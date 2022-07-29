use stm32f0xx_hal as hal;
use libm::sinf;
use core::cmp::{max, min};
use core::sync::atomic;

type TimRegs = hal::stm32::TIM1;

// Number of sine commutation steps each electrical cycle
pub const COMM_STEPS: usize = 30;
// Maximum acceleration (initial value)
const DEFAULT_ACCEL: i32 = 2800; // RPM per second (electrical!)
// Scale factor used for fixed point calculation
pub const SPEED_SCALE: i32 = 100;
// Sets the frequency of the PWM output
// The counter runs at 48MHz, so the pwm frequency is 48MHz/RELOAD.
const RELOAD: u32 = 1600;
// Sets the maximum output duty cycle (i.e. motor voltage)
const DEFAULT_MAXDRIVE: f32 = 0.45;

// The minimum speed before the motor is shut off.
pub const MIN_SPEED: i32 = 35;
// Sets the delay between turning the FETs on and beginning acceleration. This creates a more
// reliable startup by allowing the stator to align with the commutation angle and settle
pub const ENABLE_DELAY_US: u32 = 750_000;

// The percentage of total duty cycle used at zero velocity while not accelerating
pub const DEFAULT_BASE_POWER: u32 = 50;
// An additional fraction of total available duty cycle applied while changing velocity
pub const DEFAULT_ACCEL_POWER: u32 = 25;
// Scale factor to increase motor power with increasing RPM Based initially on KV, but adjusted
// empirically to achieve constant current over speed
// Necessary because back-EMF increases with motor speed, cancelling some applied voltage
pub const DEFAULT_V_PER_MICRORPS: u32 = 10*128;

pub struct MotorState {
    cur_speed: atomic::AtomicI32, // RPS * SPEED_SCALEconst
    cmd_speed: atomic::AtomicI32, // RPS * SPEED_SCALE
    accelerating: atomic::AtomicBool,
    enabled_count: atomic::AtomicU32,
    accel_rate: atomic::AtomicI32, // electrical RPM/s
    base_power: atomic::AtomicU32, // normalized power, range 0-127
    accel_power: atomic::AtomicU32, // Normalized power, range 0-127
    power_per_microrps: atomic::AtomicU32, 
}

impl Default for MotorState {
    fn default() -> MotorState {
        MotorState {
            cur_speed: atomic::AtomicI32::new(0),
            cmd_speed: atomic::AtomicI32::new(0),
            accelerating: atomic::AtomicBool::new(false),
            enabled_count: atomic::AtomicU32::new(0),
            accel_rate: atomic::AtomicI32::new(DEFAULT_ACCEL),
            base_power: atomic::AtomicU32::new(DEFAULT_BASE_POWER),
            accel_power: atomic::AtomicU32::new(DEFAULT_ACCEL_POWER),
            power_per_microrps: atomic::AtomicU32::new(DEFAULT_V_PER_MICRORPS),
        }
    }
}

impl MotorState {
    pub fn fets_enabled(&self) -> bool {
        self.enabled_count.load(atomic::Ordering::Relaxed) > 0
    }

    pub fn accelerating(&self) -> bool {
        self.accelerating.load(atomic::Ordering::Relaxed)
    }

    pub fn cur_speed(&self) -> i32 {
        self.cur_speed.load(atomic::Ordering::Relaxed)
    }

    pub fn cmd_speed(&self) -> i32 {
        self.cmd_speed.load(atomic::Ordering::Relaxed)
    }

    pub fn set_cmd_speed(&self, val: i32) {
        self.cmd_speed.store(val, atomic::Ordering::Relaxed);
    }

    pub fn set_accel_rate(&self, val: i32) {
        self.accel_rate.store(val, atomic::Ordering::Relaxed);
    }

    pub fn set_base_power(&self, val: u32) {
        self.base_power.store(val, atomic::Ordering::Relaxed);
    }

    pub fn set_accel_power(&self, val: u32) {
        self.accel_power.store(val, atomic::Ordering::Relaxed);
    }

    pub fn set_power_per_microrps(&self, val: u32) {
        self.power_per_microrps.store(val, atomic::Ordering::Relaxed);
    }

    /* Perform acceleration/state update */
    pub fn compute_new_state(&self, period_us: u32) {
        // Convert RPM/s to RPS scaled units
        let accel_per_s: u32 = (self.accel_rate.load(atomic::Ordering::Relaxed) as f32 * SPEED_SCALE as f32 / 60.) as u32;
        let max_accel = max(accel_per_s * period_us / 1000000, 1);
        let cmd_speed = self.cmd_speed();
        let cur_speed = self.cur_speed();
        let enabled_count = self.enabled_count.load(atomic::Ordering::Relaxed);
        let ds = min(max_accel, (cmd_speed - cur_speed).abs() as u32);

        if enabled_count < ENABLE_DELAY_US {
            // We're not enabled, but now we should be
            if cmd_speed.abs() > 0 {
                self.enabled_count.store(enabled_count + period_us,atomic::Ordering::Relaxed);
            }
        } else if cur_speed.abs() < MIN_SPEED && cmd_speed.abs() < MIN_SPEED {
            // Switch to disabled
            self.enabled_count.store(0, atomic::Ordering::Relaxed);
        } else {
            if cmd_speed > cur_speed {
                self.cur_speed.store(cur_speed + ds as i32, atomic::Ordering::Relaxed);
                self.accelerating.store(true, atomic::Ordering::Relaxed);
            } else if cmd_speed < cur_speed {
                self.cur_speed.store(cur_speed - ds as i32, atomic::Ordering::Relaxed);
                self.accelerating.store(true, atomic::Ordering::Relaxed);
            } else {
                self.accelerating.store(false,atomic::Ordering::Relaxed);
            }
        }
    }

    /* Calculate the current drive power based on on current state and configuration parameters  */
    pub fn compute_power(&self) -> u32 {
        let mut power = self.base_power.load(atomic::Ordering::Relaxed);
        if self.accelerating() {
            power += self.accel_power.load(atomic::Ordering::Relaxed);
        }
        power += self.power_per_microrps.load(atomic::Ordering::Relaxed) * self.cur_speed().abs() as u32 / 1_000_000;
        power = core::cmp::min(power, 128);
        power
    }
}

// TODO: Configurable control parameters
// #[derive(Default, Clone)]
// pub struct MotorConfig {
//     accel_per_ms: u32, // RPS * SPEED_SCALE / ms
//     max_voltage: f32, // Duty cycle [0..1]
// }

pub struct PwmDriver {
    tim: TimRegs,
    sine_table: [u16; COMM_STEPS],
    step: u32,
}

pub struct PwmEnableControl {
    tim: *const TimRegs
}

// Needed for RTIC to pass the *const pointer to an interrupt
unsafe impl Send for PwmEnableControl {}

impl PwmEnableControl {
    pub fn enable_outputs(&mut self, enable: bool) {
        // Safety: This is not strictly safe, because it's possible to create more than
        // one of these, and modifying the register isn't atomic.
        // It would be better to figure out how to ensure that only one PwmEnableControl
        // can exist at once. Right now though I'm satisfied that I will only create one.
        let tim = unsafe { &(*self.tim) };
        if enable {
            tim.bdtr.modify(|_, w| w.moe().set_bit());
        } else {
            tim.bdtr.modify(|_, w| w.moe().clear_bit());
        }
    }
}

impl PwmDriver {
    pub fn new(tim: TimRegs) -> Self {
        Self { tim, sine_table: [0; COMM_STEPS], step: 0}
    }

    pub fn init(&mut self) {
        self.set_commutation_table(DEFAULT_MAXDRIVE);

        self.tim.arr.write(|w| unsafe { w.bits(RELOAD) });
        self.tim.cr1.modify(|_, w| w.arpe().set_bit());
        // Setup first three channels in PWM mode
        self.tim.ccmr1_output().modify(|_, w| w.oc1m().pwm_mode1().oc2m().pwm_mode1());
        self.tim.ccmr2_output().modify(|_, w| w.oc3m().pwm_mode1());
        // Enable compare outputs
        self.tim.ccer.modify(|_, w|
            w.cc1ne().set_bit()
            .cc1e().set_bit()
            .cc2ne().set_bit()
            .cc2e().set_bit()
            .cc3ne().set_bit()
            .cc3e().set_bit()
            );

        // Setup dead-time and set master output enable
        self.tim.bdtr.modify(|_, w| unsafe { w.dtg().bits(24).moe().set_bit() });
        // Enable timer
        self.tim.cr1.modify(|_, w| w.cen().set_bit());

        self.tim.ccr1.write(|w| w.ccr().bits((RELOAD/2) as u16));
        self.tim.ccr2.write(|w| w.ccr().bits((RELOAD/2) as u16));
        self.tim.ccr3.write(|w| w.ccr().bits((RELOAD/2) as u16));
    }

    pub fn set_commutation_table(&mut self, maxdrive: f32) {
        for i in 0..COMM_STEPS {
            let phase: f32 = 2.0 * core::f32::consts::PI * (i as f32 / COMM_STEPS as f32);
            let a: f32 = RELOAD as f32 / 2.0;
            self.sine_table[i] = ((1.0 + sinf(phase)* maxdrive) * a ) as u16;
        }
    }

    /// Increment the commutation position one step forward or backwards
    pub fn step(&mut self, reverse: bool, power: u32) {
        self.step = if reverse {
            (self.step as i32 - 1).rem_euclid(COMM_STEPS as i32) as u32
        } else {
            (self.step as i32 + 1).rem_euclid(COMM_STEPS as i32) as u32
        };

        let s0 = self.step;
        let s1 = (self.step + COMM_STEPS as u32 / 3) % COMM_STEPS as u32;
        let s2 = (self.step + COMM_STEPS as u32 * 2 / 3) % COMM_STEPS as u32;
        self.tim.ccr1.write(|w| w.ccr().bits((self.sine_table[s0 as usize] as u32 * power / 128) as u16));
        self.tim.ccr2.write(|w| w.ccr().bits((self.sine_table[s1 as usize] as u32 * power / 128) as u16));
        self.tim.ccr3.write(|w| w.ccr().bits((self.sine_table[s2 as usize] as u32 * power / 128) as u16));
    }

    /// Get a PwmEnableControl which allows toggling the output enable without holding a mut
    /// reference to the PwmDriver.
    ///
    /// This was done because I don't want the interrupt which updates MOE to block the commutation
    /// interrupt. It's not necessary to lock while writing MOE, as long as no other process is also
    /// using BDTR.
    ///
    /// I assume it's possible somehow to borrow just the BDTR register in a way that ensures only
    /// one owner can hold it, but frankly just figuring out how to split this off was enough of a
    /// chore for one trivial register write and I am done with this at least for today.
    pub fn split_enabler(&self) -> PwmEnableControl {
        PwmEnableControl { tim: core::ptr::addr_of!(self.tim) }
    }

}
