use stm32f0xx_hal as hal;
use libm::sinf;
use core::cmp::{max, min};
use core::sync::atomic;

type TimRegs = hal::stm32::TIM1;

// Number of sine commutation steps each electrical cycle
const COMM_STEPS: usize = 30;
// Maximum acceleration
const ACCEL: f32 = 2800.; // RPM per second (electrical!)
// Scale factor used for fixed point calculation
const SPEED_SCALE: i32 = 100;
// Sets the frequency of the PWM output
// The counter runs at 48MHz, so the pwm frequency is 48MHz/RELOAD.
const RELOAD: u32 = 1600;
// Sets the maximum output duty cycle (i.e. motor voltage)
const MAXDRIVE: f32 = 0.25;
// The minimum commutation step frequency before the motor is shut off.
pub const MIN_COMMUTATE_HZ: u32 = 10;

#[derive(Default)]
pub struct MotorState {
    pub cur_speed: atomic::AtomicI32, // RPS * SPEED_SCALE
    pub cmd_speed: atomic::AtomicI32, // RPS * SPEED_SCALE
    pub accelerating: atomic::AtomicBool,
}

// TODO: Configurable control parameters
// #[derive(Default, Clone)]
// pub struct MotorConfig {
//     accel_per_ms: u32, // RPS * SPEED_SCALE / ms
//     max_voltage: f32, // Duty cycle [0..1]
// }

/// Computes a new speed applying acceleration limits
pub fn compute_new_speed(state: &MotorState, period_us: u32) {
    // Convert RPM/s to RPS scaled units
    const ACCEL_PER_S: u32 = (ACCEL * SPEED_SCALE as f32 / 60.) as u32;
    let max_accel = max(ACCEL_PER_S * period_us / 1000000, 1);
    let cmd_speed = state.cmd_speed.load(atomic::Ordering::Relaxed);
    let cur_speed = state.cur_speed.load(atomic::Ordering::Relaxed);
    let ds = min(max_accel, (cmd_speed - cur_speed).abs() as u32);
    if cmd_speed > cur_speed {
        state.cur_speed.store(cur_speed + ds as i32, atomic::Ordering::Relaxed);
        state.accelerating.store(true, atomic::Ordering::Relaxed);
    } else if cmd_speed < cur_speed {
        state.cur_speed.store(cur_speed - ds as i32, atomic::Ordering::Relaxed);
        state.accelerating.store(true, atomic::Ordering::Relaxed);
    } else {
        state.accelerating.store(false,atomic::Ordering::Relaxed);
    }
}

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

        for i in 0..COMM_STEPS {
            let phase: f32 = 2.0 * core::f32::consts::PI * (i as f32 / COMM_STEPS as f32);
            let a: f32 = RELOAD as f32 / 2.0;
            self.sine_table[i] = ((1.0 + sinf(phase)* MAXDRIVE) * a ) as u16;
        }

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
