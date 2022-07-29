#![no_std]
#![no_main]
#![allow(dead_code)]

mod commute;
mod parser;
mod periodic_timer;
mod serial;

use core::mem::MaybeUninit;

// pick a panicking behavior
use panic_semihosting as _;
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

// Let other modules reference crate::hal so they don't have to know which hal library to use
use stm32f0xx_hal as hal;
use hal::pac as pac;
use hal::prelude::*;
use cortex_m_semihosting::hprintln;
use cortex_m_rt::{entry};
use stm32f0::stm32f0x1::{interrupt};
use parser::Parser;
use systick_monotonic::Systick;
use systick_monotonic::fugit::*;
use rtic_monotonic::Monotonic;

use crate::commute::{PwmDriver, PwmEnableControl, MotorState};
use crate::periodic_timer::PeriodicTimer;

// Declare global data shared with IRQs. It must be initialized later in main(), hence the
// Option/MaybeUninit wrappings. I don't see any advantage to using MaybeUninit vs Option here; but
// I wanted to demo the two options
static mut PWM_DRIVER: Option<PwmDriver> = None;
static mut MOTOR_STATE: MaybeUninit::<MotorState> = MaybeUninit::uninit();
static mut PWM_ENABLE_CONTROL: Option<PwmEnableControl> = None;
static mut COMMUTATION_TIMER: Option<PeriodicTimer::<pac::TIM14>> = None;

/*** CONFIGURATION OPTIONS ***/
/// How frequently to update motor state
const SPEED_CONTROL_FREQ: u32 = 500;

fn read_i32(bytes: &[u8]) -> i32 {
    ((bytes[0] as i32) << 0) +
    ((bytes[1] as i32) << 8) +
    ((bytes[2] as i32) << 16) +
    ((bytes[3] as i32) << 24)
}

fn read_u32(bytes: &[u8]) -> u32 {
    ((bytes[0] as u32) << 0) +
    ((bytes[1] as u32) << 8) +
    ((bytes[2] as u32) << 16) +
    ((bytes[3] as u32) << 24)
}

#[entry]
fn main() -> ! {
    let device = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut flash = device.FLASH;
    let mut rcc = device.RCC.configure().sysclk(48.mhz()).pclk(48.mhz()).freeze(&mut flash);
    let mut nvic = core.NVIC;
    let gpioa = device.GPIOA.split(&mut rcc);
    let gpiob = device.GPIOB.split(&mut rcc);
    let gpiof = device.GPIOF.split(&mut rcc);

    // PoC: sysclk().0 works because it the type is `struct Hertz(pub u32)`, but this sure seems
    // like a great example of obfuscation.
    let mut monotonic: Systick<100> = Systick::new(core.SYST, rcc.clocks.sysclk().0);

    let fake_cs = unsafe { cortex_m::interrupt::CriticalSection::new() };
    // Setup GPIOs for TIM1 motor control outputs
    gpioa.pa8.into_alternate_af2(&fake_cs); // CH1
    gpiob.pb13.into_alternate_af2(&fake_cs); // CH1N
    gpioa.pa9.into_alternate_af2(&fake_cs); // CH2
    gpiob.pb14.into_alternate_af2(&fake_cs); // CH2N
    gpioa.pa10.into_alternate_af2(&fake_cs); // CH3
    gpiob.pb15.into_alternate_af2(&fake_cs); // CH3N

    // Setup GPIOs for standby / overcurrent setting
    // When both are low, the gate driver + analog circuits are in standby.
    // Other values select the OC threshold
    // | STBY2 | STBY1 | Threshold |
    // |-------|-------|-----------|
    // |   0   |   1   | 100mV     |
    // |   1   |   0   | 250mV     |
    // |   1   |   1   | 500mV     |
    let mut oc_th_stdby1 = gpiof.pf6.into_push_pull_output(&fake_cs);
    let mut oc_th_stdby2 = gpiof.pf7.into_push_pull_output(&fake_cs);
    oc_th_stdby1.set_high().unwrap();
    oc_th_stdby2.set_high().unwrap();

    serial::serial::init(&mut rcc, 1);

    let mut pwm_driver = PwmDriver::new(device.TIM1);
    let pwm_enable = pwm_driver.split_enabler();

    // Enable TIM1 clocks
    // PoC: The RCC provided by the HAL has regs marked as pub(crate), so you can't enable a clock
    // using that object outside of the hal create?! Seems crazy, like I must have misunderstood
    // something, but this works around that by stealing an RCC in an unsafe block.
    let rcc_raw = unsafe { hal::pac::Peripherals::steal().RCC };
    rcc_raw.apb2enr.modify(|_, w| w.tim1en().set_bit());

    pwm_driver.init();

    // Timer for commutation
    let mut commutation_timer = PeriodicTimer::<pac::TIM14>::tim14(device.TIM14, 10.hz(), &mut rcc);
    // Timer for speed control
    let mut timer2 = hal::timers::Timer::<pac::TIM16>::tim16(device.TIM16, SPEED_CONTROL_FREQ.hz(), &mut rcc);

    // Enable overflow interrupts for both timers
    commutation_timer.listen(hal::timers::Event::TimeOut);
    timer2.listen(hal::timers::Event::TimeOut);

    // Store objects used by interrupts to static globals
    unsafe {
        MOTOR_STATE.write(MotorState::default());
        PWM_DRIVER = Some(pwm_driver);
        PWM_ENABLE_CONTROL = Some(pwm_enable);
        COMMUTATION_TIMER = Some(commutation_timer);
    }

    // Set IRQ priority and enable in the NVIC
    unsafe {
        nvic.set_priority(pac::Interrupt::TIM14, 3);
        nvic.set_priority(pac::Interrupt::TIM16, 2);
        pac::NVIC::unmask(pac::Interrupt::TIM14);
        pac::NVIC::unmask(pac::Interrupt::TIM16);
    }

    loop {
        let mut parser = Parser::new();
        let motor_state = unsafe { MOTOR_STATE.assume_init_ref() };

        let last_rx_time = monotonic.now();
        loop {
            let expire_time = last_rx_time + 200u32.millis();
            let cur_time = monotonic.now();
            if cur_time > expire_time {
                parser.reset();
            }

            match serial::serial::read_byte() {
                Some(b) => {
                    //last_rx_time = cur_time;
                    let packet_ready = parser.push_byte(b);
                    if packet_ready {
                        match parser.packet.id {
                            0 => {
                                let motor_speed = read_i32(&parser.packet.data[0..4]);
                                motor_state.set_cmd_speed(motor_speed);
                            },
                            1 => {
                                let accel_rate = read_i32(&parser.packet.data[0..4]);
                                motor_state.set_accel_rate(accel_rate);
                            },
                            2 => {
                                let base_power = read_u32(&parser.packet.data[0..4]);
                                motor_state.set_base_power(base_power);
                            },
                            3 => {
                                let accel_power = read_u32(&parser.packet.data[0..4]);
                                motor_state.set_accel_power(accel_power);
                            },
                            4 => {
                                let power_per_microrps = read_u32(&parser.packet.data[0..4]);
                                motor_state.set_power_per_microrps(power_per_microrps);
                            },
                            5 => {
                                // We are sharing the PWM driver with the TIM14 interrupt here
                                // We are updating the commutation table. As long as the writes of
                                // individual sine_table entries are atomic, it is fine, and I believe
                                // they are because they are < 32 bits. 
                                let pwm_driver = unsafe { PWM_DRIVER.as_mut().unwrap_unchecked() };
                                let maxdrive = read_u32(&parser.packet.data[0..4]);
                                pwm_driver.set_commutation_table(maxdrive as f32 / 1024.);
                            },
                            _ => { hprintln!("Bad packet").unwrap(); }
                        }
                    }
                },
                None => (),
            }
        }
    }
}

/// Commutation interrupt - The rate of fire depends on motor velocity.
#[interrupt]
fn TIM14() {
    // Clear the interrupt flag
    let tim14 = unsafe { hal::pac::Peripherals::steal().TIM14 };
    tim14.sr.modify(|_, w| w.uif().clear_bit());

    // Unwrap unchecked saves a few instructions, but you had better not forget to put Some value
    // into the global before this interrupt is enabled!
    let pwm_driver = unsafe { PWM_DRIVER.as_mut().unwrap_unchecked() };
    let motor_state = unsafe { MOTOR_STATE.assume_init_ref() };
    let power = motor_state.compute_power();
    let reverse = motor_state.cur_speed() < 0;
    pwm_driver.step(reverse, power);
}

/// Periodic speed update interrupt - Fires at constant rate to manage motor acceleration
#[interrupt]
fn TIM16() {
    // Clear the interrupt flag
    let tim16 = unsafe { hal::pac::Peripherals::steal().TIM16 };
    tim16.sr.modify(|_, w| w.uif().clear_bit());

    // Unsafe even for immutable refs in order to do unchecked unwrap for efficiency
    let commutation_timer = unsafe { COMMUTATION_TIMER.as_mut().unwrap_unchecked() };
    let pwm_enable = unsafe { PWM_ENABLE_CONTROL.as_mut().unwrap_unchecked() };
    let motor_state = unsafe { MOTOR_STATE.assume_init_ref() };

    motor_state.compute_new_state(1_000_000 / SPEED_CONTROL_FREQ);
    let cur_speed = motor_state.cur_speed();
    // Convert to a timer tick frequency, but never set it to zero. We need to keep a minimum
    // timer frequency to maintain responsiveness, because timer updates only happen on overflow
    let timer_hz = core::cmp::max(
        cur_speed.abs() as u32 * commute::COMM_STEPS as u32 / commute::SPEED_SCALE as u32,
        commute::MIN_SPEED as u32 * commute::COMM_STEPS as u32 / commute::SPEED_SCALE as u32
    );

    commutation_timer.update_timeout(timer_hz.hz());
    pwm_enable.enable_outputs(motor_state.fets_enabled());
}
