#![no_std]
#![no_main]

// pick a panicking behavior
use panic_semihosting as _;
// use panic_abort as _; // requires nightly
// use panic_itm as _; // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

// Let other modules reference crate::hal so they don't have to know which hal library to use
use stm32f0xx_hal as hal;

mod commute;
mod parser;
mod periodic_timer;

#[rtic::app(device = stm32f0xx_hal::pac, peripherals = true, dispatchers = [USART2])]
mod app {
    use systick_monotonic::*;

    use stm32f0xx_hal as hal;
    use stm32f0xx_hal::pac as pac;
    use hal::prelude::*;
    use hal::gpio;
    use hal::serial::Serial;
    use heapless::spsc::Queue;
    use cortex_m::{interrupt};
    use cortex_m_semihosting::hprintln;

    use crate::commute;
    use crate::commute::{PwmDriver, PwmEnableControl, MotorState};
    use crate::parser::Parser;
    use crate::periodic_timer::PeriodicTimer;

    type UartQueue = Queue::<u8, 128>;
    static mut RXQ: UartQueue = Queue::new();
    static mut TXQ: UartQueue = Queue::new();

    const SPEED_CONTROL_FREQ: u32 = 500;

    #[monotonic(binds = SysTick, default = true, priority = 1)]
    type MyMono = Systick<100>; // 100 Hz / 10 ms granularity

    // Local resources to specific tasks (cannot be shared)
    #[local]
    struct Local {
        parser: Parser,
        pwm_enable: PwmEnableControl,
    }

    // Resources shared between tasks
    #[shared]
    struct Shared {
        // PoC: I relaly have duplicating the UART pins here. Is there a way around this?
        serial: Serial<hal::pac::USART1, gpio::gpiob::PB6<gpio::Alternate<gpio::AF0>>, gpio::gpiob::PB7<gpio::Alternate<gpio::AF0>>>,
        commutation_timer: PeriodicTimer::<pac::TIM14>,
        pwm_driver: PwmDriver,
        motor_state: MotorState,
    }

    fn read_i32(bytes: &[u8]) -> i32 {
        ((bytes[0] as i32) << 0) +
        ((bytes[1] as i32) << 8) +
        ((bytes[2] as i32) << 16) +
        ((bytes[3] as i32) << 24)
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mono = Systick::new(cx.core.SYST, 12_000_000);

        let mut flash = cx.device.FLASH;
        let mut rcc = cx.device.RCC.configure().sysclk(48.mhz()).pclk(48.mhz()).freeze(&mut flash);
        let gpioa = cx.device.GPIOA.split(&mut rcc);
        let gpiob = cx.device.GPIOB.split(&mut rcc);
        let gpiof = cx.device.GPIOF.split(&mut rcc);

        // Create a fake critical section to lie to into_alternate_afx calls.
        let fake_cs = unsafe { interrupt::CriticalSection::new() };
        // Setup GPIOs for UART
        let tx = gpiob.pb6.into_alternate_af0(&fake_cs);
        let rx = gpiob.pb7.into_alternate_af0(&fake_cs);
        // Setup GPIOs for TIM1 motor control outputs
        gpioa.pa8.into_alternate_af2(&fake_cs); // CH1
        gpiob.pb13.into_alternate_af2(&fake_cs); // CH1N
        gpioa.pa9.into_alternate_af2(&fake_cs); // CH2
        gpiob.pb14.into_alternate_af2(&fake_cs); // CH2N
        gpioa.pa10.into_alternate_af2(&fake_cs); // CH3
        gpiob.pb15.into_alternate_af2(&fake_cs); // CH3n

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
        // PoC: Setting a GPIO high can return an ERROR? It's a write to a register. It should not fail.
        oc_th_stdby1.set_high().unwrap();
        oc_th_stdby2.set_high().unwrap();

        let mut serial = Serial::usart1(cx.device.USART1, (tx, rx), 9600.bps(), &mut rcc);
        let parser = Parser::new();
        let mut pwm_driver = PwmDriver::new(cx.device.TIM1);

        serial.listen(hal::serial::Event::Rxne);

        // Enable TIM1 clocks
        // PoC: The RCC provided by the HAL has regs marked as pub(crate), so you can't enable a clock
        // using that object outside of the hal create?! Seems crazy, like I must have misunderstood
        // something, but this works around that by stealing an RCC in an unsafe block.
        let rcc_raw = unsafe { hal::pac::Peripherals::steal().RCC };
        rcc_raw.apb2enr.modify(|_, w| w.tim1en().set_bit());

        pwm_driver.init();

        let pwm_enable = pwm_driver.split_enabler();

        // Timer for commutation
        let mut commutation_timer = PeriodicTimer::<pac::TIM14>::tim14(cx.device.TIM14, 10.hz(), &mut rcc);
        // Timer for speed control
        let mut timer2 = hal::timers::Timer::<pac::TIM16>::tim16(cx.device.TIM16, SPEED_CONTROL_FREQ.hz(), &mut rcc);

        // Enable overflow interrupts for both timers
        commutation_timer.listen(hal::timers::Event::TimeOut);
        timer2.listen(hal::timers::Event::TimeOut);

        send_a_byte::spawn_after(100.millis()).unwrap();

        (
            // Initialization of shared resources
            Shared { serial, commutation_timer, pwm_driver, motor_state: MotorState::default()},
            // Initialization of task local resources
            Local { parser, pwm_enable },
            // Move the monotonic timer to the RTIC run-time, this enables
            // scheduling
            init::Monotonics(mono),
        )
    }

    // Periodically send a byte
    // This was solely a debugging thing. Needs to go at some point...
    #[task(shared=[serial])]
    fn send_a_byte(cx: send_a_byte::Context) {
        let mut serial = cx.shared.serial;
        serial.lock(|s| {
            s.write(0xa5).unwrap();
        });

        send_a_byte::spawn_after(1000.millis()).unwrap();
    }

    #[idle(shared = [&motor_state], local = [parser])]
    fn idle(cx: idle::Context) -> ! {

        let parser = cx.local.parser;
        let mut rx_q_consumer = unsafe { RXQ.split().1 };
        let motor_state = cx.shared.motor_state;

        let mut last_rx_time = monotonics::now();
        loop {
            let expire_time = last_rx_time + 200.millis();
            let cur_time = monotonics::now();
            if cur_time > expire_time {
                parser.reset();
            }

            match rx_q_consumer.dequeue() {
                Some(b) => {
                    last_rx_time = cur_time;
                    let packet_ready = parser.push_byte(b);
                    if packet_ready {
                        match parser.packet.id {
                            0 => {
                                let motor_speed = read_i32(&parser.packet.data[0..4]);
                                motor_state.set_cmd_speed(motor_speed);
                            },
                            _ => { hprintln!("Bad packet").unwrap(); }
                        }
                    }
                },
                None => (),
            }
        }
    }

    // Hardware task, bound to a hardware interrupt
    #[task(binds = USART1, priority = 2, shared = [serial])]
    fn uart1_interrupt(cx: uart1_interrupt::Context) {
        let mut serial = cx.shared.serial;
        let mut rx_q_producer = unsafe { RXQ.split().0 };
        let mut tx_q_consumer = unsafe { TXQ.split().1 };
        let mut tx_q_producer = unsafe { TXQ.split().0 };
        let usart1 = unsafe { hal::pac::Peripherals::steal().USART1 };

        serial.lock(|serial| {
            match serial.read() {
                Ok(rxbyte) => {
                    rx_q_producer.enqueue(rxbyte).ok().unwrap();
                    // Echo back for debug purposes
                    //tx_q_producer.enqueue(rxbyte).ok().unwrap();
                },
                Err(_) => () // Error can be nb::WouldBlock if empty, or Other for uart error
                             // flags such as framing error etc
            };

            let isr = (*usart1).isr.read();
            if isr.txe().bit_is_set() {
                match tx_q_consumer.dequeue() {
                    Some(b) => {
                        serial.write(b).ok().unwrap();
                    },
                    None => {
                        serial.unlisten(hal::serial::Event::Txe);
                    }
                }
            }

        });
    }

    /// Periodic timer interrupt to commutate the motor
    ///
    /// Motor speed is controlled by adjusting the overflow rate of this timer
    #[task(binds = TIM14, priority = 3, shared = [pwm_driver, &motor_state])]
    fn commute_task(cx: commute_task::Context) {
        // Clear the interrupt flag
        let tim14 = unsafe { hal::pac::Peripherals::steal().TIM14 };
        tim14.sr.modify(|_, w| w.uif().clear_bit());

        let mut pwm_driver = cx.shared.pwm_driver;
        let motor_state = cx.shared.motor_state;
        let cur_speed = motor_state.cur_speed();

        pwm_driver.lock(|pwm_driver| {
            let power = if motor_state.accelerating() {128} else {64};
            if cur_speed > 0 {
                pwm_driver.step(false, power);
            } else {
                pwm_driver.step(true, power);
            }
        });
    }

    /// Periodic speed update interrupt
    #[task(binds = TIM16, priority = 2, local = [pwm_enable], shared = [&motor_state, commutation_timer])]
    fn tim16_irq(cx: tim16_irq::Context) {
        let tim16 = unsafe { hal::pac::Peripherals::steal().TIM16 };
        tim16.sr.modify(|_, w| w.uif().clear_bit());

        let motor_state = cx.shared.motor_state;
        let mut commutation_timer = cx.shared.commutation_timer;
        let pwm_enable = cx.local.pwm_enable;

        motor_state.compute_new_state(1_000_000 / SPEED_CONTROL_FREQ);
        let cur_speed = motor_state.cur_speed();
        // Convert to a timer tick frequency, but never set it to zero. We need to keep a minimum
        // timer frequency to maintain responsiveness, because timer updates only happen on overflow
        let timer_hz = core::cmp::max(
            cur_speed.abs() as u32 * commute::COMM_STEPS as u32 / commute::SPEED_SCALE as u32,
            commute::MIN_SPEED as u32 * commute::COMM_STEPS as u32 / commute::SPEED_SCALE as u32
        );

        commutation_timer.lock(|timer| {
            timer.update_timeout(timer_hz.hz());
            pwm_enable.enable_outputs(motor_state.fets_enabled());
        });
    }
}
