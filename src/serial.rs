

pub mod serial {
    use crate::interrupt;
    use crate::hal;
    use crate::hal::serial::Serial;
    use crate::hal::gpio;
    use crate::hal::pac;
    use crate::hal::prelude::*;
    use heapless::spsc::{Consumer, Producer, Queue};
    static mut TX_Q_CONSUMER: Option<Consumer<u8, 128>> = None;
    static mut TX_Q_PRODUCER: Option<Producer<u8, 128>> = None;
    static mut RX_Q_CONSUMER: Option<Consumer<u8, 128>> = None;
    static mut RX_Q_PRODUCER: Option<Producer<u8, 128>> = None;
    // static mut RXQ: Queue<u8, 128> = Queue::new();
    // static mut TXQ: Queue<u8, 128> = Queue::new();
    static mut SERIAL: Option<
        Serial<
            pac::USART1,
            gpio::gpiob::PB6<gpio::Alternate<gpio::AF0>>,
            gpio::gpiob::PB7<gpio::Alternate<gpio::AF0>>
        >
    > = None;

    /// Must be called once during application initialization
    pub fn init(rcc: &mut hal::rcc::Rcc, irq_prio: u8) {
        let device = unsafe { pac::Peripherals::steal() };
        //let mut nvic = unsafe { cortex_m::Peripherals::steal().NVIC };
        let core = unsafe { pac::CorePeripherals::steal() };
        let gpiob = device.GPIOB.split(rcc);
        let mut nvic = core.NVIC;

        static mut RX_Q: Queue<u8, 128>= Queue::new();
        static mut TX_Q: Queue<u8, 128> = Queue::new();
        unsafe {
            let (p, c) = RX_Q.split();
            RX_Q_CONSUMER = Some(c);
            RX_Q_PRODUCER = Some(p);
        }
        unsafe {
            let (p, c) = TX_Q.split();
            TX_Q_CONSUMER = Some(c);
            TX_Q_PRODUCER = Some(p);
        }

        // Setup GPIOs for UART
        let (tx, rx) = cortex_m::interrupt::free(|cs| {
            ( 
                gpiob.pb6.into_alternate_af0(&cs),
                gpiob.pb7.into_alternate_af0(&cs),
            )
        });
        let mut serial = Serial::usart1(device.USART1, (tx, rx), 9600.bps(), rcc);
        serial.listen(hal::serial::Event::Rxne);
        unsafe { SERIAL = Some(serial) };

        unsafe {
            nvic.set_priority(crate::pac::Interrupt::USART1, irq_prio);
            pac::NVIC::unmask(crate::pac::Interrupt::USART1);
        }
    }

    pub fn read_byte() -> Option<u8> {
        let rx_q_consumer = unsafe { RX_Q_CONSUMER.as_mut().unwrap_unchecked() };
        return rx_q_consumer.dequeue();
    }

    pub fn send_byte(b: u8) {
        let tx_q_producer = unsafe { TX_Q_PRODUCER.as_mut().unwrap_unchecked() };
        let _ = tx_q_producer.enqueue(b);
    }

    #[interrupt]
    fn USART1() {
        let serial = unsafe{ SERIAL.as_mut().unwrap_unchecked() };
        let rx_q_producer = unsafe { RX_Q_PRODUCER.as_mut().unwrap_unchecked() };
        let tx_q_consumer = unsafe { TX_Q_CONSUMER.as_mut().unwrap_unchecked() };
        let usart1 = unsafe { crate::hal::pac::Peripherals::steal().USART1 };

        // Read any available bytes from the serial port
        match serial.read() {
            Ok(rxbyte) => {
                rx_q_producer.enqueue(rxbyte).ok().unwrap();
                // Echo back for debug purposes
                //tx_q_producer.enqueue(rxbyte).ok().unwrap();
            },
            Err(_) => () // Error can be nb::WouldBlock if empty, or Other for uart error
                            // flags such as framing error etc
        };

        // Check if there is room to transmit a byte
        let isr = (*usart1).isr.read();
        if isr.txe().bit_is_set() {
            match tx_q_consumer.dequeue() {
                Some(b) => {
                    // If there's a byte available in the Q, send it
                    serial.write(b).ok().unwrap();
                },
                None => {
                    // If the Q is empty, mask the TXE interrupt. It must be re-enabled when data is written to the queue
                    serial.unlisten(hal::serial::Event::Txe);
                }
            }
        }

    }
}