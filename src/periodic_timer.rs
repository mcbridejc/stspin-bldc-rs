use crate::hal::rcc::{Clocks, Rcc};
use crate::hal::time::Hertz;
use crate::hal::timers::Event;

/// Periodic timer, very similar to the Timer in the stm32f0xx_hal, but with
/// support for changing the frequency without restarting the timer. Frequency
/// updates will take effect on the next update event (overflow).
pub struct PeriodicTimer<TIM> {
    clocks: Clocks,
    pub tim: TIM
}

macro_rules! create_timer {
    ($($TIM:ident: ($tim:ident, $timXen:ident, $timXrst:ident, $apbenr:ident, $apbrstr:ident),)+) => {
        $(
            use crate::hal::pac::$TIM;
            impl PeriodicTimer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, rcc: &Rcc) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    let rccregs = unsafe { &(*crate::hal::pac::RCC::ptr()) };
                    rccregs.$apbenr.modify(|_, w| w.$timXen().set_bit());
                    rccregs.$apbrstr.modify(|_, w| w.$timXrst().set_bit());
                    rccregs.$apbrstr.modify(|_, w| w.$timXrst().clear_bit());

                    let mut timer = PeriodicTimer {
                        clocks: rcc.clocks,
                        tim,
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> $TIM {
                    let rcc = unsafe { &(*crate::hal::pac::RCC::ptr()) };
                    // Pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // Disable timer
                    rcc.$apbenr.modify(|_, w| w.$timXen().clear_bit());
                    self.tim
                }

                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // restart counter
                    self.tim.cnt.reset();

                    // Setup prescaller and reload values to achieve desired frequency
                    self.update_timeout(timeout);

                    // Enable preload of ARR. PSC register is always shadowed.
                    self.tim.cr1.modify(|_, w| w.arpe().set_bit());

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                pub fn update_timeout<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    let frequency = timeout.into().0;
                    // If pclk is prescaled from hclk, the frequency fed into the timers is doubled
                    let tclk = if self.clocks.hclk().0 == self.clocks.pclk().0 {
                        self.clocks.pclk().0
                    } else {
                        self.clocks.pclk().0 * 2
                    };
                    let ticks = tclk / frequency;

                    let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| w.psc().bits(psc));

                    let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });
                }
            }
        )+
    }
}

create_timer! {
    TIM1: (tim1, tim1en, tim1rst, apb2enr, apb2rstr),
    TIM3: (tim3, tim3en, tim3rst, apb1enr, apb1rstr),
    TIM14: (tim14, tim14en, tim14rst, apb1enr, apb1rstr),
    TIM16: (tim16, tim16en, tim16rst, apb2enr, apb2rstr),
    TIM17: (tim17, tim17en, tim17rst, apb2enr, apb2rstr),
}