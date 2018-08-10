use riscvimac;
use riscvimac::plic;
// use cortexm0::nvic;
// use i2c;
use kernel;
// use nrf5x;
// use nrf5x::peripheral_interrupts;
// use radio;
// use uart;
use interrupts;
use uart;

pub struct E310x(());

impl E310x {
    pub unsafe fn new() -> E310x {
        E310x(())
    }
}

impl kernel::Chip for E310x {
    type MPU = ();
    type SysTick = ();

    fn mpu(&self) -> &Self::MPU {
        &self.0
    }

    fn systick(&self) -> &Self::SysTick {
        &self.0
    }

    fn service_pending_interrupts(&mut self) {
        unsafe {
            while let Some(interrupt) = plic::next_pending() {
                // debug_gpio!(0, toggle);

                if interrupt == 1 {
                    debug_gpio!(0, toggle);
                }

                match interrupt {
                    interrupts::WATCHDOG => { /* Not sure why this interrupt is happening. */}
                    interrupts::UART0 => uart::UART0.handle_interrupt(),
                    // peripheral_interrupts::GPIOTE => nrf5x::gpio::PORT.handle_interrupt(),
                    // peripheral_interrupts::RADIO => radio::RADIO.handle_interrupt(),
                    // peripheral_interrupts::RNG => nrf5x::trng::TRNG.handle_interrupt(),
                    // peripheral_interrupts::RTC1 => nrf5x::rtc::RTC.handle_interrupt(),
                    // peripheral_interrupts::TEMP => nrf5x::temperature::TEMP.handle_interrupt(),
                    // peripheral_interrupts::TIMER0 => nrf5x::timer::TIMER0.handle_interrupt(),
                    // peripheral_interrupts::TIMER1 => nrf5x::timer::ALARM1.handle_interrupt(),
                    // peripheral_interrupts::TIMER2 => nrf5x::timer::TIMER2.handle_interrupt(),
                    // peripheral_interrupts::UART0 => uart::UART0.handle_interrupt(),
                    // peripheral_interrupts::SPI0_TWI0 => {
                    //     // SPI0 and TWI0 share interrupts.
                    //     // Dispatch the correct handler.
                    //     // match (spi::SPIM0.is_enabled(), i2c::TWIM0.is_enabled()) {
                    //     match (false, i2c::TWIM0.is_enabled()) {
                    //         (false, false) => (),
                    //         (true, false) => panic!("SPI is not yet implemented"),
                    //         // spi::SPIM0.handle_interrupt(),
                    //         (false, true) => i2c::TWIM0.handle_interrupt(),
                    //         (true, true) => debug_assert!(
                    //             false,
                    //             "SPIM0 and TWIM0 cannot be \
                    //              enabled at the same time."
                    //         ),
                    //     }
                    // }
                    // peripheral_interrupts::SPI1_TWI1 => {
                    //     // SPI1 and TWI1 share interrupts.
                    //     // Dispatch the correct handler.
                    //     // match (spi::SPIM1.is_enabled(), i2c::TWIM1.is_enabled()) {
                    //     match (false, i2c::TWIM1.is_enabled()) {
                    //         (false, false) => (),
                    //         (true, false) => panic!("SPI is not yet implemented"),
                    //         // spi::SPIM1.handle_interrupt(),
                    //         (false, true) => i2c::TWIM1.handle_interrupt(),
                    //         (true, true) => debug_assert!(
                    //             false,
                    //             "SPIM1 and TWIM1 cannot be \
                    //              enabled at the same time."
                    //         ),
                    //     }
                    // }
                    _ => debug!("PLIC index not supported by Tock"),
                }
                plic::clear_pending(interrupt);
                // let n = nvic::Nvic::new(interrupt);
                // n.clear_pending();
                // n.enable();
            }
        }
    }

    fn has_pending_interrupts(&self) -> bool {
        unsafe { plic::has_pending() }
    }

    fn sleep(&self) {
        unsafe {
            // riscvimac::support::wfi();
            riscvimac::support::nop();
        }
    }

    unsafe fn atomic<F, R>(&self, f: F) -> R
    where
        F: FnOnce() -> R,
    {
        riscvimac::support::atomic(f)
    }
}
