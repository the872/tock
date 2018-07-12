// use core::fmt::Write;
use core::panic::PanicInfo;

use cortexm4;
use kernel::debug;
use kernel::hil::led;
use nrf5x;

// struct Writer {}

// static mut WRITER: Writer = Writer {};

// impl Write for Writer {
//     fn write_str(&mut self, _: &str) -> ::core::fmt::Result {
//         Ok(())
//     }
// }

/// Panic.
#[cfg(not(test))]
#[no_mangle]
#[panic_implementation]
pub unsafe extern "C" fn panic_fmt(pi: &PanicInfo) -> ! {
    const LED1_PIN: usize = 22;
    let led = &mut led::LedLow::new(&mut nrf5x::gpio::PORT[LED1_PIN]);
    // let writer = &mut WRITER;
    // debug::panic(&mut [led], writer, pi, &cortexm4::support::nop)
    debug::panic_blink_forever(&mut [led])
}
