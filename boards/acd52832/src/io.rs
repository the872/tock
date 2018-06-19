use core::fmt::{Arguments, Write};
use kernel::debug;
use kernel::hil::led;
use nrf5x;

struct Writer {}

static mut WRITER: Writer = Writer {};

impl Write for Writer {
    fn write_str(&mut self, _: &str) -> ::core::fmt::Result {
        Ok(())
    }
}

#[cfg(not(test))]
#[no_mangle]
#[lang = "panic_fmt"]
/// Panic handler
pub unsafe extern "C" fn panic_fmt(args: Arguments, file: &'static str, line: u32) -> ! {
    const LED1_PIN: usize = 14;
    let led = &mut led::LedLow::new(&mut nrf5x::gpio::PORT[LED1_PIN]);
    let writer = &mut WRITER;
    debug::panic(led, writer, args, file, line)
}
