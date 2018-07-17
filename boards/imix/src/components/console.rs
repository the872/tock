//! Component for Console on the imix board.
//!
//! This provides one Component, ConsoleComponent, which implements
//! a buffered read/write console over a serial port. This is typically
//! USART3 (the DEBUG USB connector). It attaches kernel debug output
//! to this console (for panic!, print!, debug!, etc.).
//!
//! Usage
//! -----
//! ```rust
//! let spi_syscalls = SpiSyscallComponent::new(mux_spi).finalize();
//! let rf233_spi = SpiComponent::new(mux_spi).finalize();
//! ```

// Author: Philip Levis <pal@cs.stanford.edu>
// Last modified: 6/20/2018

#![allow(dead_code)] // Components are intended to be conditionally included

use capsules::console;
use capsules::virtual_uart::{UartDevice,UartMux};
use kernel;
use kernel::component::Component;
use kernel::Grant;
use kernel::hil::uart::UART;

pub struct ConsoleComponent {
    uart: &'static UartMux<'static>,
}

impl ConsoleComponent {
    pub fn new(uart: &'static UartMux<'static>) -> ConsoleComponent {
        ConsoleComponent {
            uart: uart,
        }
    }
}

impl Component for ConsoleComponent {
    type Output = &'static console::Console<'static, UartDevice<'static>>;

    unsafe fn finalize(&mut self) -> Self::Output {
        let device = static_init!(UartDevice<'static>,
                                  UartDevice::new(self.uart, false));
        let console = static_init!(
            console::Console<UartDevice<'static>>,
            console::Console::new(
                device,
                &mut console::WRITE_BUF,
                &mut console::READ_BUF,
                Grant::create()
            )
        );
        device.set_client(console);

        // Attach the kernel debug interface to this console
        let kc = static_init!(console::App, console::App::default());
        kernel::debug::assign_console_driver(Some(console), kc);

        console
    }
}
