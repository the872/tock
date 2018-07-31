//! Board file for EK-TM4C1294XL development platform.

#![no_std]
#![no_main]
#![feature(asm, const_fn, panic_implementation)]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(debug, static_init)]
extern crate kernel;
extern crate cortexm4;
extern crate tm4c129x;

use capsules::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules::virtual_uart::{UartDevice, UartMux};
use kernel::hil;
use kernel::hil::Controller;
use kernel::Platform;

#[macro_use]
pub mod io;

// State for loading and holding applications.

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::procs::FaultResponse = kernel::procs::FaultResponse::Panic;

// RAM to be shared by all application processes.
#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 10240] = [0; 10240];

// Actual memory for holding the active process structures.
static mut PROCESSES: [Option<&'static kernel::procs::ProcessType>; NUM_PROCS] =
    [None, None, None, None];

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

/// A structure representing this platform that holds references to all
/// capsules for this platform.
struct EkTm4c1294xl {
    console: &'static capsules::console::Console<'static, UartDevice<'static>>,
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        VirtualMuxAlarm<'static, tm4c129x::gpt::AlarmTimer>,
    >,
    gpio: &'static capsules::gpio::GPIO<'static, tm4c129x::gpio::GPIOPin>,
    ipc: kernel::ipc::IPC,
    led: &'static capsules::led::LED<'static, tm4c129x::gpio::GPIOPin>,
    button: &'static capsules::button::Button<'static, tm4c129x::gpio::GPIOPin>,
}

/// Mapping of integer syscalls to objects that implement syscalls.
impl Platform for EkTm4c1294xl {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            _ => f(None),
        }
    }
}

/// Reset Handler
#[no_mangle]
pub unsafe fn reset_handler() {
    tm4c129x::init();

    tm4c129x::sysctl::PSYSCTLM
        .setup_system_clock(tm4c129x::sysctl::SystemClockSource::PllPioscAt120MHz);

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux = static_init!(
        UartMux<'static>,
        UartMux::new(&tm4c129x::uart::UART0, &mut capsules::virtual_uart::RX_BUF)
    );
    hil::uart::UART::set_client(&tm4c129x::uart::UART0, uart_mux);

    // Create a UartDevice for the console.
    let console_uart = static_init!(UartDevice, UartDevice::new(uart_mux, true));
    console_uart.setup();

    let console = static_init!(
        capsules::console::Console<UartDevice>,
        capsules::console::Console::new(
            console_uart,
            115200,
            &mut capsules::console::WRITE_BUF,
            &mut capsules::console::READ_BUF,
            board_kernel.create_grant()
        )
    );
    hil::uart::UART::set_client(console_uart, console);
    tm4c129x::uart::UART0.specify_pins(&tm4c129x::gpio::PA[0], &tm4c129x::gpio::PA[1]);

    // Create virtual device for kernel debug.
    let debugger_uart = static_init!(UartDevice, UartDevice::new(uart_mux, false));
    debugger_uart.setup();
    let debugger = static_init!(
        kernel::debug::DebugWriter,
        kernel::debug::DebugWriter::new(
            debugger_uart,
            &mut kernel::debug::OUTPUT_BUF,
            &mut kernel::debug::INTERNAL_BUF,
        )
    );
    hil::uart::UART::set_client(debugger_uart, debugger);

    let debug_wrapper = static_init!(
        kernel::debug::DebugWriterWrapper,
        kernel::debug::DebugWriterWrapper::new(debugger)
    );
    kernel::debug::set_debug_writer_wrapper(debug_wrapper);

    // Alarm
    let alarm_timer = &tm4c129x::gpt::TIMER0;
    let mux_alarm = static_init!(
        MuxAlarm<'static, tm4c129x::gpt::AlarmTimer>,
        MuxAlarm::new(alarm_timer)
    );
    alarm_timer.configure(mux_alarm);
    let virtual_alarm1 = static_init!(
        VirtualMuxAlarm<'static, tm4c129x::gpt::AlarmTimer>,
        VirtualMuxAlarm::new(mux_alarm)
    );
    let alarm = static_init!(
        capsules::alarm::AlarmDriver<'static, VirtualMuxAlarm<'static, tm4c129x::gpt::AlarmTimer>>,
        capsules::alarm::AlarmDriver::new(virtual_alarm1, board_kernel.create_grant())
    );
    virtual_alarm1.set_client(alarm);

    // LEDs
    let led_pins = static_init!(
        [(
            &'static tm4c129x::gpio::GPIOPin,
            capsules::led::ActivationMode
        ); 4],
        [
            (
                &tm4c129x::gpio::PF[0],
                capsules::led::ActivationMode::ActiveHigh
            ), // D1
            (
                &tm4c129x::gpio::PF[4],
                capsules::led::ActivationMode::ActiveHigh
            ), // D2
            (
                &tm4c129x::gpio::PN[0],
                capsules::led::ActivationMode::ActiveHigh
            ), // D3
            (
                &tm4c129x::gpio::PN[1],
                capsules::led::ActivationMode::ActiveHigh
            ), // D4
        ]
    );
    let led = static_init!(
        capsules::led::LED<'static, tm4c129x::gpio::GPIOPin>,
        capsules::led::LED::new(led_pins)
    );

    // BUTTONs
    let button_pins = static_init!(
        [(&'static tm4c129x::gpio::GPIOPin, capsules::button::GpioMode); 2],
        [
            (
                &tm4c129x::gpio::PJ[0],
                capsules::button::GpioMode::LowWhenPressed
            ), //USR_SW1
            (
                &tm4c129x::gpio::PJ[1],
                capsules::button::GpioMode::LowWhenPressed
            ), //USR_SW2
        ]
    );
    let button = static_init!(
        capsules::button::Button<'static, tm4c129x::gpio::GPIOPin>,
        capsules::button::Button::new(button_pins, board_kernel.create_grant())
    );
    for &(btn, _) in button_pins.iter() {
        btn.set_client(button);
    }

    // set GPIO driver controlling remaining GPIO pins
    let gpio_pins = static_init!(
        [&'static tm4c129x::gpio::GPIOPin; 4],
        [
            &tm4c129x::gpio::PM[3],
            &tm4c129x::gpio::PH[2],
            &tm4c129x::gpio::PC[6],
            &tm4c129x::gpio::PC[7],
        ]
    );
    let gpio = static_init!(
        capsules::gpio::GPIO<'static, tm4c129x::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }

    let tm4c1294 = EkTm4c1294xl {
        console: console,
        alarm: alarm,
        gpio: gpio,
        ipc: kernel::ipc::IPC::new(board_kernel),
        led: led,
        button: button,
    };

    let mut chip = tm4c129x::chip::Tm4c129x::new();

    tm4c1294.console.initialize();

    debug!("Initialization complete. Entering main loop...\r");

    extern "C" {
        /// Beginning of the ROM region containing app images.
        ///
        /// This symbol is defined in the linker script.
        static _sapps: u8;
    }
    kernel::procs::load_processes(
        board_kernel,
        &cortexm4::syscall::SysCall::new(),
        &_sapps as *const u8,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
    );
    board_kernel.kernel_loop(&tm4c1294, &mut chip, Some(&tm4c1294.ipc));
}
