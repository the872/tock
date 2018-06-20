//! Tock kernel for the Aconno ACD52832 board based on the Nordic nRF52832 MCU.

#![no_std]
#![no_main]
#![feature(lang_items)]
#![deny(missing_docs)]

extern crate capsules;
#[allow(unused_imports)]
#[macro_use(debug, debug_verbose, debug_gpio, static_init)]
extern crate kernel;
extern crate nrf52;
extern crate nrf5x;

use capsules::virtual_alarm::VirtualMuxAlarm;
use kernel::hil::gpio::Pin;
use kernel::hil::pwm::Pwm;
use kernel::hil;
use nrf5x::rtc::Rtc;

const LED1_PIN: usize = 26;
const LED2_PIN: usize = 22;
const LED3_PIN: usize = 23;
const LED4_PIN: usize = 24;

const BUTTON1_PIN: usize = 25;
const BUTTON2_PIN: usize = 14;
const BUTTON3_PIN: usize = 15;
const BUTTON4_PIN: usize = 16;
const BUTTON_RST_PIN: usize = 19;

/// UART Writer
#[macro_use]
pub mod io;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::procs::FaultResponse = kernel::procs::FaultResponse::Panic;

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 4;

#[link_section = ".app_memory"]
static mut APP_MEMORY: [u8; 32768] = [0; 32768];

static mut PROCESSES: [Option<&'static mut kernel::procs::Process<'static>>; NUM_PROCS] =
    [None, None, None, None];

/// Supported drivers by the platform
pub struct Platform {
    ble_radio: &'static capsules::ble_advertising_driver::BLE<
        'static,
        nrf52::radio::Radio,
        VirtualMuxAlarm<'static, Rtc>,
    >,
    button: &'static capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
    console: &'static capsules::console::Console<
        'static,
        capsules::segger_rtt::SeggerRtt<'static, VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>>,
    >,
    gpio: &'static capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
    led: &'static capsules::led::LED<'static, nrf5x::gpio::GPIOPin>,
    rng: &'static capsules::rng::SimpleRng<'static, nrf5x::trng::Trng<'static>>,
    temp: &'static capsules::temperature::TemperatureSensor<'static>,
    ipc: kernel::ipc::IPC,
    alarm:
        &'static capsules::alarm::AlarmDriver<'static, VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>>,
    gpio_async:
        &'static capsules::gpio_async::GPIOAsync<'static, capsules::mcp230xx::MCP230xx<'static>>,
    light:
        &'static capsules::ambient_light::AmbientLight<'static>,
}

impl kernel::Platform for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&kernel::Driver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temp)),
            capsules::gpio_async::DRIVER_NUM => f(Some(self.gpio_async)),
            capsules::ambient_light::DRIVER_NUM => f(Some(self.light)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

/// Entry point in the vector table called on hard reset.
#[no_mangle]
pub unsafe fn reset_handler() {
    // Loads relocations and clears BSS
    nrf52::init();

    // GPIOs
    let gpio_pins = static_init!(
        [&'static nrf5x::gpio::GPIOPin; 14],
        [
            &nrf5x::gpio::PORT[3], // Bottom right header on DK board
            &nrf5x::gpio::PORT[4],
            &nrf5x::gpio::PORT[28],
            &nrf5x::gpio::PORT[29],
            &nrf5x::gpio::PORT[30],
            // &nrf5x::gpio::PORT[31], // -----
            &nrf5x::gpio::PORT[12], // Top mid header on DK board
            &nrf5x::gpio::PORT[11], // -----
            &nrf5x::gpio::PORT[27], // Top left header on DK board
            &nrf5x::gpio::PORT[26],
            &nrf5x::gpio::PORT[2],
            &nrf5x::gpio::PORT[25],
            &nrf5x::gpio::PORT[24],
            &nrf5x::gpio::PORT[23],
            &nrf5x::gpio::PORT[22], // -----
        ]
    );

    // LEDs
    let led_pins = static_init!(
        [(&'static nrf5x::gpio::GPIOPin, capsules::led::ActivationMode); 4],
        [
            (
                &nrf5x::gpio::PORT[LED1_PIN],
                capsules::led::ActivationMode::ActiveLow
            ),
            (
                &nrf5x::gpio::PORT[LED2_PIN],
                capsules::led::ActivationMode::ActiveLow
            ),
            (
                &nrf5x::gpio::PORT[LED3_PIN],
                capsules::led::ActivationMode::ActiveLow
            ),
            (
                &nrf5x::gpio::PORT[LED4_PIN],
                capsules::led::ActivationMode::ActiveLow
            ),
        ]
    );

    let button_pins = static_init!(
        [(&'static nrf5x::gpio::GPIOPin, capsules::button::GpioMode); 4],
        [
            (
                &nrf5x::gpio::PORT[BUTTON1_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ), // 13
            (
                &nrf5x::gpio::PORT[BUTTON2_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ), // 14
            (
                &nrf5x::gpio::PORT[BUTTON3_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ), // 15
            (
                &nrf5x::gpio::PORT[BUTTON4_PIN],
                capsules::button::GpioMode::LowWhenPressed
            ), // 16
        ]
    );

    // Make non-volatile memory writable and activate the reset button
    let uicr = nrf52::uicr::Uicr::new();
    nrf52::nvmc::NVMC.erase_uicr();
    nrf52::nvmc::NVMC.configure_writeable();
    while !nrf52::nvmc::NVMC.is_ready() {}
    uicr.set_psel0_reset_pin(BUTTON_RST_PIN);
    while !nrf52::nvmc::NVMC.is_ready() {}
    uicr.set_psel1_reset_pin(BUTTON_RST_PIN);

    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(
        Some(&nrf5x::gpio::PORT[LED2_PIN]),
        Some(&nrf5x::gpio::PORT[LED3_PIN]),
        Some(&nrf5x::gpio::PORT[LED4_PIN]),
    );

    let gpio = static_init!(
        capsules::gpio::GPIO<'static, nrf5x::gpio::GPIOPin>,
        capsules::gpio::GPIO::new(gpio_pins)
    );
    for pin in gpio_pins.iter() {
        pin.set_client(gpio);
    }

    // LEDs
    let led = static_init!(
        capsules::led::LED<'static, nrf5x::gpio::GPIOPin>,
        capsules::led::LED::new(led_pins)
    );

    // Buttons
    let button = static_init!(
        capsules::button::Button<'static, nrf5x::gpio::GPIOPin>,
        capsules::button::Button::new(button_pins, kernel::Grant::create())
    );
    for &(btn, _) in button_pins.iter() {
        use kernel::hil::gpio::PinCtl;
        btn.set_input_mode(kernel::hil::gpio::InputMode::PullUp);
        btn.set_client(button);
    }

    let rtc = &nrf5x::rtc::RTC;
    rtc.start();
    let mux_alarm = static_init!(
        capsules::virtual_alarm::MuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::MuxAlarm::new(&nrf5x::rtc::RTC)
    );
    rtc.set_client(mux_alarm);

    let virtual_alarm1 = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );
    let alarm = static_init!(
        capsules::alarm::AlarmDriver<
            'static,
            capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        >,
        capsules::alarm::AlarmDriver::new(virtual_alarm1, kernel::Grant::create())
    );
    virtual_alarm1.set_client(alarm);
    let ble_radio_virtual_alarm = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );

    // RTT and Console
    let virtual_alarm_rtt = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );

    let rtt_memory = static_init!(
        capsules::segger_rtt::SeggerRttMemory,
        capsules::segger_rtt::SeggerRttMemory::new(
            b"Terminal\0",
            &mut capsules::segger_rtt::UP_BUFFER,
            b"Terminal\0",
            &mut capsules::segger_rtt::DOWN_BUFFER
        )
    );

    let rtt = static_init!(
        capsules::segger_rtt::SeggerRtt<VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>>,
        capsules::segger_rtt::SeggerRtt::new(
            virtual_alarm_rtt,
            rtt_memory,
            &mut capsules::segger_rtt::UP_BUFFER,
            &mut capsules::segger_rtt::DOWN_BUFFER
        )
    );
    virtual_alarm_rtt.set_client(rtt);

    // rtt.say();

    let console = static_init!(
        capsules::console::Console<
            capsules::segger_rtt::SeggerRtt<VirtualMuxAlarm<'static, nrf5x::rtc::Rtc>>,
        >,
        capsules::console::Console::new(
            rtt,
            115200,
            &mut capsules::console::WRITE_BUF,
            &mut capsules::console::READ_BUF,
            kernel::Grant::create()
        )
    );
    kernel::hil::uart::UART::set_client(rtt, console);
    console.initialize();

    // Attach the kernel debug interface to this console
    let kc = static_init!(capsules::console::App, capsules::console::App::default());
    kernel::debug::assign_console_driver(Some(console), kc);

    let i2c_mux = static_init!(
        capsules::virtual_i2c::MuxI2C<'static>,
        capsules::virtual_i2c::MuxI2C::new(&nrf52::i2c::TWIM0)
    );
    nrf52::i2c::TWIM0.configure(
        nrf5x::pinmux::Pinmux::new(21),
        nrf5x::pinmux::Pinmux::new(20),
    );
    nrf52::i2c::TWIM0.set_client(i2c_mux);

    // Configure the MCP23017. Device address 0x20.
    let mcp23017_i2c = static_init!(
        capsules::virtual_i2c::I2CDevice,
        capsules::virtual_i2c::I2CDevice::new(i2c_mux, 0x40)
    );
    let mcp23017 = static_init!(
        capsules::mcp230xx::MCP230xx<'static>,
        capsules::mcp230xx::MCP230xx::new(
            mcp23017_i2c,
            Some(&nrf5x::gpio::PORT[11]),
            Some(&nrf5x::gpio::PORT[12]),
            &mut capsules::mcp230xx::BUFFER,
            8,
            2
        )
    );
    mcp23017_i2c.set_client(mcp23017);
    nrf5x::gpio::PORT[11].set_client(mcp23017);
    nrf5x::gpio::PORT[12].set_client(mcp23017);

    // Create an array of the GPIO extenders so we can pass them to an
    // administrative layer that provides a single interface to them all.
    let async_gpio_ports = static_init!([&'static capsules::mcp230xx::MCP230xx; 1], [mcp23017]);

    // `gpio_async` is the object that manages all of the extenders.
    let gpio_async = static_init!(
        capsules::gpio_async::GPIOAsync<'static, capsules::mcp230xx::MCP230xx<'static>>,
        capsules::gpio_async::GPIOAsync::new(async_gpio_ports)
    );
    // Setup the clients correctly.
    for port in async_gpio_ports.iter() {
        port.set_client(gpio_async);
    }

    let ble_radio = static_init!(
        capsules::ble_advertising_driver::BLE<
            'static,
            nrf52::radio::Radio,
            VirtualMuxAlarm<'static, Rtc>,
        >,
        capsules::ble_advertising_driver::BLE::new(
            &mut nrf52::radio::RADIO,
            kernel::Grant::create(),
            &mut capsules::ble_advertising_driver::BUF,
            ble_radio_virtual_alarm
        )
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_receive_client(
        &nrf52::radio::RADIO,
        ble_radio,
    );
    kernel::hil::ble_advertising::BleAdvertisementDriver::set_transmit_client(
        &nrf52::radio::RADIO,
        ble_radio,
    );
    ble_radio_virtual_alarm.set_client(ble_radio);

    let temp = static_init!(
        capsules::temperature::TemperatureSensor<'static>,
        capsules::temperature::TemperatureSensor::new(
            &mut nrf5x::temperature::TEMP,
            kernel::Grant::create()
        )
    );
    kernel::hil::sensors::TemperatureDriver::set_client(&nrf5x::temperature::TEMP, temp);

    let rng = static_init!(
        capsules::rng::SimpleRng<'static, nrf5x::trng::Trng>,
        capsules::rng::SimpleRng::new(&mut nrf5x::trng::TRNG, kernel::Grant::create())
    );
    nrf5x::trng::TRNG.set_client(rng);

    // Setup Analog Light Sensor
    let analog_light_sensor = static_init!(
         capsules::analog_light_sensor::AnalogLightSensor<'static, nrf52::adc::Adc>,
         capsules::analog_light_sensor::AnalogLightSensor::new(
            &nrf52::adc::ADC,
            &nrf52::adc::AdcChannel::AnalogInput5));
    nrf52::adc::ADC.set_client(analog_light_sensor);

    let light = static_init!(
         capsules::ambient_light::AmbientLight<'static>,
         capsules::ambient_light::AmbientLight::new(analog_light_sensor,
             kernel::Grant::create()));
     hil::sensors::AmbientLight::set_client(analog_light_sensor, light);



    // Start all of the clocks. Low power operation will require a better
    // approach than this.
    nrf52::clock::CLOCK.low_stop();
    nrf52::clock::CLOCK.high_stop();

    nrf52::clock::CLOCK.low_set_source(nrf52::clock::LowClockSource::XTAL);
    nrf52::clock::CLOCK.low_start();
    nrf52::clock::CLOCK.high_set_source(nrf52::clock::HighClockSource::XTAL);
    nrf52::clock::CLOCK.high_start();
    while !nrf52::clock::CLOCK.low_started() {}
    while !nrf52::clock::CLOCK.high_started() {}

    // debug_gpio!(0, make_output);
    // debug_gpio!(0, clear);

    let platform = Platform {
        button: button,
        ble_radio: ble_radio,
        console: console,
        led: led,
        gpio: gpio,
        rng: rng,
        temp: temp,
        alarm: alarm,
        gpio_async: gpio_async,
        light: light,
        ipc: kernel::ipc::IPC::new(),
    };

    let mut chip = nrf52::chip::NRF52::new();

    nrf5x::gpio::PORT[31].make_output();
    nrf5x::gpio::PORT[31].clear();
    let buzzer_pinmux = nrf5x::pinmux::Pinmux::new(31);
    // nrf52::pwm::PWM0.start(&buzzer_pinmux, 2400, 1200);

    // debug!("Initialization complete. Entering main loop\r");
    // debug!("{}", &nrf52::ficr::FICR_INSTANCE);

    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
    }
    kernel::procs::load_processes(
        &_sapps as *const u8,
        &mut APP_MEMORY,
        &mut PROCESSES,
        FAULT_RESPONSE,
    );

    // nrf52::pwm::PWM0.stop(&buzzer_pinmux);

    kernel::kernel_loop(&platform, &mut chip, &mut PROCESSES, Some(&platform.ipc));
}
