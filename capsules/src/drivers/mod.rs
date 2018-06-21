//! Collection of capsules that provide userspace `Driver` interfaces.

pub mod adc;
pub mod alarm;
pub mod ambient_light;
pub mod app_flash;
pub mod ble_advertising;
pub mod button;
pub mod console;
pub mod crc;
pub mod dac;
pub mod gpio;
pub mod gpio_async;
pub mod humidity;
pub mod i2c_master_slave;
pub mod led;
pub mod ninedof;
pub mod nonvolatile_storage;
pub mod nrf51822_serialization;
pub mod rng;
pub mod spi;
pub mod temperature;
pub mod usb_user;
