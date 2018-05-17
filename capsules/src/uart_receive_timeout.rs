//! Convert the normal UART interface to one with a timeout.

use core::cmp;
use kernel::common::take_cell::{MapCell, TakeCell};
use kernel::hil;




pub struct UartReceiveTimeout<'a, U: hil::uart::UART + 'a> {
    uart: &'a U,
    tx_buffer: TakeCell<'static, [u8]>,
    rx_buffer: TakeCell<'static, [u8]>,
}

impl<'a, U: hil::uart::UART> UartReceiveTimeout<'a, U> {
    pub fn new(
        uart: &'a U,
        tx_buffer: &'static mut [u8],
        rx_buffer: &'static mut [u8],
    ) -> Nrf51822Serialization<'a, U> {
        Nrf51822Serialization {
            uart: uart,
            app: MapCell::new(App::default()),
            tx_buffer: TakeCell::new(tx_buffer),
            rx_buffer: TakeCell::new(rx_buffer),
        }
    }

    pub fn initialize(&self) {
        self.uart.init(uart::UARTParams {
            baud_rate: 250000,
            stop_bits: uart::StopBits::One,
            parity: uart::Parity::Even,
            hw_flow_control: true,
        });
    }
}

impl hil::uart::UART for USART {
    fn set_client(&self, client: &'static hil::uart::Client) {
        self.client.set(Some(c));
        self.uart.set_client(self);
    }

    fn init(&self, params: hil::uart::UARTParams) {
        self.uart.init(params);
    }

    fn transmit(&self, tx_data: &'static mut [u8], tx_len: usize) {
        self.uart.transmit(tx_data, tx_len);
    }

    fn receive(&self, rx_buffer: &'static mut [u8], rx_len: usize) {
        self.uart.receive(rx_buffer, rx_len);
    }

    fn abort_receive(&self) {
        self.uart.abort_receive();
    }
}

impl hil::uart::UARTAdvanced for USART {
    fn receive_automatic(&self, rx_buffer: &'static mut [u8], interbyte_timeout: u8) {

    }

    fn receive_until_terminator(&self, rx_buffer: &'static mut [u8], terminator: u8) {

    }
}

// Callbacks from the underlying UART driver.
impl<'a, U: hil::uart::UART> hil::uart::Client for UartReceiveTimeout<'a, U> {
    // Called when the UART TX has finished.
    fn transmit_complete(&self, buffer: &'static mut [u8], error: uart::Error) {
        self.client.map(|client| {
            client.transmit_complete(buffer, error);
        });
    }

    // Called when a buffer is received on the UART.
    fn receive_complete(&self, buffer: &'static mut [u8], rx_len: usize, _error: uart::Error) {
        self.rx_buffer.replace(buffer);

        self.app.map(|appst| {
            appst.rx_buffer = appst.rx_buffer.take().map(|mut rb| {
                // Figure out length to copy.
                let max_len = cmp::min(rx_len, rb.len());

                // Copy over data to app buffer.
                self.rx_buffer.map(|buffer| {
                    for idx in 0..max_len {
                        rb.as_mut()[idx] = buffer[idx];
                    }
                });
                appst.callback.as_mut().map(|cb| {
                    // Notify the serialization library in userspace about the
                    // received buffer.
                    cb.schedule(4, rx_len, 0);
                });

                rb
            });
        });

        // Restart the UART receive.
        self.rx_buffer
            .take()
            .map(|buffer| self.uart.receive_automatic(buffer, 250));
    }
}
