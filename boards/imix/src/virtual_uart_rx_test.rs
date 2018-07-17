use capsules::test::virtual_uart::TestVirtualUartReceive;
use kernel::hil::uart::{Client, UART};
use capsules::virtual_uart::{UartDevice, UartMux};

pub unsafe fn run_virtual_uart_receive(mux: &'static UartMux<'static>) {
    let small = static_init_test_receive_small(mux);
    let large = static_init_test_receive_large(mux);

    small.run();
    large.run();
}

unsafe fn static_init_test_receive_small(mux: &'static UartMux<'static>) -> &'static TestVirtualUartReceive {
    static mut BUFFER: [u8; 17] = [0; 17];
    let device = static_init!(
        UartDevice<'static>,
        UartDevice::new(mux, true)
    ); 
    let test = static_init!(
        TestVirtualUartReceive,
        TestVirtualUartReceive::new(device, &mut BUFFER)
    );
    device.set_client(test);
    test
}


unsafe fn static_init_test_receive_large(mux: &'static UartMux<'static>) -> &'static TestVirtualUartReceive {
    static mut BUFFER: [u8; 23] = [0; 23];
    let device = static_init!(
        UartDevice<'static>,
        UartDevice::new(mux, true)
    ); 
    let test = static_init!(
        TestVirtualUartReceive,
        TestVirtualUartReceive::new(device, &mut BUFFER)
    );
    device.set_client(test);
    test
}
