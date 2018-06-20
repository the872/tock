//! Basic driver for an analog light sensor.

use kernel::common::cells::OptionalCell;
use kernel::hil;
use kernel::ReturnCode;

pub struct AnalogLightSensor<'a, A: hil::adc::Adc + 'a> {
    adc: &'a A,
    channel: &'a <A as hil::adc::Adc>::Channel,
    client: OptionalCell<&'a hil::sensors::AmbientLightClient>,
}

impl<'a, A: hil::adc::Adc + 'a> AnalogLightSensor<'a, A> {
    pub fn new(adc: &'a A, channel: &'a <A as hil::adc::Adc>::Channel) -> AnalogLightSensor<'a, A> {
        AnalogLightSensor {
            adc: adc,
            channel: channel,
            client: OptionalCell::empty(),
        }
    }
}

/// Callbacks from the ADC driver
impl<'a, A: hil::adc::Adc + 'a> hil::adc::Client for AnalogLightSensor<'a, A> {
    fn sample_ready(&self, sample: u16) {
        self.client.map(|client| client.callback(sample as usize));
    }
}

impl<'a, A: hil::adc::Adc + 'a> hil::sensors::AmbientLight for AnalogLightSensor<'a, A> {
    fn set_client(&self, client: &'static hil::sensors::AmbientLightClient) {
        self.client.set(client);
    }

    fn read_light_intensity(&self) -> ReturnCode {
        // self.start_read_lux();
        // ReturnCode::SUCCESS

        self.adc.sample(self.channel)
    }
}
