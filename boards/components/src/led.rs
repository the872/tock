//! Component for imix board LEDs.
//!
//! This provides one Component, LedComponent, which implements
//! a userspace syscall interface to the two imix on-board LEDs.
//!
//! Usage
//! -----
//! ```rust
//! let led = LedComponent::new().finalize();
//! ```

// Author: Philip Levis <pal@cs.stanford.edu>
// Last modified: 6/20/2018

#![allow(dead_code)] // Components are intended to be conditionally included

use core::marker::PhantomData;

use capsules::led;
use kernel::component::Component;
use kernel::hil::gpio;

pub struct LedComponent<T: 'static + gpio::Pin + gpio::PinCtl> {
    led_pins: &'static [(T, led::ActivationMode)],
    phantom: PhantomData<T>,
}

impl<T: 'static + gpio::Pin + gpio::PinCtl> LedComponent<T> {
    pub fn new(led_pins: &'static [(T, led::ActivationMode)]) -> LedComponent<T> {
        LedComponent {
            led_pins,
            phantom: PhantomData,
        }
    }

    fn um<R>(&self) -> R {
        static_init!(
            led::LED<'static, T>,
            led::LED::new(self.led_pins)
        )
    }
}

impl<T: 'static + gpio::Pin + gpio::PinCtl> Component for LedComponent<T> {
    type Output = &'static led::LED<'static, T>;

    unsafe fn finalize(&mut self) -> Self::Output {
        // static_init!(
        //     led::LED<'static, P>,
        //     led::LED::new(self.led_pins)
        // )
        self.um::<T, Self::Output>()
    }
}
