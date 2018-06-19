//! Interfaces for Pulse Width Modulation output.

use returncode::ReturnCode;

/// PWM control for a single pin.
pub trait Pwm {
    /// The chip-dependent type of a PWM pin.
    type Pin;

    /// Generate a PWM single on the given pin at the given frequency and duty
    /// cycle.
    ///
    /// - `frequency_hz` is specified in Hertz.
    /// - `duty_cycle` is specified in hundredths of a percent. So setting
    ///   `duty_cycle` to 1 would mean a PWM output with a 0.01% duty cycle.
    ///   Passing 5000 would result in a 50% duty cycle.
    fn start(&self, pin: &Self::Pin, frequency_hz: usize, duty_cycle: usize) -> ReturnCode;

    /// Stop a PWM pin output.
    fn stop(&self, pin: &Self::Pin) -> ReturnCode;
}
