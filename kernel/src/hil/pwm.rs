//! Interfaces for Pulse Width Modulation output.

use returncode::ReturnCode;

/// PWM control for a single pin.
pub trait Pwm {
    /// The chip-dependent type of a PWM pin.
    type Pin;

    /// Generate a PWM single on the given pin at the given frequency and duty
    /// cycle.
    fn start(&self, pin: &Self::Pin, frequency: usize, duty_cycle: usize) -> ReturnCode;

    /// Stop a PWM pin output.
    fn stop(&self, pin: &Self::Pin) -> ReturnCode;
}
