//! Mechanism for inspecting the status of the kernel.
//!
//! In particular this provides functions for getting the status of processes
//! on the board. It potentially could be expanded to other kernel state.

use core::cell::Cell;

use sched::Kernel;
use process;
use common::cells::NumericCellExt;

/// This struct provides the introspection functions.
struct Introspection {
    kernel: &'static Kernel,
}

impl Introspection {
    pub fn new(kernel: &'static Kernel) -> Introspection {
        Introspection {
            kernel: kernel,
        }
    }

    /// Returns how many processes have been loaded on this platform. This is
    /// functionally equivalent to how many of the process slots have been used
    /// on the board. This does not consider what state the process is in, as
    /// long as it has been loaded.
    pub fn number_loaded_processes(&self) -> usize {
    	let count: Cell<usize> = Cell::new(0);
    	self.kernel.process_each_enumerate(|_, _| count.increment());
    	count.get()
    }

    /// Returns how many processes are considered to be active. This includes
    /// processes in the `Running` and `Yield` states. This does not include
    /// processes which have faulted, or processes which the kernel is no longer
    /// scheduling because they have faulted too frequently or for some other
    /// reason.
    pub fn number_active_processes(&self) -> usize {
    	let count: Cell<usize> = Cell::new(0);
    	self.kernel.process_each_enumerate(|i, process| {
    		match process.current_state() {
    			process::State::Running => count.increment(),
    			process::State::Yielded => count.increment(),
    			process::State::Fault => {},
    		}
    	});
    	count.get()
    }

    /// Returns how many processes are considered to be inactive. This includes
    /// processes in the `Fault` state and processes which the kernel is not
    /// scheduling for any reason.
    pub fn number_inactive_processes(&self) -> usize {
    	let count: Cell<usize> = Cell::new(0);
    	self.kernel.process_each_enumerate(|i, process| {
    		match process.current_state() {
    			process::State::Running => {},
    			process::State::Yielded => {},
    			process::State::Fault => count.increment(),
    		}
    	});
    	count.get()
    }
}
