//! Mechanism for inspecting the status of the kernel.
//!
//! In particular this provides functions for getting the status of processes
//! on the board. It potentially could be expanded to other kernel state.

use core::cell::Cell;

use callback::AppId;
use common::cells::NumericCellExt;
use process;
use sched::Kernel;

/// This struct provides the introspection functions.
struct Introspection {
    kernel: &'static Kernel,
}

impl Introspection {
    pub fn new(kernel: &'static Kernel) -> Introspection {
        Introspection { kernel: kernel }
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
        self.kernel
            .process_each_enumerate(|i, process| match process.current_state() {
                process::State::Running => count.increment(),
                process::State::Yielded => count.increment(),
                process::State::Fault => {}
            });
        count.get()
    }

    /// Returns how many processes are considered to be inactive. This includes
    /// processes in the `Fault` state and processes which the kernel is not
    /// scheduling for any reason.
    pub fn number_inactive_processes(&self) -> usize {
        let count: Cell<usize> = Cell::new(0);
        self.kernel
            .process_each_enumerate(|i, process| match process.current_state() {
                process::State::Running => {}
                process::State::Yielded => {}
                process::State::Fault => count.increment(),
            });
        count.get()
    }

    /// Get the name of the process.
    pub fn app_name(&self, app: AppId) -> &'static str {
    	self.kernel.process_map_or("", app.idx(), |process| {
    		process.get_package_name()
    	})
    }
    // pub fn app_name(&self, app: AppId) -> &[u8] {
    // 	self.kernel.process_map_or(&[], app.idx(), |process| {
    // 		process.get_package_name()
    // 	})
    // }

    /// Returns the number of syscalls the app has called.
    pub fn number_app_syscalls(&self, app: AppId) -> usize {
    	self.kernel.process_map_or(0, app.idx(), |process| {
    		process.debug_syscall_count()
    	})
    }

    /// Returns the number of dropped callbacks the app has experience.
    /// Callbacks can be dropped if the queue for the app is full when a capsule
    /// tries to schedule a callback.
    pub fn number_app_dropped_callbacks(&self, app: AppId) -> usize {
    	self.kernel.process_map_or(0, app.idx(), |process| {
    		process.debug_dropped_callback_count()
    	})
    }

    /// Returns the number of time this app has been restarted.
    pub fn number_app_restarts(&self, app: AppId) -> usize {
    	self.kernel.process_map_or(0, app.idx(), |process| {
    		process.debug_restart_count()
    	})
    }
}
