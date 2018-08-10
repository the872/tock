//! Platform Level Interrupt Control

use kernel::common::StaticRef;
use kernel::common::registers::{self, ReadOnly, ReadWrite, WriteOnly};

#[repr(C)]
struct PlicRegisters {
	/// Interrupt Priority Register
	priority: [ReadWrite<u32>; 255],
	_reserved0: [u8; 3076],
	/// Interrupt Pending Register
	pending: [ReadWrite<u32>; 8],
	_reserved1: [u8; 4064],
	/// Interrupt Enable Register
	enable: [ReadWrite<u32>; 8],
	_reserved2: [u8; 2088928],
	/// Priority Threshold Register
	threshold: ReadWrite<u32>,
	/// Claim/Complete Register
	claim: ReadWrite<u32>,
}

const PLIC_BASE: StaticRef<PlicRegisters> =
    unsafe { StaticRef::new(0x0c00_0000 as *const PlicRegisters) };


/// Clear all pending interrupts.
pub unsafe fn clear_all_pending() {
    let plic: &PlicRegisters = &*PLIC_BASE;
    for pending in plic.pending.iter() {
        pending.set(0);
    }
}

/// Enable all interrupts.
pub unsafe fn enable_all() {
    let plic: &PlicRegisters = &*PLIC_BASE;
    for enable in plic.enable.iter() {
        enable.set(0xFFFF_FFFF);
    }
}

/// Disable all interrupts.
pub unsafe fn disable_all() {
    let plic: &PlicRegisters = &*PLIC_BASE;
    for enable in plic.enable.iter() {
        enable.set(0);
    }
}

/// Get the index (0-256) of the lowest number pending interrupt, or `None` if
/// none is pending.
pub unsafe fn next_pending() -> Option<u32> {
    let plic: &PlicRegisters = &*PLIC_BASE;

    for (block, pending) in plic.pending.iter().enumerate() {
        let pending = pending.get();

        // If there are any high bits there is a pending interrupt
        if pending != 0 {
            // trailing_zeros == index of first high bit
            // let bit = (pending & !0x3).trailing_zeros();
            let bit = pending.trailing_zeros();
            return Some(block as u32 * 32 + bit);
        }
    }
    None
}

/// Return `true` if there are any pending interrupts in the PLIC, `false`
/// otherwise.
pub unsafe fn has_pending() -> bool {
    let plic: &PlicRegisters = &*PLIC_BASE;

    plic.pending.iter().fold(0, |i, pending| pending.get() | i) != 0
}

/// Clear the pending interrupt passed in by index.
pub unsafe fn clear_pending(index: u32) {
    let plic: &PlicRegisters = &*PLIC_BASE;
    let index = index as usize;

    let old = plic.pending[index / 32].get();
    plic.pending[index / 32].set(old & !(1 << (index & 31)));
}
