#![feature(asm, concat_idents, const_fn, const_cell_new, try_from, used)]
#![no_std]
#![crate_name = "e310x"]
#![crate_type = "rlib"]

extern crate riscvimac;

#[allow(unused_imports)]
#[macro_use(
    debug,
    debug_verbose,
    debug_gpio,
    register_bitfields,
    register_bitmasks
)]
extern crate kernel;

pub mod chip;
pub mod gpio;
pub mod uart;
