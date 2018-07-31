#![no_std]
#![feature(in_band_lifetimes)]

extern crate capsules;
#[macro_use(static_init)]
extern crate kernel;

pub mod led;
