#![crate_name = "riscvimac"]
#![crate_type = "rlib"]
#![feature(asm, const_fn, lang_items)]
#![no_std]

#[macro_use(register_bitfields, register_bitmasks)]
extern crate kernel;

extern "C" {
    // NOTE `rustc` forces this signature on us. See `src/lang_items.rs`
    fn main() -> isize;

    // Boundaries of the .bss section
    static mut _ebss: u32;
    static mut _sbss: u32;

    // Boundaries of the .data section
    static mut _edata: u32;
    static mut _sdata: u32;

    // Initial values of the .data section (stored in Flash)
    static _sidata: u32;

    // Address of _start_trap
    static _start_trap: u32;
}


/// Entry point of all programs (_start).
///
/// It initializes DWARF call frame information, the stack pointer, the
/// frame pointer (needed for closures to work in start_rust) and the global
/// pointer. Then it calls _start_rust.
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(r#"
.section .init
.globl _start
_start:
  .cfi_startproc
  .cfi_undefined ra

  // .option push
  // .option norelax
  lui gp, %hi(__global_pointer$)
  addi gp, gp, %lo(__global_pointer$)
  // .option pop

  lui sp, %hi(_stack_start)
  addi sp, sp, %lo(_stack_start)

  add s0, sp, zero

  jal zero, _start_rust

  .cfi_endproc
"#);


/// Rust entry point (_start_rust)
///
/// Zeros bss section, initializes data section and calls main. This function
/// never returns.
#[naked]
#[link_section = ".init.rust"]
#[export_name = "_start_rust"]
pub extern "C" fn start_rust() -> ! {
    unsafe {
        r0::zero_bss(&mut _sbss, &mut _ebss);
        r0::init_data(&mut _sdata, &mut _edata, &_sidata);
    }

    // TODO: Enable FPU when available

    // Set mtvec to _start_trap
    #[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
    unsafe {
        //mtvec::write(_start_trap as usize, mtvec::TrapMode::Direct);
        asm!("csrrw zero, 0x305, $0"
             :
             : "r"(&_start_trap)
             :
             : "volatile");
    }

    // Neither `argc` or `argv` make sense in bare metal context so we
    // just stub them
    unsafe {
        main();
    }

    // If `main` returns, then we go into "reactive" mode and simply attend
    // interrupts as they occur.
    loop {
        asm::wfi();
    }
}


/// Trap entry point (_start_trap)
///
/// Saves caller saved registers ra, t0..6, a0..7, calls _start_trap_rust,
/// restores caller saved registers and then returns.
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(r#"
  .section .trap
  .align 4
  .global _start_trap

_start_trap:
  addi sp, sp, -16*4

  sw ra, 0*4(sp)
  sw t0, 1*4(sp)
  sw t1, 2*4(sp)
  sw t2, 3*4(sp)
  sw t3, 4*4(sp)
  sw t4, 5*4(sp)
  sw t5, 6*4(sp)
  sw t6, 7*4(sp)
  sw a0, 8*4(sp)
  sw a1, 9*4(sp)
  sw a2, 10*4(sp)
  sw a3, 11*4(sp)
  sw a4, 12*4(sp)
  sw a5, 13*4(sp)
  sw a6, 14*4(sp)
  sw a7, 15*4(sp)

  jal ra, _start_trap_rust

  lw ra, 0*4(sp)
  lw t0, 1*4(sp)
  lw t1, 2*4(sp)
  lw t2, 3*4(sp)
  lw t3, 4*4(sp)
  lw t4, 5*4(sp)
  lw t5, 6*4(sp)
  lw t6, 7*4(sp)
  lw a0, 8*4(sp)
  lw a1, 9*4(sp)
  lw a2, 10*4(sp)
  lw a3, 11*4(sp)
  lw a4, 12*4(sp)
  lw a5, 13*4(sp)
  lw a6, 14*4(sp)
  lw a7, 15*4(sp)

  addi sp, sp, 16*4
  mret
"#);


/// Trap entry point rust (_start_trap_rust)
///
/// mcause is read to determine the cause of the trap. XLEN-1 bit indicates
/// if it's an interrupt or an exception. The result is converted to an element
/// of the Interrupt or Exception enum and passed to handle_interrupt or
/// handle_exception.
#[link_section = ".trap.rust"]
#[export_name = "_start_trap_rust"]
pub extern "C" fn start_trap_rust() {
    // dispatch trap to handler
    trap_handler(mcause::read().cause());
    // mstatus, remain in M-mode after mret
    unsafe {
        mstatus::set_mpp(mstatus::MPP::Machine);
    }
}


/// Default Trap Handler
#[no_mangle]
#[linkage = "weak"]
pub fn trap_handler(_: mcause::Trap) {}

// Make sure there is an abort when linking
#[cfg(any(target_arch = "riscv32", target_arch = "riscv64"))]
global_asm!(r#"
.section .init
.globl abort
abort:
  jal zero, _start
"#);
