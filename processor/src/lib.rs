#[cfg(any(feature = "m6502", feature = "m6502x", feature = "w65"))]
pub mod m6502;

#[cfg(feature = "z80")]
pub mod z80;

#[cfg(feature = "gbz80")]
pub mod gbz80;

#[cfg(feature = "m68k")]
pub mod m68k;

pub trait Processor {
    fn tick(&mut self);
}
