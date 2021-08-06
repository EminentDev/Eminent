#[cfg(any(feature = "m6502", feature = "m6502x", feature = "w65"))]
pub mod m6502;

#[cfg(feature = "gbz80")]
pub mod gbz80;

pub trait Processor {
    fn tick(&mut self);
}
