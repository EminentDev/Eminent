#[cfg(any(feature = "m6502", feature = "m6502x", feature = "w65"))]
pub mod m6502;

pub trait Processor {
    fn tick(&mut self);
}
