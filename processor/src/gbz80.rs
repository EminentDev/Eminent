use common::ReadSeek;
use crate::Processor;

pub struct GbZ80 {
}

impl GbZ80 {
    pub fn new(_: &mut dyn ReadSeek) -> GbZ80 {
        GbZ80 {}
    }
}

impl Processor for GbZ80 {
    fn tick(&mut self) {}
}
