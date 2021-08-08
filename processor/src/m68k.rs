use crate::Processor;

pub struct M68K {}

impl M68K {
    pub fn new() -> M68K {
        M68K {}
    }
}

impl Processor for M68K {
    fn tick(&mut self) {}
}
