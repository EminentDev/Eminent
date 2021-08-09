use crate::Processor;
use common::Bus;

#[derive(Debug)]
pub struct M68KFault {}

pub struct M68K<'a> {
    bus: &'a Bus<'a, M68KFault, 16>,
}

impl<'a> M68K<'_> {
    pub fn new(bus: &'a mut Bus<'a, M68KFault, 16>) -> M68K<'a> {
        M68K { bus }
    }

    fn read(&self, addr: usize) -> (Option<M68KFault>, &[u8; 16], u64) {
        unsafe {
            // SAFETY: We have the only reference to the Bus.
            self.bus.read(addr)
        }
    }
}

impl Processor for M68K<'_> {
    fn tick(&mut self) {
        println!("{:?}", self.read(0x0000 as usize));
    }
}
