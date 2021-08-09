use crate::Processor;
use common::Bus;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug)]
pub struct M68KFault {}

pub struct M68K<'a> {
    bus: Rc<RefCell<Bus<'a, M68KFault, 16>>>,
}

impl<'a> M68K<'a> {
    pub fn new(bus: Rc<RefCell<Bus<'a, M68KFault, 16>>>) -> M68K<'a> {
        M68K { bus }
    }

    fn read(&self, addr: usize) -> (Option<M68KFault>, &'a [u8; 16], u64) {
        unsafe {
            // SAFETY: Assume we're fine.
            self.bus.borrow().read(addr)
        }
    }
}

impl Processor for M68K<'_> {
    fn tick(&mut self) {
        println!("{:?}", self.read(0x0000_usize));
    }
}
