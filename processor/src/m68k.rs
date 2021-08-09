use crate::Processor;
use common::Bus;
use std::cell::RefCell;
use std::rc::Rc;

#[derive(Debug)]
pub struct M68KFault {}

pub struct M68K {
    bus: Rc<RefCell<Bus<M68KFault, 2>>>,
}

impl M68K {
    pub fn new(bus: Rc<RefCell<Bus<M68KFault, 2>>>) -> M68K {
        M68K { bus }
    }

    fn read(&self, addr: usize) -> (Option<M68KFault>, &[u8; 2], u64) {
        unsafe {
            // SAFETY: Assume we're fine.
            self.bus.borrow().read(addr)
        }
    }
}

impl Processor for M68K {
    fn tick(&mut self) {
        println!("{:?}", self.read(0x0000_usize));
    }
}
