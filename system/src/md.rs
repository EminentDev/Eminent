use crate::{FileLoader, System};
use common::{Bus, ReadSeek};
use processor::m68k::{M68KFault, M68K};
use processor::Processor;
use std::cell::RefCell;
use std::ffi::OsStr;
use std::path::Path;
use std::rc::Rc;

pub struct MdFileLoader {}

impl FileLoader for MdFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("md")
    }

    fn load(&self, _: &mut dyn ReadSeek) -> Box<dyn System> {
        let m68k_bus = Rc::new(RefCell::new(Bus::new()));
        let m68k = M68K::new(Rc::clone(&m68k_bus));
        Box::new(MdSystem {
            m68k_bus,
            m68k,
            cycle_index: 0,
        })
    }
}

pub struct MdSystem<'a> {
    m68k_bus: Rc<RefCell<Bus<'a, M68KFault, 16>>>,
    m68k: M68K<'a>,
    cycle_index: u16,
}

impl<'a> System for MdSystem<'a> {
    fn tick(&mut self, num_cycles: u64) {
        for _ in 0..num_cycles {
            if self.cycle_index % 7 == 0 {
                self.m68k.tick()
            }
            self.cycle_index = (self.cycle_index + 1) % 420; // One processor divides by 15, one divides by 7, one diviides by 4.
        }
    }
    fn clock_speed(&self) -> u64 {
        53_693_175
    }
    fn set_clock_speed(&mut self, _: u64) {}
}
