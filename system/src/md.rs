use crate::{FileLoader, System};
use common::{Bus, ReadSeek};
use processor::m68k::{M68K, M68KFault};
use std::ffi::OsStr;
use std::path::Path;

pub struct MdFileLoader {}

impl FileLoader for MdFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("md")
    }

    fn load(&self, _: &mut dyn ReadSeek) -> Box<dyn System> {
        let mut m68k_bus = Bus::new();
        let m68k = M68K::new(&mut m68k_bus);
        Box::new(MdSystem { m68k_bus, m68k, cycle_index: 0 })
    }
}

pub struct MdSystem<'a> {
    m68k_bus: Bus<'a, M68KFault, 16>,
    m68k: M68K<'a>,
    cycle_index: u8,
}

impl<'a> System for MdSystem<'a> {
    fn tick(&mut self, _: u64) {
    }
    fn clock_speed(&self) -> u64 {
        53_693_175
    }
    fn set_clock_speed(&mut self, _: u64) {}
}
