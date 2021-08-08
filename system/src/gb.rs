use crate::{FileLoader, System};
use common::ReadSeek;
use std::ffi::OsStr;
use std::path::Path;

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("gb")
    }

    fn load(&self, _: &mut dyn ReadSeek) -> Box<dyn System> {
        Box::new(GbSystem {})
    }
}

pub struct GbSystem {}

impl System for GbSystem {
    fn tick(&mut self, _: u64) {}
    fn clock_speed(&self) -> u64 { 4_194_304 }
    fn set_clock_speed(&mut self, _: u64) {}
}
