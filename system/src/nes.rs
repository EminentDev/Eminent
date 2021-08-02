use crate::{FileLoader, System};
use common::ReadSeek;

pub struct NesFileLoader {}

impl FileLoader for NesFileLoader {
    fn can_load(&self, _: &str, _: &mut dyn ReadSeek) -> bool {
        true
    }

    fn load(&self, file: &mut dyn ReadSeek) -> Box<dyn System> {
        Box::from(NesSystem::from_rom(file))
    }
}

pub struct NesSystem {}

impl NesSystem {
    fn from_rom(_: &mut dyn ReadSeek) -> NesSystem {
        NesSystem {}
    }
}

impl System for NesSystem {}
