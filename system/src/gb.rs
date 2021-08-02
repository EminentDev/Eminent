use crate::{FileLoader, System};
use common::ReadSeek;

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, _: &str, _: &mut dyn ReadSeek) -> bool {
        false // Temporary
    }

    fn load(&self, file: &mut dyn ReadSeek) -> Box<dyn System> {
        Box::from(GbSystem::from_rom(file))
    }
}

pub struct GbSystem {}

impl GbSystem {
    fn from_rom(_: &mut dyn ReadSeek) -> GbSystem {
        GbSystem {}
    }
}

impl System for GbSystem {}
