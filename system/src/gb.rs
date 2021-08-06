use crate::{FileLoader, System};
use common::ReadSeek;
use std::path::Path;
use std::ffi::OsStr;

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("gb")
    }

    fn load(&self, _: &mut dyn ReadSeek) -> System {
        System {
            processors: Vec::new(),
            schedule: vec![0; 1],
            clocks_per_second: 4_194_304,
            cycle: 0,
        }
    }
}
