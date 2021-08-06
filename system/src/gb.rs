use crate::{FileLoader, System};
use common::ReadSeek;
use processor::gbz80::GbZ80;
use std::ffi::OsStr;
use std::path::Path;

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("gb")
    }

    fn load(&self, file: &mut dyn ReadSeek) -> System {
        let gbz80 = GbZ80::new(file);
        System {
            processors: vec![Box::new(gbz80)],
            schedule: vec![1], // Tick the main processor every clock cycle
            clocks_per_second: 4_194_304,
            cycle: 0,
        }
    }
}
