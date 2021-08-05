use crate::{FileLoader, System};
use common::ReadSeek;

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, _: &str, _: &mut dyn ReadSeek) -> bool {
        false // Temporary
    }

    fn load(&self, _: &mut dyn ReadSeek) -> System {
        System { processors: Vec::new(), schedule: vec![0; 1], clocks_per_second: 4_194_304, cycle: 0 }
    }
}
