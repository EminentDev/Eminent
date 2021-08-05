use crate::{FileLoader, System};
use common::ReadSeek;
use std::io::SeekFrom;

pub struct NesFileLoader {}

impl FileLoader for NesFileLoader {
    fn can_load(&self, _: &str, file: &mut dyn ReadSeek) -> bool {
        let mut buf: [u8; 4] = [0; 4];
        if file.read_exact(&mut buf).is_err() {
            return false;
        }
        if file.seek(SeekFrom::Start(0)).is_err() {
            return false;
        }
        buf[0] == b'N' && buf[1] == b'E' && buf[2] == b'S' && buf[3] == 0x1A
    }

    fn load(&self, _: &mut dyn ReadSeek) -> System {
        System {
            processors: Vec::new(),
            schedule: vec![0; 1],
            clocks_per_second: 21_477_272,
            cycle: 0,
        }
    }
}
