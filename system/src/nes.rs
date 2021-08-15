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

    fn load(&self, _: &mut dyn ReadSeek) -> Box<dyn System> {
        Box::new(NesSystem {})
    }
}

pub struct NesSystem {}

impl System for NesSystem {
    fn tick(&mut self, _: u64) {}
    fn clock_speed(&self) -> u64 {
        21_477_272
    }
    fn set_clock_speed(&mut self, _: u64) {}
    fn cycles_per_frame(&self) -> u64 { 357_366 } // TODO: Actually slightly more complicated than this
}
