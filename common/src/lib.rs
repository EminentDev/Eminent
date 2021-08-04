use std::io::{Read, Seek};

pub trait ReadSeek: Read + Seek {}
impl<T: Read + Seek + ?Sized> ReadSeek for T {}

pub enum BusResult {
    Hit,
    Fault,
    OpenBus,
}

pub trait BusDevice {
    fn read(&mut self, addr: usize, value: &mut [u8]) -> BusResult;
    fn write(&mut self, addr: usize, value: &[u8]) -> BusResult; // OpenBus here is equivalent to Fault
}

pub struct Bus<'a> {
    devices: Vec<&'a mut dyn BusDevice>,
}
