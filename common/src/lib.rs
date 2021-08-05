use std::io::{Read, Seek};

pub trait ReadSeek: Read + Seek {}
impl<T: Read + Seek + ?Sized> ReadSeek for T {}

pub enum BusStatus {
    Hit,
    OpenBus,
}

pub trait BusDevice<F, const WIDTH: usize> {
    fn read(&mut self, addr: usize, value: &mut [u8; WIDTH]) -> (Option<F>, [BusStatus; WIDTH]);
    fn write(&mut self, addr: usize, value: &[u8; WIDTH]) -> Option<F>;
}

pub trait BusTransform {
    fn transform(&self, addr: usize) -> usize;
}

pub struct Bus<'a, F, const WIDTH: usize> {
    devices: Vec<(&'a mut dyn BusDevice<F, WIDTH>, &'a dyn BusTransform)>,
}

impl<'a, F, const WIDTH: usize> Bus<'a, F, WIDTH> {
    pub fn read(_addr: usize) -> (Option<F>, Box<[u8]>) {
        (None, Vec::with_capacity(WIDTH).into_boxed_slice())
    }

    pub fn write(_addr: usize, _value: &[u8; WIDTH]) -> Option<F> {
        None
    }
}
