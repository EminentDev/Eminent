use std::io::{Read, Seek};

pub trait ReadSeek: Read + Seek {}
impl<T: Read + Seek + ?Sized> ReadSeek for T {}

#[derive(Clone, Copy, PartialEq)]
pub enum BusStatus {
    Hit,
    OpenBus,
}

pub trait BusDevice<F, const WIDTH: usize> {
    fn read(&mut self, addr: usize, value: &mut [u8; WIDTH]) -> (Option<F>, [BusStatus; WIDTH]);
    fn write(&mut self, addr: usize, value: &[u8; WIDTH]) -> Option<F>;
}

pub trait BusTransform {
    fn transform(&self, addr: usize) -> Option<usize>;
}

pub struct Bus<'a, F, const WIDTH: usize> {
    devices: Vec<(&'a mut dyn BusDevice<F, WIDTH>, &'a dyn BusTransform)>,
    bus_state: Box<[u8; WIDTH]>,
}

impl<'a, F, const WIDTH: usize> Bus<'a, F, WIDTH> {
    pub fn read(&mut self, addr: usize) -> (Option<F>, &[u8; WIDTH]) {
        let mut new_state = [0; WIDTH];
        let mut bus_status = [BusStatus::OpenBus; WIDTH];
        let mut scratch_state = [0; WIDTH];
        for (device, transform) in &mut self.devices {
            if let Some(addr) = transform.transform(addr) {
                let (fault, cur_bus_status) = device.read(addr, &mut scratch_state);
                if let Some(fault) = fault {
                    return (Some(fault), &*self.bus_state); // Temporary
                }
                for i in 0..WIDTH {
                    if cur_bus_status[i] != BusStatus::OpenBus {
                        if bus_status[i] == BusStatus::OpenBus {
                            new_state[i] = scratch_state[i];
                            bus_status[i] = BusStatus::Hit;
                        } else {
                            // TODO: Support custom bus conflict handling
                            new_state[i] &= scratch_state[i];
                        }
                    }
                }
            }
        }
        for i in 0..WIDTH {
            if bus_status[i] == BusStatus::OpenBus {
                new_state[i] = self.bus_state[i];
            }
        }
        *self.bus_state = new_state;
        (None, &*self.bus_state)
    }

    pub fn write(&mut self, addr: usize, value: &[u8; WIDTH]) -> Option<F> {
        for (device, transform) in &mut self.devices {
            if let Some(addr) = transform.transform(addr) {
                let fault = device.write(addr, value);
                if let Some(fault) = fault {
                    return Some(fault); // Temporary
                }
            }
        }
        None
    }

    pub fn add_device(
        &mut self,
        device: &'a mut dyn BusDevice<F, WIDTH>,
        transform: &'a dyn BusTransform,
    ) {
        self.devices.push((device, transform));
    }
}
