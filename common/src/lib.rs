use std::cell::UnsafeCell;
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
    devices: UnsafeCell<Vec<(&'a mut dyn BusDevice<F, WIDTH>, &'a dyn BusTransform)>>,
    bus_state: UnsafeCell<[u8; WIDTH]>,
}

impl<'a, F, const WIDTH: usize> Bus<'a, F, WIDTH> {
    pub fn new() -> Self {
        Bus {
            devices: Vec::new().into(),
            bus_state: [0; WIDTH].into(),
        }
    }

    /// # Safety
    /// Only safe if the Bus can be accessed from one place at a time.
    pub unsafe fn read(&self, addr: usize) -> (Option<F>, &'a [u8; WIDTH], u64) {
        let devices = &mut *self.devices.get();
        let bus_state = &mut *self.bus_state.get();
        let mut new_state = [0; WIDTH];
        let mut bus_status = [BusStatus::OpenBus; WIDTH];
        let mut scratch_state = [0; WIDTH];
        for (device, transform) in devices {
            if let Some(addr) = transform.transform(addr) {
                let (fault, cur_bus_status) = device.read(addr, &mut scratch_state);
                if let Some(fault) = fault {
                    return (Some(fault), &*self.bus_state.get(), 1); // Temporary
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
                new_state[i] = bus_state[i];
            }
        }
        *bus_state = new_state;
        (None, &*self.bus_state.get(), 1)
    }

    /// # Safety
    /// Only safe if the Bus can be accessed from one place at a time.
    pub unsafe fn write(&self, addr: usize, value: &[u8; WIDTH]) -> (Option<F>, u64) {
        let devices = &mut *self.devices.get();
        // TODO: update open bus
        for (device, transform) in devices {
            if let Some(addr) = transform.transform(addr) {
                let fault = device.write(addr, value);
                if let Some(fault) = fault {
                    return (Some(fault), 1); // Temporary
                }
            }
        }
        (None, 1)
    }

    pub fn add_device(
        &mut self,
        device: &'a mut dyn BusDevice<F, WIDTH>,
        transform: &'a dyn BusTransform,
    ) {
        unsafe {
            // SAFETY: `add_device` obtains a mutable reference to self, therefore this is the only usage of the UnsafeCell.
            (*self.devices.get()).push((device, transform));
        }
    }
}

impl<'a, F, const WIDTH: usize> Default for Bus<'a, F, WIDTH> {
    fn default() -> Self {
        Self::new()
    }
}
