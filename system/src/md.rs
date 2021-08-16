use crate::{FileLoader, System};
use common::{Bus, BusDevice, BusStatus, BusTransform, ReadSeek};
use processor::m68k::{M68KFault, M68K};
use processor::Processor;
use std::cell::RefCell;
use std::ffi::OsStr;
use std::io::SeekFrom;
use std::path::Path;
use std::rc::Rc;

struct MdRomCartridge {
    data: Box<[u16; 0x200000]>, // 0x400000 bytes
}

impl BusDevice<M68KFault, 2> for MdRomCartridge {
    fn read(&mut self, addr: usize, result: &mut [u8; 2]) -> (Option<M68KFault>, [BusStatus; 2]) {
        result[0] = ((self.data[addr / 2] >> 8) & 0xFF) as u8; // m68k is big-endian
        result[1] = (self.data[addr / 2] & 0xFF) as u8;
        (None, [BusStatus::Hit; 2])
    }
    fn write(&mut self, _: usize, _: &[u8; 2]) -> Option<M68KFault> {
        None // I don't think this raises a fault?
    }
}

impl MdRomCartridge {
    fn new(file: &mut dyn ReadSeek) -> MdRomCartridge {
        file.seek(SeekFrom::Start(0)).unwrap();
        let mut temp_data = Vec::new();
        file.read_to_end(&mut temp_data).unwrap();
        let mut data = Box::new([0u16; 0x200000]);
        for i in 0..0x200000 {
            data[i] = (temp_data[(i * 2) % temp_data.len()] as u16) * 256
                + (temp_data[(i * 2 + 1) % temp_data.len()] as u16);
        }
        MdRomCartridge { data }
    }
}

struct MdCartridgeMapping {}

impl BusTransform for MdCartridgeMapping {
    fn transform(&self, addr: usize) -> Option<usize> {
        if (addr & 0x00FFFFFF) < 0x400000 {
            Some(addr & 0x00FFFFFE)
        } else {
            None
        }
    }
}

struct MdIoAreaDevice {}

impl BusDevice<M68KFault, 2> for MdIoAreaDevice {
    fn read(&mut self, addr: usize, result: &mut [u8; 2]) -> (Option<M68KFault>, [BusStatus; 2]) {
        result[0] = 0; // TODO
        result[1] = 0;
        (None, [BusStatus::Hit; 2])
    }
    fn write(&mut self, _: usize, _: &[u8; 2]) -> Option<M68KFault> {
        None // TODO
    }
}

struct MdIoAreaMapping {}

impl BusTransform for MdIoAreaMapping {
    fn transform(&self, addr: usize) -> Option<usize> {
        if (addr & 0x00FFFFFF) >= 0xA10000 && (addr & 0x00FFFFFF) < 0xA10020 {
            Some(addr & 0x0000001E)
        } else {
            None
        }
    }
}

struct MdVdpAreaDevice {}

impl BusDevice<M68KFault, 2> for MdVdpAreaDevice {
    fn read(&mut self, addr: usize, result: &mut [u8; 2]) -> (Option<M68KFault>, [BusStatus; 2]) {
        result[0] = 0; // TODO
        result[1] = 0;
        (None, [BusStatus::Hit; 2])
    }
    fn write(&mut self, _: usize, _: &[u8; 2]) -> Option<M68KFault> {
        None // TODO
    }
}

struct MdVdpAreaMapping {}

impl BusTransform for MdVdpAreaMapping {
    fn transform(&self, addr: usize) -> Option<usize> {
        if (addr & 0x00FFFFFF) >= 0xC00000 && (addr & 0x00FFFFFF) < 0xC00010 {
            Some(addr & 0x0000000E)
        } else {
            None
        }
    }
}

pub struct MdFileLoader {}

impl FileLoader for MdFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("md")
    }

    fn load(&self, file: &mut dyn ReadSeek) -> Box<dyn System> {
        let m68k_bus = Rc::new(RefCell::new(Bus::new()));
        m68k_bus.borrow_mut().add_device(
            Box::new(MdRomCartridge::new(file)),
            Box::new(MdCartridgeMapping {}),
        );
        m68k_bus
            .borrow_mut()
            .add_device(Box::new(MdIoAreaDevice {}), Box::new(MdIoAreaMapping {}));
        let m68k = M68K::new(Rc::clone(&m68k_bus));
        Box::new(MdSystem {
            m68k_bus,
            m68k,
            cycle_index: 0,
        })
    }
}

pub struct MdSystem {
    m68k_bus: Rc<RefCell<Bus<M68KFault, 2>>>,
    m68k: M68K,
    cycle_index: u16,
}

impl System for MdSystem {
    fn tick(&mut self, num_cycles: u64) {
        for _ in 0..num_cycles {
            if self.cycle_index % 7 == 0 {
                self.m68k.tick()
            }
            self.cycle_index = (self.cycle_index + 1) % 420; // One processor divides by 15, one divides by 7, one diviides by 4.
        }
    }
    fn clock_speed(&self) -> u64 {
        53_693_175
    }
    fn set_clock_speed(&mut self, _: u64) {} // But no
    fn cycles_per_frame(&self) -> u64 {
        896_040
    }
}
