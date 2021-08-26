use common::{BusDevice, BusStatus, BusTransform};
use processor::gbz80::GbZ80Fault;

const KIB: usize = 1024;

// ########## Banks ##########

/// A 16KiB ROM bank.
pub struct RomBank {
    data: Box<[u8; 16 * KIB]>,
}

impl RomBank {
    pub fn new(rom: [u8; 16 * KIB]) -> RomBank {
        RomBank { data : Box::new(rom) }
    }
}

/// A 256 bytes ROM.
pub struct BootRom {
    data: Box<[u8; 256]>,
}

impl BootRom {
    pub fn new(rom: [u8; 256]) -> BootRom {
        BootRom { data : Box::new(rom) }
    }
}

/// A 8KiB RAM bank.
pub struct RamBank {
    data: Box<[u8; 8 * KIB]>,
}

impl RamBank {
    pub fn new() -> RamBank {
        RamBank { data : Box::new([0; 8 * KIB]) }
    }
}

impl Default for RamBank {
    fn default() -> Self {
        Self::new()
    }
}

// ########## BusDevices ##########

impl BusDevice<GbZ80Fault, 1> for RomBank {
    fn read(&mut self, addr: usize, value: &mut [u8; 1])
            -> (Option<GbZ80Fault>, [BusStatus; 1]) {
        value[0] = self.data[addr % (16 * KIB)];
        (None, [BusStatus::Hit])
    }

    fn write(&mut self, _: usize, _: &[u8; 1]) -> Option<GbZ80Fault> {
        None
    }
}

impl BusDevice<GbZ80Fault, 1> for BootRom {
    fn read(&mut self, addr: usize, value: &mut [u8; 1])
            -> (Option<GbZ80Fault>, [BusStatus; 1]) {
        value[0] = self.data[addr % 256];
        (None, [BusStatus::Hit])
    }

    fn write(&mut self, _: usize, _: &[u8; 1]) -> Option<GbZ80Fault> {
        None
    }
}

impl BusDevice<GbZ80Fault, 1> for RamBank {
    fn read(&mut self, addr: usize, value: &mut [u8; 1])
            -> (Option<GbZ80Fault>, [BusStatus; 1]) {
        value[0] = self.data[addr % (8 * KIB)];
        (None, [BusStatus::Hit])
    }

    fn write(&mut self, addr: usize, value: &[u8; 1]) -> Option<GbZ80Fault> {
        self.data[addr % (8 * KIB)] = value[0];
        None
    }
}

pub struct MBC1 {
    rombanks: Vec<RomBank>,
    rambanks: Vec<RamBank>,
    low_rom_id: u8,
    high_rom_id: u8,
    ram_id: u8,
    ram_enable: bool,
    /// false for mode 0, true for mode 1
    banking_mode: bool,
}

impl MBC1 {
    pub fn new(rombanks: Vec<RomBank>, rambanks: Vec<RamBank>) -> MBC1 {
        MBC1 {
            rombanks,
            rambanks,
            low_rom_id: 0,
            high_rom_id: 1,
            ram_id: 0,
            ram_enable: false,
            banking_mode: false,
        }
    }
}

impl BusDevice<GbZ80Fault, 1> for MBC1 {
    fn read(&mut self, addr: usize, value: &mut [u8; 1])
            -> (Option<GbZ80Fault>, [BusStatus; 1]) {
        if addr < 0x4000 {
            self.rombanks.get_mut(self.low_rom_id as usize)
                .expect("Accessing ROM bank")
                .read(addr, value)
        } else if addr < 0x8000 {
            self.rombanks.get_mut(self.high_rom_id as usize)
                .expect("Accessing ROM bank")
                // RomBank::read truncates addr, no need to map it here
                .read(addr, value)
        } else if self.ram_enable {
            self.rambanks.get_mut(self.ram_id as usize)
                .expect("Accessing RAM bank")
                // Ditto for RamBank
                .read(addr, value)
        } else {
            // RAM is disabled
            value[0] = 0xFF;
            (None, [BusStatus::Hit])
        }
    }

    fn write(&mut self, addr: usize, value: &[u8; 1]) -> Option<GbZ80Fault> {
        // We expect RAM writes to be more frequent than ROM ones, so we
        // manage RAM first, with early exits.
        if (0xA000..0xC000).contains(&addr) {
            if self.ram_enable {
                return self.rambanks.get_mut(self.ram_id as usize)
                    .expect("Accessing RAM bank")
                    .write(addr, value)
            }
            return None;
        }

        if addr < 0x2000 {
            // RAM Enable register
            self.ram_enable = value[0] & 0b1111 == 0xA
        } else if addr < 0x4000 {
            // ROM Bank Number register
            let mut banknum = value[0];
            // Keep only the low 5 bits
            banknum &= 0b11111;
            // Minimum allowed value: 1
            if banknum == 0 {
                banknum = 1;
            }

            // Merge with the existing upper bits
            banknum |= self.high_rom_id & 0b0110_0000;
            // At least one rombank => this can't panic
            self.high_rom_id = banknum % self.rombanks.len() as u8;
        } else if addr < 0x6000 {
            // RAM Bank Number register / Upper bits of ROM Bank Number
            if !self.banking_mode { // Mode 0
                let upper_bits = value[0] & 0b11;
                // Merge with the existing low bits
                let banknum = (upper_bits << 5) | (self.high_rom_id & 0b11111);
                // Ditto
                self.high_rom_id = banknum % self.rombanks.len() as u8;
                // Difference with 0 in the low bits is already enforced
            } else { // Mode 1
               if self.rambanks.len() > 1 {
                   self.ram_id = value[0] & 0b11;
               } else {
                    let upper_bits = value[0] & 0b11;
                    // Same as Mode 0
                    let banknum = (upper_bits << 5)
                        | (self.high_rom_id & 0b11111);
                    self.high_rom_id = banknum % self.rombanks.len() as u8;

                    // Also for the low ROM
                    let banknum = upper_bits << 5;
                    self.low_rom_id = banknum % self.rombanks.len() as u8;
               }
            }
        } else if addr < 0x8000 {
            // Banking Mode Select
            self.banking_mode = value[0] & 0b1 == 0b1;
        }

        None
    }
}

pub struct MBC2 {
    rombanks: Vec<RomBank>,
    ram: [u8; 512],
    high_rom_id: u8,
    ram_enable: bool,
}

impl MBC2 {
    pub fn new(rombanks: Vec<RomBank>, ram: [u8; 512]) -> MBC2 {
        MBC2 {
            rombanks,
            ram,
            high_rom_id: 1,
            ram_enable: false,
        }
    }
}

impl BusDevice<GbZ80Fault, 1> for MBC2 {
    fn read(&mut self, addr: usize, value: &mut [u8; 1])
            -> (Option<GbZ80Fault>, [BusStatus; 1]) {
        if addr < 0x4000 {
            self.rombanks.get_mut(0)
                .expect("Accessing ROM bank")
                .read(addr, value)
        } else if addr < 0x8000 {
            self.rombanks.get_mut(self.high_rom_id as usize)
                .expect("Accessing ROM bank")
                // RomBank::read truncates addr, no need to map it here
                .read(addr, value)
        } else {
            value[0] = if self.ram_enable {
                self.ram[addr % 512] | 0b1111_0000
            } else {
                0xFF
            };
            (None, [BusStatus::Hit])
        }
    }

    fn write(&mut self, addr: usize, value: &[u8; 1]) -> Option<GbZ80Fault> {
        // Not the same considerations as for MBC1 since this is limited RAM,
        // probably used for saving rather than working.
        if addr < 0x4000 {
            if addr & 0b1_0000_0000 == 0 {
                // RAM Enable
                self.ram_enable = value[0] & 0b1111 == 0xA;
            } else {
                // ROM Number
                let mut banknum = value[0] & 0b1111;
                if banknum == 0 {
                    banknum = 1;
                }
                self.high_rom_id = banknum % self.rombanks.len() as u8;
            }
        } else if (0xA000..0xC000).contains(&addr) && self.ram_enable {
            self.ram[addr % 512] = value[0] & 0b1111;
        }

        None
    }
}

// ########## BusTransforms ##########

/// Naive mapping of a device.
///
/// This implementation maps a single area of memory, and maps the offset into
/// the area to the internal address.
pub struct SimpleMapping {
    base: usize,
    size: usize
}

impl SimpleMapping {
    pub fn new(base: usize, size: usize) -> SimpleMapping {
        SimpleMapping { base, size }
    }
}

impl BusTransform for SimpleMapping {
    fn transform(&self, addr: usize) -> Option<usize> {
        if self.base <= addr && addr < self.base + self.size {
            Some(addr - self.base)
        } else {
            None
        }
    }
}

/// Mapping of all ROM and switchable RAM areas.
///
/// This mapper triggers on addresses `0x0000-0x7FFF` and `0xA000-0xBFFF` and
/// maps them to themselves.
///
/// This is intended for use with a BusDevice that can react to accesses in all
/// those areas.
pub struct ROMswRAMMapping {}

impl BusTransform for ROMswRAMMapping {
    fn transform(&self, addr: usize) -> Option<usize> {
        if addr < 0x8000 || (0xA000..0xC000).contains(&addr) {
            Some(addr)
        } else {
            None
        }
    }
}
