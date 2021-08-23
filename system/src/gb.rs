use std::cell::RefCell;
use std::ffi::OsStr;
use std::io::SeekFrom;
use std::path::Path;
use std::rc::Rc;

use crate::{FileLoader, System};
use common::{Bus, BusDevice, BusStatus, BusTransform, ReadSeek};
use processor::gbz80::{GbZ80Fault, GbZ80};

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
struct BootRom {
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

struct MBC1 {
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
    fn new(rombanks: Vec<RomBank>, rambanks: Vec<RamBank>) -> MBC1 {
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
            (None, [BusStatus::OpenBus])
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
            self.banking_mode = value[0] == 0x01;
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

// ########## GbSystem implementation ##########

/// When bootROM emulation is activated, which step will be executed next tick.
pub enum BootStep {
    /// Setup display and sound system
    SETUP,
    /// Put Nintendo logo on screen
    DISPLAY,
    /// Scroll up screen and play sounds
    SCROLL(u8),
    /// Perform the logo check
    CHECK,
    /// No bootROM emulation
    DONE,
}

pub struct GbSystem {
    bus: Rc<RefCell<Bus<GbZ80Fault, 1>>>,
    proc: GbZ80,
    step: BootStep,
}

impl GbSystem {
    /// Split a ROM into multiple 16KiB ROM banks.
    ///
    /// # Panics
    ///
    /// Panic on `rom.seek()` or `rom.read()` error.
    fn split_rom(rom: &mut dyn ReadSeek) -> Vec<RomBank> {
        rom.seek(SeekFrom::Start(0)).expect("Reading GB ROM");

        let mut rombanks = Vec::new();
        loop {
            let mut buf: [u8; 16 * KIB] = [0; 16 * KIB];
            let size = rom.read(&mut buf).expect("Reading GB ROM");
            if size == 16 * KIB {
                rombanks.push(RomBank::new(buf));
            } else if size == 0 {
                // End of file
                break;
            } else {
                // Don't trust the contents of `buf` after `size` bytes.
                // This should get optimized into a memset().
                buf.iter_mut().skip(size).for_each(|b| *b = 0);
                rombanks.push(RomBank::new(buf));
                // Also, assume end of file
                break;
            }
        }

        rombanks
    }

    /// Create a GbSystem with no bootROM.
    ///
    /// The behaviour of the bootROM will be emulated. This can provide better 
    /// information about what's happening during the boot process, at the
    /// expense of less accurate information
    ///
    /// # Panics
    ///
    /// Panic on `rom.seek()` or `rom.read()` error.
    pub fn new_emulate_bootrom(rom: &mut dyn ReadSeek) -> GbSystem {
        let bus = Rc::new(RefCell::new(Bus::<GbZ80Fault, 1>::new()));
        let proc = GbZ80::new(bus.clone());

        GbSystem::instanciate(rom, bus, proc, BootStep::SETUP)
    }

    /// Create a GbSystem with a bootROM.
    ///
    /// The execution will start in the bootROM. If you feed an actual GB
    /// bootROM, this is the most accurate way to emulate, but may lead to a
    /// more obscure boot process.
    ///
    /// If `bootrom`'s contents are longer than 256 bytes, they will be
    /// truncated to 256 bytes. If they are shorter than 256 bytes, the rest of
    /// the bootROM space will be filled with zeroes (= `NOOP`s).
    ///
    /// # Panics
    ///
    /// Panic on `seek()` or `read()` error from either parameter.
    pub fn new_with_bootrom(bootrom: &mut dyn ReadSeek, rom: &mut dyn ReadSeek)
            -> GbSystem {
        bootrom.seek(SeekFrom::Start(0)).expect("Reading GB BootROM");
        let mut buf: [u8; 256] = [0; 256];
        let size = bootrom.read(&mut buf).expect("Reading GB BootROM");
        // Don't trust the contents of `buf` after `size` bytes.
        // This should get optimized into a memset().
        buf.iter_mut().skip(size).for_each(|b| *b = 0);
        let bootrom = Box::new(BootRom::new(buf));

        let mut bus = Bus::<GbZ80Fault, 1>::new();
        bus.add_device(bootrom, Box::new(SimpleMapping::new(0, 256)));

        let bus = Rc::new(RefCell::new(bus));
        let proc = GbZ80::new(bus.clone());

        GbSystem::instanciate(rom, bus, proc, BootStep::DONE)
    }

    /// Create a GbSystem generically.
    ///
    /// From the ROM:
    /// * build the ROM banks
    /// * setup the bus according to the ROM header
    /// * finalize the construction of the GbSystem
    ///
    /// This should be the final step; any specific setup of the bus or proc
    /// should be done before calling this function.
    ///
    /// # Panics
    ///
    /// Panic on `rom.seek()` or `rom.read()` error.
    fn instanciate(rom: &mut dyn ReadSeek, _bus: Rc<RefCell<Bus<GbZ80Fault, 1>>>,
                   _proc: GbZ80, _step: BootStep) -> GbSystem {
        let mut _rombanks = GbSystem::split_rom(rom);
        unimplemented!() // TODO
    }
}

impl System for GbSystem {
    fn tick(&mut self, _: u64) {}
    fn clock_speed(&self) -> u64 {
        4_194_304
    }
    fn set_clock_speed(&mut self, _: u64) {} // TODO
    fn cycles_per_frame(&self) -> u64 {
        70_221
    }
}

pub struct GbFileLoader {}

impl FileLoader for GbFileLoader {
    fn can_load(&self, filename: &str, _: &mut dyn ReadSeek) -> bool {
        Path::new(filename).extension().and_then(OsStr::to_str) == Some("gb")
    }

    fn load(&self, _: &mut dyn ReadSeek) -> Box<dyn System> {
        unimplemented!() // TODO
    }
}
