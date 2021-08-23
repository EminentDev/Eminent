use std::cell::RefCell;
use std::ffi::OsStr;
use std::io::SeekFrom;
use std::path::Path;
use std::rc::Rc;

use crate::{FileLoader, System};
use common::{Bus, ReadSeek};
use processor::gbz80::{GbZ80Fault, GbZ80};

pub mod bus;

const KIB: usize = 1024;

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
    fn split_rom(rom: &mut dyn ReadSeek) -> Vec<bus::RomBank> {
        rom.seek(SeekFrom::Start(0)).expect("Reading GB ROM");

        let mut rombanks = Vec::new();
        loop {
            let mut buf: [u8; 16 * KIB] = [0; 16 * KIB];
            let size = rom.read(&mut buf).expect("Reading GB ROM");
            if size == 16 * KIB {
                rombanks.push(bus::RomBank::new(buf));
            } else if size == 0 {
                // End of file
                break;
            } else {
                // Don't trust the contents of `buf` after `size` bytes.
                // This should get optimized into a memset().
                buf.iter_mut().skip(size).for_each(|b| *b = 0);
                rombanks.push(bus::RomBank::new(buf));
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
        let bootrom = Box::new(bus::BootRom::new(buf));

        let mut bus = Bus::<GbZ80Fault, 1>::new();
        bus.add_device(bootrom, Box::new(bus::SimpleMapping::new(0, 256)));

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
