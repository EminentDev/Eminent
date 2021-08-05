use common::ReadSeek;
use processor::Processor;

#[cfg(feature = "gb")]
pub mod gb;
#[cfg(feature = "nes")]
pub mod nes;

pub trait FileLoader {
    fn can_load(&self, filename: &str, file: &mut dyn ReadSeek) -> bool;
    fn load(&self, file: &mut dyn ReadSeek) -> System;
}

pub struct System {
    processors: Vec<Box<dyn Processor>>,
    schedule: Vec<u64>, // e.g. 0b100101 ticks the 1st, 3rd, and 6th processors in order
    pub clocks_per_second: u64,
    cycle: u64,
}

impl System {
    pub fn tick(&mut self) {
        let mut to_tick = self.schedule[self.cycle as usize];
        for i in 0..64 {
            if to_tick == 0 {
                break;
            }
            if to_tick & 1 == 1 {
                self.processors[i].tick();
            }
            to_tick >>= 1;
        }
        self.cycle += 1;
        if self.cycle == self.schedule.len() as u64 {
            self.cycle = 0;
        }
    }
}

macro_rules! init_loaders_array{
    [$($loader:expr => $name:literal),*] => {
        {
            #[allow(unused_mut)]
            let mut vec = Vec::<&'static (dyn FileLoader + Sync)>::new();
            $({
                #[cfg(feature=$name)]
                {
                    vec.push(& $loader)
                }
            })*
            vec
        }
    };
}

lazy_static::lazy_static! {
    static ref LOADERS: Vec<&'static (dyn FileLoader + Sync)> = init_loaders_array![nes::NesFileLoader{} => "nes", gb::GbFileLoader{} => "gb"];
}

pub fn get_file_loader(
    filename: &str,
    file: &mut dyn ReadSeek,
) -> Option<&'static (dyn FileLoader + Sync)> {
    for loader in &*LOADERS {
        if loader.can_load(filename, file) {
            return Some(*loader);
        }
    }
    None
}
