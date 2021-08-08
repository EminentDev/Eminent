use common::ReadSeek;

#[cfg(feature = "gb")]
pub mod gb;
#[cfg(feature = "nes")]
pub mod nes;

pub trait FileLoader {
    fn can_load(&self, filename: &str, file: &mut dyn ReadSeek) -> bool;
    fn load(&self, file: &mut dyn ReadSeek) -> Box<dyn System>;
}

pub trait System {
    fn tick(&mut self, num_cycles: u64);
    fn clock_speed(&self) -> u64;
    fn set_clock_speed(&mut self, speed: u64);
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
