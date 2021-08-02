use common::ReadSeek;

pub mod nes;

pub trait FileLoader {
    fn can_load(&self, filename: &str, file: &mut dyn ReadSeek) -> bool;
    fn load(&self, file: &mut dyn ReadSeek) -> Box<dyn System>;
}

pub trait System {}

const LOADERS: [&dyn FileLoader; 1] = [&nes::NesFileLoader {}];

pub fn get_file_loader(filename: &str, file: &mut dyn ReadSeek) -> Option<&'static dyn FileLoader> {
    for loader in LOADERS {
        if loader.can_load(filename, file) {
            return Some(loader);
        }
    }
    None
}
