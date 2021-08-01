use common::ReadSeek;

pub mod gb;

pub trait System {
    fn detect<F: ReadSeek>(filename: &str, file: &mut F) -> bool;
}
