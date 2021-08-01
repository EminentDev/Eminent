use crate::System;
use common::ReadSeek;

pub struct GbSystem {}

impl System for GbSystem {
    fn detect<F: ReadSeek>(_: &str, _: &mut F) -> bool {
        true
    }
}
