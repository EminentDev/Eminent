use std::{cell::RefCell, rc::Rc, fmt};

use common::Bus;

use crate::Processor;

const DEBUG: bool = true;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum M68KFault {}

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum M68KSize {
    Byte,
    Word,
    Long,
}

impl M68KSize {
    pub fn from_type_a<A: TryInto<u8>>(size: A) -> Self {
        match size.try_into().ok().unwrap() {
            0 => M68KSize::Byte,
            1 => M68KSize::Word,
            2 => M68KSize::Long,
            _ => panic!("invalid size"),
        }
    }
}

impl fmt::Display for M68KSize {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            M68KSize::Byte => ".b",
            M68KSize::Word => ".w",
            M68KSize::Long => ".l",
        }.fmt(f)
    }
}

impl From<M68KSize> for u8 {
    // Note: returns standard (type a)
    fn from(size: M68KSize) -> Self {
        match size {
            M68KSize::Byte => 0,
            M68KSize::Word => 1,
            M68KSize::Long => 2,
        }
    }
}

pub struct M68KEffectiveAddress {
    size: M68KSize,
    m: u8,
    xn: u8,
    resolution: Option<u32>,
}

impl M68KEffectiveAddress {
    pub fn new<A: TryInto<u8>, B: TryInto<u8>, C: TryInto<u8>>(size: A, m: B, xn: C) -> Self {
        Self {
            size: M68KSize::from_type_a(size),
            m: m.try_into().ok().unwrap(),
            xn: xn.try_into().ok().unwrap(),
            resolution: None,
        }
    }

    pub fn resolve(&mut self, m68k: &mut M68K) {
        match self.m {
            5 => {
                // Sign extension in Rust be like:
                let disp = m68k.read_u16(m68k.pc) as i16 as i32 as u32;
                m68k.pc += 2;
                let addr = m68k.a(self.xn) + disp;
                if DEBUG {
                    println!("Resolving (0x{:04X}, A{}) as {:08X}", disp, self.xn, addr);
                }
                self.resolution = Some(addr);
            }
            6 | 7 => todo!(),
            _ => {}
        }
    }

    pub fn read<D: TryFrom<u32>>(&self, m68k: &mut M68K) -> D {
        D::try_from(match (self.size, self.m, self.xn) {
            (M68KSize::Byte, 0, xn) => m68k.d[xn as usize] & 0xFF,
            (M68KSize::Long, 5, _) => m68k.read_u32(self.resolution.unwrap()),
            (a, b, c) => todo!("{:?}, {}, {}", a, b, c),
        })
        .ok()
        .unwrap()
    }

    pub fn write<D: Into<u32>>(&self, m68k: &mut M68K, data: D) {
        match (self.size, self.m, self.xn) {
            (M68KSize::Long, 5, _) => m68k.write_u32(self.resolution.unwrap(), data.into()),
            (a, b, c) => todo!("{:?}, {}, {}", a, b, c),
        }
    }
}

pub type M68KBus = Rc<RefCell<Bus<M68KFault, 2>>>;

pub struct M68K {
    bus: M68KBus,
    d: [u32; 8],
    a: [u32; 7],
    usp: u32,
    ssp: u32,
    pc: u32,
    sr: u16,
}

impl M68K {
    fn a<X: Into<usize>>(&self, an: X) -> u32 {
        let an = an.into();
        if an == 7 {
            todo!()
        } else {
            self.a[an]
        }
    }

    pub fn new(bus: M68KBus) -> Self {
        let ssp: u32 = unsafe {
            let ms = bus.borrow().read(0).1.clone();
            let ls = bus.borrow().read(2).1;
            u32::from(ms[0]) << 24
                | u32::from(ms[1]) << 16
                | u32::from(ls[0]) << 8
                | u32::from(ls[1])
        };
        let pc: u32 = unsafe {
            let ms = bus.borrow().read(4).1.clone();
            let ls = bus.borrow().read(6).1;
            u32::from(ms[0]) << 24
                | u32::from(ms[1]) << 16
                | u32::from(ls[0]) << 8
                | u32::from(ls[1])
        };
        Self {
            bus,
            d: [0, 0, 0, 0, 0, 0, 0, 0],
            a: [0, 0, 0, 0, 0, 0, 0],
            usp: 0,
            ssp,
            pc,
            sr: 0x0700,
        }
    }

    fn read_u8(&mut self, addr: u32) -> u8 {
        let addr = usize::try_from(addr).unwrap();
        let result = unsafe { self.bus.borrow().read(addr & 0xFFFFFFFE) };
        let result = if result.0.is_some() {
            todo!();
        } else if addr & 1 == 0 {
            result.1[0]
        } else {
            result.1[1]
        };
        if DEBUG {
            println!("reading 0x{:08X} -> 0x{:02X}", addr, result);
        }
        result
    }

    fn read_u16(&mut self, addr: u32) -> u16 {
        let addr = usize::try_from(addr).unwrap();
        let result = unsafe { self.bus.borrow().read(addr & 0xFFFFFFFE) };
        let result = if result.0.is_some() {
            todo!();
        } else {
            u16::from(result.1[0]) << 8 | u16::from(result.1[1])
        };
        if DEBUG {
            println!("reading 0x{:08X} -> 0x{:04X}", addr, result);
        }
        result
    }

    fn read_u32(&mut self, addr: u32) -> u32 {
        let addr = usize::try_from(addr).unwrap();
        let result1 = unsafe { self.bus.borrow().read(addr & 0xFFFFFFFE) };
        if result1.0.is_some() {
            todo!();
        }
        let result1 = result1.1.clone();
        let result2 = unsafe { self.bus.borrow().read((addr + 2) & 0xFFFFFFFE) };
        println!("{:?} {:?}", result1, result2.1);
        let result = if result2.0.is_some() {
            todo!();
        } else {
            u32::from(result1[0]) << 24
                | u32::from(result1[1]) << 16
                | u32::from(result2.1[0]) << 8
                | u32::from(result2.1[1])
        };
        if DEBUG {
            println!("reading 0x{:08X} -> 0x{:08X}", addr, result);
        }
        result
    }

    fn write_u16(&mut self, addr: u32, data: u16) {
        let addr = usize::try_from(addr).unwrap();
        unsafe {
            self.bus
                .borrow()
                .write(addr & 0xFFFFFFFE, &[(data >> 8) as u8, data as u8]);
        }
        if DEBUG {
            println!("writing 0x{:04X} -> 0x{:08X}", data, addr);
        }
    }

    fn write_u32(&mut self, addr: u32, data: u32) {
        let addr = usize::try_from(addr).unwrap();
        unsafe {
            self.bus
                .borrow()
                .write(addr & 0xFFFFFFFE, &[(data >> 24) as u8, (data >> 16) as u8]);
            self.bus
                .borrow()
                .write((addr + 2) & 0xFFFFFFFE, &[(data >> 8) as u8, data as u8]);
        }
        if DEBUG {
            println!("writing 0x{:08X} -> 0x{:08X}", data, addr);
        }
    }
}

impl Processor for M68K {
    fn tick(&mut self) {
        let insn = self.read_u16(self.pc);
        self.pc += 2;
        match insn & 0xF000 {
            0 => {
                if insn & 0x0100 == 0 {
                    match insn & 0x0E00 {
                        0x0200 => {
                            if insn == 0x023C || insn == 0x027C {
                                todo!("ANDI to SR");
                            }
                            let size = M68KSize::from_type_a((insn & 0x00C0) >> 6);
                            let mut ea = M68KEffectiveAddress::new(
                                size,
                                (insn & 0x0038) >> 3,
                                insn & 0x0007,
                            );
                            ea.resolve(self);
                            let a: u32 = ea.read(self);
                            let b = match size {
                                M68KSize::Byte => {
                                    let result = self.read_u8(self.pc);
                                    self.pc += 2;
                                    result.into()
                                }
                                M68KSize::Word => {
                                    let result = self.read_u16(self.pc);
                                    self.pc += 2;
                                    result.into()
                                }
                                M68KSize::Long => {
                                    let result = self.read_u32(self.pc);
                                    self.pc += 4;
                                    result
                                }
                            };
                            let result = a & b;
                            if DEBUG {
                                println!("ANDI{} #0x{:08X}, 0x{:08X} = 0x{:08X}", size, b, a, result);
                            }
                            self.sr &= 0xFFF0;
                            if result == 0 {
                                self.sr |= 4;
                            }
                            if match size {
                                M68KSize::Byte => result & 0x80,
                                M68KSize::Word => result & 0x8000,
                                M68KSize::Long => result & 0x80000000,
                            } != 0
                            {
                                self.sr |= 8;
                            }
                            ea.write(self, result);
                        }
                        _ => todo!("0x{:04X}", insn),
                    }
                } else {
                    todo!("0x01__");
                }
            }
            x if x & 0xC000 == 0 => todo!("MOVE instructions"),
            x => todo!("0x{:04X}", x),
        }
    }
}
