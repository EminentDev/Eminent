use std::{cell::RefCell, fmt, rc::Rc, result};

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

    pub fn from_type_b<A: TryInto<u8>>(size: A) -> Self {
        match size.try_into().ok().unwrap() {
            1 => M68KSize::Byte,
            3 => M68KSize::Word,
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
        }
        .fmt(f)
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

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum M68KConditionCode {
    True,
    False,
    High,
    LowOrSame,
    CarryClear,
    CarrySet,
    NotEqual,
    Equal,
    OverflowClear,
    OverflowSet,
    Plus,
    Minus,
    GreaterOrEqual,
    LessThan,
    GreaterThan,
    LessOrEqual,
}

impl M68KConditionCode {
    pub fn eval(&self, m68k: &M68K) -> bool {
        match self {
            Self::True => true,
            Self::False => false,
            Self::CarryClear => m68k.sr & 1 == 0,
            Self::CarrySet => m68k.sr & 1 != 0,
            x => todo!("{:?}", x),
        }
    }
}

impl fmt::Display for M68KConditionCode {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::True => "T",
            Self::False => "F",
            _ => todo!(),
        }
        .fmt(f)
    }
}

impl From<u8> for M68KConditionCode {
    fn from(cc: u8) -> Self {
        match cc {
            0 => Self::True,
            1 => Self::False,
            2 => Self::High,
            3 => Self::LowOrSame,
            4 => Self::CarryClear,
            5 => Self::CarrySet,
            6 => Self::NotEqual,
            7 => Self::Equal,
            8 => Self::OverflowClear,
            9 => Self::OverflowSet,
            10 => Self::Plus,
            11 => Self::Minus,
            12 => Self::GreaterOrEqual,
            13 => Self::LessThan,
            14 => Self::GreaterThan,
            15 => Self::LessOrEqual,
            _ => unreachable!(),
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
            3 => {
                let val = m68k.a(self.xn);
                *m68k.a_mut(self.xn) += 1;
                if DEBUG {
                    println!("Resolving (A{})+ as {:08X}", self.xn, val);
                }
                self.resolution = Some(val);
            }
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
            6 => todo!(),
            7 => match self.xn {
                1 => {
                    let val = m68k.read_u32(m68k.pc);
                    m68k.pc += 4;
                    if DEBUG {
                        println!("Resolving 0x{:08X}", val);
                    }
                    self.resolution = Some(val);
                }
                4 => {
                    let val = match self.size {
                        M68KSize::Byte => {
                            let result = m68k.read_u8(m68k.pc);
                            m68k.pc += 2;
                            result.into()
                        }
                        M68KSize::Word => {
                            let result = m68k.read_u16(m68k.pc);
                            m68k.pc += 2;
                            result.into()
                        }
                        M68KSize::Long => {
                            let result = m68k.read_u32(m68k.pc);
                            m68k.pc += 4;
                            result
                        }
                    };
                    if DEBUG {
                        println!("Resolving immediate #0x{:08X}", val);
                    }
                    self.resolution = Some(val);
                }
                x => todo!("{}", x),
            },
            _ => {}
        }
    }

    pub fn read<D: TryFrom<u32>>(&self, m68k: &mut M68K) -> D {
        D::try_from(match (self.size, self.m, self.xn) {
            (M68KSize::Byte, 0, dn) => m68k.d[dn as usize] & 0xFF,
            (M68KSize::Word, 0, dn) => m68k.d[dn as usize] & 0xFFFF,
            (M68KSize::Long, 0, dn) => m68k.d[dn as usize],
            (M68KSize::Word, 3, _) => m68k.read_u16(self.resolution.unwrap()) as u32,
            (M68KSize::Long, 5, _) => m68k.read_u32(self.resolution.unwrap()),
            (_, 7, 4) => self.resolution.unwrap(),
            (a, b, c) => todo!("{:?}, {}, {}", a, b, c),
        })
        .ok()
        .unwrap()
    }

    pub fn write<D: Into<u32>>(&self, m68k: &mut M68K, data: D) {
        match (self.size, self.m, self.xn) {
            (M68KSize::Word, 0, dn) => {
                let dn = dn as usize;
                let mut val = m68k.d[dn];
                val &= 0xFF00;
                val |= data.into();
                m68k.d[dn] = val;
            }
            (M68KSize::Long, 0, dn) => {
                m68k.d[dn as usize] = data.into();
            }
            (M68KSize::Long, 1, an) => {
                m68k.a[an as usize] = data.into();
            }
            (M68KSize::Word, 3, _) | (M68KSize::Word, 7, 1) => {
                m68k.write_u16(self.resolution.unwrap(), data.into() as u16)
            }
            (M68KSize::Long, 3, _) | (M68KSize::Long, 7, 1) => {
                m68k.write_u32(self.resolution.unwrap(), data.into())
            }
            (M68KSize::Long, 5, _) => m68k.write_u32(self.resolution.unwrap(), data.into()),
            (a, b, c) => todo!("{:?}, {}, {}", a, b, c),
        }
    }

    pub fn ea(&self) -> u32 {
        match (self.m, self.xn) {
            (7, 1) => self.resolution.unwrap(),
            (a, b) => todo!("{}, {}", a, b),
        }
    }
}

impl fmt::Display for M68KEffectiveAddress {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match (self.m, self.xn) {
            (0, dn) => write!(f, "D{}", dn),
            (1, an) => write!(f, "A{}", an),
            (3, an) => write!(f, "(A{})+", an),
            (7, 1) => write!(f, "0x{:08X}", self.resolution.unwrap()),
            (7, 4) => write!(f, "#0x{:08X}", self.resolution.unwrap()),
            (a, b) => todo!("{}, {}", a, b),
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
    fn a_mut<X: Into<usize>>(&mut self, an: X) -> &mut u32 {
        let an = an.into();
        if an == 7 {
            todo!()
        } else {
            &mut self.a[an]
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
                            if DEBUG {
                                println!("ANDI{} #0x{:08X}, {} = 0x{:08X}", size, b, ea, result);
                            }
                        }
                        0x0600 => {
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
                            let result = a.wrapping_add(b);
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
                            if DEBUG {
                                println!("ADDI{} #0x{:08X}, {} = 0x{:08X}", size, b, ea, result);
                            }
                        }
                        _ => todo!("0x{:04X}", insn),
                    }
                } else {
                    todo!("0x01__");
                }
            }
            x if x & 0xC000 == 0 => {
                let size = M68KSize::from_type_b((insn & 0x3000) >> 12);
                let mut dest_ea =
                    M68KEffectiveAddress::new(size, (insn & 0x01C0) >> 6, (insn & 0x0E00) >> 9);
                let mut src_ea =
                    M68KEffectiveAddress::new(size, (insn & 0x0038) >> 3, insn & 0x0007);
                src_ea.resolve(self);
                let val: u32 = src_ea.read(self);
                dest_ea.resolve(self);
                dest_ea.write(self, val);
                if DEBUG {
                    println!("MOVE{} {}, {}", size, dest_ea, src_ea);
                }
            }
            0x4000 => match insn {
                x if x & 0x0FC0 == 0x06C0 => {
                    let mut ea = M68KEffectiveAddress::new(1, (insn & 0x0038) >> 3, insn & 0x0007);
                    ea.resolve(self);
                    self.sr = ea.read(self);
                    if DEBUG {
                        println!("MOVE {}, SR", ea);
                    }
                }
                x if x & 0x0F00 == 0x0200 => {
                    let mut ea = M68KEffectiveAddress::new(1, (insn & 0x0038) >> 3, insn & 0x0007);
                    ea.resolve(self);
                    ea.write(self, 0u32);
                    if DEBUG {
                        println!("CLR {}", ea);
                    }
                }
                x if x & 0x01C0 == 0x01C0 => {
                    let mut ea = M68KEffectiveAddress::new(2, (insn & 0x0038) >> 3, insn & 0x0007);
                    ea.resolve(self);
                    let an = (insn & 0x0E00) >> 9;
                    let ea = ea.ea();
                    *self.a_mut(an) = ea;
                    if DEBUG {
                        println!("LEA 0x{:08X}, A{}", ea, an);
                    }
                }
                x => todo!("0x{:04X}", x),
            },
            0x5000 => match insn {
                x if x & 0x00F8 == 0x00C8 => {
                    let next_pc = self
                        .pc
                        .wrapping_add(self.read_u16(self.pc) as i16 as i32 as u32);
                    self.pc += 2;
                    let dn = (x & 0x0007) as usize;
                    let cc = M68KConditionCode::from(((x & 0x0F00) >> 8) as u8);
                    if !cc.eval(self) {
                        let mut val = self.d[dn] as u16;
                        val = val.wrapping_sub(1);
                        self.d[dn] = (self.d[dn] & 0xFF00) | val as u32;
                        if val != 0xFFFF {
                            self.pc = next_pc;
                        }
                    }
                    if DEBUG {
                        println!("DB{} D{}, 0x{:04X}", cc, dn, next_pc);
                    }
                }
                x if x & 0x00C0 == 0x00C0 => todo!("Scc"),
                x if x & 0x0100 == 0 => {
                    let size = M68KSize::from_type_a((insn & 0x00C0) >> 6);
                    let mut ea =
                        M68KEffectiveAddress::new(size, (insn & 0x0038) >> 3, insn & 0x0007);
                    ea.resolve(self);
                    let a: u32 = ea.read(self);
                    let mut b = ((insn & 0x0E00) >> 9) as u32;
                    if b == 0 {
                        b = 8;
                    }
                    let result = a.wrapping_add(b);
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
                    if DEBUG {
                        println!("ADDQ{} #0x{:01X}, {} = 0x{:08X}", size, b, ea, result);
                    }
                }
                x => todo!("0x{:04X}", x),
            },
            0x6000 => {
                if insn & 0x0F00 == 0x0100 {
                    todo!("BSR");
                }
                let old_pc = self.pc;
                let mut disp = (insn & 0xFF) as u8 as i8 as i32 as u32;
                if disp == 0 {
                    disp = self.read_u16(self.pc) as i16 as i32 as u32;
                    self.pc += 2;
                } else if disp == 0xFFFFFFFF {
                    disp = self.read_u32(self.pc);
                    self.pc += 4;
                }
                let cc = M68KConditionCode::from(((insn & 0x0F00) >> 8) as u8);
                let next_pc = old_pc.wrapping_add(disp);
                if cc.eval(self) {
                    self.pc = next_pc;
                }
                if DEBUG {
                    println!("B{} 0x{:08X}", cc, next_pc);
                }
            }
            0x7000 => {
                assert!(insn & 0x0100 == 0);
                let val: u32 = (insn & 0xFF) as i8 as i64 as u32;
                let dn = (insn & 0x0E00) >> 9;
                self.d[dn as usize] = val;
                if DEBUG {
                    println!("MOVEQ #0x{:08X}, D{}", val, dn);
                }
            }
            x => todo!("0x{:04X}", x),
        }
    }
}
