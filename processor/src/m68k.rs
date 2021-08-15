use crate::Processor;
use common::Bus;
use std::cell::RefCell;
use std::rc::Rc;

pub struct M68KFault {}

enum M68KState {
    Reset(u8),
    Fetch,
}

struct M68KRegisters {
    a: [u32; 7], // A7 is stored separately
    d: [u32; 8],
    pc: u32,
    sr: u16,  // Also contains CCR
    usp: u32, // A7 in user mode
    ssp: u32, // A7 in supervisor mode
}

impl Default for M68KRegisters {
    fn default() -> Self {
        M68KRegisters {
            a: [0; 7],
            d: [0; 8],
            pc: 0,
            sr: 0,
            usp: 0,
            ssp: 0,
        }
    }
}

#[derive(Clone, Copy)]
enum M68KSize {
    Byte,
    Word,
    Long,
}

impl M68KSize {
    fn from_two_bit(size: u8) -> Self {
        match size {
            0 => M68KSize::Byte,
            1 => M68KSize::Word,
            2 => M68KSize::Long,
            3 => panic!("Illegal two bit size code"),
            _ => unreachable!(),
        }
    }
}

pub struct M68K {
    bus: Rc<RefCell<Bus<M68KFault, 2>>>,
    state: M68KState,
    temp: u16,
    stall: u64,
    regs: M68KRegisters,
}

impl M68K {
    pub fn new(bus: Rc<RefCell<Bus<M68KFault, 2>>>) -> M68K {
        M68K {
            bus,
            state: M68KState::Reset(0),
            temp: 0,
            stall: 0,
            regs: M68KRegisters::default(),
        }
    }

    fn ea_val(&mut self, size: M68KSize, mode: u8, reg: u8) -> u32 {
        match mode {
            7 => match reg {
                1 => {
                    let (_, value) = self.read(self.regs.pc as usize);
                    self.temp = value;
                    self.regs.pc += 2;
                    let (_, value) = self.read(self.regs.pc as usize);
                    let addr = ((self.temp as u32) << 16 | value as u32) & 0x00FFFFFF;
                    self.regs.pc += 2;
                    match size {
                        M68KSize::Byte => todo!(),
                        M68KSize::Word => todo!(),
                        M68KSize::Long => {
                            let (_, value) = self.read(addr as usize);
                            self.temp = value;
                            let (_, value) = self.read((addr + 2) as usize);
                            let data = (self.temp as u32) << 16 | value as u32;
                            println!("EA read-only resolution: ${:06X}.L = ${:08X}", addr, data); // TODO: Make this use a debug flag
                            data
                        }
                    }
                }
                _ => todo!(),
            },
            _ => todo!(),
        }
    }

    fn read(&mut self, addr: usize) -> (Option<M68KFault>, u16) {
        unsafe {
            // SAFETY: Assume we're fine.
            let (fault, value, _) = self.bus.borrow().read(addr);
            self.stall += 4; // Each read takes 4 cycles. No matter what.
            (fault, (value[0] as u16) << 8 | value[1] as u16)
        }
    }
}

fn sign_extend(size: M68KSize, data: u32) -> u32 {
    match size {
        M68KSize::Byte => {
            if data & 0x80 != 0 {
                data | 0xFFFFFF00
            } else {
                data
            }
        }
        M68KSize::Word => {
            if data & 0x8000 != 0 {
                data | 0xFFFF0000
            } else {
                data
            }
        }
        M68KSize::Long => data,
    }
}

impl Processor for M68K {
    fn tick(&mut self) {
        if self.stall == 0 {
            match self.state {
                M68KState::Reset(0) => {
                    let (_, value) = self.read(0x000000_usize);
                    self.temp = value;
                    self.state = M68KState::Reset(1)
                }
                M68KState::Reset(1) => {
                    let (_, value) = self.read(0x000002_usize);
                    self.regs.ssp = (self.temp as u32) << 16 | value as u32;
                    self.state = M68KState::Reset(2)
                }
                M68KState::Reset(2) => {
                    let (_, value) = self.read(0x000004_usize);
                    self.temp = value;
                    self.state = M68KState::Reset(3)
                }
                M68KState::Reset(3) => {
                    let (_, value) = self.read(0x000006_usize);
                    self.regs.pc = (self.temp as u32) << 16 | value as u32;
                    self.state = M68KState::Fetch
                }
                M68KState::Fetch => {
                    let (_, inst) = self.read(self.regs.pc as usize);
                    self.regs.pc += 2;
                    match inst & 0xF000 {
                        0x4000 => match inst & 0x0FFF {
                            0x0AFC => todo!(),
                            x if x & 0x0FC0 == 0x0AC0 => todo!(),
                            x if x & 0x0F00 == 0x0A00 => {
                                let size = M68KSize::from_two_bit(((x & 0x00C0) >> 6) as u8);
                                let mode = (x & 0x0038) >> 3;
                                let reg = x & 0x0007;
                                let ea =
                                    sign_extend(size, self.ea_val(size, mode as u8, reg as u8));
                                println!("TST #${:08X}", ea);
                                self.regs.sr &= 0xFFF0;
                                println!("C flag cleared");
                                println!("V flag cleared");
                                if ea == 0 {
                                    self.regs.sr |= 0x4;
                                    println!("Z flag set");
                                } else {
                                    println!("Z flag cleared");
                                }
                                if ea & 0x80000000 != 0 {
                                    self.regs.sr |= 0x8;
                                    println!("N flag set");
                                } else {
                                    println!("N flag cleared");
                                }
                            }
                            _ => todo!(),
                        },
                        _ => todo!(),
                    }
                }
                _ => unreachable!(),
            }
        }
        self.stall -= 1;
    }
}
