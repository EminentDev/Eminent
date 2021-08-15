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

    fn read(&mut self, addr: usize) -> (Option<M68KFault>, u16) {
        unsafe {
            // SAFETY: Assume we're fine.
            let (fault, value, _) = self.bus.borrow().read(addr);
            self.stall = 3; // Each read takes 4 cycles. No matter what.
            (fault, (value[0] as u16) << 8 | value[1] as u16)
        }
    }
}

impl Processor for M68K {
    fn tick(&mut self) {
        if self.stall != 0 {
            self.stall -= 1;
        } else {
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
                    println!("Instruction: {:#06X}", inst);
                }
                _ => unreachable!(),
            }
        }
    }
}
