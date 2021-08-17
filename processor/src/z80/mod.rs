//! Almost complete implementation of a Zilog Z80 processor.
//!
//! This implementation has all the features of the Z80, except for maskable
//! interrupts and I/O, as they require architecture specificities we don't have
//! yet. If you wan to use this processor, be aware that those features are not
//! available, and that some designing will have to come before they are
//! implementable.

use std::cell::RefCell;
use std::rc::Rc;

use crate::Processor;
use common::Bus;

pub mod instructions;

/// The restart address on reception of a Non-Maskable Interrupt.
pub const NMI_RESTART_ADDR: u16 = 0x0066;

/// A description of an error in the memory bus.
///
/// Unused.
pub struct Z80Fault {}

enum Flag {
    S,
    Z,
    H,
    PV,
    N,
    C,
}

/// The various interrupt modes available on the Z80.
pub enum InterruptMode {
    IM0,
    IM1,
    IM2,
}

/// The state of the CPU. This structure holds all the necessary information
/// for the execution of the CPU.
#[allow(non_snake_case)]
pub struct State {
    pub A: u8,
    pub Aprime: u8,
    pub F: u8,
    pub Fprime: u8,
    pub B: u8,
    pub Bprime: u8,
    pub C: u8,
    pub Cprime: u8,
    pub D: u8,
    pub Dprime: u8,
    pub E: u8,
    pub Eprime: u8,
    pub H: u8,
    pub Hprime: u8,
    pub L: u8,
    pub Lprime: u8,
    pub I: u8,
    pub R: u8,
    pub IX: u16,
    pub IY: u16,
    pub SP: u16,
    pub PC: u16,
    pub IFF1: bool,
    pub IFF2: bool,
    pub IM: InterruptMode,
    pub halted: bool,
}

/// Get the u16 value associated with a compound register
macro_rules! get_reg16 {
    ($state: expr, $high: ident, $low: ident) => {
        u16::from_be_bytes([$state.$high, $state.$low])
    };
}

/// Write a u16 value to a compound register
macro_rules! set_reg16 {
    ($state: expr, $high: ident, $low: ident, $value: expr) => {
        let bytes = $value.to_be_bytes(); // [high, low]
        $state.$high = bytes[0];
        $state.$low = bytes[1];
    };
}

/// The main structure of the CPU.
pub struct Z80 {
    state: State,
    bus: Rc<RefCell<Bus<Z80Fault, 1>>>,
}

impl Z80 {
    /// Instanciate a new Z80 Processor.
    pub fn new(bus: Rc<RefCell<Bus<Z80Fault, 1>>>) -> Z80 {
        let state = State {
            A: 0,
            Aprime: 0,
            F: 0,
            Fprime: 0,
            B: 0,
            Bprime: 0,
            C: 0,
            Cprime: 0,
            D: 0,
            Dprime: 0,
            E: 0,
            Eprime: 0,
            H: 0,
            Hprime: 0,
            L: 0,
            Lprime: 0,
            I: 0,
            R: 0,
            IX: 0,
            IY: 0,
            SP: 0,
            PC: 0,
            IFF1: false,
            IFF2: false,
            IM: InterruptMode::IM0,
            halted: false,
        };
        Z80 { state, bus }
    }

    /// Get a value from memory
    fn read(&self, addr: u16) -> (Option<Z80Fault>, u8) {
        let (fault, byte_array, _) = unsafe {
            // Safety concern: make sure we are the only thread accessing the
            // bus
            self.bus.borrow().read(addr as usize)
        };
        (fault, byte_array[0])
    }

    /// Write data to memory
    fn write(&mut self, addr: u16, data: u8) -> Option<Z80Fault> {
        let (fault, _) = unsafe {
            // Safety concern: make sure we are the only thread accessing the
            // bus
            Rc::get_mut(&mut self.bus)
                .unwrap()
                .get_mut()
                .write(addr as usize, &[data])
        };
        fault
    }

    /// Dereference an operand. May perform memory accesses.
    fn get_value8(&self, operand: instructions::Operand8) -> u8 {
        match operand {
            instructions::Operand8::Immediate(n) => n,
            instructions::Operand8::Deref(o) => {
                let (_, n) = self.read(self.get_value16(o));
                n
            }
            instructions::Operand8::A => self.state.A,
            instructions::Operand8::B => self.state.B,
            instructions::Operand8::C => self.state.C,
            instructions::Operand8::D => self.state.D,
            instructions::Operand8::E => self.state.E,
            instructions::Operand8::H => self.state.H,
            instructions::Operand8::L => self.state.L,
            instructions::Operand8::I => self.state.I,
            instructions::Operand8::R => self.state.R,
            _ => panic!("Cannot get_value8({:?})", operand),
        }
    }

    /// Dereference an operand. May perform memory accesses.
    fn get_value16(&self, operand: instructions::Operand16) -> u16 {
        match operand {
            instructions::Operand16::Immediate(nn) => nn,
            instructions::Operand16::BC => get_reg16!(self.state, B, C),
            instructions::Operand16::DE => get_reg16!(self.state, D, E),
            instructions::Operand16::HL => get_reg16!(self.state, H, L),
            instructions::Operand16::IX => self.state.IX,
            instructions::Operand16::IY => self.state.IY,
            instructions::Operand16::IXOffset(d) => {
                // "<i8> as u16" performs a sign extension, wrapping_add does the same with unsigned as you would with signed
                self.state.IX.wrapping_add(d as u16)
            }
            instructions::Operand16::IYOffset(d) => self.state.IY.wrapping_add(d as u16),
            instructions::Operand16::SP => self.state.SP,
            instructions::Operand16::AF => get_reg16!(self.state, A, F),
            instructions::Operand16::AFprime => get_reg16!(self.state, Aprime, Fprime),
            instructions::Operand16::DerefSP => {
                let (_, low) = self.read(self.state.SP);
                let (_, high) = self.read(self.state.SP.wrapping_add(1));
                u16::from_be_bytes([high, low])
            }
            instructions::Operand16::Deref(addr) => {
                let (_, low) = self.read(addr);
                let (_, high) = self.read(addr.wrapping_add(1));
                u16::from_be_bytes([high, low])
            }
        }
    }

    /// Dereference an operand. May perform memory accesses.
    fn set_value8(&mut self, operand: instructions::Operand8, value: u8) {
        match operand {
            instructions::Operand8::Deref(o) => {
                let _ = self.write(self.get_value16(o), value);
            }
            instructions::Operand8::A => {
                self.state.A = value;
            }
            instructions::Operand8::B => {
                self.state.B = value;
            }
            instructions::Operand8::C => {
                self.state.C = value;
            }
            instructions::Operand8::D => {
                self.state.D = value;
            }
            instructions::Operand8::E => {
                self.state.E = value;
            }
            instructions::Operand8::H => {
                self.state.H = value;
            }
            instructions::Operand8::L => {
                self.state.L = value;
            }
            instructions::Operand8::I => {
                self.state.I = value;
            }
            instructions::Operand8::R => {
                self.state.R = value;
            }
            _ => panic!("Cannot set_value8({:?}, {})", operand, value),
        }
    }

    /// Dereference an operand. May perform memory accesses.
    fn set_value16(&mut self, operand: instructions::Operand16, value: u16) {
        match operand {
            instructions::Operand16::BC => {
                set_reg16!(self.state, B, C, value);
            }
            instructions::Operand16::DE => {
                set_reg16!(self.state, D, E, value);
            }
            instructions::Operand16::HL => {
                set_reg16!(self.state, H, L, value);
            }
            instructions::Operand16::IX => {
                self.state.IX = value;
            }
            instructions::Operand16::IY => {
                self.state.IY = value;
            }
            instructions::Operand16::SP => {
                self.state.SP = value;
            }
            instructions::Operand16::AF => {
                set_reg16!(self.state, A, F, value);
            }
            instructions::Operand16::AFprime => {
                set_reg16!(self.state, Aprime, Fprime, value);
            }
            instructions::Operand16::DerefSP => {
                let bytes = value.to_be_bytes(); // [high, low]
                self.write(self.state.SP.wrapping_add(1), bytes[0]);
                self.write(self.state.SP, bytes[1]);
            }
            instructions::Operand16::Deref(addr) => {
                let bytes = value.to_be_bytes(); // [high, low]
                self.write(addr.wrapping_add(1), bytes[0]);
                self.write(addr, bytes[1]);
            }
            _ => panic!("Cannot set_value8({:?}, {})", operand, value),
        }
    }

    fn get_flag_mask(flag: Flag) -> u8 {
        match flag {
            Flag::S => 0b10000000,
            Flag::Z => 0b01000000,
            Flag::H => 0b00010000,
            Flag::PV => 0b00000100,
            Flag::N => 0b00000010,
            Flag::C => 0b00000001,
        }
    }

    fn set_flag(&mut self, flag: Flag, value: bool) {
        let mask = Z80::get_flag_mask(flag);
        if value {
            self.state.F |= mask;
        } else {
            self.state.F &= !mask;
        }
    }

    fn test_flag(&self, flag: Flag) -> bool {
        self.state.F & Z80::get_flag_mask(flag) != 0
    }

    fn test_condition(&self, condition: instructions::Condition) -> bool {
        match condition {
            instructions::Condition::NZ => !self.test_flag(Flag::Z),
            instructions::Condition::Z => self.test_flag(Flag::Z),
            instructions::Condition::NC => !self.test_flag(Flag::C),
            instructions::Condition::C => self.test_flag(Flag::C),
            instructions::Condition::PO => !self.test_flag(Flag::PV),
            instructions::Condition::PE => self.test_flag(Flag::PV),
            instructions::Condition::P => !self.test_flag(Flag::S),
            instructions::Condition::M => self.test_flag(Flag::S),
        }
    }

    /// Update S flag according to the given value
    #[allow(non_snake_case)]
    fn check_S(&mut self, value: u8) {
        self.set_flag(Flag::S, value & 0b1000_0000 != 0);
    }

    /// Update Z flag according to the given value
    #[allow(non_snake_case)]
    fn check_Z(&mut self, value: u8) {
        self.set_flag(Flag::Z, value == 0);
    }

    /// Update H flag according to the given value
    #[allow(non_snake_case)]
    fn check_H(&mut self, value: u8) {
        self.set_flag(Flag::H, value & 0b0001_0000 != 0);
    }

    /// Update P flag according to the given value
    #[allow(non_snake_case)]
    fn check_P(&mut self, value: u8) {
        self.set_flag(Flag::PV, value % 2 == 0);
    }

    /// Run an instruction on the CPU.
    pub fn execute(&mut self, instruction: instructions::Instruction) {
        match instruction {
            instructions::Instruction::LD(lhs, rhs) => {
                self.set_value8(lhs, self.get_value8(rhs.clone()));
                match rhs {
                    instructions::Operand8::I | instructions::Operand8::R => {
                        self.check_S(self.state.A);
                        self.check_Z(self.state.A);
                        self.set_flag(Flag::H, false);
                        self.set_flag(Flag::PV, self.state.IFF2);
                        self.set_flag(Flag::N, false);
                    }
                    _ => (),
                }
            }
            instructions::Instruction::LD16(lhs, rhs) => {
                self.set_value16(lhs, self.get_value16(rhs))
            }
            instructions::Instruction::PUSH(op) => {
                let bytes = self.get_value16(op).to_be_bytes(); // [high, low]
                self.write(self.state.SP.wrapping_sub(2), bytes[1]);
                self.write(self.state.SP.wrapping_sub(1), bytes[0]);
                self.state.SP = self.state.SP.wrapping_sub(2);
            }
            instructions::Instruction::POP(op) => {
                let (_, low) = self.read(self.state.SP);
                let (_, high) = self.read(self.state.SP.wrapping_add(1));
                self.set_value16(op, u16::from_be_bytes([high, low]));
                self.state.SP = self.state.SP.wrapping_add(2);
            }
            instructions::Instruction::EX(op1, op2) => {
                let val1 = self.get_value16(op1.clone());
                self.set_value16(op1, self.get_value16(op2.clone()));
                self.set_value16(op2, val1);
            }
            instructions::Instruction::EXX => {
                macro_rules! exchange {
                    ($op1: ident, $op2: ident) => {
                        let tmp = self.state.$op1;
                        self.state.$op1 = self.state.$op2;
                        self.state.$op2 = tmp;
                    };
                }
                exchange!(B, Bprime);
                exchange!(C, Cprime);
                exchange!(D, Dprime);
                exchange!(E, Eprime);
                exchange!(H, Hprime);
                exchange!(L, Lprime);
            }
            instructions::Instruction::LDI => {
                let destination = get_reg16!(self.state, D, E);
                let source = get_reg16!(self.state, H, L);

                let (_, value) = self.read(source);
                self.write(destination, value);

                set_reg16!(self.state, D, E, destination.wrapping_add(1));
                set_reg16!(self.state, H, L, source.wrapping_add(1));
                let byte_counter = get_reg16!(self.state, B, C).wrapping_sub(1);
                set_reg16!(self.state, B, C, byte_counter);

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::LDIR => {
                let mut byte_counter = get_reg16!(self.state, B, C);
                let mut destination = get_reg16!(self.state, D, E);
                let mut source = get_reg16!(self.state, H, L);

                while byte_counter != 0 {
                    let (_, value) = self.read(source);
                    self.write(destination, value);

                    destination = destination.wrapping_add(1);
                    source = source.wrapping_add(1);
                    byte_counter = byte_counter.wrapping_sub(1);
                }

                set_reg16!(self.state, D, E, destination);
                set_reg16!(self.state, H, L, source);
                self.state.B = 0;
                self.state.C = 0;

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::PV, false);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::LDD => {
                let destination = get_reg16!(self.state, D, E);
                let source = get_reg16!(self.state, H, L);

                let (_, value) = self.read(source);
                self.write(destination, value);

                set_reg16!(self.state, D, E, destination.wrapping_sub(1));
                set_reg16!(self.state, H, L, source.wrapping_sub(1));
                let byte_counter = get_reg16!(self.state, B, C).wrapping_sub(1);
                set_reg16!(self.state, B, C, byte_counter);

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::LDDR => {
                let mut byte_counter = get_reg16!(self.state, B, C);
                let mut destination = get_reg16!(self.state, D, E);
                let mut source = get_reg16!(self.state, H, L);

                while byte_counter != 0 {
                    let (_, value) = self.read(source);
                    self.write(destination, value);

                    destination = destination.wrapping_sub(1);
                    source = source.wrapping_sub(1);
                    byte_counter = byte_counter.wrapping_sub(1);
                }

                set_reg16!(self.state, D, E, destination);
                set_reg16!(self.state, H, L, source);
                self.state.B = 0;
                self.state.C = 0;

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::PV, false);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::CPI => {
                let source = get_reg16!(self.state, H, L);

                let (_, value) = self.read(source);
                let result = self.state.A.wrapping_sub(value);

                set_reg16!(self.state, H, L, source.wrapping_add(1));
                let byte_counter = get_reg16!(self.state, B, C).wrapping_sub(1);
                set_reg16!(self.state, B, C, byte_counter);

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);

                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, true);
            }
            instructions::Instruction::CPIR => {
                let mut byte_counter = get_reg16!(self.state, B, C);
                let mut source = get_reg16!(self.state, H, L);

                let mut result = 1;

                while byte_counter != 0 && result != 0 {
                    let (_, value) = self.read(source);

                    result = self.state.A.wrapping_sub(value);

                    source = source.wrapping_add(1);
                    byte_counter = byte_counter.wrapping_sub(1);
                }

                set_reg16!(self.state, H, L, source);
                set_reg16!(self.state, B, C, byte_counter);

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);

                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, true);
            }
            instructions::Instruction::CPD => {
                let source = get_reg16!(self.state, H, L);

                let (_, value) = self.read(source);
                let result = self.state.A.wrapping_sub(value);

                set_reg16!(self.state, H, L, source.wrapping_sub(1));
                let byte_counter = get_reg16!(self.state, B, C).wrapping_sub(1);
                set_reg16!(self.state, B, C, byte_counter);

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);

                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, true);
            }
            instructions::Instruction::CPDR => {
                let mut byte_counter = get_reg16!(self.state, B, C);
                let mut source = get_reg16!(self.state, H, L);

                let mut result = 1;

                while byte_counter != 0 && result != 0 {
                    let (_, value) = self.read(source);

                    result = self.state.A.wrapping_sub(value);

                    source = source.wrapping_sub(1);
                    byte_counter = byte_counter.wrapping_sub(1);
                }

                set_reg16!(self.state, H, L, source);
                set_reg16!(self.state, B, C, byte_counter);

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);

                self.set_flag(Flag::PV, byte_counter != 0);
                self.set_flag(Flag::N, true);
            }
            instructions::Instruction::ADD(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_add(value);
                self.state.A = result;

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::ADC(op) => {
                let value = self.get_value8(op);
                let (mut result, mut overflow) = self.state.A.overflowing_add(value);
                if self.test_flag(Flag::C) {
                    let (result2, overflow2) = result.overflowing_add(1);
                    result = result2;
                    overflow |= overflow2;
                }
                self.state.A = result;

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::SUB(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_sub(value);
                self.state.A = result;

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::SBC(op) => {
                let value = self.get_value8(op);
                let (mut result, mut overflow) = self.state.A.overflowing_sub(value);
                if self.test_flag(Flag::C) {
                    let (result2, overflow2) = result.overflowing_sub(1);
                    result = result2;
                    overflow |= overflow2;
                }
                self.state.A = result;

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::AND(_)
            | instructions::Instruction::OR(_)
            | instructions::Instruction::XOR(_) => {
                match instruction {
                    instructions::Instruction::AND(op) => {
                        self.state.A &= self.get_value8(op);
                    }
                    instructions::Instruction::OR(op) => {
                        self.state.A |= self.get_value8(op);
                    }
                    instructions::Instruction::XOR(op) => {
                        self.state.A ^= self.get_value8(op);
                    }
                    _ => unreachable!(),
                }

                self.check_S(self.state.A);
                self.check_Z(self.state.A);
                self.check_H(self.state.A);
                self.check_P(self.state.A);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, false);
            }
            instructions::Instruction::CP(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_sub(value);

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::INC(op) => {
                self.set_value8(op.clone(), self.get_value8(op).wrapping_add(1));
            }
            instructions::Instruction::DEC(op) => {
                self.set_value8(op.clone(), self.get_value8(op).wrapping_sub(1));
            }
            instructions::Instruction::DAA => {
                let mut overflow = false;
                if self.test_flag(Flag::H) || self.state.A & 0b1111 >= 10 {
                    // There has been a BCD overflow
                    if self.test_flag(Flag::N) {
                        // Previous operation was an ADD. Overflow means the
                        // carry happened at 16 instead of 10, compensate by
                        // adding another 6
                        let (result, o) = self.state.A.overflowing_add(6);
                        self.state.A = result;
                        overflow = o;
                    } else {
                        // Previous operation was a SUB... and I'm getting lost
                        // in the algorithms
                        unimplemented!("DAA not implemented for SUB");
                    }
                }

                self.check_S(self.state.A);
                self.check_Z(self.state.A);
                self.check_H(self.state.A);
                self.check_P(self.state.A);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::CPL => {
                self.state.A = !self.state.A;

                self.set_flag(Flag::H, true);
                self.set_flag(Flag::N, true);
            }
            instructions::Instruction::NEG => {
                let (result, overflow) = self.state.A.overflowing_neg();
                self.state.A = result;

                self.check_S(result);
                self.check_Z(result);
                self.check_H(result);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::CCF => {
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, !self.test_flag(Flag::C));
            }
            instructions::Instruction::SCF => {
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, true);
            }
            instructions::Instruction::NOP => (),
            instructions::Instruction::HALT => {
                self.state.halted = true;
            }
            instructions::Instruction::DI => {
                self.state.IFF1 = false;
                self.state.IFF2 = false;
            }
            instructions::Instruction::EI => {
                self.state.IFF1 = true;
                self.state.IFF2 = true;
            }
            instructions::Instruction::IM0 => {
                self.state.IM = InterruptMode::IM0;
            }
            instructions::Instruction::IM1 => {
                self.state.IM = InterruptMode::IM1;
            }
            instructions::Instruction::IM2 => {
                self.state.IM = InterruptMode::IM2;
            }
            instructions::Instruction::ADD16(lhs, rhs) => {
                let (result, overflow) = self
                    .get_value16(lhs.clone())
                    .overflowing_add(self.get_value16(rhs));
                self.set_value16(lhs, result);

                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::ADC16(op) => {
                let lhs = get_reg16!(self.state, H, L);
                let rhs = self.get_value16(op);
                let (mut result, mut overflow) = lhs.overflowing_add(rhs);
                if self.test_flag(Flag::C) {
                    let (result2, overflow2) = result.overflowing_add(1);
                    result = result2;
                    overflow |= overflow2;
                }
                set_reg16!(self.state, H, L, result);

                self.set_flag(Flag::S, result & 0b1000_0000_000_0000 != 0);
                self.set_flag(Flag::Z, result == 0);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::SBC16(op) => {
                let lhs = get_reg16!(self.state, H, L);
                let rhs = self.get_value16(op);
                let (mut result, mut overflow) = lhs.overflowing_sub(rhs);
                if self.test_flag(Flag::C) {
                    let (result2, overflow2) = result.overflowing_sub(1);
                    result = result2;
                    overflow |= overflow2;
                }
                set_reg16!(self.state, H, L, result);

                self.set_flag(Flag::S, result & 0b1000_0000_000_0000 != 0);
                self.set_flag(Flag::Z, result == 0);
                self.set_flag(Flag::PV, overflow);
                self.set_flag(Flag::N, true);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::INC16(op) => {
                self.set_value16(op.clone(), self.get_value16(op).wrapping_add(1));
            }
            instructions::Instruction::DEC16(op) => {
                self.set_value16(op.clone(), self.get_value16(op).wrapping_sub(1));
            }
            instructions::Instruction::RLCA => {
                let out_bit = self.state.A & 0b1000_0000 != 0;

                self.state.A = self.state.A.rotate_left(1);

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RLA => {
                let out_bit = self.state.A & 0b1000_0000 != 0;

                self.state.A = self.state.A.wrapping_shl(1);
                if self.test_flag(Flag::C) {
                    self.state.A |= 0b0000_0001;
                }

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRCA => {
                let out_bit = self.state.A & 0b0000_0001 != 0;

                self.state.A = self.state.A.rotate_right(1);

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRA => {
                let out_bit = self.state.A & 0b0000_0001 != 0;

                self.state.A = self.state.A.wrapping_shr(1);
                if self.test_flag(Flag::C) {
                    self.state.A |= 0b1000_0000;
                }

                self.set_flag(Flag::H, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RLC(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b1000_0000 != 0;

                value = value.rotate_left(1);
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RL(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b1000_0000 != 0;

                value = value.wrapping_shl(1);
                if self.test_flag(Flag::C) {
                    value |= 0b0000_0001;
                }
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRC(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;

                value = value.rotate_right(1);
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RR(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;

                value = value.wrapping_shr(1);
                if self.test_flag(Flag::C) {
                    value |= 0b1000_0000;
                }
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::SLA(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b1000_0000 != 0;

                value = value.wrapping_shl(1);
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::SRA(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;
                let top_bit = value & 0b1000_0000 != 0;

                value = value.wrapping_shr(1);
                if top_bit {
                    value |= 0b1000_0000;
                }
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::SRL(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;

                value = value.wrapping_shr(1);
                self.set_value8(op, value);

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RLD => {
                let al = self.state.A & 0b0000_1111; // low nibble for A
                let (_, value) = self.read(get_reg16!(self.state, H, L));
                let vh = value.wrapping_shr(4); // high nibble for (HL)
                let vl = value & 0b0000_1111; // low nibble for (HL)

                let new_value = vl.wrapping_shl(4) | al;
                let new_a = (self.state.A & 0b1111_0000) | vh;

                self.write(get_reg16!(self.state, H, L), new_value);
                self.state.A = new_a;

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::RRD => {
                let al = self.state.A & 0b0000_1111; // low nibble for A
                let (_, value) = self.read(get_reg16!(self.state, H, L));
                let vh = value.wrapping_shr(4); // high nibble for (HL)
                let vl = value & 0b0000_1111; // low nibble for (HL)

                let new_value = al.wrapping_shl(4) | vh;
                let new_a = (self.state.A & 0b1111_0000) | vl;

                self.write(get_reg16!(self.state, H, L), new_value);
                self.state.A = new_a;

                self.check_S(value);
                self.check_Z(value);
                self.set_flag(Flag::H, false);
                self.check_P(value);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::BIT(b, op) => {
                let value = self.get_value8(op);

                self.set_flag(Flag::Z, value & 1u8.wrapping_shl(b as u32) != 0);
                self.set_flag(Flag::H, true);
                self.set_flag(Flag::N, false);
            }
            instructions::Instruction::SET(b, op) => {
                let value = self.get_value8(op.clone());
                self.set_value8(op, value | 1u8.wrapping_shl(b as u32));
            }
            instructions::Instruction::RES(b, op) => {
                let value = self.get_value8(op.clone());
                self.set_value8(op, value & (!1u8.wrapping_shl(b as u32)));
            }
            instructions::Instruction::JP(op) => {
                self.state.PC = self.get_value16(op);
            }
            instructions::Instruction::JPConditional(condition, addr) => {
                if self.test_condition(condition) {
                    self.state.PC = addr;
                }
            }
            instructions::Instruction::JR(offset) => {
                // "<i8> as u16" performs a sign extension, wrapping_add does
                // the same with unsigned as you would with signed
                self.state.PC = self.state.PC.wrapping_add(offset as u16);
            }
            instructions::Instruction::JRConditional(condition, offset) => {
                if self.test_condition(condition) {
                    self.state.PC = self.state.PC.wrapping_add(offset as u16);
                }
            }
            instructions::Instruction::DJNZ(offset) => {
                self.state.B = self.state.B.wrapping_sub(1);
                if self.state.B != 0 {
                    self.state.PC = self.state.PC.wrapping_add(offset as u16);
                }
            }
            instructions::Instruction::CALL(addr) => {
                let bytes = self.state.PC.to_be_bytes(); // [high, low]
                self.write(self.state.SP.wrapping_sub(2), bytes[1]);
                self.write(self.state.SP.wrapping_sub(1), bytes[0]);
                self.state.SP = self.state.SP.wrapping_sub(2);
                self.state.PC = addr;
            }
            instructions::Instruction::CALLConditional(condition, addr) => {
                if self.test_condition(condition) {
                    let bytes = self.state.PC.to_be_bytes(); // [high, low]
                    self.write(self.state.SP.wrapping_sub(2), bytes[1]);
                    self.write(self.state.SP.wrapping_sub(1), bytes[0]);
                    self.state.SP = self.state.SP.wrapping_sub(2);
                    self.state.PC = addr;
                }
            }
            instructions::Instruction::RET => {
                let (_, low) = self.read(self.state.SP);
                let (_, high) = self.read(self.state.SP.wrapping_add(1));
                self.state.PC = u16::from_be_bytes([high, low]);
                self.state.SP = self.state.SP.wrapping_add(2);
            }
            instructions::Instruction::RETConditional(condition) => {
                if self.test_condition(condition) {
                    let (_, low) = self.read(self.state.SP);
                    let (_, high) = self.read(self.state.SP.wrapping_add(1));
                    self.state.PC = u16::from_be_bytes([high, low]);
                    self.state.SP = self.state.SP.wrapping_add(2);
                }
            }
            instructions::Instruction::RETI => {
                panic!("Interrupts not implemented for Z80")
            }
            instructions::Instruction::RETN => {
                self.state.IFF1 = self.state.IFF2;
                let (_, low) = self.read(self.state.SP);
                let (_, high) = self.read(self.state.SP.wrapping_add(1));
                self.state.PC = u16::from_be_bytes([high, low]);
                self.state.SP = self.state.SP.wrapping_add(2);
            }
            instructions::Instruction::RST(addr) => {
                let bytes = self.state.PC.to_be_bytes(); // [high, low]
                self.write(self.state.SP.wrapping_sub(2), bytes[1]);
                self.write(self.state.SP.wrapping_sub(1), bytes[0]);
                self.state.SP = self.state.SP.wrapping_sub(2);
                self.state.PC = addr as u16; // zero-extension
            }
            instructions::Instruction::IN(_, _)
            | instructions::Instruction::INI
            | instructions::Instruction::INIR
            | instructions::Instruction::IND
            | instructions::Instruction::INDR
            | instructions::Instruction::OUT(_, _)
            | instructions::Instruction::OUTI
            | instructions::Instruction::OTIR
            | instructions::Instruction::OUTD
            | instructions::Instruction::OTDR => {
                panic!("I/O not implemented for Z80")
            }
            _ => {
                panic!("Cannot execute {:?}", instruction)
            }
        }
    }

    /// Send a Non-Maskable Interrupt to the processor.
    pub fn nmi(&mut self) {
        self.state.IFF1 = false;
        self.state.halted = false;
        // Essentially a CALL 0x0066
        let bytes = self.state.PC.to_be_bytes(); // [high, low]
        self.write(self.state.SP.wrapping_sub(2), bytes[1]);
        self.write(self.state.SP.wrapping_sub(1), bytes[0]);
        self.state.SP = self.state.SP.wrapping_sub(2);
        self.state.PC = NMI_RESTART_ADDR;
    }

    /// Send an interrupt to the processor.
    ///
    /// Will always panic as implementation details, especially considering the
    /// different interrupt modes, have not been figured out.
    ///
    /// # Panics
    ///
    /// Always.
    pub fn int(&mut self) {
        panic!("Interrupts not implemented for Z80")
    }
}

impl Processor for Z80 {
    fn tick(&mut self) {
        if self.state.halted {
            return;
        }

        let (_, byte1) = self.read(self.state.PC);
        self.state.PC = self.state.PC.wrapping_add(1);

        if let Some(instruction) = instructions::decode1byte(byte1) {
            self.execute(instruction);
            return;
        }

        let (_, byte2) = self.read(self.state.PC);
        self.state.PC = self.state.PC.wrapping_add(1);

        if let Some(instruction) = instructions::decode2bytes(byte1, byte2) {
            self.execute(instruction);
            return;
        }

        let (_, byte3) = self.read(self.state.PC);
        self.state.PC = self.state.PC.wrapping_add(1);

        if let Some(instruction) = instructions::decode3bytes(byte1, byte2, byte3) {
            self.execute(instruction);
            return;
        }

        let (_, byte4) = self.read(self.state.PC);
        self.state.PC = self.state.PC.wrapping_add(1);

        if let Some(instruction) = instructions::decode4bytes(byte1, byte2, byte3, byte4) {
            self.execute(instruction);
            return;
        }

        // decode4bytes() always returns Some
        unreachable!()
    }
}
