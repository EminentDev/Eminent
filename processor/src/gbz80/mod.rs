//! Implementation of the GbZ80 processor.

#![allow(clippy::unusual_byte_groupings)]

use std::cell::RefCell;
use std::rc::Rc;

use crate::Processor;
use common::Bus;

pub mod instructions;

/// The address of the IE (Interrupt Enable) I/O register.
const IE_ADDR: u16 = 0xFFFF;

/// The address of the IF (Interrupt Flag) I/O register.
const IF_ADDR: u16 = 0xFF0F;

/// The address for the VBLANK interrupt handler.
const VBLANK_IH_ADDR: u16 = 0x0040;

/// The address for the LCDSTATUS interrupt handler.
const LCDSTATUS_IH_ADDR: u16 = 0x0048;

/// The address for the TIMER interrupt handler.
const TIMER_IH_ADDR: u16 = 0x0050;

/// The address for the SERIAL interrupt handler.
const SERIAL_IH_ADDR: u16 = 0x0058;

/// The address for the JOYPAD interrupt handler.
const JOYPAD_IH_ADDR: u16 = 0x0060;

/// A mask for all the interrupt bits.
const INTERRUPT_MASK: u8 = 0b11111;

/// A description of an error in the memory bus.
///
/// Unused.
pub struct GbZ80Fault {}

enum Flag {
    Z,
    N,
    H,
    C,
}

/// The interrupts this CPU can handle.
#[derive(Clone, PartialEq)]
pub enum Interrupt {
    VBLANK,
    LCDSTATUS,
    TIMER,
    SERIAL,
    JOYPAD,
}

/// The execution mode of the CPU.
#[derive(PartialEq)]
pub enum Mode {
    /// Normal CPU operation
    ON,
    /// No CPU operation until interrupt reception
    HALTED,
    /// No CPU operation until joypad input
    STOPPED,
}

/// The state of the CPU. This structure holds all the necessary information
/// for the execution of the CPU.
#[allow(non_snake_case)]
pub struct State {
    pub A: u8,
    pub F: u8,
    pub B: u8,
    pub C: u8,
    pub D: u8,
    pub E: u8,
    pub H: u8,
    pub L: u8,
    pub SP: u16,
    pub PC: u16,
    pub IME: bool,
    pub mode: Mode,
    /// Part of the State because the Processor needs to access it without the Bus.
    pub IE: u8,
    /// Part of the State because the Processor needs to access it without the Bus.
    pub IF: u8,
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
pub struct GbZ80 {
    state: State,
    bus: Rc<RefCell<Bus<GbZ80Fault, 1>>>,
}

impl GbZ80 {
    /// Instanciate a new GbZ80 Processor.
    pub fn new(bus: Rc<RefCell<Bus<GbZ80Fault, 1>>>) -> GbZ80 {
        let state = State {
            A: 0,
            F: 0,
            B: 0,
            C: 0,
            D: 0,
            E: 0,
            H: 0,
            L: 0,
            SP: 0,
            PC: 0,
            IME: false,
            mode: Mode::ON,
            IE: 0,
            IF: 0,
        };
        GbZ80 { state, bus }
    }

    /// Get a value from memory
    fn read(&self, addr: u16) -> (Option<GbZ80Fault>, u8) {
        match addr {
            IE_ADDR => (None, self.state.IE),
            IF_ADDR => (None, self.state.IF),
            _ => {
                let (fault, byte_array, _) = unsafe {
                    // Safety concern: make sure we are the only thread accessing the bus
                    self.bus.borrow().read(addr as usize)
                };
                (fault, byte_array[0])
            }
        }
    }

    /// Write data to memory
    fn write(&mut self, addr: u16, data: u8) -> Option<GbZ80Fault> {
        match addr {
            IE_ADDR => {
                self.state.IE = data;
                None
            }
            IF_ADDR => {
                self.state.IF = data;
                None
            }
            _ => {
                let (fault, _) = unsafe {
                    // Safety concern: make sure we are the only thread accessing the bus
                    Rc::get_mut(&mut self.bus)
                        .unwrap()
                        .get_mut()
                        .write(addr as usize, &[data])
                };
                fault
            }
        }
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
        }
    }

    /// Dereference an operand. May perform memory accesses.
    fn get_value16(&self, operand: instructions::Operand16) -> u16 {
        match operand {
            instructions::Operand16::Immediate(nn) => nn,
            instructions::Operand16::BC => get_reg16!(self.state, B, C),
            instructions::Operand16::DE => get_reg16!(self.state, D, E),
            instructions::Operand16::HL => get_reg16!(self.state, H, L),
            instructions::Operand16::SP => self.state.SP,
            instructions::Operand16::AF => get_reg16!(self.state, A, F),
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
            instructions::Operand16::FF00Offset(offset) => {
                0xFF00 + (offset as u16) // zero-extension
            }
            instructions::Operand16::FF00C => {
                0xFF00 + (self.state.C as u16) // zero-extension
            }
            instructions::Operand16::SPOffset(offset) => {
                // sign-extension
                self.state.SP.wrapping_add(offset as u16)
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
            instructions::Operand16::SP => {
                self.state.SP = value;
            }
            instructions::Operand16::AF => {
                set_reg16!(self.state, A, F, value);
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
            Flag::Z => 0b10000000,
            Flag::N => 0b01000000,
            Flag::H => 0b00100000,
            Flag::C => 0b00010000,
        }
    }

    fn set_flag(&mut self, flag: Flag, value: bool) {
        let mask = GbZ80::get_flag_mask(flag);
        if value {
            self.state.F |= mask;
        } else {
            self.state.F &= !mask;
        }
    }

    fn test_flag(&self, flag: Flag) -> bool {
        self.state.F & GbZ80::get_flag_mask(flag) != 0
    }

    fn test_condition(&self, condition: instructions::Condition) -> bool {
        match condition {
            instructions::Condition::NZ => !self.test_flag(Flag::Z),
            instructions::Condition::Z => self.test_flag(Flag::Z),
            instructions::Condition::NC => !self.test_flag(Flag::C),
            instructions::Condition::C => self.test_flag(Flag::C),
        }
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

    /// Push a 16-bit value onto the stack.
    fn push(&mut self, value: u16) {
        let bytes = value.to_be_bytes(); // [high, low]
        self.write(self.state.SP.wrapping_sub(2), bytes[1]);
        self.write(self.state.SP.wrapping_sub(1), bytes[0]);
        self.state.SP = self.state.SP.wrapping_sub(2);
    }

    /// Pop a 16-bit value from the stack.
    fn pop(&mut self) -> u16 {
        let (_, low) = self.read(self.state.SP);
        let (_, high) = self.read(self.state.SP.wrapping_add(1));
        self.state.SP = self.state.SP.wrapping_add(2);
        u16::from_be_bytes([high, low])
    }

    fn get_interrupt_mask(interrupt: Interrupt) -> u8 {
        match interrupt {
            Interrupt::VBLANK => 1 << 0,
            Interrupt::LCDSTATUS => 1 << 1,
            Interrupt::TIMER => 1 << 2,
            Interrupt::SERIAL => 1 << 3,
            Interrupt::JOYPAD => 1 << 4,
        }
    }

    /// Get the address for the interrupt handler of a specific interrupt.
    fn get_interrupt_handler(interrupt: Interrupt) -> u16 {
        match interrupt {
            Interrupt::VBLANK => VBLANK_IH_ADDR,
            Interrupt::LCDSTATUS => LCDSTATUS_IH_ADDR,
            Interrupt::TIMER => TIMER_IH_ADDR,
            Interrupt::SERIAL => SERIAL_IH_ADDR,
            Interrupt::JOYPAD => JOYPAD_IH_ADDR,
        }
    }

    /// Retrieve the highest priority interrupt on.
    fn get_interrupt(&self) -> Option<Interrupt> {
        let flags = self.state.IF & self.state.IE & INTERRUPT_MASK;
        macro_rules! match_flag {
            ($interrupt: ident) => {
                if flags & GbZ80::get_interrupt_mask(Interrupt::$interrupt) != 0 {
                    return Some(Interrupt::$interrupt);
                }
            };
        }
        match_flag!(VBLANK);
        match_flag!(LCDSTATUS);
        match_flag!(TIMER);
        match_flag!(SERIAL);
        match_flag!(JOYPAD);
        None
    }

    /// Run an instruction on the CPU.
    pub fn execute(&mut self, instruction: instructions::Instruction) {
        match instruction {
            instructions::Instruction::LD(lhs, rhs) => {
                self.set_value8(lhs, self.get_value8(rhs));
            }
            instructions::Instruction::LD16(
                instructions::Operand16::HL,
                instructions::Operand16::SPOffset(offset),
            ) => {
                // sign-extension
                let (result, overflow) = self.state.SP.overflowing_add(offset as u16);
                set_reg16!(self.state, H, L, result);

                self.set_flag(Flag::Z, false);
                self.set_flag(Flag::N, false);
                self.check_H(result as u8);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::LD16(lhs, rhs) => {
                self.set_value16(lhs, self.get_value16(rhs))
            }
            instructions::Instruction::LDI(lhs, rhs) => {
                self.set_value8(lhs, self.get_value8(rhs));
                set_reg16!(
                    self.state,
                    H,
                    L,
                    get_reg16!(self.state, H, L).wrapping_add(1)
                );
            }
            instructions::Instruction::LDD(lhs, rhs) => {
                self.set_value8(lhs, self.get_value8(rhs));
                set_reg16!(
                    self.state,
                    H,
                    L,
                    get_reg16!(self.state, H, L).wrapping_sub(1)
                );
            }
            instructions::Instruction::PUSH(op) => {
                self.push(self.get_value16(op));
            }
            instructions::Instruction::POP(op) => {
                let value = self.pop();
                self.set_value16(op, value);
            }
            instructions::Instruction::ADD(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_add(value);
                self.state.A = result;

                self.check_Z(result);
                self.set_flag(Flag::N, false);
                self.check_H(result);
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

                self.check_Z(result);
                self.set_flag(Flag::N, false);
                self.check_H(result);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::SUB(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_sub(value);
                self.state.A = result;

                self.check_Z(result);
                self.set_flag(Flag::N, true);
                self.check_H(result);
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

                self.check_Z(result);
                self.set_flag(Flag::N, true);
                self.check_H(result);
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

                self.check_Z(self.state.A);
                self.set_flag(Flag::N, false);
                self.check_H(self.state.A);
                self.set_flag(Flag::C, false);
            }
            instructions::Instruction::CP(op) => {
                let value = self.get_value8(op);
                let (result, overflow) = self.state.A.overflowing_sub(value);

                self.check_Z(result);
                self.set_flag(Flag::N, true);
                self.check_H(result);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::INC(op) => {
                let result = self.get_value8(op.clone()).wrapping_add(1);
                self.set_value8(op, result);

                self.check_Z(result);
                self.set_flag(Flag::N, false);
                self.check_H(result);
            }
            instructions::Instruction::DEC(op) => {
                let result = self.get_value8(op.clone()).wrapping_sub(1);
                self.set_value8(op, result);

                self.check_Z(result);
                self.set_flag(Flag::N, true);
                self.check_H(result);
            }
            instructions::Instruction::ADD16(lhs, rhs) => {
                let (result, overflow) = self
                    .get_value16(lhs.clone())
                    .overflowing_add(self.get_value16(rhs));
                self.set_value16(lhs, result);

                self.set_flag(Flag::Z, false);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, result & 0b1_0000_0000_0000 != 0);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::INC16(op) => {
                self.set_value16(op.clone(), self.get_value16(op).wrapping_add(1));
            }
            instructions::Instruction::DEC16(op) => {
                self.set_value16(op.clone(), self.get_value16(op).wrapping_sub(1));
            }
            instructions::Instruction::SWAP(op) => {
                let value = self.get_value8(op.clone());
                let high_nibble = value >> 4;
                let low_nibble = value & 0b1111;
                let result = (low_nibble << 4) | high_nibble;
                self.set_value8(op, result);

                self.check_Z(result);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, false);
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

                self.check_Z(self.state.A);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, overflow);
            }
            instructions::Instruction::CPL => {
                self.state.A = !self.state.A;

                self.set_flag(Flag::N, true);
                self.set_flag(Flag::H, true);
            }
            instructions::Instruction::CCF => {
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, !self.test_flag(Flag::C));
            }
            instructions::Instruction::SCF => {
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, true);
            }
            instructions::Instruction::NOP => (),
            instructions::Instruction::HALT => {
                self.state.mode = Mode::HALTED;
            }
            instructions::Instruction::STOP => {
                self.state.mode = Mode::STOPPED;
            }
            instructions::Instruction::DI => {
                self.state.IME = false;
            }
            instructions::Instruction::EI => {
                self.state.IME = true;
            }
            instructions::Instruction::RLCA => {
                let out_bit = self.state.A & 0b1000_0000 != 0;

                self.state.A = self.state.A.rotate_left(1);

                self.check_Z(self.state.A);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RLA => {
                let out_bit = self.state.A & 0b1000_0000 != 0;

                self.state.A = self.state.A.wrapping_shl(1);
                if self.test_flag(Flag::C) {
                    self.state.A |= 0b0000_0001;
                }

                self.check_Z(self.state.A);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRCA => {
                let out_bit = self.state.A & 0b0000_0001 != 0;

                self.state.A = self.state.A.rotate_right(1);

                self.check_Z(self.state.A);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRA => {
                let out_bit = self.state.A & 0b0000_0001 != 0;

                self.state.A = self.state.A.wrapping_shr(1);
                if self.test_flag(Flag::C) {
                    self.state.A |= 0b1000_0000;
                }

                self.check_Z(self.state.A);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RLC(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b1000_0000 != 0;

                value = value.rotate_left(1);
                self.set_value8(op, value);

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
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

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::RRC(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;

                value = value.rotate_right(1);
                self.set_value8(op, value);

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
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

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::SLA(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b1000_0000 != 0;

                value = value.wrapping_shl(1);
                self.set_value8(op, value);

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
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

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::SRL(op) => {
                let mut value = self.get_value8(op.clone());
                let out_bit = value & 0b0000_0001 != 0;

                value = value.wrapping_shr(1);
                self.set_value8(op, value);

                self.check_Z(value);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, false);
                self.set_flag(Flag::C, out_bit);
            }
            instructions::Instruction::BIT(b, op) => {
                let value = self.get_value8(op);

                self.set_flag(Flag::Z, value & 1u8.wrapping_shl(b as u32) != 0);
                self.set_flag(Flag::N, false);
                self.set_flag(Flag::H, true);
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
                // sign-extension
                self.state.PC = self.state.PC.wrapping_add(offset as u16);
            }
            instructions::Instruction::JRConditional(condition, offset) => {
                if self.test_condition(condition) {
                    self.state.PC = self.state.PC.wrapping_add(offset as u16);
                }
            }
            instructions::Instruction::CALL(addr) => {
                self.push(self.state.PC);
                self.state.PC = addr;
            }
            instructions::Instruction::CALLConditional(condition, addr) => {
                if self.test_condition(condition) {
                    self.push(self.state.PC);
                    self.state.PC = addr;
                }
            }
            instructions::Instruction::RST(addr) => {
                self.push(self.state.PC);
                self.state.PC = addr as u16; // zero-extension
            }
            instructions::Instruction::RET => {
                self.state.PC = self.pop();
            }
            instructions::Instruction::RETConditional(condition) => {
                if self.test_condition(condition) {
                    self.state.PC = self.pop();
                }
            }
            instructions::Instruction::RETI => {
                self.state.PC = self.pop();
                self.state.IME = true;
            }
            _ => {
                panic!("Cannot execute {:?}", instruction)
            }
        }
    }

    /// Send an interrupt to the CPU.
    ///
    /// Interrupt is only registered, it will be handled:
    /// * on the next `tick()`
    /// * if interrupts are enabled
    /// * if this specific interrupt is enabled via the `IE` register
    /// * if it is the highest priority interrupt registered
    pub fn interrupt(&mut self, interrupt: Interrupt) {
        self.state.IF |= GbZ80::get_interrupt_mask(interrupt);
    }
}

impl Processor for GbZ80 {
    fn tick(&mut self) {
        if let Some(interrupt) = self.get_interrupt() {
            if self.state.mode == Mode::HALTED
                || (self.state.mode == Mode::STOPPED && interrupt == Interrupt::JOYPAD)
            {
                self.state.mode = Mode::ON;
            }
            if self.state.IME {
                self.state.IME = false;
                self.state.IF &= !GbZ80::get_interrupt_mask(interrupt.clone());
                self.push(self.state.PC);
                self.state.PC = GbZ80::get_interrupt_handler(interrupt);
            }
        }

        if self.state.mode == Mode::ON {
            let (_, byte1) = self.read(self.state.PC);
            self.state.PC = self.state.PC.wrapping_add(1);

            let instruction = match instructions::decode1byte(byte1) {
                Ok(instruction) => instruction,
                Err(2) => {
                    let (_, byte2) = self.read(self.state.PC);
                    self.state.PC = self.state.PC.wrapping_add(1);
                    instructions::decode2bytes(byte1, byte2)
                }
                Err(3) => {
                    let (_, byte2) = self.read(self.state.PC);
                    self.state.PC = self.state.PC.wrapping_add(1);
                    let (_, byte3) = self.read(self.state.PC);
                    self.state.PC = self.state.PC.wrapping_add(1);
                    instructions::decode3bytes(byte1, byte2, byte3)
                }
                x => panic!("Unexpected decoded value: {:?}", x),
            };

            self.execute(instruction);
        }
    }
}
