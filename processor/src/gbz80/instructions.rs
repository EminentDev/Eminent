//! A model for the instructions of the Nintendo Game Boy CPU.
//!
//! Note that not all instructions allowed to be built with this model are
//! legal, i.e. have a binary representation, and one that would be understood
//! by an actual GBZ80 CPU. However, the opposite is true: all legal
//! instructions can be represented with this model.

/// An 8 bits operand
#[derive(Debug, PartialEq)]
pub enum Operand8 {
    Immediate(u8),
    Deref(Operand16),
    A,
    B,
    C,
    D,
    E,
    H,
    L,
}

/// A 16 bits operand
#[derive(Clone, Debug, PartialEq)]
pub enum Operand16 {
    Immediate(u16),
    BC,
    DE,
    HL,
    SP,
    AF,
    DerefSP,
    Deref(u16),
    FF00Offset(u8),
    FF00C,
    SPOffset(i8),
}

/// A condition for conditional jumps and calls
#[derive(Debug, PartialEq)]
pub enum Condition {
    NZ,
    Z,
    NC,
    C,
}

/// All the instructions of the GBZ80
#[derive(Debug, PartialEq)]
pub enum Instruction {
    NOP,
    LD(Operand8, Operand8),
    LD16(Operand16, Operand16),
    PUSH(Operand16),
    POP(Operand16),
    LDI(Operand8, Operand8),
    LDD(Operand8, Operand8),
    ADD(Operand8),
    ADD16(Operand16, Operand16),
    ADC(Operand8),
    SUB(Operand8),
    SBC(Operand8),
    AND(Operand8),
    OR(Operand8),
    XOR(Operand8),
    CP(Operand8),
    INC(Operand8),
    DEC(Operand8),
    INC16(Operand16),
    DEC16(Operand16),
    DAA,
    CPL,
    CCF,
    SCF,
    HALT,
    DI,
    EI,
    RLCA,
    RLA,
    RRCA,
    RRA,
    RLC(Operand8),
    RL(Operand8),
    RRC(Operand8),
    RR(Operand8),
    SLA(Operand8),
    SRA(Operand8),
    SRL(Operand8),
    BIT(u8, Operand8),
    SET(u8, Operand8),
    RES(u8, Operand8),
    JP(Operand16),
    JPConditional(Condition, u16),
    /// The stored value is `e - 2`, not `e`
    JR(i8),
    /// The stored value is `e - 2`, not `e`
    JRConditional(Condition, i8),
    CALL(u16),
    CALLConditional(Condition, u16),
    RET,
    RETConditional(Condition),
    RETI,
    /// The stored value is `p`, not `t`
    RST(u8),
    STOP,
    SWAP(Operand8),
    /// This variant is specific to this implementation of the decoding process
    IllegalOpcode,
    /// This variant is specific to this implementation of the decoding process
    UnfinishedOpcode,
}

/// Mapping of the opcode tribit to an 8 bits register
///
/// This function truncates its argument to the three low bits before mapping.
///
/// # Examples
/// ```rust
/// use processor::gbz80::instructions;
/// assert_eq!(instructions::map_register8(0b111), instructions::Operand8::A);
/// assert_eq!(instructions::map_register8(0b1000), instructions::Operand8::B);
/// ```
pub fn map_register8(tribit: u8) -> Operand8 {
    match tribit & 0b111 {
        0b000 => Operand8::B,
        0b001 => Operand8::C,
        0b010 => Operand8::D,
        0b011 => Operand8::E,
        0b100 => Operand8::H,
        0b101 => Operand8::L,
        0b110 => Operand8::Deref(Operand16::HL),
        0b111 => Operand8::A,
        _ => unreachable!(),
    }
}

/// Mapping of the opcode dibit to a 16 bits register
///
/// This function truncates its argument to the two low bits before mapping.
///
/// # Examples
/// ```rust
/// use processor::gbz80::instructions;
/// assert_eq!(instructions::map_register16(0b11),
///            instructions::Operand16::SP);
/// assert_eq!(instructions::map_register16(0b110),
///            instructions::Operand16::HL);
/// ```
pub fn map_register16(dibit: u8) -> Operand16 {
    match dibit & 0b11 {
        0b00 => Operand16::BC,
        0b01 => Operand16::DE,
        0b10 => Operand16::HL,
        0b11 => Operand16::SP,
        _ => unreachable!(),
    }
}

/// Mapping of the opcode dibit to a condition
///
/// This function truncates its argument to the two low bits before mapping.
///
/// # Examples
/// ```rust
/// use processor::gbz80::instructions;
/// assert_eq!(instructions::map_condition(0b11), instructions::Condition::C);
/// assert_eq!(instructions::map_condition(0b101), instructions::Condition::Z);
/// ```
pub fn map_condition(dibit: u8) -> Condition {
    match dibit & 0b11 {
        0b00 => Condition::NZ,
        0b01 => Condition::Z,
        0b10 => Condition::NC,
        0b11 => Condition::C,
        _ => unreachable!(),
    }
}

/// Try to decode an instruction from a single byte.
///
/// If this is a one byte instructions, return `Ok`, otherwise return an `Err`
/// with the number of bytes needed (2 or 3).
pub fn decode1byte(opcode: u8) -> Result<Instruction, u8> {
    match opcode {
        0x00 => Ok(Instruction::NOP),
        0x02 => Ok(Instruction::LD(Operand8::Deref(Operand16::BC), Operand8::A)),
        0x07 => Ok(Instruction::RLCA),
        0x08 => Err(3), // LD (nn), SP
        0x0A => Ok(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::BC))),
        0x0F => Ok(Instruction::RRCA),
        0x10 => Ok(Instruction::STOP),
        0x12 => Ok(Instruction::LD(Operand8::Deref(Operand16::DE), Operand8::A)),
        0x17 => Ok(Instruction::RLA),
        0x18 => Err(2), // JR e
        0x1A => Ok(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::DE))),
        0x1F => Ok(Instruction::RRA),
        0x22 => Ok(Instruction::LDI(
            Operand8::Deref(Operand16::HL),
            Operand8::A,
        )),
        0x27 => Ok(Instruction::DAA),
        0x2A => Ok(Instruction::LDI(
            Operand8::A,
            Operand8::Deref(Operand16::HL),
        )),
        0x2F => Ok(Instruction::CPL),
        0x32 => Ok(Instruction::LDD(
            Operand8::Deref(Operand16::HL),
            Operand8::A,
        )),
        0x37 => Ok(Instruction::SCF),
        0x3A => Ok(Instruction::LDD(
            Operand8::A,
            Operand8::Deref(Operand16::HL),
        )),
        0x3F => Ok(Instruction::CCF),
        0x76 => Ok(Instruction::HALT),
        0xC3 => Err(3), // JP nn
        0xC9 => Ok(Instruction::RET),
        0xCD => Err(3), // CALL nn
        0xD9 => Ok(Instruction::RETI),
        0xE0 => Err(2), // LD (FF00 + e), A
        0xE2 => Ok(Instruction::LD(
            Operand8::Deref(Operand16::FF00C),
            Operand8::A,
        )),
        0xE8 => Err(2), // ADD SP, d
        0xE9 => Ok(Instruction::JP(Operand16::HL)),
        0xEA => Err(3), // LD (nn), A
        0xF0 => Err(2), // LD A, (FF00 + e)
        0xF2 => Ok(Instruction::LD(
            Operand8::A,
            Operand8::Deref(Operand16::FF00C),
        )),
        0xF3 => Ok(Instruction::DI),
        0xF8 => Err(2), // LD HL, SP + e
        0xF9 => Ok(Instruction::LD16(Operand16::SP, Operand16::HL)),
        0xFA => Err(3), // LD A, (nn)
        0xFB => Ok(Instruction::EI),

        0xCB => Err(2), // All 0xCB instructions

        _ => {
            match opcode >> 6 {
                0b00 => {
                    match opcode & 0b111 {
                        0b000 => Err(2), // JR F, e
                        0b001 => {
                            if opcode & 0b1000 == 0 {
                                Err(3) // LD qq, nn
                            } else {
                                Ok(Instruction::ADD16(
                                    Operand16::HL,
                                    map_register16(opcode >> 4),
                                ))
                            }
                        }
                        // We have already matched all 8 00_XXX_010 through
                        // constants
                        //0b010 => unreachable!()
                        0b011 => {
                            let qq = map_register16(opcode >> 4);
                            if opcode & 0b1000 == 0 {
                                Ok(Instruction::INC16(qq))
                            } else {
                                Ok(Instruction::DEC16(qq))
                            }
                        }
                        0b100 => Ok(Instruction::INC(map_register8(opcode >> 3))),
                        0b101 => Ok(Instruction::DEC(map_register8(opcode >> 3))),
                        0b110 => Err(2), // LD r, d
                        0b111 => {
                            // We have already matched all 4 00_1XX_111 though
                            // constants
                            match (opcode >> 3) & 0b11 {
                                0b00 => Ok(Instruction::RLCA),
                                0b01 => Ok(Instruction::RRCA),
                                0b10 => Ok(Instruction::RLA),
                                0b11 => Ok(Instruction::RRA),
                                _ => unreachable!(),
                            }
                        }
                        _ => unreachable!(),
                    }
                }
                // 0b01_110_110 => HALT, already matched by constant
                0b01 => Ok(Instruction::LD(
                    map_register8(opcode >> 3),
                    map_register8(opcode),
                )),
                0b10 => {
                    let r = map_register8(opcode);
                    match (opcode >> 3) & 0b111 {
                        0b000 => Ok(Instruction::ADD(r)),
                        0b001 => Ok(Instruction::ADC(r)),
                        0b010 => Ok(Instruction::SUB(r)),
                        0b011 => Ok(Instruction::SBC(r)),
                        0b100 => Ok(Instruction::AND(r)),
                        0b101 => Ok(Instruction::XOR(r)),
                        0b110 => Ok(Instruction::OR(r)),
                        0b111 => Ok(Instruction::CP(r)),
                        _ => unreachable!(),
                    }
                }
                0b11 => {
                    match opcode & 0b111 {
                        0b000 => {
                            if opcode & 0b100_000 != 0 {
                                Err(2) // Various operations
                            } else {
                                Ok(Instruction::RETConditional(map_condition(opcode >> 3)))
                            }
                        }
                        // All 0b11_XX1_001 have already been matched through
                        // constants
                        0b001 => Ok(Instruction::POP(map_register16(opcode >> 4))),
                        0b010 => {
                            if opcode & 0b100_000 == 0 {
                                Err(3) // JP cc, nn
                            } else {
                                match (opcode & 0b11_000) >> 3 {
                                    0b00 => Ok(Instruction::LD(
                                        Operand8::Deref(Operand16::FF00C),
                                        Operand8::A,
                                    )),
                                    0b01 => Ok(Instruction::LD(
                                        Operand8::A,
                                        Operand8::Deref(Operand16::FF00C),
                                    )),
                                    _ => Err(3), // LD (nn), A and LD A, (nn)
                                }
                            }
                        }
                        // All matched by constants
                        0b011 => Ok(Instruction::IllegalOpcode),
                        0b100 => {
                            if opcode & 0b100_000 == 0 {
                                Err(3) // CALL cc, nn
                            } else {
                                Ok(Instruction::IllegalOpcode)
                            }
                        }
                        0b101 => {
                            if opcode & 0b1000 == 0 {
                                Ok(Instruction::PUSH(map_register16(opcode >> 4)))
                            } else {
                                Ok(Instruction::IllegalOpcode)
                            }
                        }
                        0b110 => Err(2), // [ALU] A, d
                        0b111 => Ok(Instruction::RST(opcode & 0b111_000)),
                        _ => unreachable!(),
                    }
                }
                _ => unreachable!(),
            }
        }
    }
}

/// Decode an instruction from two bytes.
///
/// # Requirements
///
/// This function assumes you already tried `instructions::decode1byte(byte1)`
/// and it returned `Err(2)`.
///
/// # Panics
///
/// On unmet requirements.
pub fn decode2bytes(byte1: u8, byte2: u8) -> Instruction {
    match byte1 {
        0x18 => Instruction::JR(byte2 as i8),
        0xE0 => Instruction::LD(Operand8::Deref(Operand16::FF00Offset(byte2)), Operand8::A),
        0xF0 => Instruction::LD(Operand8::A, Operand8::Deref(Operand16::FF00Offset(byte2))),
        0xE8 => Instruction::ADD16(Operand16::SP, Operand16::Immediate(byte2 as i8 as u16)), // sign extension
        0xF8 => Instruction::LD16(Operand16::HL, Operand16::SPOffset(byte2 as i8)),

        0xCB => {
            let r = map_register8(byte2);
            let b = (byte2 & 0b00_111_000) >> 3;
            match byte2 >> 6 {
                0b00 => match b {
                    0b000 => Instruction::RLC(r),
                    0b001 => Instruction::RRC(r),
                    0b010 => Instruction::RL(r),
                    0b011 => Instruction::RR(r),
                    0b100 => Instruction::SLA(r),
                    0b101 => Instruction::SRA(r),
                    0b110 => Instruction::SWAP(r),
                    0b111 => Instruction::SRL(r),
                    _ => unreachable!(),
                },
                0b01 => Instruction::BIT(b, r),
                0b10 => Instruction::RES(b, r),
                0b11 => Instruction::SET(b, r),
                _ => unreachable!(),
            }
        }

        x if x & 0b11_000_111 == 0b00_000_110 => {
            Instruction::LD(map_register8(byte1 >> 3), Operand8::Immediate(byte2))
        }
        x if x & 0b11_100_111 == 0b00_100_000 => {
            Instruction::JRConditional(map_condition(byte1 >> 3), byte2 as i8)
        }
        x if x & 0b11_000_111 == 0b11_000_110 => {
            let d = Operand8::Immediate(byte2);
            match (byte1 & 0b00_111_000) >> 3 {
                0b000 => Instruction::ADD(d),
                0b001 => Instruction::ADC(d),
                0b010 => Instruction::SUB(d),
                0b011 => Instruction::SBC(d),
                0b100 => Instruction::AND(d),
                0b101 => Instruction::XOR(d),
                0b110 => Instruction::OR(d),
                0b111 => Instruction::CP(d),
                _ => unreachable!(),
            }
        }

        _ => panic!(
            "Requirement unmet: byte 0x{:02X} is not a 2 byte \
                    instruction prefix",
            byte1
        ),
    }
}

/// Decode an instruction from three bytes.
///
/// # Requirements
///
/// This function assumes you already tried `instructions::decode1byte(byte1)`
/// and it returned `Err(3)`.
///
/// # Panics
///
/// On unmet requirements.
pub fn decode3bytes(byte1: u8, byte2: u8, byte3: u8) -> Instruction {
    let nn = u16::from_le_bytes([byte2, byte3]);
    let nnimmediate = Operand16::Immediate(nn);
    let addr8 = Operand8::Deref(nnimmediate.clone());
    match byte1 {
        0x08 => Instruction::LD16(Operand16::Deref(nn), Operand16::SP),
        0xC3 => Instruction::JP(nnimmediate),
        0xCD => Instruction::CALL(nn),
        0xEA => Instruction::LD(addr8, Operand8::A),
        0xFA => Instruction::LD(Operand8::A, addr8),

        x if x & 0b11_001_111 == 0b00_000_001 => {
            Instruction::LD16(map_register16(byte1 >> 4), nnimmediate)
        }
        x if x & 0b11_100_111 == 0b11_000_010 => {
            Instruction::JPConditional(map_condition(byte1 >> 3), nn)
        }
        x if x & 0b11_100_111 == 0b11_000_100 => {
            Instruction::CALLConditional(map_condition(byte1 >> 3), nn)
        }

        _ => panic!(
            "Requirement unmet: byte 0x{:02X} is not a 3 byte \
                    instruction prefix",
            byte1
        ),
    }
}

/// An instruction iterator, yielding instructions and their address.
pub struct Decoder<I> {
    bytes: I,
    addr: u16,
}

impl<I: Iterator<Item = u8>> Decoder<I> {
    /// Create a new instruction iterator.
    ///
    /// # Params
    /// * bytes: an `Iterator` yielding `u8`s
    /// * origin: the address of `bytes`
    pub fn new(bytes: I, origin: u16) -> Decoder<I> {
        Decoder::<I> {
            bytes,
            addr: origin,
        }
    }
}

impl<I: Iterator<Item = u8>> Iterator for Decoder<I> {
    type Item = (Instruction, u16);

    fn next(&mut self) -> Option<Self::Item> {
        let byte1 = self.bytes.next()?;
        let base_addr = self.addr;
        self.addr += 1;

        match decode1byte(byte1) {
            Ok(instruction) => Some((instruction, base_addr)),
            Err(2) => match self.bytes.next() {
                Some(byte2) => {
                    self.addr += 1;
                    Some((decode2bytes(byte1, byte2), base_addr))
                }
                None => Some((Instruction::UnfinishedOpcode, base_addr)),
            },
            Err(3) => match self.bytes.next() {
                Some(byte2) => match self.bytes.next() {
                    Some(byte3) => {
                        self.addr += 2;
                        Some((decode3bytes(byte1, byte2, byte3), base_addr))
                    }
                    None => Some((Instruction::UnfinishedOpcode, base_addr)),
                },
                None => Some((Instruction::UnfinishedOpcode, base_addr)),
            },
            x => panic!("Unhandled value: {:?}", x),
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::gbz80::instructions;

    #[test]
    fn decode_all_bytes() {
        for byte in 0..=u8::MAX {
            //assert_ne!(
            //    instructions::decode1byte(byte),
            //    Ok(instructions::Instruction::IllegalOpcode),
            //    "Byte {:#010b} could not be decoded",
            //    byte
            //)
            // There are unused bytes. Just run a smoke test.
            let _ = instructions::decode1byte(byte);
        }
    }

    #[test]
    fn decode_all_2bytes() {
        for byte1 in 0..=u8::MAX {
            if let Err(2) = instructions::decode1byte(byte1) {
                for byte2 in 0..=u8::MAX {
                    assert_ne!(
                        instructions::decode2bytes(byte1, byte2),
                        instructions::Instruction::IllegalOpcode,
                        "Bytes {:#010b} {:08b} could not be decoded",
                        byte1,
                        byte2
                    );
                }
            }
        }
    }

    #[test]
    fn decode_all_3bytes() {
        for byte1 in 0..=u8::MAX {
            if let Err(3) = instructions::decode1byte(byte1) {
                for byte2 in 0..=u8::MAX {
                    for byte3 in 0..=u8::MAX {
                        assert_ne!(
                            instructions::decode3bytes(byte1, byte2, byte3),
                            instructions::Instruction::IllegalOpcode,
                            "Bytes {:#010b} {:08b} {:08b} could not be decoded",
                            byte1,
                            byte2,
                            byte3
                        )
                    }
                }
            }
        }
    }

    #[test]
    #[rustfmt::skip]
    fn decode_all_instructions() {
        let bytes = [
            0x00,                     // 0X0000 NOP
            0x08, 0xAD, 0xDE,         // 0x0001 LD (0xDEAD), SP
            0b00_00_0001, 0xEF, 0xBE, // 0x0004 LD BC, 0xBEEF
            0b00_01_1001,             // 0x0007 ADD HL, DE
            0b00_00_0010,             // 0x0008 LD (BC), A
            0b00_01_1010,             // 0x0009 LD A, (DE)
            0b00_10_0011,             // 0x000A INC HL
            0b00_11_1011,             // 0x000B DEC SP
            0b00_000_100,             // 0x000C INC B
            0b00_001_101,             // 0x000D DEC C
            0b00_010_110, 42,         // 0x000E LD D, 42
            0x07,                     // 0x0010 RLCA
            0x0F,                     // 0x0011 RRCA
            0x17,                     // 0x0012 RLA
            0x1F,                     // 0x0013 RRA
            0x10,                     // 0x0014 STOP
            0x18, 20,                 // 0x0015 JR 22
            0b00_100_000, 21,         // 0x0017 JR NZ, 23
            0x22,                     // 0x0019 LDI (HL), A
            0x2A,                     // 0x001A LDI A, (HL)
            0x32,                     // 0x001B LDD (HL), A
            0x3A,                     // 0x001C LDD A, (HL)
            0x27,                     // 0x001D DAA
            0x2F,                     // 0x001E CPL
            0x37,                     // 0x001F SCF
            0x3F,                     // 0x0020 CCF
            0b01_011_100,             // 0x0021 LD E, H
            0x76,                     // 0x0022 HALT
            0b10_000_101,             // 0x0023 ADD A, L
            0b10_001_110,             // 0x0024 ADC A, (HL)
            0b10_010_111,             // 0x0025 SUB A, A
            0b10_011_000,             // 0x0026 SBC A, B
            0b10_100_001,             // 0x0027 AND A, C
            0b10_101_010,             // 0x0028 XOR A, D
            0b10_110_011,             // 0x0029 OR A, E
            0b10_111_100,             // 0x002A CP A, H
            0b11_000_110, 43,         // 0x002B ADD A, 43
            0b11_00_0001,             // 0x002D POP BC
            0b11_01_0101,             // 0x002E PUSH DE
            0b11_101_111,             // 0x002F RST 0x28
            0b11_001_000,             // 0x0030 RET Z
            0xC9,                     // 0x0031 RET
            0xD9,                     // 0x0032 RETI
            0b11_010_010, 0xAE, 0xDE, // 0x0033 JP NC, 0xDEAE
            0xC3, 0xAF, 0xDE,         // 0x0036 JP 0xDEAF
            0b11_011_100, 0xB0, 0xDE, // 0x0039 CALL C, 0xDEB0
            0xCD, 0xB1, 0xDE,         // 0x003C CALL 0xDEB1
            0xE8, 44,                 // 0x003F ADD SP, 44
            0xF8, 45,                 // 0x0041 LD HL, SP + 45
            0xE0, 46,                 // 0x0043 LD (FF00 + 46), A
            0xF0, 47,                 // 0x0045 LD A, (FF00 + 47)
            0xE2,                     // 0x0047 LD (FF00 + C), A
            0xF2,                     // 0x0048 LD A, (FF00 + C)
            0xEA, 0xB2, 0xDE,         // 0x0049 LD (0xDEB2), A
            0xFA, 0xB3, 0xDE,         // 0x004C LD A, (0xDEB3)
            0xE9,                     // 0x004F JP HL
            0xF9,                     // 0x0050 LD SP, HL
            0xF3,                     // 0x0051 DI
            0xFB,                     // 0x0052 EI
            0xCB, 0b00_000_101,       // 0x0053 RLC L
            0xCB, 0b00_001_110,       // 0x0055 RRC (HL)
            0xCB, 0b00_010_111,       // 0x0057 RL A
            0xCB, 0b00_011_000,       // 0x0059 RR B
            0xCB, 0b00_100_001,       // 0x005B SLA C
            0xCB, 0b00_101_010,       // 0x005D SRA D
            0xCB, 0b00_110_011,       // 0x005F SWAP E
            0xCB, 0b00_111_100,       // 0x0061 SRL H
            0xCB, 0b01_000_101,       // 0x0063 BIT 0, L
            0xCB, 0b10_001_110,       // 0x0065 RES 1, (HL)
            0xCB, 0b11_010_111,       // 0x0067 SET 2, A
        ];

        let mut iter = instructions::Decoder::new(IntoIterator::into_iter(bytes), 0x0000);

        assert_eq!(iter.next(), Some((instructions::Instruction::NOP, 0x0000)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::Deref(0xDEAD),
                    instructions::Operand16::SP
                ),
                0x0001
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::BC,
                    instructions::Operand16::Immediate(0xBEEF)
                ),
                0x0004
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD16(
                    instructions::Operand16::HL,
                    instructions::Operand16::DE
                ),
                0x0007
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::BC),
                    instructions::Operand8::A
                ),
                0x0008
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::DE)
                ),
                0x0009
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC16(instructions::Operand16::HL),
                0x000A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC16(instructions::Operand16::SP),
                0x000B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC(instructions::Operand8::B),
                0x000C
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC(instructions::Operand8::C),
                0x000D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::D,
                    instructions::Operand8::Immediate(42)
                ),
                0x000E
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RLCA, 0x0010)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RRCA, 0x0011)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RLA, 0x0012)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RRA, 0x0013)));
        assert_eq!(iter.next(), Some((instructions::Instruction::STOP, 0x0014)));
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::JR(20), 0x0015))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JRConditional(instructions::Condition::NZ, 21),
                0x0017
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LDI(
                    instructions::Operand8::Deref(instructions::Operand16::HL),
                    instructions::Operand8::A
                ),
                0x0019
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LDI(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x001A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LDD(
                    instructions::Operand8::Deref(instructions::Operand16::HL),
                    instructions::Operand8::A
                ),
                0x001B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LDD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x001C
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::DAA, 0x001D)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPL, 0x001E)));
        assert_eq!(iter.next(), Some((instructions::Instruction::SCF, 0x001F)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CCF, 0x0020)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::E, instructions::Operand8::H),
                0x0021
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::HALT, 0x0022)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD(instructions::Operand8::L),
                0x0023
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADC(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x0024
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SUB(instructions::Operand8::A),
                0x0025
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SBC(instructions::Operand8::B),
                0x0026
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::AND(instructions::Operand8::C),
                0x0027
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::XOR(instructions::Operand8::D),
                0x0028
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::OR(instructions::Operand8::E),
                0x0029
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::CP(instructions::Operand8::H),
                0x002A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD(instructions::Operand8::Immediate(43)),
                0x002B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::POP(instructions::Operand16::BC),
                0x002D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::PUSH(instructions::Operand16::DE),
                0x002E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::RST(0x28), 0x002F))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RETConditional(instructions::Condition::Z),
                0x0030
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RET, 0x0031)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RETI, 0x0032)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JPConditional(instructions::Condition::NC, 0xDEAE),
                0x0033
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::Immediate(0xDEAF)),
                0x0036
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::CALLConditional(instructions::Condition::C, 0xDEB0),
                0x0039
            ))
        );
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::CALL(0xDEB1), 0x003C))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD16(
                    instructions::Operand16::SP,
                    instructions::Operand16::Immediate(44)
                ),
                0x003F
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::HL,
                    instructions::Operand16::SPOffset(45)
                ),
                0x0041
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::FF00Offset(46)),
                    instructions::Operand8::A
                ),
                0x0043
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::FF00Offset(47))
                ),
                0x0045
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::FF00C),
                    instructions::Operand8::A
                ),
                0x0047
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::FF00C)
                ),
                0x0048
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::Immediate(0xDEB2)),
                    instructions::Operand8::A
                ),
                0x0049
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::Immediate(0xDEB3))
                ),
                0x004C
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::HL),
                0x004F
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::SP,
                    instructions::Operand16::HL
                ),
                0x0050
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::DI, 0x0051)));
        assert_eq!(iter.next(), Some((instructions::Instruction::EI, 0x0052)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RLC(instructions::Operand8::L),
                0x0053
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RRC(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x0055
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RL(instructions::Operand8::A),
                0x0057
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RR(instructions::Operand8::B),
                0x0059
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SLA(instructions::Operand8::C),
                0x005B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SRA(instructions::Operand8::D),
                0x005D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SWAP(instructions::Operand8::E),
                0x005F
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SRL(instructions::Operand8::H),
                0x0061
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::BIT(0, instructions::Operand8::L),
                0x0063
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RES(
                    1,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x0065
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SET(2, instructions::Operand8::A),
                0x0067
            ))
        );
        assert_eq!(iter.next(), None);
    }
}
