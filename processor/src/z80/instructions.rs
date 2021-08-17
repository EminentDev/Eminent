//! A model for the instructions of the Zilog z80 CPU.
//!
//! Note that not all instructions allowed to be built with this model are
//! legal, i.e. have a binary representation, and one that would be understood
//! by an actual z80 CPU. However, the opposite is true: all legal instructions
//! can be represented with this model.

/// An 8 bits operand
#[derive(Clone, Debug, PartialEq)]
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
    I,
    R,
    /// This variant is specific to this implementation of the decoding process
    OneOneZero,
}

/// A 16 bits operand
#[derive(Clone, Debug, PartialEq)]
pub enum Operand16 {
    Immediate(u16),
    BC,
    DE,
    HL,
    IX,
    IY,
    IXOffset(i8),
    IYOffset(i8),
    SP,
    AF,
    AFprime,
    DerefSP,
    Deref(u16),
}

/// A condition for conditional jumps and calls
#[derive(Debug, PartialEq)]
pub enum Condition {
    NZ,
    Z,
    NC,
    C,
    PO,
    PE,
    P,
    M,
}

/// All the instructions of the Zilog z80
#[derive(Debug, PartialEq)]
pub enum Instruction {
    LD(Operand8, Operand8),
    LD16(Operand16, Operand16),
    PUSH(Operand16),
    POP(Operand16),
    EX(Operand16, Operand16),
    EXX,
    LDI,
    LDIR,
    LDD,
    LDDR,
    CPI,
    CPIR,
    CPD,
    CPDR,
    ADD(Operand8),
    ADC(Operand8),
    SUB(Operand8),
    SBC(Operand8),
    AND(Operand8),
    OR(Operand8),
    XOR(Operand8),
    CP(Operand8),
    INC(Operand8),
    DEC(Operand8),
    DAA,
    CPL,
    NEG,
    CCF,
    SCF,
    NOP,
    HALT,
    DI,
    EI,
    IM0,
    IM1,
    IM2,
    ADD16(Operand16, Operand16),
    ADC16(Operand16),
    SBC16(Operand16),
    INC16(Operand16),
    DEC16(Operand16),
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
    RLD,
    RRD,
    BIT(u8, Operand8),
    SET(u8, Operand8),
    RES(u8, Operand8),
    JP(Operand16),
    JPConditional(Condition, u16),
    /// The stored value is `e - 2`, not `e`
    JR(i8),
    /// The stored value is `e - 2`, not `e`
    JRConditional(Condition, i8),
    /// The stored value is `e - 2`, not `e`
    DJNZ(i8),
    CALL(u16),
    CALLConditional(Condition, u16),
    RET,
    RETConditional(Condition),
    RETI,
    RETN,
    /// The stored value is `p`, not `t`
    RST(u8),
    IN(Operand8, Operand8),
    INI,
    INIR,
    IND,
    INDR,
    OUT(Operand8, Operand8),
    OUTI,
    OTIR,
    OUTD,
    OTDR,
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
/// use processor::z80::instructions;
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
        0b110 => Operand8::OneOneZero,
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
/// use processor::z80::instructions;
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

/// Mapping of the opcode tribit to a condition
///
/// This function truncates its argument to the three low bits before mapping.
///
/// # Examples
/// ```rust
/// use processor::z80::instructions;
/// assert_eq!(instructions::map_condition(0b011), instructions::Condition::C);
/// assert_eq!(instructions::map_condition(0b1001), instructions::Condition::Z);
/// ```
pub fn map_condition(tribit: u8) -> Condition {
    match tribit & 0b111 {
        0b000 => Condition::NZ,
        0b001 => Condition::Z,
        0b010 => Condition::NC,
        0b011 => Condition::C,
        0b100 => Condition::PO,
        0b101 => Condition::PE,
        0b110 => Condition::P,
        0b111 => Condition::M,
        _ => unreachable!(),
    }
}

/// Try to decode an instruction from a single byte.
///
/// Return `Some(Instruction)` if sucessful, otherwise `None`. In the latter
/// case, more bytes may be needed.
pub fn decode1byte(opcode: u8) -> Option<Instruction> {
    match opcode {
        // Non-variable prefixes for multi-byte instructions
        0x10 | 0x18 | 0x20 | 0x22 | 0x28 | 0x2A | 0x30 | 0x32 | 0x36 | 0x38 | 0x3A | 0xC3
        | 0xCB | 0xCD | 0xD3 | 0xDB | 0xDD | 0xED | 0xFD => None,

        // Non-variable 1-byte instructions
        0x0A => Some(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::BC))),
        0x1A => Some(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::DE))),
        0x02 => Some(Instruction::LD(Operand8::Deref(Operand16::BC), Operand8::A)),
        0x12 => Some(Instruction::LD(Operand8::Deref(Operand16::DE), Operand8::A)),
        0xF9 => Some(Instruction::LD16(Operand16::SP, Operand16::HL)),
        0xEB => Some(Instruction::EX(Operand16::DE, Operand16::HL)),
        0x08 => Some(Instruction::EX(Operand16::AF, Operand16::AFprime)),
        0xD9 => Some(Instruction::EXX),
        0xE3 => Some(Instruction::EX(Operand16::DerefSP, Operand16::HL)),
        0x27 => Some(Instruction::DAA),
        0x2F => Some(Instruction::CPL),
        0x3F => Some(Instruction::CCF),
        0x37 => Some(Instruction::SCF),
        0x00 => Some(Instruction::NOP),
        0x76 => Some(Instruction::HALT),
        0xF3 => Some(Instruction::DI),
        0xFB => Some(Instruction::EI),
        0x07 => Some(Instruction::RLCA),
        0x17 => Some(Instruction::RLA),
        0x0F => Some(Instruction::RRCA),
        0x1F => Some(Instruction::RRA),
        0xE9 => Some(Instruction::JP(Operand16::HL)),
        0xC9 => Some(Instruction::RET),

        // 00 xxx xxx
        x if x >> 6 == 0b00 => {
            match opcode & 0b00_000_111 {
                0b001 => {
                    if opcode & 0b1000 == 0 {
                        // bit 3 not set => LD dd, nn => multi-byte
                        None
                    } else {
                        // bit 3 set => ADD HL, ss
                        Some(Instruction::ADD16(
                            Operand16::HL,
                            map_register16(opcode >> 4),
                        ))
                    }
                }
                0b011 => {
                    if opcode & 0b1000 == 0 {
                        Some(Instruction::INC16(map_register16(opcode >> 4)))
                    } else {
                        Some(Instruction::DEC16(map_register16(opcode >> 4)))
                    }
                }
                0b100 => Some(Instruction::INC(match map_register8(opcode >> 3) {
                    Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                    r => r,
                })),
                0b101 => Some(Instruction::DEC(match map_register8(opcode >> 3) {
                    Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                    r => r,
                })),
                0b110 => None, // LD r, n => multi-byte
                _ => {
                    //Some(Instruction::IllegalOpcode)
                    // From tests::decode_all_bytes, we know this is unreachable.
                    unreachable!()
                }
            }
        }

        // 01 xxx xxx
        x if x >> 6 == 0b01 => {
            match (map_register8(opcode >> 3), map_register8(opcode)) {
                // 110 110 => HALT, already matched as constant 0x76
                (Operand8::OneOneZero, r) => {
                    Some(Instruction::LD(Operand8::Deref(Operand16::HL), r))
                }
                (r, Operand8::OneOneZero) => {
                    Some(Instruction::LD(r, Operand8::Deref(Operand16::HL)))
                }
                (r1, r2) => Some(Instruction::LD(r1, r2)),
            }
        }

        // 10 xxx xxx
        x if x >> 6 == 0b10 => {
            let r = match map_register8(opcode) {
                Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                r => r,
            };
            match (opcode & 0b00_111_000) >> 3 {
                0b000 => Some(Instruction::ADD(r)),
                0b001 => Some(Instruction::ADC(r)),
                0b010 => Some(Instruction::SUB(r)),
                0b011 => Some(Instruction::SBC(r)),
                0b100 => Some(Instruction::AND(r)),
                0b101 => Some(Instruction::XOR(r)),
                0b110 => Some(Instruction::OR(r)),
                0b111 => Some(Instruction::CP(r)),
                _ => unreachable!(),
            }
        }

        // 11 xxx xxx
        _ => {
            match opcode & 0b111 {
                0b010 | 0b011 | 0b100 | 0b110 => None, // Multi-byte instruction
                0b000 => Some(Instruction::RETConditional(map_condition(opcode >> 3))),
                0b001 | 0b101 => {
                    let qq = match map_register16(opcode >> 4) {
                        Operand16::SP => Operand16::AF,
                        r => r,
                    };
                    if opcode & 0b1000 == 0 {
                        if opcode & 0b100 == 0 {
                            Some(Instruction::POP(qq))
                        } else {
                            Some(Instruction::PUSH(qq))
                        }
                    } else {
                        //Some(Instruction::IllegalOpcode)
                        // From tests::decode_all_bytes, we know this is unreachable.
                        unreachable!()
                    }
                }
                0b111 => Some(Instruction::RST(opcode & 0b00_111_000)),
                _ => unreachable!(),
            }
        }
    }
}

/// Try to decode an instruction from two bytes.
///
/// Return `Some(Instruction)` if sucessful, otherwise `None`. In the latter
/// case, more bytes may be needed.
///
/// # Requirements
///
/// This function assumes you already tried `instructions::decode1byte(byte1)`
/// and it returned `None`.
pub fn decode2bytes(byte1: u8, byte2: u8) -> Option<Instruction> {
    match byte1 {
        0xDD | 0xFD => {
            let ir = if byte1 == 0xDD {
                Operand16::IX
            } else {
                Operand16::IY
            };
            match byte2 {
                0xF9 => Some(Instruction::LD16(Operand16::SP, ir)),
                0xE5 => Some(Instruction::PUSH(ir)),
                0xE1 => Some(Instruction::POP(ir)),
                0xE3 => Some(Instruction::EX(Operand16::DerefSP, ir)),
                0x23 => Some(Instruction::INC16(ir)),
                0x2B => Some(Instruction::DEC16(ir)),
                0xE9 => Some(Instruction::JP(ir)),

                0x21 | 0x22 | 0x2A | 0x36 | 0xCB => None, // 4 bytes

                x if x & 0b11_001_111 == 0b01_001_001 => Some(Instruction::ADD16(
                    ir.clone(),
                    match map_register16(byte2 >> 4) {
                        Operand16::HL => ir,
                        r => r,
                    },
                )),

                // LD r, (ir+d) => 3 bytes
                x if x & 0b11_000_111 == 0b01_000_110 => None,
                // LD (ir+d), r => 3 bytes
                x if x & 0b11_111_000 == 0b01_110_000 => None,
                // <arithmetic or logic operation> A, (ir+d) => 3 bytes
                x if x & 0b11_000_111 == 0b10_000_110 => None,
                // <INC|DEC> (ir+d) => 3 bytes
                x if x & 0b11_111_110 == 0b00_110_100 => None,

                _ => Some(Instruction::IllegalOpcode),
            }
        }
        0xED => {
            match byte2 {
                0x57 => Some(Instruction::LD(Operand8::A, Operand8::I)),
                0x5F => Some(Instruction::LD(Operand8::A, Operand8::R)),
                0x47 => Some(Instruction::LD(Operand8::I, Operand8::A)),
                0x4F => Some(Instruction::LD(Operand8::R, Operand8::A)),
                0xA0 => Some(Instruction::LDI),
                0xB0 => Some(Instruction::LDIR),
                0xA8 => Some(Instruction::LDD),
                0xB8 => Some(Instruction::LDDR),
                0xA1 => Some(Instruction::CPI),
                0xB1 => Some(Instruction::CPIR),
                0xA9 => Some(Instruction::CPD),
                0xB9 => Some(Instruction::CPDR),
                0x44 => Some(Instruction::NEG),
                0x46 => Some(Instruction::IM0),
                0x56 => Some(Instruction::IM1),
                0x5E => Some(Instruction::IM2),
                0x6F => Some(Instruction::RLD),
                0x67 => Some(Instruction::RRD),
                0x4D => Some(Instruction::RETI),
                0x45 => Some(Instruction::RETN),
                0xA2 => Some(Instruction::INI),
                0xB2 => Some(Instruction::INIR),
                0xAA => Some(Instruction::IND),
                0xBA => Some(Instruction::INDR),
                0xA3 => Some(Instruction::OUTI),
                0xB3 => Some(Instruction::OTIR),
                0xAB => Some(Instruction::OUTD),
                0xBB => Some(Instruction::OTDR),

                // LD dd, (nn) and LD (nn), dd => 4 bytes
                x if x & 0b11_000_111 == 0b01_000_011 => None,
                x if x & 0b11_000_111 == 0b01_000_010 => {
                    let r = map_register16(byte2 >> 4);
                    if byte2 & 0b1000 == 0 {
                        Some(Instruction::SBC16(r))
                    } else {
                        Some(Instruction::ADC16(r))
                    }
                }
                x if x & 0b11_000_110 == 0b01_000_000 => {
                    let r = map_register8(byte2 >> 3);
                    if byte2 & 0b1 == 0 {
                        // Keep OneOneZero for execution step
                        Some(Instruction::IN(r, Operand8::C))
                    } else {
                        // Nothing in spec about OneOneZero, assume illegal
                        if r == Operand8::OneOneZero {
                            Some(Instruction::IllegalOpcode)
                        } else {
                            Some(Instruction::OUT(Operand8::C, r))
                        }
                    }
                }

                _ => Some(Instruction::IllegalOpcode),
            }
        }
        0xCB => {
            let r = match map_register8(byte2) {
                Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                r => r,
            };
            match byte2 >> 6 {
                0b00 => match byte2 >> 3 {
                    0b000 => Some(Instruction::RLC(r)),
                    0b001 => Some(Instruction::RRC(r)),
                    0b010 => Some(Instruction::RL(r)),
                    0b011 => Some(Instruction::RR(r)),
                    0b100 => Some(Instruction::SLA(r)),
                    0b101 => Some(Instruction::SRA(r)),
                    0b110 => Some(Instruction::IllegalOpcode),
                    0b111 => Some(Instruction::SRL(r)),
                    _ => unreachable!(),
                },
                0b01 => Some(Instruction::BIT((byte2 >> 3) & 0b111, r)),
                0b11 => Some(Instruction::SET((byte2 >> 3) & 0b111, r)),
                0b10 => Some(Instruction::RES((byte2 >> 3) & 0b111, r)),
                _ => unreachable!(),
            }
        }

        0x36 => Some(Instruction::LD(
            Operand8::Deref(Operand16::HL),
            Operand8::Immediate(byte2),
        )),
        0x18 => Some(Instruction::JR(i8::from_be_bytes([byte2]))),
        0x38 => Some(Instruction::JRConditional(
            Condition::C,
            i8::from_be_bytes([byte2]),
        )),
        0x30 => Some(Instruction::JRConditional(
            Condition::NC,
            i8::from_be_bytes([byte2]),
        )),
        0x28 => Some(Instruction::JRConditional(
            Condition::Z,
            i8::from_be_bytes([byte2]),
        )),
        0x20 => Some(Instruction::JRConditional(
            Condition::NZ,
            i8::from_be_bytes([byte2]),
        )),
        0x10 => Some(Instruction::DJNZ(i8::from_be_bytes([byte2]))),
        0xDB => Some(Instruction::IN(Operand8::A, Operand8::Immediate(byte2))),
        0xD3 => Some(Instruction::OUT(Operand8::Immediate(byte2), Operand8::A)),

        0x22 | 0x2A | 0x32 | 0x3A | 0xC3 | 0xCD => None, // 3 bytes

        x if x & 0b11_000_111 == 0b00_000_110 => Some(Instruction::LD(
            map_register8(byte1 >> 3),
            Operand8::Immediate(byte2),
        )),
        x if x & 0b11_000_111 == 0b11_000_110 => match (byte1 >> 3) & 0b111 {
            0b000 => Some(Instruction::ADD(Operand8::Immediate(byte2))),
            0b001 => Some(Instruction::ADC(Operand8::Immediate(byte2))),
            0b010 => Some(Instruction::SUB(Operand8::Immediate(byte2))),
            0b011 => Some(Instruction::SBC(Operand8::Immediate(byte2))),
            0b100 => Some(Instruction::AND(Operand8::Immediate(byte2))),
            0b101 => Some(Instruction::XOR(Operand8::Immediate(byte2))),
            0b110 => Some(Instruction::OR(Operand8::Immediate(byte2))),
            0b111 => Some(Instruction::CP(Operand8::Immediate(byte2))),
            _ => unreachable!(),
        },

        x if x & 0b11_001_111 == 0b00_000_001 => None, // LD dd, nn => 3 bytes
        x if x & 0b11_000_111 == 0b11_000_010 => None, // JP cc, nn => 3 bytes
        x if x & 0b11_000_111 == 0b11_000_100 => None, // CALL cc, nn => 3 bytes
        _ => {
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_2bytes, we know this is unreachable.
            unreachable!()
        }
    }
}

/// Try to decode an instruction from three bytes.
///
/// Return `Some(Instruction)` if sucessful, otherwise `None`. In the latter
/// case, more bytes may be needed.
///
/// # Requirements
///
/// This function assumes you already tried
/// `instructions::decode2bytes(byte1, byte2)` and it returned `None`.
pub fn decode3bytes(byte1: u8, byte2: u8, byte3: u8) -> Option<Instruction> {
    let nn = u16::from_le_bytes([byte2, byte3]);
    let nnimmediate = Operand16::Immediate(nn.clone());
    match byte1 {
        0xDD | 0xFD => {
            let iroffset = if byte1 == 0xDD {
                Operand8::Deref(Operand16::IXOffset(i8::from_be_bytes([byte3])))
            } else {
                Operand8::Deref(Operand16::IYOffset(i8::from_be_bytes([byte3])))
            };
            match byte2 {
                0x34 => Some(Instruction::INC(iroffset)),
                0x35 => Some(Instruction::DEC(iroffset)),
                0x21 | 0x22 | 0x2A | 0x36 | 0xCB => None, // 4 bytes

                x if x & 0b11_000_111 == 0b01_000_110 => {
                    Some(Instruction::LD(map_register8(byte2 >> 3), iroffset))
                }
                x if x & 0b11_111_000 == 0b01_110_000 => {
                    Some(Instruction::LD(iroffset, map_register8(byte2)))
                }
                x if x & 0b11_000_111 == 0b10_000_110 => match (byte2 >> 3) & 0b111 {
                    0b000 => Some(Instruction::ADD(iroffset)),
                    0b001 => Some(Instruction::ADC(iroffset)),
                    0b010 => Some(Instruction::SUB(iroffset)),
                    0b011 => Some(Instruction::SBC(iroffset)),
                    0b100 => Some(Instruction::AND(iroffset)),
                    0b101 => Some(Instruction::XOR(iroffset)),
                    0b110 => Some(Instruction::OR(iroffset)),
                    0b111 => Some(Instruction::CP(iroffset)),
                    _ => unreachable!(),
                },

                _ => Some(Instruction::IllegalOpcode),
            }
        }

        0x3A => Some(Instruction::LD(Operand8::A, Operand8::Deref(nnimmediate))),
        0x32 => Some(Instruction::LD(Operand8::Deref(nnimmediate), Operand8::A)),
        0x2A => Some(Instruction::LD16(Operand16::HL, Operand16::Deref(nn))),
        0x22 => Some(Instruction::LD16(Operand16::Deref(nn), Operand16::HL)),
        0xC3 => Some(Instruction::JP(Operand16::Immediate(nn))),
        0xCD => Some(Instruction::CALL(nn)),

        0xED => None, // 4 bytes, assuming requirements have been followed

        x if x & 0b11_001_111 == 0b00_000_001 => {
            Some(Instruction::LD16(map_register16(byte1 >> 4), nnimmediate))
        }
        x if x & 0b11_000_111 == 0b11_000_010 => {
            Some(Instruction::JPConditional(map_condition(byte1 >> 3), nn))
        }
        x if x & 0b11_000_111 == 0b11_000_100 => {
            Some(Instruction::CALLConditional(map_condition(byte1 >> 3), nn))
        }

        _ => {
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_3bytes, we know this is unreachable.
            unreachable!()
        }
    }
}

/// Try to decode an instruction from four bytes.
///
/// Although the interface calls for an `Option<Instruction>` to match the other
/// functions in this module, this function always returns `Some` as there are
/// no opcodes longer than 4 bytes.
///
/// # Requirements
///
/// This function assumes you already tried
/// `instructions::decode3bytes(byte1, byte2, byte3)` and it returned `None`.
pub fn decode4bytes(byte1: u8, byte2: u8, byte3: u8, byte4: u8) -> Option<Instruction> {
    let nn = u16::from_le_bytes([byte3, byte4]);
    match byte1 {
        0xDD | 0xFD => {
            let (ir, iroffset) = if byte1 == 0xDD {
                (
                    Operand16::IX,
                    Operand8::Deref(Operand16::IXOffset(i8::from_be_bytes([byte3]))),
                )
            } else {
                (
                    Operand16::IY,
                    Operand8::Deref(Operand16::IYOffset(i8::from_be_bytes([byte3]))),
                )
            };
            match byte2 {
                0x36 => Some(Instruction::LD(iroffset, Operand8::Immediate(byte4))),
                0x21 => Some(Instruction::LD16(ir, Operand16::Immediate(nn))),
                0x2A => Some(Instruction::LD16(ir, Operand16::Deref(nn))),
                0x22 => Some(Instruction::LD16(Operand16::Deref(nn), ir)),
                0xCB => {
                    if byte4 & 0b00_000_111 == 0b110 {
                        match byte4 >> 6 {
                            0b00 => match byte4 >> 3 {
                                0b000 => Some(Instruction::RLC(iroffset)),
                                0b001 => Some(Instruction::RRC(iroffset)),
                                0b010 => Some(Instruction::RL(iroffset)),
                                0b011 => Some(Instruction::RR(iroffset)),
                                0b100 => Some(Instruction::SLA(iroffset)),
                                0b101 => Some(Instruction::SRA(iroffset)),
                                0b110 => Some(Instruction::IllegalOpcode),
                                0b111 => Some(Instruction::SRL(iroffset)),
                                _ => unreachable!(),
                            },
                            0b01 => Some(Instruction::BIT((byte4 >> 3) & 0b111, iroffset)),
                            0b10 => Some(Instruction::RES((byte4 >> 3) & 0b111, iroffset)),
                            0b11 => Some(Instruction::SET((byte4 >> 3) & 0b111, iroffset)),
                            _ => unreachable!(),
                        }
                    } else {
                        Some(Instruction::IllegalOpcode)
                    }
                }
                _ => {
                    //Some(Instruction::IllegalOpcode)
                    // From tests::decode_all_4bytes, we know this is unreachable.
                    unreachable!()
                }
            }
        }
        0xED => {
            if byte2 & 0b11_000_111 == 0b01_000_011 {
                if byte2 & 0b1000 == 0 {
                    Some(Instruction::LD16(
                        Operand16::Deref(nn),
                        map_register16(byte2 >> 4),
                    ))
                } else {
                    Some(Instruction::LD16(
                        map_register16(byte2 >> 4),
                        Operand16::Deref(nn),
                    ))
                }
            } else {
                //Some(Instruction::IllegalOpcode)
                // From tests::decode_all_4bytes, we know this is unreachable.
                unreachable!()
            }
        }

        _ => {
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_4bytes, we know this is unreachable.
            unreachable!()
        }
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

        if let Some(instruction) = decode1byte(byte1) {
            return Some((instruction, base_addr));
        }

        let byte2 = match self.bytes.next() {
            Some(b) => b,
            None => return Some((Instruction::UnfinishedOpcode, base_addr)),
        };
        self.addr += 1;

        if let Some(instruction) = decode2bytes(byte1, byte2) {
            return Some((instruction, base_addr));
        }

        let byte3 = match self.bytes.next() {
            Some(b) => b,
            None => return Some((Instruction::UnfinishedOpcode, base_addr)),
        };
        self.addr += 1;

        if let Some(instruction) = decode3bytes(byte1, byte2, byte3) {
            return Some((instruction, base_addr));
        }

        let byte4 = match self.bytes.next() {
            Some(b) => b,
            None => return Some((Instruction::UnfinishedOpcode, base_addr)),
        };
        self.addr += 1;

        Some((decode4bytes(byte1, byte2, byte3, byte4)?, base_addr))
    }
}

#[cfg(test)]
mod tests {
    use crate::z80::instructions;

    #[test]
    fn decode_all_bytes() {
        for byte in 0..=u8::MAX {
            assert_ne!(
                instructions::decode1byte(byte),
                Some(instructions::Instruction::IllegalOpcode),
                "Byte {:#010b} could not be decoded",
                byte
            )
        }
    }

    #[test]
    fn decode_all_2bytes() {
        for byte1 in 0..=u8::MAX {
            if let None = instructions::decode1byte(byte1) {
                for byte2 in 0..=u8::MAX {
                    //assert_ne!(instructions::decode2bytes(byte1, byte2),
                    //    Some(instructions::Instruction::IllegalOpcode),
                    //    "Bytes {:#010b} {:08b} could not be decoded",
                    //    byte1, byte2);
                    // There are unused byte sequences. Just run a smoke test.
                    let _ = instructions::decode2bytes(byte1, byte2);
                }
            }
        }
    }

    #[test]
    fn decode_all_3bytes() {
        for byte1 in 0..=u8::MAX {
            if let None = instructions::decode1byte(byte1) {
                for byte2 in 0..=u8::MAX {
                    if let None = instructions::decode2bytes(byte1, byte2) {
                        for byte3 in 0..=u8::MAX {
                            //assert_ne!(
                            //    instructions::decode3bytes(byte1, byte2, byte3),
                            //    Some(instructions::Instruction::IllegalOpcode),
                            //    "Bytes {:#010b} {:08b} {:08b} could not be decoded",
                            //    byte1, byte2, byte3)
                            // There are unused byte sequences. Just run a smoke test.
                            let _ = instructions::decode3bytes(byte1, byte2, byte3);
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn decode_all_4bytes() {
        for byte1 in 0..=u8::MAX {
            if let None = instructions::decode1byte(byte1) {
                for byte2 in 0..=u8::MAX {
                    if let None = instructions::decode2bytes(byte1, byte2) {
                        for byte3 in 0..=u8::MAX {
                            if let None = instructions::decode3bytes(byte1, byte2, byte3) {
                                for byte4 in 0..=u8::MAX {
                                    //assert_ne!(
                                    //    instructions::decode4bytes(byte1, byte2, byte3, byte4),
                                    //    Some(instructions::Instruction::IllegalOpcode),
                                    //    "Bytes {:#010b} {:08b} {:08b} {:08b} could not be decoded",
                                    //    byte1, byte2, byte3, byte4)
                                    // There are unused byte sequences. Just run a smoke test.
                                    let _ = instructions::decode4bytes(byte1, byte2, byte3, byte4);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    #[rustfmt::skip]
    fn decode_all_instructions() {
        let bytes = [
            0b01_010_111u8,                 // 0X0000 LD D, A
            0b00_001_110, 42,               // 0X0001 LD C, 42
            0b01_011_110,                   // 0X0003 LD E, (HL)
            0xDD, 0b01_100_110, 3,          // 0X0004 LD H, (IX + 3)
            0xFD, 0b01_101_110, 4,          // 0X0007 LD L, (IY + 4)
            0b01_110_000,                   // 0X000A LD (HL), B
            0xDD, 0b01_110_111, 5,          // 0X000B LD (IX + 5), A
            0xFD, 0b01_110_010, 6,          // 0X000E LD (IY + 6), D
            0x36, 43,                       // 0X0011 LD (HL), 43
            0xDD, 0x36, 7, 44,              // 0X0013 LD (IX + 7), 44
            0xFD, 0x36, 8, 45,              // 0X0017 LD (IY + 8), 45
            0x0A,                           // 0X001B LD A, (BC)
            0x1A,                           // 0X001C LD A, (DE)
            0x3A, 0xAD, 0xDE,               // 0X001D LD A, (0xDEAD)
            0x02,                           // 0X0020 LD (BC), A
            0x12,                           // 0X0021 LD (DE), A
            0x32, 0xAE, 0xDE,               // 0X0022 LD (0xDEAE), A
            0xED, 0x57,                     // 0X0025 LD A, I
            0xED, 0x5F,                     // 0X0027 LD A, R
            0xED, 0x47,                     // 0X0029 LD I, A
            0xED, 0x4F,                     // 0X002B LD R, A
            0b00_000_001, 0xEF, 0xBE,       // 0X002D LD BC, 0xBEEF
            0xDD, 0x21, 0xF0, 0xBE,         // 0X0030 LD IX, 0xBEF0
            0xFD, 0x21, 0xF1, 0xBE,         // 0X0034 LD IY, 0xBEF1
            0x2A, 0xAF, 0xDE,               // 0X0038 LD HL, (0xDEAF)
            0xED, 0b01_011_011, 0xB0, 0xDE, // 0X003B LD DE, (0xDEB0)
            0xDD, 0x2A, 0xB1, 0xDE,         // 0X003F LD IX, (0xDEB1)
            0xFD, 0x2A, 0xB2, 0xDE,         // 0X0043 LD IY, (0xDEB2)
            0x22, 0xB3, 0xDE,               // 0X0047 LD (0xDEB3), HL
            0xED, 0b01_110_011, 0xB4, 0xDE, // 0X004A LD (0xDEB4), SP
            0xDD, 0x22, 0xB5, 0xDE,         // 0X004E LD (0xDEB5), IX
            0xFD, 0x22, 0xB6, 0xDE,         // 0X0052 LD (0xDEB6), IY
            0xF9,                           // 0X0056 LD SP, HL
            0xDD, 0xF9,                     // 0X0057 LD SP, IX
            0xFD, 0xF9,                     // 0X0059 LD SP, IY
            0b11_110_101,                   // 0X005B PUSH AF
            0xDD, 0xE5,                     // 0X005C PUSH IX
            0xFD, 0xE5,                     // 0X005E PUSH IY
            0b11_000_001,                   // 0X0060 POP BC
            0xDD, 0xE1,                     // 0X0061 POP IX
            0xFD, 0xE1,                     // 0X0063 POP IY
            0xEB,                           // 0X0065 EX DE, HL
            0x08,                           // 0X0066 EX AF, AF'
            0xD9,                           // 0X0067 EXX
            0xE3,                           // 0X0068 EX (SP), HL
            0xDD, 0xE3,                     // 0X0069 EX (SP), IX
            0xFD, 0xE3,                     // 0X006B EX (SP), IY
            0xED, 0xA0,                     // 0X006D LDI
            0xED, 0xB0,                     // 0X006F LDIR
            0xED, 0xA8,                     // 0X0071 LDD
            0xED, 0xB8,                     // 0X0073 LDDR
            0xED, 0xA1,                     // 0X0075 CPI
            0xED, 0xB1,                     // 0X0077 CPIR
            0xED, 0xA9,                     // 0X0079 CPD
            0xED, 0xB9,                     // 0X007B CPDR
            0b10_000_000,                   // 0X007D ADD A, B
            0b11_001_110, 46,               // 0X007E ADC A, 46
            0b10_010_110,                   // 0X0080 SUB A, (HL)
            0xDD, 0b10_011_110, 9,          // 0X0081 SBC A, (IX + 9)
            0xFD, 0b10_100_110, 10,         // 0X0084 AND A, (IY + 10)
            0b10_110_001,                   // 0X0087 OR A, C
            0b11_101_110, 47,               // 0X0088 XOR A, 47
            0b10_111_110,                   // 0X008A CP A, (HL)
            0b00_010_100,                   // 0X008B INC D
            0b00_110_101,                   // 0X008C DEC (HL)
            0xDD, 0b00_110_100, 11,         // 0X008D INC (IX + 11)
            0xFD, 0b00_110_101, 12,         // 0X0090 DEC (IY + 12)
            0x27,                           // 0X0093 DAA
            0x2F,                           // 0X0094 CPL
            0xED, 0x44,                     // 0X0095 NEG
            0x3F,                           // 0X0097 CCF
            0x37,                           // 0X0098 SCF
            0x00,                           // 0X0099 NOP
            0x76,                           // 0X009A HALT
            0xF3,                           // 0X009B DI
            0xFB,                           // 0X009C EI
            0xED, 0x46,                     // 0X009D IM 0
            0xED, 0x56,                     // 0X009F IM 1
            0xED, 0x5E,                     // 0X00A1 IM 2
            0b00_111_001,                   // 0X00A3 ADD HL, SP
            0xED, 0b01_001_010,             // 0X00A4 ADC HL, BC
            0xED, 0b01_010_010,             // 0X00A6 SBC HL, DE
            0xDD, 0b01_101_001,             // 0X00A8 ADD IX, IX
            0xFD, 0b01_111_001,             // 0X00AA ADD IY, SP
            0b00_100_011,                   // 0X00AC INC HL
            0xDD, 0x23,                     // 0X00AD INC IX
            0xFD, 0x23,                     // 0X00AF INC IY
            0b00_001_011,                   // 0X00B1 DEC BC
            0xDD, 0x2B,                     // 0X00B2 DEC IX
            0xFD, 0x2B,                     // 0X00B4 DEC IY
            0x07,                           // 0X00B6 RLCA
            0x17,                           // 0X00B7 RLA
            0x0F,                           // 0X00B8 RRCA
            0x1F,                           // 0X00B9 RRA
            0xCB, 0b00_000_100,             // 0X00BA RLC H
            0xCB, 0b00_010_110,             // 0X00BC RL (HL)
            0xDD, 0xCB, 13, 0b00_001_110,   // 0X00BE RRC (IX + 13)
            0xFD, 0xCB, 14, 0b00_011_110,   // 0X00C2 RR (IY + 14)
            0xCB, 0b00_100_101,             // 0X00C6 SLA L
            0xCB, 0b00_101_110,             // 0X00C8 SRA (HL)
            0xDD, 0xCB, 15, 0b00_111_110,   // 0X00CA SRL (IX + 15)
            0xED, 0x6F,                     // 0X00CE RLD
            0xED, 0x67,                     // 0X00D0 RRD
            0xCB, 0b01_000_111,             // 0X00D2 BIT 0, A
            0xCB, 0b01_001_110,             // 0X00D4 BIT 1, (HL)
            0xDD, 0xCB, 16, 0b01_010_110,   // 0X00D6 BIT 2, (IX + 16)
            0xFD, 0xCB, 17, 0b01_011_110,   // 0X00DA BIT 3, (IY + 17)
            0xCB, 0b11_100_000,             // 0X00DE SET 4, B
            0xCB, 0b10_101_110,             // 0X00E0 RES 5, (HL)
            0xDD, 0xCB, 18, 0b11_110_110,   // 0X00E2 SET 6, (IX + 18)
            0xFD, 0xCB, 19, 0b10_111_110,   // 0X00E6 RES 7, (IY + 19)
            0xC3, 0xB7, 0xDE,               // 0X00EA JP 0xDEB7
            0b11_000_010, 0xB8, 0xDE,       // 0X00ED JP NZ, 0xDEB8
            0x18, 20,                       // 0X00F0 JR 22
            0x38, 21,                       // 0X00F2 JR C, 23
            0x30, 22,                       // 0X00F4 JR NC, 24
            0x28, 23,                       // 0X00F6 JR Z, 25
            0x20, 24,                       // 0X00F8 JR NZ, 26
            0xE9,                           // 0X00FA JP (HL)
            0xDD, 0xE9,                     // 0X00FB JP (IX)
            0xFD, 0xE9,                     // 0X00FD JP (IY)
            0x10, 25,                       // 0X00FF DJNZ 27
            0xCD, 0xB9, 0xDE,               // 0X0101 CALL 0xDEB9
            0b11_001_100, 0xBA, 0xDE,       // 0X0104 CALL Z, 0xDEBA
            0xC9,                           // 0X0107 RET
            0b11_010_000,                   // 0X0108 RET NC
            0xED, 0x4D,                     // 0X0109 RETI
            0xED, 0x45,                     // 0X010B RETN
            0b11_101_111,                   // 0X010D RST 0x28
            0xDB, 47,                       // 0X010E IN A, (47)
            0xED, 0b01_010_000,             // 0X0110 IN D, (C)
            0xED, 0xA2,                     // 0X0112 INI
            0xED, 0xB2,                     // 0X0114 INIR
            0xED, 0xAA,                     // 0X0116 IND
            0xED, 0xBA,                     // 0X0118 INDR
            0xD3, 48,                       // 0X011A OUT (48), A
            0xED, 0b01_011_001,             // 0X011C OUT (C), E
            0xED, 0xA3,                     // 0X011E OUTI
            0xED, 0xB3,                     // 0X0120 OTIR
            0xED, 0xAB,                     // 0X0122 OUTD
            0xED, 0xBB,                     // 0X0124 OTDR
            ];

        let mut iter = instructions::Decoder::new(IntoIterator::into_iter(bytes), 0x0000);

        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::D, instructions::Operand8::A),
                0x0000
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::C,
                    instructions::Operand8::Immediate(42)
                ),
                0x0001
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::E,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x0003
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::H,
                    instructions::Operand8::Deref(instructions::Operand16::IXOffset(3))
                ),
                0x0004
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::L,
                    instructions::Operand8::Deref(instructions::Operand16::IYOffset(4))
                ),
                0x0007
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::HL),
                    instructions::Operand8::B
                ),
                0x000A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::IXOffset(5)),
                    instructions::Operand8::A
                ),
                0x000B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::IYOffset(6)),
                    instructions::Operand8::D
                ),
                0x000E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::HL),
                    instructions::Operand8::Immediate(43)
                ),
                0x0011
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::IXOffset(7)),
                    instructions::Operand8::Immediate(44)
                ),
                0x0013
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::IYOffset(8)),
                    instructions::Operand8::Immediate(45)
                ),
                0x0017
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::BC)
                ),
                0x001B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::DE)
                ),
                0x001C
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::A,
                    instructions::Operand8::Deref(instructions::Operand16::Immediate(0xDEAD))
                ),
                0x001D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::BC),
                    instructions::Operand8::A
                ),
                0x0020
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::DE),
                    instructions::Operand8::A
                ),
                0x0021
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(
                    instructions::Operand8::Deref(instructions::Operand16::Immediate(0xDEAE)),
                    instructions::Operand8::A
                ),
                0x0022
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::A, instructions::Operand8::I),
                0x0025
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::A, instructions::Operand8::R),
                0x0027
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::I, instructions::Operand8::A),
                0x0029
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD(instructions::Operand8::R, instructions::Operand8::A),
                0x002B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::BC,
                    instructions::Operand16::Immediate(0xBEEF)
                ),
                0x002D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::IX,
                    instructions::Operand16::Immediate(0xBEF0)
                ),
                0x0030
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::IY,
                    instructions::Operand16::Immediate(0xBEF1)
                ),
                0x0034
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::HL,
                    instructions::Operand16::Deref(0xDEAF)
                ),
                0x0038
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::DE,
                    instructions::Operand16::Deref(0xDEB0)
                ),
                0x003B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::IX,
                    instructions::Operand16::Deref(0xDEB1)
                ),
                0x003F
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::IY,
                    instructions::Operand16::Deref(0xDEB2)
                ),
                0x0043
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::Deref(0xDEB3),
                    instructions::Operand16::HL
                ),
                0x0047
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::Deref(0xDEB4),
                    instructions::Operand16::SP
                ),
                0x004A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::Deref(0xDEB5),
                    instructions::Operand16::IX
                ),
                0x004E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::Deref(0xDEB6),
                    instructions::Operand16::IY
                ),
                0x0052
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::SP,
                    instructions::Operand16::HL
                ),
                0x0056
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::SP,
                    instructions::Operand16::IX
                ),
                0x0057
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::LD16(
                    instructions::Operand16::SP,
                    instructions::Operand16::IY
                ),
                0x0059
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::PUSH(instructions::Operand16::AF),
                0x005B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::PUSH(instructions::Operand16::IX),
                0x005C
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::PUSH(instructions::Operand16::IY),
                0x005E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::POP(instructions::Operand16::BC),
                0x0060
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::POP(instructions::Operand16::IX),
                0x0061
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::POP(instructions::Operand16::IY),
                0x0063
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::EX(
                    instructions::Operand16::DE,
                    instructions::Operand16::HL
                ),
                0x0065
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::EX(
                    instructions::Operand16::AF,
                    instructions::Operand16::AFprime
                ),
                0x0066
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::EXX, 0x0067)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::EX(
                    instructions::Operand16::DerefSP,
                    instructions::Operand16::HL
                ),
                0x0068
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::EX(
                    instructions::Operand16::DerefSP,
                    instructions::Operand16::IX
                ),
                0x0069
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::EX(
                    instructions::Operand16::DerefSP,
                    instructions::Operand16::IY
                ),
                0x006B
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::LDI, 0x006D)));
        assert_eq!(iter.next(), Some((instructions::Instruction::LDIR, 0x006F)));
        assert_eq!(iter.next(), Some((instructions::Instruction::LDD, 0x0071)));
        assert_eq!(iter.next(), Some((instructions::Instruction::LDDR, 0x0073)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPI, 0x0075)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPIR, 0x0077)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPD, 0x0079)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPDR, 0x007B)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD(instructions::Operand8::B),
                0x007D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADC(instructions::Operand8::Immediate(46)),
                0x007E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SUB(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x0080
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SBC(instructions::Operand8::Deref(
                    instructions::Operand16::IXOffset(9)
                )),
                0x0081
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::AND(instructions::Operand8::Deref(
                    instructions::Operand16::IYOffset(10)
                )),
                0x0084
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::OR(instructions::Operand8::C),
                0x0087
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::XOR(instructions::Operand8::Immediate(47)),
                0x0088
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::CP(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x008A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC(instructions::Operand8::D),
                0x008B
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x008C
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC(instructions::Operand8::Deref(
                    instructions::Operand16::IXOffset(11)
                )),
                0x008D
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC(instructions::Operand8::Deref(
                    instructions::Operand16::IYOffset(12)
                )),
                0x0090
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::DAA, 0x0093)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CPL, 0x0094)));
        assert_eq!(iter.next(), Some((instructions::Instruction::NEG, 0x0095)));
        assert_eq!(iter.next(), Some((instructions::Instruction::CCF, 0x0097)));
        assert_eq!(iter.next(), Some((instructions::Instruction::SCF, 0x0098)));
        assert_eq!(iter.next(), Some((instructions::Instruction::NOP, 0x0099)));
        assert_eq!(iter.next(), Some((instructions::Instruction::HALT, 0x009A)));
        assert_eq!(iter.next(), Some((instructions::Instruction::DI, 0x009B)));
        assert_eq!(iter.next(), Some((instructions::Instruction::EI, 0x009C)));
        assert_eq!(iter.next(), Some((instructions::Instruction::IM0, 0x009D)));
        assert_eq!(iter.next(), Some((instructions::Instruction::IM1, 0x009F)));
        assert_eq!(iter.next(), Some((instructions::Instruction::IM2, 0x00A1)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD16(
                    instructions::Operand16::HL,
                    instructions::Operand16::SP
                ),
                0x00A3
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADC16(instructions::Operand16::BC),
                0x00A4
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SBC16(instructions::Operand16::DE),
                0x00A6
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD16(
                    instructions::Operand16::IX,
                    instructions::Operand16::IX
                ),
                0x00A8
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::ADD16(
                    instructions::Operand16::IY,
                    instructions::Operand16::SP
                ),
                0x00AA
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC16(instructions::Operand16::HL),
                0x00AC
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC16(instructions::Operand16::IX),
                0x00AD
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::INC16(instructions::Operand16::IY),
                0x00AF
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC16(instructions::Operand16::BC),
                0x00B1
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC16(instructions::Operand16::IX),
                0x00B2
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::DEC16(instructions::Operand16::IY),
                0x00B4
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RLCA, 0x00B6)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RLA, 0x00B7)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RRCA, 0x00B8)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RRA, 0x00B9)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RLC(instructions::Operand8::H),
                0x00BA
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RL(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x00BC
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RRC(instructions::Operand8::Deref(
                    instructions::Operand16::IXOffset(13)
                )),
                0x00BE
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RR(instructions::Operand8::Deref(
                    instructions::Operand16::IYOffset(14)
                )),
                0x00C2
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SLA(instructions::Operand8::L),
                0x00C6
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SRA(instructions::Operand8::Deref(
                    instructions::Operand16::HL
                )),
                0x00C8
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SRL(instructions::Operand8::Deref(
                    instructions::Operand16::IXOffset(15)
                )),
                0x00CA
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RLD, 0x00CE)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RRD, 0x00D0)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::BIT(0, instructions::Operand8::A),
                0x00D2
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::BIT(
                    1,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x00D4
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::BIT(
                    2,
                    instructions::Operand8::Deref(instructions::Operand16::IXOffset(16))
                ),
                0x00D6
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::BIT(
                    3,
                    instructions::Operand8::Deref(instructions::Operand16::IYOffset(17))
                ),
                0x00DA
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SET(4, instructions::Operand8::B),
                0x00DE
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RES(
                    5,
                    instructions::Operand8::Deref(instructions::Operand16::HL)
                ),
                0x00E0
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::SET(
                    6,
                    instructions::Operand8::Deref(instructions::Operand16::IXOffset(18))
                ),
                0x00E2
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RES(
                    7,
                    instructions::Operand8::Deref(instructions::Operand16::IYOffset(19))
                ),
                0x00E6
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::Immediate(0xDEB7)),
                0x00EA
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JPConditional(instructions::Condition::NZ, 0xDEB8),
                0x00ED
            ))
        );
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::JR(20), 0x00F0))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JRConditional(instructions::Condition::C, 21),
                0x00F2
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JRConditional(instructions::Condition::NC, 22),
                0x00F4
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JRConditional(instructions::Condition::Z, 23),
                0x00F6
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JRConditional(instructions::Condition::NZ, 24),
                0x00F8
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::HL),
                0x00FA
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::IX),
                0x00FB
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::JP(instructions::Operand16::IY),
                0x00FD
            ))
        );
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::DJNZ(25), 0x00FF))
        );
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::CALL(0xDEB9), 0x0101))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::CALLConditional(instructions::Condition::Z, 0xDEBA),
                0x0104
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RET, 0x0107)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::RETConditional(instructions::Condition::NC),
                0x0108
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::RETI, 0x0109)));
        assert_eq!(iter.next(), Some((instructions::Instruction::RETN, 0x010B)));
        assert_eq!(
            iter.next(),
            Some((instructions::Instruction::RST(0x28), 0x010D))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::IN(
                    instructions::Operand8::A,
                    instructions::Operand8::Immediate(47)
                ),
                0x010E
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::IN(instructions::Operand8::D, instructions::Operand8::C),
                0x0110
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::INI, 0x0112)));
        assert_eq!(iter.next(), Some((instructions::Instruction::INIR, 0x0114)));
        assert_eq!(iter.next(), Some((instructions::Instruction::IND, 0x0116)));
        assert_eq!(iter.next(), Some((instructions::Instruction::INDR, 0x0118)));
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::OUT(
                    instructions::Operand8::Immediate(48),
                    instructions::Operand8::A
                ),
                0x011A
            ))
        );
        assert_eq!(
            iter.next(),
            Some((
                instructions::Instruction::OUT(
                    instructions::Operand8::C,
                    instructions::Operand8::E
                ),
                0x011C
            ))
        );
        assert_eq!(iter.next(), Some((instructions::Instruction::OUTI, 0x011E)));
        assert_eq!(iter.next(), Some((instructions::Instruction::OTIR, 0x0120)));
        assert_eq!(iter.next(), Some((instructions::Instruction::OUTD, 0x0122)));
        assert_eq!(iter.next(), Some((instructions::Instruction::OTDR, 0x0124)));
        assert_eq!(iter.next(), None);
    }
}
