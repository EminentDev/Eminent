//! A model for the instructions of the Zilog z80 CPU.
//!
//! Note that not all instructions allowed to be built with this model are
//! legal, i.e. have a binary representation, and one that would be understood
//! by an actual z80 CPU. However, the opposite is true: all legal instructions
//! can be represented with this model.

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
    I,
    R,
    /// This variant is specific to this implementation of the decoding process
    OneOneZero
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
    Deref(u16)
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
    M
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
    UnfinishedOpcode
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
        _ => unreachable!()
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
        _ => unreachable!()
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
        _ => unreachable!()
    }
}

/// Try to decode an instruction from a single byte.
///
/// Return `Some(Instruction)` if sucessful, otherwise `None`. In the latter
/// case, more bytes may be needed.
pub fn decode1byte(opcode: u8) -> Option<Instruction> {
    match opcode {
        // Non-variable prefixes for multi-byte instructions
        0x10 | 0x18 | 0x20 | 0x22 | 0x28 | 0x2A | 0x30 | 0x32 | 0x36 | 0x38
             | 0x3A | 0xC3 | 0xCB | 0xCD | 0xD3 | 0xDB | 0xDD | 0xED | 0xFD =>
             None,

        // Non-variable 1-byte instructions
        0x0A =>
            Some(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::BC))),
        0x1A =>
            Some(Instruction::LD(Operand8::A, Operand8::Deref(Operand16::DE))),
        0x02 =>
            Some(Instruction::LD(Operand8::Deref(Operand16::BC), Operand8::A)),
        0x12 =>
            Some(Instruction::LD(Operand8::Deref(Operand16::DE), Operand8::A)),
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
                        Some(Instruction::ADD16(Operand16::HL,
                                                map_register16(opcode >> 4)))
                    }
                }
                0b011 => {
                    if opcode & 0b1000 == 0 {
                        Some(Instruction::INC16(map_register16(opcode >> 4)))
                    } else {
                        Some(Instruction::DEC16(map_register16(opcode >> 4)))
                    }
                }
                0b100 =>
                    Some(Instruction::INC(match map_register8(opcode >> 3) {
                        Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                        r => r
                    })),
                0b101 =>
                    Some(Instruction::DEC(match map_register8(opcode >> 3) {
                        Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                        r => r
                    })),
                0b110 => None, // LD r, n => multi-byte
                _ =>
                    //Some(Instruction::IllegalOpcode)
                    // From tests::decode_all_bytes, we know this is
                    // unreachable.
                    unreachable!()
            }
        }

        // 01 xxx xxx
        x if x >> 6 == 0b01 => {
            match (map_register8(opcode >> 3), map_register8(opcode)) {
                // 110 110 => HALT, already matched as constant 0x76
                (Operand8::OneOneZero, r) =>
                    Some(Instruction::LD(Operand8::Deref(Operand16::HL), r)),
                (r, Operand8::OneOneZero) =>
                    Some(Instruction::LD(r, Operand8::Deref(Operand16::HL))),
                (r1, r2) => Some(Instruction::LD(r1, r2))
            }
        }

        // 10 xxx xxx
        x if x >> 6 == 0b10 => {
            let r = match map_register8(opcode) {
                Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                r => r
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
                _ => unreachable!()
            }
        }

        // 11 xxx xxx
        _ => {
            match opcode & 0b111 {
                0b010 | 0b011 | 0b100 | 0b110 => None, // Multi-byte instruction
                0b000 =>
                    Some(Instruction::RETConditional(
                            map_condition(opcode >> 3))),
                0b001 | 0b101 => {
                    let qq = match map_register16(opcode >> 4) {
                        Operand16::SP => Operand16::AF,
                        r => r
                    };
                    if opcode & 0b1000 == 0 {
                        if opcode & 0b100 == 0 {
                            Some(Instruction::POP(qq))
                        } else {
                            Some(Instruction::PUSH(qq))
                        }
                    } else {
                        //Some(Instruction::IllegalOpcode)
                        // From tests::decode_all_bytes, we know this is
                        // unreachable.
                        unreachable!()
                    }
                }
                0b111 => Some(Instruction::RST(opcode & 0b00_111_000)),
                _ => unreachable!()
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

                x if x & 0b11_001_111 == 0b01_001_001 =>
                    Some(Instruction::ADD16(ir.clone(),
                                            match map_register16(byte2 >> 4) {
                                                Operand16::HL => ir,
                                                r => r
                                            })),

                // LD r, (ir+d) => 3 bytes
                x if x & 0b11_000_111 == 0b01_000_110 => None,
                // LD (ir+d), r => 3 bytes
                x if x & 0b11_111_000 == 0b01_110_000 => None,
                // <arithmetic or logic operation> A, (ir+d) => 3 bytes
                x if x & 0b11_000_111 == 0b10_000_110 => None,
                // <INC|DEC> (ir+d) => 3 bytes
                x if x & 0b11_111_110 == 0b00_110_100 => None,

                _ => Some(Instruction::IllegalOpcode)
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

                _ => Some(Instruction::IllegalOpcode)
            }
        }
        0xCB => {
            let r = match map_register8(byte2) {
                Operand8::OneOneZero => Operand8::Deref(Operand16::HL),
                r => r
            };
            match byte2 >> 6 {
                0b00 => {
                    match byte2 >> 3 {
                        0b000 => Some(Instruction::RLC(r)),
                        0b001 => Some(Instruction::RRC(r)),
                        0b010 => Some(Instruction::RL(r)),
                        0b011 => Some(Instruction::RR(r)),
                        0b100 => Some(Instruction::SLA(r)),
                        0b101 => Some(Instruction::SRA(r)),
                        0b110 => Some(Instruction::IllegalOpcode),
                        0b111 => Some(Instruction::SRL(r)),
                        _ => unreachable!()
                    }
                }
                0b01 => Some(Instruction::BIT((byte2 >> 3) & 0b111, r)),
                0b11 => Some(Instruction::SET((byte2 >> 3) & 0b111, r)),
                0b10 => Some(Instruction::RES((byte2 >> 3) & 0b111, r)),
                _ => unreachable!()
            }
        }

        0x36 =>
            Some(Instruction::LD(Operand8::Deref(Operand16::HL),
                                 Operand8::Immediate(byte2))),
        0x18 => Some(Instruction::JR(i8::from_be_bytes([byte2]))),
        0x38 => Some(Instruction::JRConditional(Condition::C,
                                                i8::from_be_bytes([byte2]))),
        0x30 => Some(Instruction::JRConditional(Condition::NC,
                                                i8::from_be_bytes([byte2]))),
        0x28 => Some(Instruction::JRConditional(Condition::Z,
                                                i8::from_be_bytes([byte2]))),
        0x20 => Some(Instruction::JRConditional(Condition::NZ,
                                                i8::from_be_bytes([byte2]))),
        0x10 => Some(Instruction::DJNZ(i8::from_be_bytes([byte2]))),
        0xDB => Some(Instruction::IN(Operand8::A, Operand8::Immediate(byte2))),
        0xD3 => Some(Instruction::OUT(Operand8::Immediate(byte2), Operand8::A)),

        0x22 | 0x2A | 0x32 | 0x3A | 0xC3 | 0xCD => None, // 3 bytes

        x if x & 0b11_000_111 == 0b00_000_110 =>
            Some(Instruction::LD(map_register8(byte1 >> 3),
                                 Operand8::Immediate(byte2))),
        x if x & 0b11_000_111 == 0b11_000_110 => {
            match (byte1 >> 3) & 0b111 {
                0b000 => Some(Instruction::ADD(Operand8::Immediate(byte2))),
                0b001 => Some(Instruction::ADC(Operand8::Immediate(byte2))),
                0b010 => Some(Instruction::SUB(Operand8::Immediate(byte2))),
                0b011 => Some(Instruction::SBC(Operand8::Immediate(byte2))),
                0b100 => Some(Instruction::AND(Operand8::Immediate(byte2))),
                0b101 => Some(Instruction::XOR(Operand8::Immediate(byte2))),
                0b110 => Some(Instruction::OR(Operand8::Immediate(byte2))),
                0b111 => Some(Instruction::CP(Operand8::Immediate(byte2))),
                _ => unreachable!()
            }
        }

        x if x & 0b11_001_111 == 0b00_000_001 => None, // LD dd, nn => 3 bytes
        x if x & 0b11_000_111 == 0b11_000_010 => None, // JP cc, nn => 3 bytes
        x if x & 0b11_000_111 == 0b11_000_100 => None, // CALL cc, nn => 3 bytes
        _ =>
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_2bytes, we know this is
            // unreachable.
            unreachable!()
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

                x if x & 0b11_000_111 == 0b01_000_110 =>
                    Some(Instruction::LD(map_register8(byte2 >> 3), iroffset)),
                x if x & 0b11_111_000 == 0b01_110_000 =>
                    Some(Instruction::LD(iroffset, map_register8(byte2))),
                x if x & 0b11_000_111 == 0b10_000_110 => {
                    match (byte2 >> 3) & 0b111 {
                        0b000 => Some(Instruction::ADD(iroffset)),
                        0b001 => Some(Instruction::ADC(iroffset)),
                        0b010 => Some(Instruction::SUB(iroffset)),
                        0b011 => Some(Instruction::SBC(iroffset)),
                        0b100 => Some(Instruction::AND(iroffset)),
                        0b101 => Some(Instruction::XOR(iroffset)),
                        0b110 => Some(Instruction::OR(iroffset)),
                        0b111 => Some(Instruction::CP(iroffset)),
                        _ => unreachable!()
                    }
                }

                _ => Some(Instruction::IllegalOpcode)
            }
        }

        0x3A =>
            Some(Instruction::LD(Operand8::A, Operand8::Deref(nnimmediate))),
        0x32 =>
            Some(Instruction::LD(Operand8::Deref(nnimmediate), Operand8::A)),
        0x2A => Some(Instruction::LD16(Operand16::HL, Operand16::Deref(nn))),
        0x22 => Some(Instruction::LD16(Operand16::Deref(nn), Operand16::HL)),
        0xC3 => Some(Instruction::JP(Operand16::Immediate(nn))),
        0xCD => Some(Instruction::CALL(nn)),

        0xED => None, // 4 bytes, assuming requirements have been followed

        x if x & 0b11_001_111 == 0b00_000_001 =>
            Some(Instruction::LD16(map_register16(byte1 >> 4), nnimmediate)),
        x if x & 0b11_000_111 == 0b11_000_010 =>
            Some(Instruction::JPConditional(map_condition(byte1 >> 3), nn)),
        x if x & 0b11_000_111 == 0b11_000_100 =>
            Some(Instruction::CALLConditional(map_condition(byte1 >> 3), nn)),

        _ =>
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_3bytes, we know this is
            // unreachable.
            unreachable!()
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
pub fn decode4bytes(byte1: u8, byte2: u8, byte3: u8, byte4: u8)
    -> Option<Instruction> {
    let nn = u16::from_le_bytes([byte3, byte4]);
    match byte1 {
        0xDD | 0xFD => {
            let (ir, iroffset) = if byte1 == 0xDD {
                (Operand16::IX, Operand8::Deref(
                        Operand16::IXOffset(i8::from_be_bytes([byte3]))))
            } else {
                (Operand16::IY, Operand8::Deref(
                        Operand16::IYOffset(i8::from_be_bytes([byte3]))))
            };
            match byte2 {
                0x36 =>
                    Some(Instruction::LD(iroffset, Operand8::Immediate(byte4))),
                0x21 => Some(Instruction::LD16(ir, Operand16::Immediate(nn))),
                0x2A => Some(Instruction::LD16(ir, Operand16::Deref(nn))),
                0x22 => Some(Instruction::LD16(Operand16::Deref(nn), ir)),
                0xCB => {
                    if byte4 & 0b00_000_111 == 0b110 {
                        match byte4 >> 6 {
                            0b00 => {
                                match byte4 >> 3 {
                                    0b000 => Some(Instruction::RLC(iroffset)),
                                    0b001 => Some(Instruction::RRC(iroffset)),
                                    0b010 => Some(Instruction::RL(iroffset)),
                                    0b011 => Some(Instruction::RR(iroffset)),
                                    0b100 => Some(Instruction::SLA(iroffset)),
                                    0b101 => Some(Instruction::SRA(iroffset)),
                                    0b110 => Some(Instruction::IllegalOpcode),
                                    0b111 => Some(Instruction::SRL(iroffset)),
                                    _ => unreachable!()
                                }
                            }
                            0b01 => Some(Instruction::BIT((byte4 >> 3) & 0b111,
                                                          iroffset)),
                            0b10 => Some(Instruction::RES((byte4 >> 3) & 0b111,
                                                          iroffset)),
                            0b11 => Some(Instruction::SET((byte4 >> 3) & 0b111,
                                                          iroffset)),
                            _ => unreachable!()
                        }
                    } else {
                        Some(Instruction::IllegalOpcode)
                    }
                }
                _ =>
                    //Some(Instruction::IllegalOpcode)
                    // From tests::decode_all_4bytes, we know this is
                    // unreachable.
                    unreachable!()
            }
        }
        0xED => {
            if byte2 & 0b11_000_111 == 0b01_000_011 {
                if byte2 & 0b1000 == 0 {
                    Some(Instruction::LD16(Operand16::Deref(nn),
                                           map_register16(byte2 >> 4)))
                } else {
                    Some(Instruction::LD16(map_register16(byte2 >> 4),
                                           Operand16::Deref(nn)))
                }
            } else {
                //Some(Instruction::IllegalOpcode)
                // From tests::decode_all_4bytes, we know this is
                // unreachable.
                unreachable!()
            }
        }

        _ =>
            //Some(Instruction::IllegalOpcode)
            // From tests::decode_all_4bytes, we know this is
            // unreachable.
            unreachable!()
    }
}

#[cfg(test)]
mod tests {
    use crate::z80::instructions;

    #[test]
    fn decode_all_bytes() {
        for byte in 0..=u8::MAX {
            assert_ne!(instructions::decode1byte(byte),
                       Some(instructions::Instruction::IllegalOpcode),
                       "Byte {:#010b} could not be decoded", byte)
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
}
