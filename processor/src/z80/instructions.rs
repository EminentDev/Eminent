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
#[derive(Debug, PartialEq)]
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
}
