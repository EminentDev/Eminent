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
