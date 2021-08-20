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
    fn from_alt_two_bit(size: u8) -> Self {
        match size {
            0 => unreachable!(), // This is only used in the case of 0 being invalid for some other reason
            1 => M68KSize::Byte,
            2 => M68KSize::Long,
            3 => M68KSize::Word,
            _ => unreachable!(),
        }
    }
}

enum M68KEffectiveAddress {
    D(u8),
    A(u8),
    Mem(u32),
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

    fn ea_addr(&mut self, size: M68KSize, mode: u8, reg: u8) -> M68KEffectiveAddress {
        match mode {
            0 => {
                println!("EA address resolution: D{}", reg);
                M68KEffectiveAddress::D(reg)
            }
            1 => {
                println!("EA address resolution: A{}", reg);
                M68KEffectiveAddress::A(reg)
            }
            2 => {
                if reg == 7 {
                    todo!();
                }
                let (_, value) = self.read(self.regs.a[reg as usize] as usize);
                self.temp = value;
                let (_, value) = self.read((self.regs.a[reg as usize] + 2) as usize);
                let addr = (self.temp as u32) << 16 | (value as u32);
                println!("EA address resolution: (A{}) (${:06X})", reg, addr);
                M68KEffectiveAddress::Mem(addr)
            }
            7 => match reg {
                2 => {
                    let pc = self.regs.pc; // PC is saved here, before reading displacement
                    let (_, value) = self.read(self.regs.pc as usize);
                    self.regs.pc += 2;
                    let addr = pc.wrapping_add(sign_extend(M68KSize::Word, value as u32));
                    println!(
                        "EA address resolution: (${:04X}, PC) (${:06X})",
                        value, addr
                    );
                    M68KEffectiveAddress::Mem(addr)
                }
                _ => todo!(),
            },
            _ => todo!(),
        }
    }

    fn ea_val(&mut self, size: M68KSize, mode: u8, reg: u8) -> u32 {
        match mode {
            0 => {
                let value = match size {
                    M68KSize::Byte => self.regs.d[reg as usize] & 0x00FF,
                    M68KSize::Word => self.regs.d[reg as usize] & 0xFFFF,
                    M68KSize::Long => self.regs.d[reg as usize],
                };
                println!("EA read-only resolution: D{} = ${:08X}", reg, value);
                value
            }
            2 => {
                if reg == 7 {
                    todo!();
                }
                let addr = self.regs.a[reg as usize];
                match size {
                    M68KSize::Word => {
                        let (_, value) = self.read(addr as usize);
                        println!(
                            "EA read-only resolution: (A{}) (${:06X}) = ${:04X}",
                            reg, addr, value
                        );
                        value as u32
                    }
                    _ => todo!(),
                }
            }
            3 => {
                if reg == 7 {
                    todo!();
                }
                match size {
                    M68KSize::Byte => {
                        let addr = self.regs.a[reg as usize];
                        self.regs.a[reg as usize] += 1;
                        let (_, data) = self.read((addr & 0xFFFFFE) as usize);
                        if addr & 0x000001 != 0 {
                            // Low byte, high address
                            println!(
                                "EA read-only resolution: (A{})+ (${:06X}) = $({:02X}){:02X}",
                                reg,
                                addr,
                                (data & 0xFF00) >> 8,
                                data & 0x00FF
                            );
                            (data & 0x00FF) as u32
                        } else {
                            // High byte, low address
                            println!(
                                "EA read-only resolution: (A{})+ (${:06X}) = ${:02X}({:02X})",
                                reg,
                                addr,
                                (data & 0xFF00) >> 8,
                                data & 0x00FF
                            );
                            ((data & 0xFF00) >> 8) as u32
                        }
                    }
                    M68KSize::Word => {
                        let addr = self.regs.a[reg as usize];
                        self.regs.a[reg as usize] += 2;
                        let (_, value) = self.read(addr as usize);
                        println!(
                            "EA read-only resolution: (A{})+ (${:06X}) = ${:04X}",
                            reg, addr, value
                        );
                        value as u32
                    }
                    M68KSize::Long => {
                        let addr = self.regs.a[reg as usize];
                        self.regs.a[reg as usize] += 4;
                        let (_, value) = self.read(addr as usize);
                        self.temp = value;
                        let (_, value) = self.read((addr + 2) as usize);
                        let data = (self.temp as u32) << 16 | value as u32;
                        println!(
                            "EA read-only resolution: (A{})+ (${:06X}) = ${:08X}",
                            reg, addr, data
                        );
                        data
                    }
                }
            }
            5 => {
                if reg == 7 {
                    todo!(); // A7 is hard
                }
                let (_, value) = self.read(self.regs.pc as usize);
                self.regs.pc += 2;
                let addr = self.regs.a[reg as usize]
                    .wrapping_add(sign_extend(M68KSize::Word, value as u32));
                self.stall += 2;
                match size {
                    M68KSize::Byte => {
                        let (_, data) = self.read((addr & 0xFFFFFE) as usize);
                        if addr & 0x000001 != 0 {
                            // Low byte, high address
                            println!(
                                "EA read-only resolution: (A{}, ${:04X}) (${:06X}) = $({:02X}){:02X}",
                                reg,
                                value as i32,
                                addr,
                                (data & 0xFF00) >> 8,
                                data & 0x00FF
                            );
                            (data & 0x00FF) as u32
                        } else {
                            // High byte, low address
                            println!(
                                "EA read-only resolution: (A{}, ${:04X}) (${:06X}) = ${:02X}({:02X})",
                                reg,
                                value as i32,
                                addr,
                                (data & 0xFF00) >> 8,
                                data & 0x00FF
                            );
                            ((data & 0xFF00) >> 8) as u32
                        }
                    }
                    _ => todo!(),
                }
            }
            7 => match reg {
                1 => {
                    let (_, value) = self.read(self.regs.pc as usize);
                    self.temp = value;
                    self.regs.pc += 2;
                    let (_, value) = self.read(self.regs.pc as usize);
                    let addr = ((self.temp as u32) << 16 | value as u32) & 0x00FFFFFF;
                    self.regs.pc += 2;
                    match size {
                        M68KSize::Byte => {
                            // This gets fun.
                            let (_, data) = self.read((addr & 0xFFFFFE) as usize); // Get both bytes. Welcome to the system.
                            if addr & 0x000001 != 0 {
                                // Low byte, high address
                                println!(
                                    "EA read-only resolution: ${:06X}.L = $({:02X}){:02X}",
                                    addr,
                                    (data & 0xFF00) >> 8,
                                    data & 0x00FF
                                );
                                (data & 0x00FF) as u32
                            } else {
                                // High byte, low address
                                println!(
                                    "EA read-only resolution: ${:06X}.L = ${:02X}({:02X})",
                                    addr,
                                    (data & 0xFF00) >> 8,
                                    data & 0x00FF
                                );
                                ((data & 0xFF00) >> 8) as u32
                            }
                        }
                        M68KSize::Word => {
                            let (_, data) = self.read(addr as usize);
                            println!("EA read-only resolution: ${:06X}.L = ${:04X}", addr, data); // TODO: Make this use a debug flag
                            data as u32
                        }
                        M68KSize::Long => {
                            let (_, value) = self.read(addr as usize);
                            self.temp = value;
                            let (_, value) = self.read((addr + 2) as usize);
                            let data = (self.temp as u32) << 16 | value as u32;
                            println!("EA read-only resolution: ${:06X}.L = ${:08X}", addr, data);
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

    fn write(&mut self, addr: usize, data: u16) -> Option<M68KFault> {
        unsafe {
            // SAFETY: Assume we're fine.
            let (fault, _) = self
                .bus
                .borrow()
                .write(addr, &[((data & 0xFF00) >> 8) as u8, (data & 0xFF) as u8]);
            self.stall += 4; // Each write takes 4 cycles. No matter what.
            fault
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
                        0x0000 => match inst & 0x0FFF {
                            x if x & 0x0F00 == 0x0200 => {
                                let size = M68KSize::from_two_bit(((inst & 0x00C0) >> 6) as u8);
                                let mode = (inst & 0x0038) >> 3;
                                let reg = inst & 0x0007;
                                let imm = match size {
                                    M68KSize::Long => todo!(), // More work
                                    _ => {
                                        let (_, imm) = self.read(self.regs.pc as usize);
                                        self.regs.pc += 2;
                                        imm
                                    }
                                };
                                let addr = self.ea_addr(size, mode as u8, reg as u8);
                                match addr {
                                    M68KEffectiveAddress::D(x) => {
                                        println!("ANDI #{:08X}, D{}", imm, x);
                                        self.regs.d[x as usize] &= imm as u32;
                                        self.regs.sr &= 0xFFFC;
                                        println!("C flag cleared");
                                        println!("V flag cleared");
                                        if self.regs.d[x as usize] == 0 {
                                            self.regs.sr |= 0x4;
                                            println!("Z flag set");
                                        } else {
                                            self.regs.sr &= 0xFFFB;
                                            println!("Z flag cleared");
                                        }
                                        match size {
                                            M68KSize::Byte => {
                                                if self.regs.d[x as usize] & 0x80 != 0 {
                                                    self.regs.sr |= 0x8;
                                                    println!("N flag set");
                                                } else {
                                                    self.regs.sr &= 0xFFF7;
                                                    println!("N flag cleared");
                                                }
                                            }
                                            _ => todo!(),
                                        }
                                    }
                                    _ => todo!(),
                                }
                            }
                            x if x & 0x0800 == 0x0800 => {
                                let mode = (inst & 0x0038) >> 3;
                                let reg = inst & 0x0007;
                                let (_, shift) = self.read(self.regs.pc as usize);
                                self.regs.pc += 2;
                                let (value, shift) = if mode <= 1 {
                                    // Dn or An
                                    self.stall += 2; // Apparently something has to happen internally
                                    (
                                        self.ea_val(M68KSize::Long, mode as u8, reg as u8),
                                        shift & 0x1F,
                                    )
                                } else {
                                    // Memory operand
                                    (
                                        self.ea_val(M68KSize::Byte, mode as u8, reg as u8),
                                        shift & 0x07,
                                    )
                                };
                                println!("BTST #{}, #${:02X}", shift, value); // TODO: Correctly print 32-bit value of Dn / An
                                if value & (1 << shift) == 0 {
                                    self.regs.sr |= 0x4;
                                    println!("Z flag set");
                                } else {
                                    self.regs.sr &= 0xFFFB;
                                    println!("Z flag cleared");
                                }
                            }
                            _ => todo!(),
                        },
                        x if x & 0xC000 == 0 => {
                            // 0x1000 - 0x3000 (and 0x0000, but it's covered by the above case)
                            let size = M68KSize::from_alt_two_bit(((inst & 0x3000) >> 12) as u8);
                            let dest_reg = (inst & 0x0E00) >> 9;
                            let dest_mode = (inst & 0x01C0) >> 6;
                            let src_mode = (inst & 0x0038) >> 3;
                            let src_reg = inst & 0x0007;
                            // TODO: I currently assume src is resolved first; I don't have proof of this.
                            let src = self.ea_val(size, src_mode as u8, src_reg as u8);
                            let dest = self.ea_addr(size, dest_mode as u8, dest_reg as u8);
                            match dest {
                                M68KEffectiveAddress::D(x) => {
                                    match size {
                                        M68KSize::Byte => {
                                            println!("MOVE.B #${:02X}, D{}", src, x);
                                            self.regs.d[x as usize] &= 0xFFFFFF00;
                                        }
                                        M68KSize::Word => {
                                            println!("MOVE.W #${:04X}, D{}", src, x);
                                            self.regs.d[x as usize] &= 0xFFFF0000;
                                        }
                                        M68KSize::Long => {
                                            println!("MOVE.L #${:08X}, D{}", src, x);
                                            self.regs.d[x as usize] = 0;
                                        }
                                    }
                                    self.regs.d[x as usize] |= src;
                                }
                                M68KEffectiveAddress::A(x) => {
                                    if x == 7 {
                                        todo!();
                                    }
                                    match size {
                                        M68KSize::Byte => {
                                            println!("MOVE.B #${:02X}, A{}", src, x);
                                            self.regs.a[x as usize] &= 0xFFFFFF00;
                                        }
                                        M68KSize::Word => {
                                            println!("MOVE.W #${:04X}, A{}", src, x);
                                            self.regs.a[x as usize] &= 0xFFFF0000;
                                        }
                                        M68KSize::Long => {
                                            println!("MOVE.L #${:08X}, A{}", src, x);
                                            self.regs.a[x as usize] = 0;
                                        }
                                    }
                                    self.regs.a[x as usize] |= src;
                                }
                                M68KEffectiveAddress::Mem(x) => match size {
                                    M68KSize::Word => {
                                        println!("MOVE.W #${:04X}, ${:06X}", src, x);
                                        self.write(x as usize, src as u16);
                                    }
                                    M68KSize::Long => {
                                        println!("MOVE.L #${:08X}, ${:06X}", src, x);
                                        self.write(((x & 0xFF00) >> 8) as usize, src as u16);
                                        self.write((x & 0x00FF) as usize, (src + 2) as u16);
                                    }
                                    _ => todo!(),
                                },
                            }
                        }
                        0x4000 => match inst & 0x0FFF {
                            0x0AFC => todo!(),
                            x if x & 0x0FC0 == 0x0AC0 => todo!(),
                            x if x & 0x0F00 == 0x0A00 => {
                                let size = M68KSize::from_two_bit(((inst & 0x00C0) >> 6) as u8);
                                let mode = (inst & 0x0038) >> 3;
                                let reg = inst & 0x0007;
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
                            x if x & 0x0FF0 == 0x0E60 => {
                                let reg = inst & 0x0007;
                                if inst & 0x0008 == 0 {
                                    println!("MOVE A{}, USP", reg);
                                    self.regs.usp = self.regs.a[reg as usize];
                                } else {
                                    println!("MOVE USP, A{}", reg);
                                    self.regs.a[reg as usize] = self.regs.usp;
                                }
                            }
                            x if x & 0x0F80 == 0x0C80 => {
                                let (_, mut list) = self.read(self.regs.pc as usize);
                                self.regs.pc += 2;
                                self.stall += 4; // Read cycle for some unknown reason
                                let mode = (inst & 0x0038) >> 3;
                                let reg = inst & 0x0007;
                                let list_str = "<formatting register lists is hard>";
                                if inst & 0x0040 == 0 {
                                    // Word addressing
                                    match mode {
                                        3 => {
                                            println!("MOVEM.W (A{})+, {}", reg, list_str);
                                            for i in 0..8 {
                                                // Data registers
                                                if list & 1 == 1 {
                                                    let (_, value) = self
                                                        .read(self.regs.a[reg as usize] as usize);
                                                    self.regs.d[i as usize] = value as u32;
                                                    self.regs.a[reg as usize] += 2;
                                                }
                                                list >>= 1;
                                            }
                                            for i in 0..7 {
                                                // Address registers
                                                if list & 1 == 1 {
                                                    let (_, value) = self
                                                        .read(self.regs.a[reg as usize] as usize);
                                                    self.regs.a[i as usize] = value as u32;
                                                    self.regs.a[reg as usize] += 2;
                                                }
                                                list >>= 1;
                                            }
                                            if list == 1 {
                                                todo!(); // You're trying to load into the stack pointer? Too complicated.
                                            }
                                        }
                                        _ => todo!(),
                                    }
                                } else {
                                    // Long addressing
                                    match mode {
                                        3 => {
                                            println!("MOVEM.L (A{})+, {}", reg, list_str);
                                            for i in 0..8 {
                                                // Data registers
                                                if list & 1 == 1 {
                                                    let (_, value) = self
                                                        .read(self.regs.a[reg as usize] as usize);
                                                    self.temp = value;
                                                    let (_, value) = self.read(
                                                        (self.regs.a[reg as usize] + 2) as usize,
                                                    );
                                                    let value =
                                                        (self.temp as u32) << 16 | value as u32;
                                                    self.regs.a[reg as usize] += 4;
                                                    self.regs.d[i as usize] = value as u32;
                                                }
                                                list >>= 1;
                                            }
                                            for i in 0..7 {
                                                // Address registers
                                                if list & 1 == 1 {
                                                    let (_, value) = self
                                                        .read(self.regs.a[reg as usize] as usize);
                                                    self.temp = value;
                                                    let (_, value) = self.read(
                                                        (self.regs.a[reg as usize] + 2) as usize,
                                                    );
                                                    let value =
                                                        (self.temp as u32) << 16 | value as u32;
                                                    self.regs.a[reg as usize] += 4;
                                                    self.regs.a[i as usize] = value as u32;
                                                }
                                                list >>= 1;
                                            }
                                            if list == 1 {
                                                todo!(); // You're trying to load into the stack pointer? Too complicated.
                                            }
                                        }
                                        _ => todo!(),
                                    }
                                }
                            }
                            x if x & 0x01C0 == 0x01C0 => {
                                let dest = (inst & 0x0E00) >> 9;
                                let mode = (inst & 0x0038) >> 3;
                                let reg = inst & 0x0007;
                                let ea = self.ea_addr(M68KSize::Byte, mode as u8, reg as u8);
                                if let M68KEffectiveAddress::Mem(ea) = ea {
                                    println!("LEA ${:06X}, A{}", ea, dest);
                                    if dest == 7 {
                                        // A7 is weird
                                        todo!();
                                    } else {
                                        self.regs.a[dest as usize] = ea;
                                    }
                                } else {
                                    panic!("Insert fault here"); // Should be an illegal instruction fault tbh
                                }
                            }
                            _ => todo!(),
                        },
                        0x5000 => match inst & 0x0FFF {
                            x if x & 0x00F8 == 0x00C8 => {
                                let cond = (inst & 0x0F00) >> 8;
                                let (mut passed, cc) = match cond {
                                    1 => (false, "F"), // DBF
                                    _ => todo!(),
                                };
                                let reg = inst & 0x0003;
                                let (_, disp) = self.read(self.regs.pc as usize);
                                let disp = sign_extend(M68KSize::Word, disp as u32);
                                let addr = self.regs.pc.wrapping_add(disp) & 0x00FFFFFF;
                                self.regs.pc += 2;
                                println!("DB{} D{}, ${:06X}", cc, reg, addr);
                                if !passed {
                                    self.stall += 2;
                                    passed = self.regs.d[reg as usize] == 0; // If it's zero, we don't want to branch.
                                    self.regs.d[reg as usize] =
                                        self.regs.d[reg as usize].wrapping_sub(1);
                                }
                                if !passed {
                                    // So, funnily enough, we only branch if the condition *hasn't* passed.
                                    self.stall += 4;
                                    println!("Condition passed");
                                    self.regs.pc = addr;
                                } else {
                                    self.stall += 8; // Pipeline miss?
                                }
                            }
                            _ => todo!(),
                        },
                        0x6000 => {
                            let cond = (inst & 0x0F00) >> 8;
                            if cond == 0x1 {
                                todo!();
                            } else {
                                let (passed, cc) = match cond {
                                    0 => (true, "RA"),                    // BRA
                                    1 => unreachable!(), // BSR; covered separately above
                                    6 => (self.regs.sr & 0x4 == 0, "NE"), // BNE
                                    7 => (self.regs.sr & 0x4 != 0, "EQ"), // BEQ
                                    _ => todo!(),
                                };
                                let disp = sign_extend(M68KSize::Byte, (inst & 0x00FF) as u32);
                                if disp == 0 || disp == 0xFFFF {
                                    todo!();
                                }
                                let dest = self.regs.pc.wrapping_add(disp) & 0x00FFFFFF;
                                println!("B{} ${:06X}", cc, dest);
                                if passed {
                                    println!("Condition passed");
                                    self.stall += 2;
                                    self.regs.pc = dest;
                                }
                            }
                        }
                        0x7000 => {
                            if inst & 0x0100 != 0 {
                                panic!("Illegal instruction");
                            }
                            let reg = (inst & 0x0E00) >> 9;
                            let data = sign_extend(M68KSize::Byte, (inst & 0x00FF) as u32);
                            println!("MOVEQ #${:08X}, D{}", data, reg);
                            self.regs.d[reg as usize] = data;
                        }
                        0xD000 => match inst & 0x0FFF {
                            x if x & 0x00C0 == 0x00C0 => todo!(),
                            x if x & 0x0130 == 0x0100 => todo!(),
                            _ => {
                                if inst & 0x0100 == 0 {
                                    let size = M68KSize::from_two_bit(((inst & 0x01C0) >> 6) as u8);
                                    let mode = (inst & 0x0038) >> 3;
                                    let reg = inst & 0x0007;
                                    let breg = (inst & 0x0E00) >> 9;
                                    let a = self.ea_val(size, mode as u8, reg as u8);
                                    let b = self.regs.d[breg as usize];
                                    let (result, overflow, carry) = match size {
                                        M68KSize::Word => {
                                            println!("ADD.W #${:04X}, D{}", a, breg);
                                            let result = a as u32 + b as u32;
                                            // Credit where credit is due to StackOverflow:
                                            let overflow =
                                                ((!(a ^ b)) & (a ^ result) & 0x8000) != 0;
                                            let carry = result > 0xFFFF;
                                            self.regs.d[breg as usize] &= 0xFFFF0000;
                                            self.regs.d[breg as usize] |= result;
                                            (result & 0x0000FFFF, overflow, carry)
                                        }
                                        _ => todo!(),
                                    };
                                    if carry {
                                        println!("C flag set");
                                        self.regs.sr |= 0x0001;
                                    } else {
                                        println!("C flag cleared");
                                        self.regs.sr &= 0xFFFE;
                                    }
                                    if overflow {
                                        println!("V flag set");
                                        self.regs.sr |= 0x0002;
                                    } else {
                                        println!("V flag cleared");
                                        self.regs.sr &= 0xFFFD;
                                    }
                                    if result == 0 {
                                        println!("Z flag set");
                                        self.regs.sr |= 0x0004;
                                    } else {
                                        println!("Z flag cleared");
                                        self.regs.sr &= 0xFFFB;
                                    }
                                    if sign_extend(size, result) & 0x80000000 != 0 {
                                        println!("N flag set");
                                        self.regs.sr |= 0x0008;
                                    } else {
                                        println!("N flag cleared");
                                        self.regs.sr &= 0xFFF7;
                                    }
                                    if carry {
                                        println!("X flag set");
                                        self.regs.sr |= 0x0010;
                                    } else {
                                        println!("X flag cleared");
                                        self.regs.sr &= 0xFFEF;
                                    }
                                } else {
                                    todo!();
                                }
                            }
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
