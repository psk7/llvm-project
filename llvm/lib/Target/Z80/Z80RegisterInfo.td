//===-- Z80RegisterInfo.td - Z80 Register defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the Z80 register file
//===----------------------------------------------------------------------===//

// 8-bit General purpose register definition.
class Z80Reg<bits<16> num,
             string name,
             list<Register> subregs = [],
             list<string> altNames = []>
  : RegisterWithSubRegs<name, subregs>
{
  field bits<16> Num = num;

  let HWEncoding = num;
  let Namespace = "Z80";
  let SubRegs = subregs;
  let AltNames = altNames;
}

// Subregister indices.
let Namespace = "Z80" in
{
  def sub_lo : SubRegIndex<8>;
  def sub_hi : SubRegIndex<8, 8>;
}

let Namespace = "Z80" in {
  def ptr : RegAltNameIndex;
}


//===----------------------------------------------------------------------===//
//  8-bit general purpose registers
//===----------------------------------------------------------------------===//

def A  : Z80Reg<7,  "a">,  DwarfRegNum<[0]>;
def B  : Z80Reg<0,  "b">,  DwarfRegNum<[1]>;
def C  : Z80Reg<1,  "c">,  DwarfRegNum<[2]>;
def D  : Z80Reg<2,  "d">,  DwarfRegNum<[3]>;
def E  : Z80Reg<3,  "e">,  DwarfRegNum<[3]>;
def H  : Z80Reg<4,  "h">,  DwarfRegNum<[4]>;
def L  : Z80Reg<5,  "l">,  DwarfRegNum<[5]>;
def XH  : Z80Reg<4,  "xh">,  DwarfRegNum<[5]>;
def XL  : Z80Reg<5,  "xl">,  DwarfRegNum<[6]>;
def YH  : Z80Reg<4,  "yh">,  DwarfRegNum<[7]>;
def YL  : Z80Reg<5,  "yl">,  DwarfRegNum<[8]>;

let SubRegIndices = [sub_lo, sub_hi],
CoveredBySubRegs = 1 in
{
  // 16 bit GPR pairs.
  def HL     : Z80Reg<2, "hl",      [L, H]>, DwarfRegNum<[5]>;
  def DE     : Z80Reg<1, "de",      [E, D]>, DwarfRegNum<[3]>;
  def BC     : Z80Reg<0, "bc",      [C, B]>, DwarfRegNum<[1]>;

  def IX     : Z80Reg<2, "ix",      [XL, XH]>, DwarfRegNum<[5]>;
  def IY     : Z80Reg<2, "iy",      [YL, YH]>, DwarfRegNum<[7]>;
}

def SP     : Z80Reg<3, "sp">, DwarfRegNum<[88]>;


//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

// Main 8-bit register class.
def GPR8 : RegisterClass<"Z80", [i8], 8,
  (
    add A, B, C, D, E, H, L

    // Return value and argument registers.
    // add R24, R25, R18, R19, R20, R21, R22, R23,
    // Scratch registers.
    // R30, R31, R26, R27,
    // Callee saved registers.
    // R28, R29, R17, R16, R15, R14, R13, R12, R11, R10,
    // R9, R8, R7, R6, R5, R4, R3, R2, R0, R1
  )>;

def ACC : RegisterClass<"Z80", [i8], 8, (add A)>;

// Simple lower registers r0..r15
//def GPR8lo : RegisterClass<"Z80", [i8], 8,
//  (
//    add R15, R14, R13, R12, R11, R10, R9, R8, R7, R6, R5, R4, R3, R2, R0, R1
//  )>;

// 8-bit register class for instructions which take immediates.
//def LD8 : RegisterClass<"Z80", [i8], 8,
//  (
//    // Return value and arguments.
//    add R24, R25, R18, R19, R20, R21, R22, R23,
//    // Scratch registers.
//    R30, R31, R26, R27,
//    // Callee saved registers.
//    R28, R29, R17, R16
//  )>;

// Simple lower registers r16..r23
//def LD8lo : RegisterClass<"Z80", [i8], 8,
//  (
//    add R23, R22, R21, R20, R19, R18, R17, R16
//  )>;

// Main 16-bit pair register class.
def DREGS : RegisterClass<"Z80", [i16], 8, ( add BC, DE, HL, SP, IX, IY )>;

def XDREGS : RegisterClass<"Z80", [i16], 8, ( add IX, IY )>;

// 16-bit pair register class for movw
//def DREGSMOVW : RegisterClass<"Z80", [i16], 8,
//  (
//    // Return value and arguments.
//    add R25R24, R19R18, R21R20, R23R22,
//    // Scratch registers.
//    R31R30, R27R26,
//    // Callee saved registers.
//    R29R28, R17R16, R15R14, R13R12, R11R10,
//    R9R8, R7R6, R5R4, R3R2, R1R0
//  )>;

// The 16-bit DREGS register class, excluding the Z pointer register.
//
// This is used by instructions which cause high pointer register
// contention which leads to an assertion in the register allocator.
//
// There is no technical reason why instructions that use this class
// cannot use Z; it's simply a workaround a regalloc bug.
//
// More information can be found in PR39553.
//def DREGS_WITHOUT_YZ_WORKAROUND : RegisterClass<"Z80", [i16], 8,
//  (
//    // Return value and arguments.
//    add R25R24, R19R18, R21R20, R23R22,
//    // Scratch registers.
//    R27R26,
//    // Callee saved registers.
//    R17R16, R15R14, R13R12, R11R10,
//    R9R8, R7R6, R5R4, R3R2, R1R0
//  )>;

// 16-bit register class for immediate instructions.
//def DLDREGS : RegisterClass<"Z80", [i16], 8,
//  (
//    // Return value and arguments.
//    add R25R24, R19R18, R21R20, R23R22,
//    // Scratch registers.
//    R31R30, R27R26,
//    // Callee saved registers.
//    R29R28, R17R16
//  )>;

// 16-bit register class for the adiw/sbiw instructions.
//def IWREGS : RegisterClass<"Z80", [i16], 8,
//  (
//    // Return value and arguments.
//    add R25R24,
//    // Scratch registers.
//    R31R30, R27R26,
//    // Callee saved registers.
//    R29R28
//  )>;

// 16-bit register class for the ld and st instructions.
// AKA X,Y, and Z
//def PTRREGS : RegisterClass<"Z80", [i16], 8,
//  (
//    add R27R26, // X
//        R29R28, // Y
//        R31R30  // Z
//  ), ptr>;

// 16-bit register class for the ldd and std instructions.
// AKA Y and Z.
def PTRDISPREGS : RegisterClass<"Z80", [i16], 8, ( add HL, IX, IY ), ptr>;

def HLREG : RegisterClass<"Z80", [i16], 8, (add HL)>;
def SPREG : RegisterClass<"Z80", [i16], 8, (add SP)>;

def HLIXIYREGS : RegisterClass<"Z80", [i16], 8, (add HL, IX, IY)>;

// Status register.
def SREG : Z80Reg<14, "FLAGS">, DwarfRegNum<[88]>;

def CCR : RegisterClass<"Z80", [i8], 8, (add SREG)>
{
  let CopyCost = -1;      // Don't allow copying of status registers
}

