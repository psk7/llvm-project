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

// Status register.
def SREG : Z80Reg<14, "F">, DwarfRegNum<[89]>;

let SubRegIndices = [sub_lo, sub_hi],
CoveredBySubRegs = 1 in
{
  def HL     : Z80Reg<2, "hl",      [L, H]>, DwarfRegNum<[5]>;
  def DE     : Z80Reg<1, "de",      [E, D]>, DwarfRegNum<[3]>;
  def BC     : Z80Reg<0, "bc",      [C, B]>, DwarfRegNum<[1]>;

  def IX     : Z80Reg<2, "ix",      [XL, XH]>, DwarfRegNum<[5]>;
  def IY     : Z80Reg<2, "iy",      [YL, YH]>, DwarfRegNum<[7]>;

  def AF     : Z80Reg<2, "af",      [A, SREG]>, DwarfRegNum<[8]>;
}

def SP     : Z80Reg<3, "sp">, DwarfRegNum<[88]>;

//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

// Main 8-bit register class.
def BGPR8 : RegisterClass<"Z80", [i8], 8, ( add A, B, C, D, E, H, L )>;
def GPR8 : RegisterClass<"Z80", [i8], 8, ( add A, B, C, D, E, H, L, XL, XH, YL, YH )>;

def GPR8_NoHL : RegisterClass<"Z80", [i8], 8, ( add A, B, C, D, E )>;

def HandL : RegisterClass<"Z80", [i8], 8, ( add H, L )>;
def XY_HandL : RegisterClass<"Z80", [i8], 8, ( add XL, XH, YL, YH )>;

def ACC : RegisterClass<"Z80", [i8], 8, (add A)>;

// Main 16-bit pair register class.
def DREGS : RegisterClass<"Z80", [i16], 8, ( add BC, DE, HL, IX, IY, SP )>;
def BDREGS : RegisterClass<"Z80", [i16], 8, ( add BC, DE, HL, SP )>;
def XDREGS : RegisterClass<"Z80", [i16], 8, ( add IX, IY )>;

def REGSTOSTACK : RegisterClass<"Z80", [i16], 8, ( add BC, DE, HL, IX, IY, AF )>;

def PTRDISPREGS : RegisterClass<"Z80", [i16], 8, ( add IX, IY )>;
def PTRREGS : RegisterClass<"Z80", [i16], 8, ( add HL, IX, IY )>;

def BCREG : RegisterClass<"Z80", [i16], 8, (add BC)>;
def HLREG : RegisterClass<"Z80", [i16], 8, (add HL)>;
def DEREG : RegisterClass<"Z80", [i16], 8, (add DE)>;
def SPREG : RegisterClass<"Z80", [i16], 8, (add SP)>;

def CCR : RegisterClass<"Z80", [i8], 8, (add SREG)>
{
  let CopyCost = -1;      // Don't allow copying of status registers
}

