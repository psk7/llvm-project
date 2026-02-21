//===-- PDPRegisterInfo.td - PDP Register defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
//  Declarations that describe the PDP register file
//===----------------------------------------------------------------------===//

// 8-bit General purpose register definition.
class PDPReg<bits<16> num, string name, list<Register> subregs = [],
             list<string> altNames = []> : RegisterWithSubRegs<name, subregs> {
  field bits<16> Num = num;

  let HWEncoding = num;
  let Namespace = "PDP";
  let SubRegs = subregs;
  let AltNames = altNames;
}

// Subregister indices.
let Namespace = "PDP" in {
  def sub_lo : SubRegIndex<8>;
  def sub_hi : SubRegIndex<8, 8>;
}

let Namespace = "PDP" in { def ptr : RegAltNameIndex; }

//===----------------------------------------------------------------------===//
//  8-bit general purpose registers
//===----------------------------------------------------------------------===//

def R0l : PDPReg<0, "R0l">, DwarfRegNum<[0]>;
def R1l : PDPReg<1, "R1l">, DwarfRegNum<[1]>;
def R2l : PDPReg<2, "R2l">, DwarfRegNum<[2]>;
def R3l : PDPReg<3, "R3l">, DwarfRegNum<[3]>;
def R4l : PDPReg<4, "R4l">, DwarfRegNum<[4]>;
def R5l : PDPReg<5, "R5l">, DwarfRegNum<[5]>;
def R6l : PDPReg<6, "R6l">, DwarfRegNum<[6]>;
def R7l : PDPReg<7, "R7l">, DwarfRegNum<[7]>;

let SubRegIndices = [sub_lo], CoveredBySubRegs = 0 in {
  def R0 : PDPReg<0, "R0", [R0l]>, DwarfRegNum<[0]>;
  def R1 : PDPReg<1, "R1", [R1l]>, DwarfRegNum<[1]>;
  def R2 : PDPReg<2, "R2", [R2l]>, DwarfRegNum<[2]>;
  def R3 : PDPReg<3, "R3", [R3l]>, DwarfRegNum<[3]>;
  def R4 : PDPReg<4, "R4", [R4l]>, DwarfRegNum<[4]>;
  def R5 : PDPReg<5, "R5", [R5l]>, DwarfRegNum<[5]>;
  def R6 : PDPReg<6, "R6", [R6l], ["SP"]>, DwarfRegNum<[6]>;
  def R7 : PDPReg<7, "R7", [R7l], ["PC"]>, DwarfRegNum<[7]>;
}

//===----------------------------------------------------------------------===//
// Register Classes
//===----------------------------------------------------------------------===//

def GPRR0 : RegisterClass<"PDP", [i16], 16, (add R0)>;
def GPRR1 : RegisterClass<"PDP", [i16], 16, (add R1)>;
def GPRR2 : RegisterClass<"PDP", [i16], 16, (add R2)>;

def GPR16 : RegisterClass<"PDP", [i16], 16, (add R0, R1, R2, R3, R4, R5, R6, R7)>;
def GPR16l : RegisterClass<"PDP", [i8], 8, (add R0l, R1l, R2l, R3l, R4l, R5l, R6l, R7l)>;

// Status register.
def SREG : PDPReg<14, "FLAGS">, DwarfRegNum<[88]>;
def CCR : RegisterClass<"PDP", [i8], 8, (add SREG)> {
  let CopyCost = -1; // Don't allow copying of status registers
}
