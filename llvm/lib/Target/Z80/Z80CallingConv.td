//===-- Z80CallingConv.td - Calling Conventions for Z80 ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for Z80 architecture.
// Normal functions use a special calling convention, solved in code.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Z80 Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Special return value calling convention for runtime functions.
def RetCC_Z80_BUILTIN : CallingConv
<[
  CCIfType<[i8], CCAssignToReg<[L, H]>>,
  CCIfType<[i16], CCAssignToReg<[HL]>>
]>;

def RetCC_Z80_BUILTIN_32BitArith : CallingConv
<[
  CCIfType<[i16], CCAssignToReg<[HL, DE]>>
]>;

def RetCC_Z80_C : CallingConv
<[
  CCIfType<[i8], CCAssignToReg<[L, H]>>,
  CCIfType<[i16], CCAssignToReg<[HL, DE]>>
]>;

//===----------------------------------------------------------------------===//
// Z80 Argument Calling Conventions
//===----------------------------------------------------------------------===//

// Calling convention for variadic functions.
def ArgCC_Z80_Builtin : CallingConv
<[
  CCIfType<[i8], CCAssignToReg<[L, H, E, D, C, B]>>,
  CCIfType<[i16], CCAssignToReg<[HL, DE, BC]>>
]>;

def ArgCC_Z80_Builtin_32BitArith : CallingConv
<[
  CCIfType<[i16], CCAssignToStack<2, 1>>,
]>;

def ArgCC_Z80_C : CallingConv
<[
  CCIfType<[i8], CCAssignToReg<[E, D, C, B]>>,
  CCIfType<[i16], CCAssignToReg<[DE, BC]>>,
  CCIfType<[i16], CCAssignToStack<2, 1>>,
  CCIfType<[i8], CCAssignToStack<1, 1>>
]>;

// Calling convention for variadic functions.
def ArgCC_Z80_Vararg : CallingConv
<[
  CCIfType<[i16], CCAssignToReg<[DE]>>,
  // i16 are always passed through the stack with an alignment of 1.
  CCAssignToStack<2, 1>
]>;

//===----------------------------------------------------------------------===//
// Callee-saved register lists.
//===----------------------------------------------------------------------===//

def CSR_Normal : CalleeSavedRegs<(add BC, DE, IX, IY)>;
def CSR_Normal_NoDE : CalleeSavedRegs<(add BC, IX, IY)>;

