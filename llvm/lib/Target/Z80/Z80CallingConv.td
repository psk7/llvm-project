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
  CCIfType<[i8], CCAssignToReg<[A, B]>>,
  CCIfType<[i16], CCAssignToReg<[DE, HL]>>
]>;

//===----------------------------------------------------------------------===//
// Z80 Argument Calling Conventions
//===----------------------------------------------------------------------===//

// The calling conventions are implemented in custom C++ code

// Calling convention for variadic functions.
def ArgCC_Z80_Vararg : CallingConv
<[
  // i16 are always passed through the stack with an alignment of 1.
  CCAssignToStack<2, 1>
]>;

//===----------------------------------------------------------------------===//
// Callee-saved register lists.
//===----------------------------------------------------------------------===//

def CSR_Normal : CalleeSavedRegs<(add BC, IX, IY)>;

