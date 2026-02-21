//===-- PDPCallingConv.td - Calling Conventions for PDP ----*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
// This describes the calling conventions for PDP architecture.
// Normal functions use a special calling convention, solved in code.
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// PDP Return Value Calling Convention
//===----------------------------------------------------------------------===//

// Special return value calling convention for runtime functions.
def RetCC_PDP_BUILTIN : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i8>>,
  CCIfType<[i8], CCAssignToReg<[R0l]>>,
  CCIfType<[i16], CCAssignToReg<[R0]>>,
]>;

def RetCC_PDP_C : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i8>>,
  CCIfType<[i8], CCAssignToReg<[R0l]>>,
  CCIfType<[i16], CCAssignToReg<[R0]>>,
]>;

//===----------------------------------------------------------------------===//
// PDP Argument Calling Conventions
//===----------------------------------------------------------------------===//

// The calling conventions are implemented in custom C++ code

// Calling convention for variadic functions.
def ArgCC_PDP_C : CallingConv<[
  CCIfType<[i1], CCPromoteToType<i8>>,

  CCIfType<[i8], CCAssignToReg<[R0l, R1l, R2l, R3l]>>,
  CCIfType<[i8], CCAssignToStack<1, 1>>,

  CCIfType<[i16], CCAssignToReg<[R0, R1, R2, R3]>>,
  CCIfType<[i16], CCAssignToStack<2, 2>>
]>;

def ArgCC_PDP_Vararg : CallingConv<[
  CCIfType<[i1, i8], CCPromoteToType<i16>>,
  CCAssignToStack<2, 2>
]>;

//===----------------------------------------------------------------------===//
// Callee-saved register lists.
//===----------------------------------------------------------------------===//

def CSR_Normal : CalleeSavedRegs<(add R1, R2, R3, R4, R5)>;
def CSR_Interrupts : CalleeSavedRegs<(add R0, R1, R2, R3, R4, R5)>;
