//===-- Z80MachineFuctionInfo.h - Z80 machine function info -----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares Z80-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_MACHINE_FUNCTION_INFO_H
#define LLVM_Z80_MACHINE_FUNCTION_INFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

/// Contains Z80-specific information for each MachineFunction.
class Z80MachineFunctionInfo : public MachineFunctionInfo {
  /// Indicates if a register has been spilled by the register
  /// allocator.
  bool HasSpills;

  /// Indicates if there are any fixed size allocas present.
  /// Note that if there are only variable sized allocas this is set to false.
  bool HasAllocas;

  /// Indicates if arguments passed using the stack are being
  /// used inside the function.
  bool HasStackArgs;

  /// Size of the callee-saved register portion of the
  /// stack frame in bytes.
  unsigned CalleeSavedFrameSize;

  /// FrameIndex for start of varargs area.
  int VarArgsFrameIndex;

public:
  Z80MachineFunctionInfo()
      : HasSpills(false), HasAllocas(false), HasStackArgs(false),
        CalleeSavedFrameSize(0), VarArgsFrameIndex(0) {}

  explicit Z80MachineFunctionInfo(MachineFunction &MF)
      : HasSpills(false), HasAllocas(false), HasStackArgs(false),
        CalleeSavedFrameSize(0), VarArgsFrameIndex(0) {
    //unsigned CallConv = MF.getFunction().getCallingConv();
  }

  bool getHasSpills() const { return HasSpills; }
  void setHasSpills(bool B) { HasSpills = B; }

  bool getHasAllocas() const { return HasAllocas; }
  void setHasAllocas(bool B) { HasAllocas = B; }

  bool getHasStackArgs() const { return HasStackArgs; }
  void setHasStackArgs(bool B) { HasStackArgs = B; }

  unsigned getCalleeSavedFrameSize() const { return CalleeSavedFrameSize; }
  void setCalleeSavedFrameSize(unsigned Bytes) { CalleeSavedFrameSize = Bytes; }

  int getVarArgsFrameIndex() const { return VarArgsFrameIndex; }
  void setVarArgsFrameIndex(int Idx) { VarArgsFrameIndex = Idx; }
};

} // end llvm namespace

#endif // LLVM_Z80_MACHINE_FUNCTION_INFO_H
