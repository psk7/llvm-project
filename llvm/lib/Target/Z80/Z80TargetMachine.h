//===-- Z80TargetMachine.h - Define TargetMachine for Z80 -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the Z80 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_TARGET_MACHINE_H
#define LLVM_Z80_TARGET_MACHINE_H

#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "Z80FrameLowering.h"
#include "Z80ISelLowering.h"
#include "Z80InstrInfo.h"
#include "Z80SelectionDAGInfo.h"
#include "Z80Subtarget.h"

namespace llvm {

/// A generic Z80 implementation.
class Z80TargetMachine : public LLVMTargetMachine {
public:
  Z80TargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   Optional<Reloc::Model> RM,
                   Optional<CodeModel::Model> CM,
                   CodeGenOpt::Level OL, bool JIT);

  const Z80Subtarget *getSubtargetImpl() const;
  const Z80Subtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
      return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  bool isMachineVerifierClean() const override {
    return false;
  }

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  Z80Subtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_Z80_TARGET_MACHINE_H
