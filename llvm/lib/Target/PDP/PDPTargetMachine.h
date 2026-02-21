//===-- PDPTargetMachine.h - Define TargetMachine for PDP -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the PDP specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_TARGET_MACHINE_H
#define LLVM_PDP_TARGET_MACHINE_H

#include "llvm/CodeGen/CodeGenTargetMachineImpl.h"
#include "llvm/IR/DataLayout.h"

#include "PDPFrameLowering.h"
#include "PDPISelLowering.h"
#include "PDPInstrInfo.h"
#include "PDPSelectionDAGInfo.h"
#include "PDPSubtarget.h"

#include <optional>

namespace llvm {

/// A generic PDP implementation.
class PDPTargetMachine : public CodeGenTargetMachineImpl {
public:
  PDPTargetMachine(const Target &T, const Triple &TT, StringRef CPU,
                   StringRef FS, const TargetOptions &Options,
                   std::optional<Reloc::Model> RM,
                   std::optional<CodeModel::Model> CM, CodeGenOptLevel OL,
                   bool JIT);

  const PDPSubtarget *getSubtargetImpl() const;
  const PDPSubtarget *getSubtargetImpl(const Function &) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override {
    return this->TLOF.get();
  }

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  MachineFunctionInfo *
  createMachineFunctionInfo(BumpPtrAllocator &Allocator, const Function &F,
                            const TargetSubtargetInfo *STI) const override;

  bool isNoopAddrSpaceCast(unsigned SrcAs, unsigned DestAs) const override {
    return getPointerSize(SrcAs) == getPointerSize(DestAs);
  }

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  PDPSubtarget SubTarget;
};

} // end namespace llvm

#endif // LLVM_PDP_TARGET_MACHINE_H
