//===-- PDPTargetMachine.cpp - Define TargetMachine for PDP ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the PDP specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "PDPTargetMachine.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/MC/TargetRegistry.h"

#include "PDP.h"
#include "PDPMachineFunctionInfo.h"
#include "PDPTargetObjectFile.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "TargetInfo/PDPTargetInfo.h"

#include <optional>

namespace llvm {

static const char *PDPDataLayout = "e-p:16:16-i8:8-i16:16-i32:16-i64:16-n8:16";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
  if (CPU.empty() || CPU == "generic") {
    return "vm1a";
  }

  return CPU;
}

static Reloc::Model getEffectiveRelocModel(std::optional<Reloc::Model> RM) {
  return RM.value_or(Reloc::Static);
}

PDPTargetMachine::PDPTargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   std::optional<Reloc::Model> RM,
                                   std::optional<CodeModel::Model> CM,
                                   CodeGenOptLevel OL, bool JIT)
    : CodeGenTargetMachineImpl(T, PDPDataLayout, TT, getCPU(CPU), FS, Options,
                               getEffectiveRelocModel(RM),
                               getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, std::string(getCPU(CPU)), std::string(FS), *this) {
  this->TLOF = std::make_unique<PDPTargetObjectFile>();
  initAsmInfo();
}

namespace {
/// PDP Code Generator Pass Configuration Options.
class PDPPassConfig : public TargetPassConfig {
public:
  PDPPassConfig(PDPTargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  PDPTargetMachine &getPDPTargetMachine() const {
    return getTM<PDPTargetMachine>();
  }

  void addIRPasses() override;
  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
};
} // namespace

TargetPassConfig *PDPTargetMachine::createPassConfig(PassManagerBase &PM) {
  return new PDPPassConfig(*this, PM);
}

void PDPPassConfig::addIRPasses() {
  // Expand instructions like
  //   %result = shl i32 %n, %amount
  // to a loop so that library calls are avoided.
  addPass(createPDPShiftExpandPass());

  TargetPassConfig::addIRPasses();
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPTarget() {
  // Register the target.
  RegisterTargetMachine<PDPTargetMachine> X(getThePDPTarget());

  auto &PR = *PassRegistry::getPassRegistry();
  initializePDPExpandPseudoPass(PR);
  initializePDPOptimizeInstructionsPass(PR);
  initializePDPShiftExpandPass(PR);
  initializePDPDAGToDAGISelLegacyPass(PR);
}

const PDPSubtarget *PDPTargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const PDPSubtarget *PDPTargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

MachineFunctionInfo *PDPTargetMachine::createMachineFunctionInfo(
    BumpPtrAllocator &Allocator, const Function &F,
    const TargetSubtargetInfo *STI) const {
  return PDPMachineFunctionInfo::create<PDPMachineFunctionInfo>(Allocator, F,
                                                                STI);
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

bool PDPPassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createPDPISelDag(getPDPTargetMachine(), getOptLevel()));
  // Create the frame analyzer pass used by the PEI pass.
  addPass(createPDPFrameAnalyzerPass());

  return false;
}

void PDPPassConfig::addPreSched2() {
  addPass(createPDPExpandPseudoPass());
  addPass(createPDPOptimizeInstructionsPass());
}

void PDPPassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  addPass(&BranchRelaxationPassID);
}

} // end of namespace llvm
