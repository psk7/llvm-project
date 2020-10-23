//===-- Z80TargetMachine.cpp - Define TargetMachine for Z80 ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the Z80 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "Z80TargetMachine.h"

#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/TargetRegistry.h"

#include "Z80.h"
#include "Z80TargetObjectFile.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "TargetInfo/Z80TargetInfo.h"

namespace llvm {

static const char *Z80DataLayout = "e-P1-p:16:8-i8:8-i16:8-i32:8-n8-a:8";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) {
/*
  if (CPU.empty() || CPU == "generic") {
    return "avr2";
  }
*/

  return "z80";
}

static Reloc::Model getEffectiveRelocModel(Optional<Reloc::Model> RM) {
  return RM.hasValue() ? *RM : Reloc::Static;
}

Z80TargetMachine::Z80TargetMachine(const Target &T, const Triple &TT,
                                   StringRef CPU, StringRef FS,
                                   const TargetOptions &Options,
                                   Optional<Reloc::Model> RM,
                                   Optional<CodeModel::Model> CM,
                                   CodeGenOpt::Level OL, bool JIT)
    : LLVMTargetMachine(T, Z80DataLayout, TT, getCPU(CPU), FS, Options,
                        getEffectiveRelocModel(RM),
                        getEffectiveCodeModel(CM, CodeModel::Tiny), OL),
      SubTarget(TT, std::string(getCPU(CPU)), std::string(FS), *this) {
  this->TLOF = std::make_unique<Z80TargetObjectFile>();
  initAsmInfo();
}

namespace {
/// Z80 Code Generator Pass Configuration Options.
class Z80PassConfig : public TargetPassConfig {
public:
  Z80PassConfig(Z80TargetMachine &TM, PassManagerBase &PM)
      : TargetPassConfig(TM, PM) {}

  Z80TargetMachine &getZ80TargetMachine() const {
    return getTM<Z80TargetMachine>();
  }

  bool addInstSelector() override;
  void addPreSched2() override;
  void addPreEmitPass() override;
  void addPreRegAlloc() override;
};
} // namespace

TargetPassConfig *Z80TargetMachine::createPassConfig(PassManagerBase &PM) {
  return new Z80PassConfig(*this, PM);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeZ80Target() {
  // Register the target.
  RegisterTargetMachine<Z80TargetMachine> X(getTheZ80Target());

  auto &PR = *PassRegistry::getPassRegistry();
  initializeZ80ExpandPseudoPass(PR);
  //initializeZ80RelaxMemPass(PR);
}

const Z80Subtarget *Z80TargetMachine::getSubtargetImpl() const {
  return &SubTarget;
}

const Z80Subtarget *Z80TargetMachine::getSubtargetImpl(const Function &) const {
  return &SubTarget;
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

bool Z80PassConfig::addInstSelector() {
  // Install an instruction selector.
  addPass(createZ80ISelDag(getZ80TargetMachine(), getOptLevel()));
  // Create the frame analyzer pass used by the PEI pass.
  addPass(createZ80FrameAnalyzerPass());

  return false;
}

void Z80PassConfig::addPreRegAlloc() {
  // Create the dynalloc SP save/restore pass to handle variable sized allocas.
  addPass(createZ80DynAllocaSRPass());
}

void Z80PassConfig::addPreSched2() {
  //addPass(createZ80RelaxMemPass());
  addPass(createZ80ExpandPseudoPass());
}

void Z80PassConfig::addPreEmitPass() {
  // Must run branch selection immediately preceding the asm printer.
  addPass(&BranchRelaxationPassID);
}

} // end of namespace llvm