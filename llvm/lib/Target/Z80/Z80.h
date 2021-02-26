//===-- Z80.h - Top-level interface for Z80 representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// Z80 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_H
#define LLVM_Z80_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class Z80TargetMachine;
class FunctionPass;

FunctionPass *createZ80ISelDag(Z80TargetMachine &TM,
                               CodeGenOpt::Level OptLevel);
ModulePass *createZ80ModuleAnalyzerPass();
FunctionPass *createZ80BranchRelaxationPass();
FunctionPass *createZ80ExpandPseudoPass();
FunctionPass *createZ80SimplifyInstructionsPass();
FunctionPass *createZ80PreRASimplifyInstructionsPass();
FunctionPass *createZ80FrameAnalyzerPass();
FunctionPass *createZ80RelaxMemPass();
FunctionPass *createZ80DynAllocaSRPass();
FunctionPass *createZ80BranchSelectionPass();

void initializeZ80ModuleAnalyzerPass(PassRegistry&);
void initializeZ80BranchRelaxationPass(PassRegistry&);
void initializeZ80ModuleAnalyzerPass(PassRegistry&);
void initializeZ80ExpandPseudoPass(PassRegistry&);
void initializeZ80PreRASimplifyInstructionsPass(PassRegistry&);
void initializeZ80SimplifyInstructionsPass(PassRegistry&);
void initializeZ80RelaxMemPass(PassRegistry&);

/// Contains the Z80 backend.
namespace Z80 {

enum AddressSpace { DataMemory, Ports = 2, ShortPorts = 3 };

} // end of namespace Z80

} // end namespace llvm

#endif // LLVM_Z80_H
