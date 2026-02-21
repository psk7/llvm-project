//===-- PDP.h - Top-level interface for PDP representation ------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// PDP back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_H
#define LLVM_PDP_H

#include "llvm/CodeGen/SelectionDAGNodes.h"
#include "llvm/Pass.h"
#include "llvm/PassRegistry.h"
#include "llvm/Target/TargetMachine.h"

namespace llvm {

class PDPTargetMachine;
class FunctionPass;
class PassRegistry;

Pass *createPDPShiftExpandPass();
FunctionPass *createPDPISelDag(PDPTargetMachine &TM, CodeGenOptLevel OptLevel);
FunctionPass *createPDPExpandPseudoPass();
FunctionPass *createPDPOptimizeInstructionsPass();
FunctionPass *createPDPFrameAnalyzerPass();
FunctionPass *createPDPBranchSelectionPass();

void initializePDPDAGToDAGISelLegacyPass(PassRegistry &);
void initializePDPExpandPseudoPass(PassRegistry &);
void initializePDPOptimizeInstructionsPass(PassRegistry &);
void initializePDPShiftExpandPass(PassRegistry &);

/// Contains the PDP backend.
namespace PDP {

/*/// An integer that identifies all of the supported PDP address spaces.
enum AddressSpace {
  DataMemory,
  ProgramMemory,
  ProgramMemory1,
  ProgramMemory2,
  ProgramMemory3,
  ProgramMemory4,
  ProgramMemory5,
  NumAddrSpaces,
};*/

/// Checks if a given type is a pointer to program memory.
/*template <typename T> bool isProgramMemoryAddress(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  return PT->getAddressSpace() == ProgramMemory ||
         PT->getAddressSpace() == ProgramMemory1 ||
         PT->getAddressSpace() == ProgramMemory2 ||
         PT->getAddressSpace() == ProgramMemory3 ||
         PT->getAddressSpace() == ProgramMemory4 ||
         PT->getAddressSpace() == ProgramMemory5;
}*/

/*template <typename T> AddressSpace getAddressSpace(T *V) {
  auto *PT = cast<PointerType>(V->getType());
  assert(PT != nullptr && "unexpected MemSDNode");
  unsigned AS = PT->getAddressSpace();
  if (AS < NumAddrSpaces)
    return static_cast<AddressSpace>(AS);
  return NumAddrSpaces;
}*/

/*inline bool isProgramMemoryAccess(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V != nullptr && isProgramMemoryAddress(V))
    return true;
  return false;
}*/

// Get the index of the program memory bank.
//  -1: not program memory
//   0: ordinary program memory
// 1~5: extended program memory
/*inline int getProgramMemoryBank(MemSDNode const *N) {
  auto *V = N->getMemOperand()->getValue();
  if (V == nullptr || !isProgramMemoryAddress(V))
    return -1;
  AddressSpace AS = getAddressSpace(V);
  assert(ProgramMemory <= AS && AS <= ProgramMemory5);
  return static_cast<int>(AS - ProgramMemory);
}*/

} // end of namespace PDP

} // end namespace llvm

#endif // LLVM_PDP_H
