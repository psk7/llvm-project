//===-- Z80SimplifyInstructions.cpp - Expand pseudo instructions
//-------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a pass that expands pseudo instructions into target
// instructions. This pass should be run after register allocation but before
// the post-regalloc scheduling pass.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "Z80.h"
#include "Z80InstrInfo.h"
#include "Z80TargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define Z80_SIMPLIFY_INSTRUCTIONS_NAME "Z80 instructions simplification pass"

namespace {

class Z80SimplifyInstructions : public MachineFunctionPass {
public:
  static char ID;

  Z80SimplifyInstructions() : MachineFunctionPass(ID) {
    initializeZ80SimplifyInstructionsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return Z80_SIMPLIFY_INSTRUCTIONS_NAME;
  }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const Z80RegisterInfo *TRI;
  const TargetInstrInfo *TII;

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode));
  }

  MachineInstrBuilder buildMI(Block &MBB, BlockIt MBBI, unsigned Opcode,
                              Register DstReg) {
    return BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(Opcode), DstReg);
  }

  MachineRegisterInfo &getRegInfo(Block &MBB) {
    return MBB.getParent()->getRegInfo();
  }

  bool expandMI(Block &MBB, BlockIt MBBI);
  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);
};

char Z80SimplifyInstructions::ID = 0;

#define DUMPMI(x) { dbgs() << "Rewrite: "; x.dump(); }

template <>
bool Z80SimplifyInstructions::expand<Z80::RLCRd>(Block &MBB, BlockIt MBBI) {
    MachineInstr &MI = *MBBI;

    if (MI.getOperand(0).getReg() != Z80::A)
      return false;

    MI.setDesc(TII->get(Z80::RLCA));
    return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RRCRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RRCA));
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RRRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RRA));
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RLRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RLA));
  return true;
}

bool Z80SimplifyInstructions::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  for (Block &MBB : MF) {
    for (auto &I : MBB) {
      Modified |= expandMI(MBB, I);
    }
  }

  return Modified;
}

bool Z80SimplifyInstructions::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)                                                             \
  case Op:                                                                     \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(Z80::RLCRd);
    EXPAND(Z80::RRCRd);
    EXPAND(Z80::RRRd);
    EXPAND(Z80::RLRd);
  }

#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(Z80SimplifyInstructions, "z80-simplify-instructions",
                Z80_SIMPLIFY_INSTRUCTIONS_NAME, false, false)
namespace llvm {

FunctionPass *createZ80SimplifyInstructionsPass() {
  return new Z80SimplifyInstructions();
}

} // end of namespace llvm
