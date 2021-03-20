//===-- Z80PreRASimplifyInstructions.cpp - Expand pseudo instructions
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

#define Z80_PRE_RA_SIMPLIFY_INSTRUCTIONS_NAME "Z80 pre RA instructions simplification pass"

namespace {

class Z80PreRASimplifyInstructions : public MachineFunctionPass {
public:
  static char ID;

  Z80PreRASimplifyInstructions() : MachineFunctionPass(ID) {
    initializeZ80PreRASimplifyInstructionsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return Z80_PRE_RA_SIMPLIFY_INSTRUCTIONS_NAME;
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

char Z80PreRASimplifyInstructions::ID = 0;

#define DUMPMI(x) { dbgs() << "Rewrite: "; x.dump(); }

template <>
bool Z80PreRASimplifyInstructions::expand<Z80::TESTBIT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MBBI == MBB.begin())
    return false;

  MachineOperand &Op0 = MI.getOperand(0);

  if (!Op0.isKill())
    return false;

  auto Bit = MI.getOperand(1).getImm();

  MachineInstr &PMI = *std::prev(MBBI);

  if (Z80::LDPTR == PMI.getOpcode()) {
    buildMI(MBB, MBBI, Z80::TESTBITPTR)
        .add(PMI.getOperand(1))
        .addImm(0)
        .addImm(Bit);
  } else if (Z80::LDDPTR == PMI.getOpcode()) {
    MachineOperand &Displacement = PMI.getOperand(2);

    buildMI(MBB, MBBI, Z80::TESTBITPTR)
        .add(PMI.getOperand(1))
        .addImm(Displacement.getImm())
        .addImm(Bit);
  } else
    return false;

  PMI.eraseFromParent();
  MI.eraseFromParent();

  return true;
}

bool Z80PreRASimplifyInstructions::runOnMachineFunction(MachineFunction &MF) {
  return false;
  bool Modified = false;

  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  for (Block &MBB : MF) {
    BlockIt MBBI = MBB.begin(), E = MBB.end();
    while (MBBI != E) {
      BlockIt NMBBI = std::next(MBBI);
      Modified |= expandMI(MBB, MBBI);
      MBBI = NMBBI;
    }
  }

  return Modified;
}

bool Z80PreRASimplifyInstructions::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)                                                             \
  case Op:                                                                     \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(Z80::TESTBIT);
  }

#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(Z80PreRASimplifyInstructions, "z80-pre-ra-simplify-instructions",
                Z80_PRE_RA_SIMPLIFY_INSTRUCTIONS_NAME, false, false)
namespace llvm {

FunctionPass *createZ80PreRASimplifyInstructionsPass() {
  return new Z80PreRASimplifyInstructions();
}

} // end of namespace llvm
