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
  const Z80InstrInfo *TII;

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
bool Z80SimplifyInstructions::expand<Z80::RLC>(Block &MBB, BlockIt MBBI) {
    MachineInstr &MI = *MBBI;

    if (MI.getOperand(0).getReg() != Z80::A)
      return false;

    MI.setDesc(TII->get(Z80::RLCA));
    return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RRC>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RRCA));
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RRA));
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::RL>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A)
    return false;

  MI.setDesc(TII->get(Z80::RLA));
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::OR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  if (MI.getOperand(0).getReg() != Z80::A ||
      MI.getOperand(1).getReg() != Z80::A ||
      MI.getOperand(2).getReg() != Z80::A)
    return false;

  auto PMIT = MBBI;

  while (PMIT != MBB.begin()) {
    PMIT = std::prev(PMIT);
    MachineInstr &PMI = *PMIT;

    MachineOperand &O = PMI.getOperand(0);

    if (O.isReg() && O.getReg() == Z80::A && O.isDef())
      break;

    for (auto &op : PMI.operands()) {
      if (op.isReg() && op.getReg() == Z80::SREG && op.isDef())
        return false;
    }
  }

  if (MBBI == MBB.begin())
    return false;

  MachineInstr &PMI = *PMIT;

  auto OpCode = PMI.getOpcode();

  if (OpCode != Z80::AND && OpCode != Z80::OR && OpCode != Z80::ANDk &&
      OpCode != Z80::ORk && OpCode != Z80::XOR &&
      OpCode != Z80::XORk && OpCode != Z80::ANDPTR &&
      OpCode != Z80::ORPTR && OpCode != Z80::XORPTR)
    return false;

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::TESTBIT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  MachineOperand &Op0 = MI.getOperand(0);
  MachineOperand &Op1 = MI.getOperand(1);

  if (!(Op0.isReg() && Z80::A == Op0.getReg() && Op0.isKill() && Op1.isImm()))
    return false;

  buildMI(MBB, MBBI, Z80::ANDk)
      .addReg(Z80::A, RegState::Define | getDeadRegState(true))
      .addReg(Z80::A, getKillRegState(true))
      .addImm(1 << Op1.getImm());

  MI.eraseFromParent();

  return true;
}

template <>
bool Z80SimplifyInstructions::expand<Z80::JRCC>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  BlockIt NMBBI = std::next(MBBI);

  if (NMBBI != MBB.end())
    return false;

  auto *FallThrough = MBB.getFallThrough();
  if (!FallThrough)
    return false;

  auto *Target = MI.getOperand(0).getMBB();

  auto *CallFallThrough = FallThrough->getFallThrough();
  if (!CallFallThrough)
    return false;

  if (Target != CallFallThrough)
    return false;

  for(auto *Predecessor : FallThrough->predecessors())
    if (Predecessor != &MBB)
      return false;

  if (FallThrough->size() != 1)
    return false;

  MachineInstr &MICall = *FallThrough->begin();

  Z80CC::CondCodes BranchCode =
      static_cast<Z80CC::CondCodes>(MI.getOperand(1).getImm());

  BranchCode = TII->getOppositeCondition(BranchCode);

  MachineInstrBuilder NewCall =
      buildMI(MBB, MBBI, Z80::CALLCCk).addImm(BranchCode);

  for(auto &Op : MICall.operands())
    NewCall.add(Op);

  MI.eraseFromParent();

  MBB.removeSuccessor(FallThrough);
  FallThrough->removeSuccessor(CallFallThrough);
  FallThrough->eraseFromParent();

  return true;
}

bool Z80SimplifyInstructions::runOnMachineFunction(MachineFunction &MF) {
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

bool Z80SimplifyInstructions::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)                                                             \
  case Op:                                                                     \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(Z80::RLC);
    EXPAND(Z80::RRC);
    EXPAND(Z80::RR);
    EXPAND(Z80::RL);
    EXPAND(Z80::OR);
    EXPAND(Z80::TESTBIT);
    EXPAND(Z80::JRCC);
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
