//===-- PDPOptimizeInstructions.cpp - Optimize instructions ---------------===//
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

#include "MCTargetDesc/PDPMCCodeEmitter.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "PDP.h"
#include "PDPInstrInfo.h"
#include "PDPTargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define PDP_OPTIMIZE_INSTRUCTIONS_NAME "PDP instruction optimization pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual PDP instructions.
class PDPOptimizeInstructions : public MachineFunctionPass {
public:
  static char ID;

  PDPOptimizeInstructions() : MachineFunctionPass(ID) {
    initializePDPOptimizeInstructionsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return PDP_OPTIMIZE_INSTRUCTIONS_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const PDPRegisterInfo *TRI;
  const PDPInstrInfo *TII;

  bool expandMBB(Block &MBB);
  bool expandMI(Block &MBB, BlockIt MBBI);
  template <unsigned OP> bool expand(Block &MBB, BlockIt MBBI);

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

  bool checkAdd(Block &MBB, BlockIt MBBI);
};

char PDPOptimizeInstructions::ID = 0;

bool IsImmediate(const MachineInstr &MI, unsigned OpNo, int &Val) {
  if (MI.getOperand(OpNo).getReg() != PDP::R7)
    return false;

  if (!MI.getOperand(OpNo +1).isImm() || MI.getOperand(OpNo +1).getImm() != 2)
    return false;

  Val = MI.getOperand(OpNo + 2).getImm() + MI.getOperand(OpNo + 3).getImm();

  return true;
}

bool PDPOptimizeInstructions::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);

    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool PDPOptimizeInstructions::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  TRI = STI.getRegisterInfo();
  TII = dyn_cast<PDPInstrInfo>(STI.getInstrInfo());

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");
      (void)ExpandCount;

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

bool PDPOptimizeInstructions::checkAdd(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

  if (Opcode != PDP::ADD)
    return false;

  if (MI.getOperand(0).getReg() != PDP::R7)
    return false;

  if (MI.getOperand(1).getImm() != 2)
    return false;

  if (!MI.getOperand(2).isImm() || !MI.getOperand(3).isImm())
    return false;

  auto Disp = MI.getOperand(2).getImm() + MI.getOperand(3).getImm();

  unsigned NewOpCode = 0;

  if (Disp == -1)
    NewOpCode = PDP::DEC;
  else if (Disp == 1)
    NewOpCode = PDP::INC;
  else
    return false;

  //MI.dump();

  const bool ImpIsDead = MI.getOperand(8).isDead();

  auto NewOp = buildMI(MBB, MBBI, NewOpCode);
  NewOp->addOperand(MI.getOperand(4));
  NewOp->addOperand(MI.getOperand(5));
  NewOp->addOperand(MI.getOperand(6));
  NewOp->addOperand(MI.getOperand(7));

  for (const auto &Mo : MI.memoperands())
    NewOp->addMemOperand(*MBB.getParent(), Mo);

  if (ImpIsDead)
    NewOp->getOperand(4).setIsDead();

  NewOp->setFlags(MI.getFlags());

  //NewOp->dump();

  MI.eraseFromParent();

  return true;
}

bool PDPOptimizeInstructions::expandMI(Block &MBB, BlockIt MBBI) {
//  MachineInstr &MI = *MBBI;
//  int Opcode = MBBI->getOpcode();

  bool Modified = checkAdd(MBB, MBBI);

  return Modified;
}

} // end of anonymous namespace

INITIALIZE_PASS(PDPOptimizeInstructions, "PDP-optimize-instructions", PDP_OPTIMIZE_INSTRUCTIONS_NAME,
                false, false)
namespace llvm {

FunctionPass *createPDPOptimizeInstructionsPass() { return new PDPOptimizeInstructions(); }

} // end of namespace llvm
