//===- BranchRelaxation.cpp -----------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Z80.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80RegisterInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "MCTargetDesc/Z80MCCodeEmitter.h"

using namespace llvm;

#define DEBUG_TYPE "z80-branch-relaxation"

#define BRANCH_RELAX_NAME "Z80 Branch relaxation pass"

namespace {

class Z80BranchRelaxation : public MachineFunctionPass {
  /// BasicBlockInfo - Information about the offset and size of a single
  /// basic block.
  struct BasicBlockInfo {
    /// Offset - Distance from the beginning of the function to the beginning
    /// of this basic block.
    ///
    /// The offset is always aligned as required by the basic block.
    unsigned Offset = 0;

    /// Size - Size of the basic block in bytes.  If the block contains
    /// inline assembly, this is a worst case estimate.
    ///
    /// The size does not include any alignment padding whether from the
    /// beginning of the block, or from an aligned jump table at the end.
    unsigned Size = 0;

    BasicBlockInfo() = default;

    /// Compute the offset immediately following this block. \p MBB is the next
    /// block.
    unsigned postOffset(const MachineBasicBlock &MBB) const {
      const unsigned PO = Offset + Size;
      const Align Alignment = MBB.getAlignment();
      const Align ParentAlign = MBB.getParent()->getAlignment();
      if (Alignment <= ParentAlign)
        return alignTo(PO, Alignment);

      // The alignment of this MBB is larger than the function's alignment, so we
      // can't tell whether or not it will insert nops. Assume that it will.
      return alignTo(PO, Alignment) + Alignment.value() - ParentAlign.value();
    }
  };

  SmallVector<BasicBlockInfo, 16> BlockInfo;

  MachineFunction *MF;
  const TargetRegisterInfo *TRI;
  const TargetInstrInfo *TII;

  bool relaxBranchInstructions();
  void scanFunction();

  void adjustBlockOffsets(MachineBasicBlock &Start);
  bool isBlockInRange(const MachineInstr &MI, const MachineBasicBlock &BB) const;

  bool fixupConditionalBranch(MachineInstr &MI);
  bool fixupUnconditionalBranch(MachineInstr &MI);
  uint64_t computeBlockSize(const MachineBasicBlock &MBB) const;
  unsigned getInstrOffset(const MachineInstr &MI) const;
  void verify();

public:
  static char ID;

  Z80BranchRelaxation() : MachineFunctionPass(ID) {
    initializeZ80BranchRelaxationPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return BRANCH_RELAX_NAME; }
};

} // end anonymous namespace

char Z80BranchRelaxation::ID = 0;

INITIALIZE_PASS(Z80BranchRelaxation, DEBUG_TYPE, BRANCH_RELAX_NAME, false, false)

namespace llvm {
FunctionPass *createZ80BranchRelaxationPass() {
  return new Z80BranchRelaxation();
}
} // namespace llvm

/// verify - check BBOffsets, BBSizes, alignment of islands
void Z80BranchRelaxation::verify() {
#ifndef NDEBUG
  unsigned PrevNum = MF->begin()->getNumber();
  for (MachineBasicBlock &MBB : *MF) {
    const unsigned Num = MBB.getNumber();
    assert(!Num || BlockInfo[PrevNum].postOffset(MBB) <= BlockInfo[Num].Offset);
    assert(BlockInfo[Num].Size == computeBlockSize(MBB));
    PrevNum = Num;
  }
#endif
}

/// scanFunction - Do the initial scan of the function, building up
/// information about each block.
void Z80BranchRelaxation::scanFunction() {
  BlockInfo.clear();
  BlockInfo.resize(MF->getNumBlockIDs());

  // First thing, compute the size of all basic blocks, and see if the function
  // has any inline assembly in it. If so, we have to be conservative about
  // alignment assumptions, as we don't know for sure the size of any
  // instructions in the inline assembly.
  for (MachineBasicBlock &MBB : *MF)
    BlockInfo[MBB.getNumber()].Size = computeBlockSize(MBB);

  // Compute block offsets and known bits.
  adjustBlockOffsets(*MF->begin());
}

/// computeBlockSize - Compute the size for MBB.
uint64_t Z80BranchRelaxation::computeBlockSize(const MachineBasicBlock &MBB) const {
  uint64_t Size = 0;
  for (const MachineInstr &MI : MBB)
    Size += TII->getInstSizeInBytes(MI);
  return Size;
}

/// getInstrOffset - Return the current offset of the specified machine
/// instruction from the start of the function.  This offset changes as stuff is
/// moved around inside the function.
unsigned Z80BranchRelaxation::getInstrOffset(const MachineInstr &MI) const {
  const MachineBasicBlock *MBB = MI.getParent();

  // The offset is composed of two things: the sum of the sizes of all MBB's
  // before this instruction's block, and the offset from the start of the block
  // it is in.
  unsigned Offset = BlockInfo[MBB->getNumber()].Offset;

  // Sum instructions before MI in MBB.
  for (MachineBasicBlock::const_iterator I = MBB->begin(); &*I != &MI; ++I) {
    assert(I != MBB->end() && "Didn't find MI in its own basic block?");
    Offset += TII->getInstSizeInBytes(*I);
  }

  return Offset;
}

void Z80BranchRelaxation::adjustBlockOffsets(MachineBasicBlock &Start) {
  unsigned PrevNum = Start.getNumber();
  for (auto &MBB :
       make_range(std::next(MachineFunction::iterator(Start)), MF->end())) {
    unsigned Num = MBB.getNumber();
    // Get the offset and known bits at the end of the layout predecessor.
    // Include the alignment of the current block.
    BlockInfo[Num].Offset = BlockInfo[PrevNum].postOffset(MBB);

    PrevNum = Num;
  }
}

bool Z80BranchRelaxation::isBlockInRange(
  const MachineInstr &MI, const MachineBasicBlock &DestBB) const {
  int64_t BrOffset = getInstrOffset(MI);
  int64_t DestOffset = BlockInfo[DestBB.getNumber()].Offset;

  return TII->isBranchOffsetInRange(MI.getOpcode(), DestOffset - BrOffset);
}

bool Z80BranchRelaxation::fixupConditionalBranch(MachineInstr &MI) {
  DebugLoc DL = MI.getDebugLoc();
  MachineBasicBlock *MBB = MI.getParent();
  MachineBasicBlock *TBB = nullptr, *FBB = nullptr;
  SmallVector<MachineOperand, 4> Cond;

  bool Fail = TII->analyzeBranch(*MBB, TBB, FBB, Cond);
  assert(!Fail && "branches to be relaxed must be analyzable");
  (void)Fail;

  unsigned &BBSize = BlockInfo[MBB->getNumber()].Size;

  BBSize -= TII->getInstSizeInBytes(MI);
  MI.setDesc(TII->get(Z80::JPCC));
  BBSize += TII->getInstSizeInBytes(MI);

  adjustBlockOffsets(*MBB);
  return true;
}

bool Z80BranchRelaxation::fixupUnconditionalBranch(MachineInstr &MI) {
  MachineBasicBlock *MBB = MI.getParent();

  unsigned &BBSize = BlockInfo[MBB->getNumber()].Size;

  BBSize -= TII->getInstSizeInBytes(MI);
  MI.setDesc(TII->get(Z80::JMPk));
  BBSize += TII->getInstSizeInBytes(MI);

  adjustBlockOffsets(*MBB);
  return true;
}

bool Z80BranchRelaxation::relaxBranchInstructions() {
  bool Changed = false;

  // Relaxing branches involves creating new basic blocks, so re-eval
  // end() for termination.
  for (MachineFunction::iterator I = MF->begin(); I != MF->end(); ++I) {
    MachineBasicBlock &MBB = *I;

    // Empty block?
    MachineBasicBlock::iterator Last = MBB.getLastNonDebugInstr();
    if (Last == MBB.end())
      continue;

    // Expand the unconditional branch first if necessary. If there is a
    // conditional branch, this will end up changing the branch destination of
    // it to be over the newly inserted indirect branch block, which may avoid
    // the need to try expanding the conditional branch first, saving an extra
    // jump.
    if (Last->isUnconditionalBranch()) {
      // Unconditional branch destination might be unanalyzable, assume these
      // are OK.
      if (MachineBasicBlock *DestBB = TII->getBranchDestBlock(*Last)) {
        if (!isBlockInRange(*Last, *DestBB)) {
          fixupUnconditionalBranch(*Last);
          Changed = true;
        }
      }
    }

    // Loop over the conditional branches.
    MachineBasicBlock::iterator Next;
    for (MachineBasicBlock::iterator J = MBB.getFirstTerminator();
         J != MBB.end(); J = Next) {
      Next = std::next(J);
      MachineInstr &MI = *J;

      if (MI.isConditionalBranch()) {
        MachineBasicBlock *DestBB = TII->getBranchDestBlock(MI);
        if (!isBlockInRange(MI, *DestBB)) {
          fixupConditionalBranch(MI);

          Changed = true;

          // This may have modified all of the terminators, so start over.
          Next = MBB.getFirstTerminator();
        }
      }
    }
  }

  return Changed;
}

bool Z80BranchRelaxation::runOnMachineFunction(MachineFunction &mf) {
  MF = &mf;

  LLVM_DEBUG(dbgs() << "***** Z80 BranchRelaxation *****\n");

  const TargetSubtargetInfo &ST = MF->getSubtarget();
  TII = ST.getInstrInfo();

  // Renumber all of the machine basic blocks in the function, guaranteeing that
  // the numbers agree with the position of the block in the function.
  MF->RenumberBlocks();

  // Do the initial scan of the function, building up information about the
  // sizes of each block.
  scanFunction();

  bool MadeChange = false;
  while (relaxBranchInstructions())
    MadeChange = true;

  // After a while, this might be made debug-only, but it is not expensive.
  verify();

  BlockInfo.clear();

  return MadeChange;
}
