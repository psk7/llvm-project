//===-- PDPInstrInfo.cpp - PDP Instruction Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PDP implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "PDPInstrInfo.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/ErrorHandling.h"

#include "PDP.h"
#include "PDPMachineFunctionInfo.h"
#include "PDPRegisterInfo.h"
#include "PDPTargetMachine.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "MCTargetDesc/PDPMCCodeEmitter.h"
#include "PDPGenInstrInfo.inc"

namespace llvm {

PDPInstrInfo::PDPInstrInfo(PDPSubtarget &STI)
    : PDPGenInstrInfo(PDP::ADJCALLSTACKDOWN, PDP::ADJCALLSTACKUP), RI(),
      STI(STI) { }

void PDPInstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc,
                               bool RenamableDest, bool RenamableSrc) const {
  const PDPRegisterInfo &TRI = *STI.getRegisterInfo();
  //unsigned Opc;

  if (SrcReg == PDP::R0l && DestReg == PDP::R0)
    return;
  if (SrcReg == PDP::R1l && DestReg == PDP::R1)
    return;
  if (SrcReg == PDP::R2l && DestReg == PDP::R2)
    return;
  if (SrcReg == PDP::R3l && DestReg == PDP::R3)
    return;
  if (SrcReg == PDP::R4l && DestReg == PDP::R4)
    return;
  if (SrcReg == PDP::R5l && DestReg == PDP::R5  )
    return;
  if (SrcReg == PDP::R0 && DestReg == PDP::R0l)
    return;
  if (SrcReg == PDP::R1 && DestReg == PDP::R1l)
    return;
  if (SrcReg == PDP::R2 && DestReg == PDP::R2l)
    return;
  if (SrcReg == PDP::R3 && DestReg == PDP::R3l)
    return;
  if (SrcReg == PDP::R4 && DestReg == PDP::R4l)
    return;
  if (SrcReg == PDP::R5 && DestReg == PDP::R5l)
    return;

  if (PDP::GPR16RegClass.contains(DestReg, SrcReg)) {
    // ReSharper disable once CppExpressionWithoutSideEffects
    auto &mi = BuildMI(MBB, MI, DL, get(PDP::MOV))
        .addReg(SrcReg, getKillRegState(KillSrc))
        .addImm(0)    // mode
        .addImm(0)
        .addImm(0)
        .addReg(DestReg)
        .addImm(0)    // mode
        .addImm(0)
        .addImm(0);

    mi->getOperand(8).setIsDead(true);

    return;
  } else if (PDP::GPR16lRegClass.contains(DestReg, SrcReg)) {
    // ReSharper disable once CppExpressionWithoutSideEffects
    auto &clr = BuildMI(MBB, MI, DL, get(PDP::CLRB))
          .addReg(TRI.getSuperReg(DestReg))
          .addImm(0)    // mode
          .addImm(0)
          .addImm(0);

    clr->getOperand(4).setIsDead(true);

    auto &bis = BuildMI(MBB, MI, DL, get(PDP::BISB))
          .addReg(TRI.getSuperReg(SrcReg), getKillRegState(KillSrc))
          .addImm(0)    // mode
          .addImm(0)
          .addImm(0)
          .addReg(TRI.getSuperReg(DestReg))
          .addImm(0)    // mode
          .addImm(0)
          .addImm(0);

    bis->getOperand(8).setIsDead(true);

    return;
  }
  else {
    dbgs() << printReg(SrcReg) << "->" << printReg(DestReg);

    llvm_unreachable("Impossible reg-to-reg copy");
  }

  /*if (PDP::DREGSRegClass.contains(DestReg, SrcReg)) {
    // If our PDP has `movw`, let's emit that; otherwise let's emit two separate
    // `mov`s.
    if (STI.hasMOVW() && PDP::DREGSMOVWRegClass.contains(DestReg, SrcReg)) {
      BuildMI(MBB, MI, DL, get(PDP::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      Register DestLo, DestHi, SrcLo, SrcHi;

      TRI.splitReg(DestReg, DestLo, DestHi);
      TRI.splitReg(SrcReg, SrcLo, SrcHi);

      // Emit the copies.
      // The original instruction was for a register pair, of which only one
      // register might have been live. Add 'undef' to satisfy the machine
      // verifier, when subreg liveness is enabled.
      // TODO: Eliminate these unnecessary copies.
      if (DestLo == SrcHi) {
        BuildMI(MBB, MI, DL, get(PDP::MOVRdRr), DestHi)
            .addReg(SrcHi, getKillRegState(KillSrc) | RegState::Undef);
        BuildMI(MBB, MI, DL, get(PDP::MOVRdRr), DestLo)
            .addReg(SrcLo, getKillRegState(KillSrc) | RegState::Undef);
      } else {
        BuildMI(MBB, MI, DL, get(PDP::MOVRdRr), DestLo)
            .addReg(SrcLo, getKillRegState(KillSrc) | RegState::Undef);
        BuildMI(MBB, MI, DL, get(PDP::MOVRdRr), DestHi)
            .addReg(SrcHi, getKillRegState(KillSrc) | RegState::Undef);
      }
    }
  } else {
    if (PDP::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = PDP::MOVRdRr;
    } else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }*/
}

Register PDPInstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  if (MI.getOpcode() == PDP::L__) {
    if (MI.getOperand(1).getReg() == PDP::R6 && MI.getOperand(2).getImm() == 6) {
      FrameIndex = MI.getOperand(3).getIndex();
      return MI.getOperand(0).getReg();
    }
  }

  return 0;

  /*switch (MI.getOpcode()) {
  case PDP::LDDRdPtrQ:
  case PDP::LDDWRdYQ: { //: FIXME: remove this once PR13375 gets fixed
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }*/

  return 0;
}

Register PDPInstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  auto opcode = MI.getOpcode();

  if (opcode == PDP::S__ || opcode == PDP::SB_) {
    if (MI.getOperand(1).getReg() == PDP::R6 && MI.getOperand(2).getImm() > 0 && MI.getOperand(3).isFI()) {
      FrameIndex = MI.getOperand(3).getIndex();
      return MI.getOperand(0).getReg();
    }

    return 0;
  } else if (opcode == PDP::MOV || opcode == PDP::MOVB) {
    if (MI.getOperand(1).getImm() == 0 && MI.getOperand(4).getReg() == PDP::R6) {
      auto tm = MI.getOperand(5).getImm();

      if (tm >= 1 && tm <=6)
        return MI.getOperand(0).getReg();

      return 0;
    }
  }

  return 0;

  /*switch (MI.getOpcode()) {
  case PDP::STDPtrQRr:
  case PDP::STDWPtrQRr: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }*/

  return 0;
}

void PDPInstrInfo::storeRegToStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg,
    bool isKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, Register VReg,
    MachineInstr::MIFlag Flags) const {

  MachineFunction &MF = *MBB.getParent();
  PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();

  AFI->setHasSpills(true);

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = PDP::SB_;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = PDP::S__;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  auto &mi = BuildMI(MBB, MI, DebugLoc(), get(Opcode))
      .addReg(SrcReg, getKillRegState(isKill))
      .addReg(TRI->getFrameRegister(MF))
      .addImm(6)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);

  mi->getOperand(5).setIsDead();
}

void PDPInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI,
                                        Register VReg,
                                        MachineInstr::MIFlag Flags) const {

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = PDP::LB_;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = PDP::LDDWRdPtrQ;
    Opcode = PDP::L__;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  auto &mi = BuildMI(MBB, MI, DebugLoc(), get(Opcode), DestReg)
      .addReg(TRI->getFrameRegister(MF))
      .addImm(6)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);

  mi->getOperand(5).setIsDead();
}

const MCInstrDesc &PDPInstrInfo::getBrCond(PDPCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case PDPCC::COND_EQ:
    return get(PDP::BEQ);
  case PDPCC::COND_NE:
    return get(PDP::BNE);

  case PDPCC::COND_GE:
    return get(PDP::BGE);
  case PDPCC::COND_GT:
    return get(PDP::BGT);
  case PDPCC::COND_LT:
    return get(PDP::BLT);
  case PDPCC::COND_LE:
    return get(PDP::BLE);

  case PDPCC::COND_SH:
    return get(PDP::BHIS);
  case PDPCC::COND_LO:
    return get(PDP::BLO);
  case PDPCC::COND_SL:
    return get(PDP::BLOS);
  case PDPCC::COND_HI:
    return get(PDP::BHI);

  case PDPCC::COND_MI:
    return get(PDP::BMI);
  case PDPCC::COND_PL:
    return get(PDP::BPL);
  }
}

PDPCC::CondCodes PDPInstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  switch (Opc) {
  default:
    return PDPCC::COND_INVALID;
  case PDP::BEQ:
    return PDPCC::COND_EQ;
  case PDP::BNE:
    return PDPCC::COND_NE;
  case PDP::BHIS:
    return PDPCC::COND_SH;
  case PDP::BLO:
    return PDPCC::COND_LO;
  case PDP::BMI:
    return PDPCC::COND_MI;
  case PDP::BPL:
    return PDPCC::COND_PL;
  case PDP::BGE:
    return PDPCC::COND_GE;
  case PDP::BGT:
    return PDPCC::COND_GT;
  case PDP::BLT:
    return PDPCC::COND_LT;
  case PDP::BLE:
    return PDPCC::COND_LE;
  }
}

PDPCC::CondCodes PDPInstrInfo::getOppositeCondition(PDPCC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Invalid condition!");
  case PDPCC::COND_EQ:
    return PDPCC::COND_NE;
  case PDPCC::COND_NE:
    return PDPCC::COND_EQ;
  case PDPCC::COND_SH:
    return PDPCC::COND_LO;
  case PDPCC::COND_LO:
    return PDPCC::COND_SH;
  case PDPCC::COND_GE:
    return PDPCC::COND_LT;
  case PDPCC::COND_LT:
    return PDPCC::COND_GE;
  case PDPCC::COND_GT:
    return PDPCC::COND_LE;
  case PDPCC::COND_LE:
    return PDPCC::COND_GT;
  case PDPCC::COND_MI:
    return PDPCC::COND_PL;
  case PDPCC::COND_PL:
    return PDPCC::COND_MI;
  }
}

bool PDPInstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end();
  MachineBasicBlock::iterator UnCondBrIter = MBB.end();

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }

    // Working from the bottom, when we see a non-terminator
    // instruction, we're done.
    if (!isUnpredicatedTerminator(*I)) {
      break;
    }

    // A terminator that isn't a branch can't easily be handled
    // by this analysis.
    if (!I->getDesc().isBranch()) {
      return true;
    }

    // Handle unconditional branches.
    //: TODO: add here jmp
    if (I->getOpcode() == PDP::BR) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      MBB.erase(std::next(I), MBB.end());

      Cond.clear();
      FBB = nullptr;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = MBB.end();
        continue;
      }

      // TBB is used to indicate the unconditinal destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    PDPCC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    if (BranchCode == PDPCC::COND_INVALID) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() && MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        unsigned JNCC = getBrCond(BranchCode).getOpcode();
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(JNCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(PDP::BR))
            .addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(BranchCode));
      continue;
    }

    // Handle subsequent conditional branches. Only handle the case where all
    // conditional branches branch to the same destination.
    assert(Cond.size() == 1);
    assert(TBB);

    // Only handle the case where all conditional branches branch to
    // the same destination.
    if (TBB != I->getOperand(0).getMBB()) {
      return true;
    }

    PDPCC::CondCodes OldBranchCode = (PDPCC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned PDPInstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL, int *BytesAdded) const {
  if (BytesAdded)
    *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "PDP branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    auto &MI = *BuildMI(&MBB, DL, get(PDP::BR)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  PDPCC::CondCodes CC = (PDPCC::CondCodes)Cond[0].getImm();
  auto &CondMI = *BuildMI(&MBB, DL, getBrCond(CC)).addMBB(TBB);

  if (BytesAdded)
    *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    auto &MI = *BuildMI(&MBB, DL, get(PDP::BR)).addMBB(FBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned PDPInstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved)
    *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //: TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    if (I->getOpcode() != PDP::BR &&
        getCondFromBranchOpc(I->getOpcode()) == PDPCC::COND_INVALID) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved)
      *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool PDPInstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid PDP branch condition!");

  auto CC = static_cast<PDPCC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned PDPInstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  switch (Opcode) {
    // A regular instruction
    default: {
      if (MI.isPseudo()) {
        MI.dump();
      }

      assert(!MI.isPseudo() && "Cant obtain size of pseudo instruction");

      const MCInstrDesc &Desc = get(Opcode);
      auto Size = Desc.getSize();

      assert(Size != 0);

      PDPMCCodeEmitter::forEachArg(Desc, [&MI, &Size](const unsigned N) -> void {
        const auto reg = MI.getOperand(N).getReg();
        const auto mode = MI.getOperand(N + 1).getImm();

        if ((mode == 2 || mode == 3) && reg == PDP::R7)
          Size += 2;
        if (mode == 6 || mode == 7)
          Size += 2;
      });

      return Size;
    }
    case TargetOpcode::EH_LABEL:
    case TargetOpcode::IMPLICIT_DEF:
    case TargetOpcode::KILL:
    case TargetOpcode::DBG_VALUE:
    case TargetOpcode::DBG_VALUE_LIST:
      return 0;
    case TargetOpcode::INLINEASM:
    case TargetOpcode::INLINEASM_BR: {
      const MachineFunction &MF = *MI.getParent()->getParent();
      const PDPTargetMachine &TM =
          static_cast<const PDPTargetMachine &>(MF.getTarget());
      const TargetInstrInfo &TII = *STI.getInstrInfo();
      return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                    *TM.getMCAsmInfo());
    }
  }
}

MachineBasicBlock *
PDPInstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  const auto OpCode = MI.getOpcode();

  if (OpCode == PDP::JMP && MI.getOperand(0).getReg() == PDP::R7 && MI.getOperand(1).getImm() == 3) {
    const auto &Op2 = MI.getOperand(2);
    if (Op2.isMBB())
      return Op2.getMBB();
  }

  switch (OpCode) {
  default:
    llvm_unreachable("unexpected opcode!");
  case PDP::BR:
  case PDP::BEQ:
  case PDP::BNE:
  case PDP::BHIS:
  case PDP::BLO:
  case PDP::BMI:
  case PDP::BPL:
  case PDP::BGE:
  case PDP::BGT:
  case PDP::BLT:
  case PDP::BLE:
  case PDP::BLOS:
  case PDP::BHI:
    return MI.getOperand(0).getMBB();
  case PDP::CBR:
  case PDP::CBRB:
    return MI.getOperand(9).getMBB();
  }
}

bool PDPInstrInfo::isBranchOffsetInRange(unsigned BranchOp, int64_t BrOffset) const {

  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");

  case PDP::JMP:
      return true;

  case PDP::BR:
  case PDP::BEQ:
  case PDP::BNE:
  case PDP::BVC:
  case PDP::BVS:

  case PDP::BPL:
  case PDP::BMI:
  case PDP::BGE:
  case PDP::BLT:
  case PDP::BGT:
  case PDP::BLE:

  case PDP::BHI:
  case PDP::BLOS:
  case PDP::BHIS:
  case PDP::BLO:

  case PDP::CBR:
  case PDP::CBRB:
    return isIntN(8, (BrOffset - 2) / 2);
  }
}

void PDPInstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                        MachineBasicBlock &NewDestBB,
                                        MachineBasicBlock &RestoreBB,
                                        const DebugLoc &DL, int64_t BrOffset,
                                        RegScavenger *RS) const {

  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/RegisterRelaxation.cpp for details.

  BuildMI(&MBB, DL, get(PDP::JMP))
      .addReg(PDP::R7)
      .addImm(3)
      .addMBB(&NewDestBB)
      .addImm(0);
}

/*bool PDPInstrInfo::isPredicated(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
    case PDP::BNE:
    case PDP::BEQ:
    case PDP::BVC:
    case PDP::BVS:
    case PDP::BPL:
    case PDP::BMI:
    case PDP::BGE:
    case PDP::BLT:
    case PDP::BGT:
    case PDP::BLE:
    case PDP::BHI:
    case PDP::BLOS:
    case PDP::BHIS:
    case PDP::BLO:
      return true;

    default:
      return false;
  }
}*/

} // end of namespace llvm
