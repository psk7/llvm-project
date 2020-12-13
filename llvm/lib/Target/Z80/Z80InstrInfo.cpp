//===-- Z80InstrInfo.cpp - Z80 Instruction Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Z80 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "Z80InstrInfo.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Function.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/TargetRegistry.h"

#include "Z80.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80RegisterInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "MCTargetDesc/Z80MCCodeEmitter.h"

#define GET_INSTRINFO_CTOR_DTOR
#include "Z80GenInstrInfo.inc"

namespace llvm {

Z80InstrInfo::Z80InstrInfo()
    : Z80GenInstrInfo(Z80::ADJCALLSTACKDOWN, Z80::ADJCALLSTACKUP), RI() {}

void Z80InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  const Z80Subtarget &STI = MBB.getParent()->getSubtarget<Z80Subtarget>();
  const Z80RegisterInfo &TRI = *STI.getRegisterInfo();
  unsigned Opc;

  if ((Z80::XDREGSRegClass.contains(DestReg) &&
       (Z80::DE == SrcReg || Z80::BC == SrcReg)) ||
      (Z80::XDREGSRegClass.contains(SrcReg) &&
       (Z80::DE == DestReg || Z80::BC == DestReg))) {
    Register DestLo, DestHi, SrcLo, SrcHi;

    TRI.splitReg(DestReg, DestLo, DestHi);
    TRI.splitReg(SrcReg, SrcLo, SrcHi);

    // Copy each individual register with the `LD` instruction.
    BuildMI(MBB, MI, DL, get(Z80::LDRdRr8), DestLo)
        .addReg(SrcLo, getKillRegState(KillSrc));
    BuildMI(MBB, MI, DL, get(Z80::LDRdRr8), DestHi)
        .addReg(SrcHi, getKillRegState(KillSrc));

    return;
  }

  if (Z80::XDREGSRegClass.contains(DestReg) && Z80::DREGSRegClass.contains(SrcReg))
  {
    BuildMI(MBB, MI, DL, get(Z80::PUSHRr))
        .addReg(SrcReg, getKillRegState(KillSrc));

    BuildMI(MBB, MI, DL, get(Z80::POPRd), DestReg);

    return;
  }

  if (Z80::DREGSRegClass.contains(DestReg, SrcReg)) {
    Register DestLo, DestHi, SrcLo, SrcHi;

    TRI.splitReg(DestReg, DestLo, DestHi);
    TRI.splitReg(SrcReg,  SrcLo,  SrcHi);

    // Copy each individual register with the `LD` instruction.
    BuildMI(MBB, MI, DL, get(Z80::LDRdRr8), DestLo)
        .addReg(SrcLo, getKillRegState(KillSrc));
    BuildMI(MBB, MI, DL, get(Z80::LDRdRr8), DestHi)
        .addReg(SrcHi, getKillRegState(KillSrc));


    /*if (STI.hasMOVW() && Z80::DREGSMOVWRegClass.contains(DestReg, SrcReg)) {
      BuildMI(MBB, MI, DL, get(Z80::MOVWRdRr), DestReg)
          .addReg(SrcReg, getKillRegState(KillSrc));
    } else {
      Register DestLo, DestHi, SrcLo, SrcHi;

      TRI.splitReg(DestReg, DestLo, DestHi);
      TRI.splitReg(SrcReg,  SrcLo,  SrcHi);

      // Copy each individual register with the `MOV` instruction.
      BuildMI(MBB, MI, DL, get(Z80::MOVRdRr), DestLo)
        .addReg(SrcLo, getKillRegState(KillSrc));
      BuildMI(MBB, MI, DL, get(Z80::MOVRdRr), DestHi)
        .addReg(SrcHi, getKillRegState(KillSrc));
    }*/
  } else {
    if (Z80::GPR8RegClass.contains(DestReg, SrcReg)) {
      Opc = Z80::LDRdRr8;
    } /*else if (SrcReg == Z80::SP && Z80::DREGSRegClass.contains(DestReg)) {
      Opc = Z80::SPREAD;
    } else if (DestReg == Z80::SP && Z80::DREGSRegClass.contains(SrcReg)) {
      Opc = Z80::SPWRITE;
    }*/ else {
      llvm_unreachable("Impossible reg-to-reg copy");
    }

    BuildMI(MBB, MI, DL, get(Opc), DestReg)
        .addReg(SrcReg, getKillRegState(KillSrc));
  }
}

unsigned Z80InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case Z80::LDDRdPtrQ:
  case Z80::LDDWRdYQ: { //:FIXME: remove this once PR13375 gets fixed
    if (MI.getOperand(1).isFI() && MI.getOperand(2).isImm() &&
        MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

unsigned Z80InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  case Z80::STDPtrQRr:
  case Z80::STDWPtrQRr: {
    if (MI.getOperand(0).isFI() && MI.getOperand(1).isImm() &&
        MI.getOperand(1).getImm() == 0) {
      FrameIndex = MI.getOperand(0).getIndex();
      return MI.getOperand(2).getReg();
    }
    break;
  }
  default:
    break;
  }

  return 0;
}

void Z80InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  Z80MachineFunctionInfo *AFI = MF.getInfo<Z80MachineFunctionInfo>();

  AFI->setHasSpills(true);

  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = Z80::STDPtrQRr;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    Opcode = Z80::STDWPtrQRr;
  } else {
    llvm_unreachable("Cannot store this register into a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode))
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addReg(SrcReg, getKillRegState(isKill))
      .addMemOperand(MMO);
}

void Z80InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  DebugLoc DL;
  if (MI != MBB.end()) {
    DL = MI->getDebugLoc();
  }

  MachineFunction &MF = *MBB.getParent();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  MachineMemOperand *MMO = MF.getMachineMemOperand(
      MachinePointerInfo::getFixedStack(MF, FrameIndex),
      MachineMemOperand::MOLoad, MFI.getObjectSize(FrameIndex),
      MFI.getObjectAlign(FrameIndex));

  unsigned Opcode = 0;
  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    Opcode = Z80::LDDRdPtrQ;
  } else if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    // Opcode = Z80::LDDWRdPtrQ;
    //:FIXME: remove this once PR13375 gets fixed
    Opcode = Z80::LDDWRdYQ;
  } else {
    llvm_unreachable("Cannot load this register from a stack slot!");
  }

  BuildMI(MBB, MI, DL, get(Opcode), DestReg)
      .addFrameIndex(FrameIndex)
      .addImm(0)
      .addMemOperand(MMO);
}

const MCInstrDesc &Z80InstrInfo::getBrCond(Z80CC::CondCodes CC) const {
  llvm_unreachable("Z80InstrInfo::getBrCond");
  /*switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case Z80CC::COND_EQ:
    return get(Z80::BREQk);
  case Z80CC::COND_NE:
    return get(Z80::BRNEk);
  case Z80CC::COND_GE:
    return get(Z80::BRGEk);
  case Z80CC::COND_LT:
    return get(Z80::BRLTk);
  case Z80CC::COND_SH:
    return get(Z80::BRSHk);
  case Z80CC::COND_LO:
    return get(Z80::BRLOk);
  case Z80CC::COND_MI:
    return get(Z80::BRMIk);
  case Z80CC::COND_PL:
    return get(Z80::BRPLk);
  }*/
}

Z80CC::CondCodes Z80InstrInfo::getCondFromBranchOpc(unsigned Opc) const {
  llvm_unreachable("Z80InstrInfo::getCondFromBranchOpc");
  /*switch (Opc) {
  default:
    return Z80CC::COND_INVALID;
  case Z80::BREQk:
    return Z80CC::COND_EQ;
  case Z80::BRNEk:
    return Z80CC::COND_NE;
  case Z80::BRSHk:
    return Z80CC::COND_SH;
  case Z80::BRLOk:
    return Z80CC::COND_LO;
  case Z80::BRMIk:
    return Z80CC::COND_MI;
  case Z80::BRPLk:
    return Z80CC::COND_PL;
  case Z80::BRGEk:
    return Z80CC::COND_GE;
  case Z80::BRLTk:
    return Z80CC::COND_LT;
  }*/
}

Z80CC::CondCodes Z80InstrInfo::getOppositeCondition(Z80CC::CondCodes CC) const {
  switch (CC) {
  default:
    llvm_unreachable("Unable to get opposite condition code");
  case Z80CC::COND_Z:
    return Z80CC::COND_NZ;
  case Z80CC::COND_NZ:
    return Z80CC::COND_Z;
  case Z80CC::COND_C:
    return Z80CC::COND_NC;
  case Z80CC::COND_NC:
    return Z80CC::COND_C;
  case Z80CC::COND_PO:
    return Z80CC::COND_PE;
  case Z80CC::COND_PE:
    return Z80CC::COND_PO;
  case Z80CC::COND_P:
    return Z80CC::COND_M;
  case Z80CC::COND_M:
    return Z80CC::COND_P;
  }
}

bool Z80InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
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
    //:TODO: add here jmp
    if (I->getOpcode() == Z80::JRk) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after a JMP, delete them.
      while (std::next(I) != MBB.end()) {
        std::next(I)->eraseFromParent();
      }

      Cond.clear();
      FBB = 0;

      // Delete the JMP if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = 0;
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
    //Z80CC::CondCodes BranchCode = getCondFromBranchOpc(I->getOpcode());
    Z80CC::CondCodes BranchCode =
        static_cast<Z80CC::CondCodes>(I->getOperand(1).getImm());

    if (I->getOpcode() == Z80::JRk) {
      return true; // Can't handle indirect branch.
    }

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (AllowModify && UnCondBrIter != MBB.end() &&
          MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jrCC L1
        //     jr L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jrnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        BranchCode = getOppositeCondition(BranchCode);
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(Z80::JRCC))
            .addMBB(UnCondBrIter->getOperand(0).getMBB()).addImm(BranchCode);
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(Z80::JRk))
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

    Z80CC::CondCodes OldBranchCode = (Z80CC::CondCodes)Cond[0].getImm();
    // If the conditions are the same, we can leave them alone.
    if (OldBranchCode == BranchCode) {
      continue;
    }

    return true;
  }

  return false;
}

unsigned Z80InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL,
                                    int *BytesAdded) const {
  if (BytesAdded) *BytesAdded = 0;

  // Shouldn't be a fall through.
  assert(TBB && "insertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 1 || Cond.size() == 0) &&
         "Z80 branch conditions have one component!");

  if (Cond.empty()) {
    assert(!FBB && "Unconditional branch with multiple successors!");
    auto &MI = *BuildMI(&MBB, DL, get(Z80::JRk)).addMBB(TBB);
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(MI);
    return 1;
  }

  // Conditional branch.
  unsigned Count = 0;
  auto &CondMI = *BuildMI(&MBB, DL, get(Z80::JRCC))
                      .addMBB(TBB)
                      .addImm(Cond[0].getImm()); /*Z80CC::CondCodes*/

  if (BytesAdded) *BytesAdded += getInstSizeInBytes(CondMI);
  ++Count;

  if (FBB) {
    // Two-way Conditional branch. Insert the second branch.
    auto &MI = *BuildMI(&MBB, DL, get(Z80::JRk)).addMBB(FBB);
    if (BytesAdded) *BytesAdded += getInstSizeInBytes(MI);
    ++Count;
  }

  return Count;
}

unsigned Z80InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  if (BytesRemoved) *BytesRemoved = 0;

  MachineBasicBlock::iterator I = MBB.end();
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugInstr()) {
      continue;
    }
    //:TODO: add here the missing jmp instructions once they are implemented
    // like jmp, {e}ijmp, and other cond branches, ...
    auto oc = I->getOpcode();
    if (oc != Z80::JRk && oc != Z80::JRCC) {
      break;
    }

    // Remove the branch.
    if (BytesRemoved) *BytesRemoved += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  return Count;
}

bool Z80InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1 && "Invalid Z80 branch condition!");

  /*Z80CC::CondCodes CC = static_cast<Z80CC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));*/

  Z80CC::CondCodes CC = static_cast<Z80CC::CondCodes>(Cond[0].getImm());
  Cond[0].setImm(getOppositeCondition(CC));

  return false;
}

unsigned Z80InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  unsigned Opcode = MI.getOpcode();

  Z80II::InstPrefixInfo P(MI);

  switch (Opcode) {
  // A regular instruction
  default: {
    return P.getSize();
  }
  case TargetOpcode::EH_LABEL:
  case TargetOpcode::IMPLICIT_DEF:
  case TargetOpcode::KILL:
  case TargetOpcode::DBG_VALUE:
    return 0;
  case TargetOpcode::INLINEASM:
  case TargetOpcode::INLINEASM_BR: {
    const MachineFunction &MF = *MI.getParent()->getParent();
    const Z80TargetMachine &TM = static_cast<const Z80TargetMachine&>(MF.getTarget());
    const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();

    return TII.getInlineAsmLength(MI.getOperand(0).getSymbolName(),
                                  *TM.getMCAsmInfo());
  }
  }
}

MachineBasicBlock *
Z80InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("unexpected opcode!");
  case Z80::JRCC:
  case Z80::JRk:
    return MI.getOperand(0).getMBB();
  /*case Z80::JMPk:
  case Z80::CALLk:
  case Z80::RJMPk:
  case Z80::BREQk:
  case Z80::BRNEk:
  case Z80::BRSHk:
  case Z80::BRLOk:
  case Z80::BRMIk:
  case Z80::BRPLk:
  case Z80::BRGEk:
  case Z80::BRLTk:
    return MI.getOperand(0).getMBB();
  case Z80::BRBSsk:
  case Z80::BRBCsk:
    return MI.getOperand(1).getMBB();
  case Z80::SBRCRrB:
  case Z80::SBRSRrB:
  case Z80::SBICAb:
  case Z80::SBISAb:*/
    llvm_unreachable("unimplemented branch instructions");
  }
}

bool Z80InstrInfo::isBranchOffsetInRange(unsigned BranchOp,
                                         int64_t BrOffset) const {
  switch (BranchOp) {
  default:
    llvm_unreachable("unexpected opcode!");
  case Z80::JRk:
  case Z80::JRCC:
    return isIntN(8, BrOffset);
  /*case Z80::JMPk:
  case Z80::CALLk:
    return true;
  case Z80::RJMPk:
    return isIntN(13, BrOffset);
  case Z80::BRBSsk:
  case Z80::BRBCsk:
  case Z80::BREQk:
  case Z80::BRNEk:
  case Z80::BRSHk:
  case Z80::BRLOk:
  case Z80::BRMIk:
  case Z80::BRPLk:
  case Z80::BRGEk:
  case Z80::BRLTk:
    return isIntN(7, BrOffset);*/
  }
}

unsigned Z80InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                            MachineBasicBlock &NewDestBB,
                                            const DebugLoc &DL,
                                            int64_t BrOffset,
                                            RegScavenger *RS) const {
  llvm_unreachable("Z80InstrInfo::insertIndirectBranch");
/*    // This method inserts a *direct* branch (JMP), despite its name.
    // LLVM calls this method to fixup unconditional branches; it never calls
    // insertBranch or some hypothetical "insertDirectBranch".
    // See lib/CodeGen/RegisterRelaxation.cpp for details.
    // We end up here when a jump is too long for a RJMP instruction.
    auto &MI = *BuildMI(&MBB, DL, get(Z80::JMPk)).addMBB(&NewDestBB);

    return getInstSizeInBytes(MI);*/
}

bool Z80InstrInfo::isCommuteAllowed(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    return TargetInstrInfo::isCommuteAllowed(MI);
  case Z80::ADDRdRr8:
  case Z80::ADDRdRr16:
  case Z80::ORrr8:
  case Z80::XORrr8:
    const MachineRegisterInfo &RI = MI.getParent()->getParent()->getRegInfo();

    auto RegClass0 = RI.getRegClass(MI.getOperand(0).getReg())->getID();
    auto RegClass1 = RI.getRegClass(MI.getOperand(1).getReg())->getID();
    auto RegClass2 = RI.getRegClass(MI.getOperand(2).getReg())->getID();

    return !(RegClass0 == Z80::ACCRegClassID &&
             RegClass1 == Z80::ACCRegClassID &&
             RegClass2 == Z80::GPR8RegClassID);
  }
}

} // end of namespace llvm

