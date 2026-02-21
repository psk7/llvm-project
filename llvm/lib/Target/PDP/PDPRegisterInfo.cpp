//===-- PDPRegisterInfo.cpp - PDP Register Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PDP implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "PDPRegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

#include "PDP.h"
#include "PDPInstrInfo.h"
#include "PDPMachineFunctionInfo.h"
#include "PDPTargetMachine.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"

#define GET_REGINFO_TARGET_DESC
#include "PDPGenRegisterInfo.inc"

namespace llvm {

PDPRegisterInfo::PDPRegisterInfo() : PDPGenRegisterInfo(0) {}

const uint16_t *
PDPRegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const PDPMachineFunctionInfo *AFI = MF->getInfo<PDPMachineFunctionInfo>();
//  const PDPSubtarget &STI = MF->getSubtarget<PDPSubtarget>();
    return AFI->isInterruptOrSignalHandler() ? CSR_Interrupts_SaveList
                                             : CSR_Normal_SaveList;
/*
  if (STI.hasTinyEncoding())
    return AFI->isInterruptOrSignalHandler() ? CSR_InterruptsTiny_SaveList
                                             : CSR_NormalTiny_SaveList;
  else
    return AFI->isInterruptOrSignalHandler() ? CSR_Interrupts_SaveList
                                             : CSR_Normal_SaveList;
*/
}

const uint32_t *
PDPRegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CC) const {
//  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  return CSR_Normal_RegMask;
//  return STI.hasTinyEncoding() ? CSR_NormalTiny_RegMask : CSR_Normal_RegMask;
}

BitVector PDPRegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  //if (MF.getInfo<PDPMachineFunctionInfo>()->getHasStacksCall())
  //  Reserved.set(PDP::R5);

  Reserved.set(PDP::R6);
  Reserved.set(PDP::R7);
  Reserved.set(PDP::R6l);
  Reserved.set(PDP::R7l);

  return Reserved;
}

const TargetRegisterClass *
PDPRegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &MF) const {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    return &PDP::GPR16RegClass;
  }

  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    return &PDP::GPR16lRegClass;
  }

  /*if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    return &PDP::GPR8RegClass;
  }*/

  llvm_unreachable("Invalid register size");
}

/// Fold a frame offset shared between two add instructions into a single one.
static void foldFrameOffset(MachineBasicBlock::iterator &II, int &Offset,
                            Register DstReg) {
  llvm_unreachable("foldFrameOffset");

  /*MachineInstr &MI = *II;
  int Opcode = MI.getOpcode();

  // Don't bother trying if the next instruction is not an add or a sub.
  if ((Opcode != PDP::SUBIWRdK) && (Opcode != PDP::ADIWRdK)) {
    return;
  }

  // Check that DstReg matches with next instruction, otherwise the instruction
  // is not related to stack address manipulation.
  if (DstReg != MI.getOperand(0).getReg()) {
    return;
  }

  // Add the offset in the next instruction to our offset.
  switch (Opcode) {
  case PDP::SUBIWRdK:
    Offset += -MI.getOperand(2).getImm();
    break;
  case PDP::ADIWRdK:
    Offset += MI.getOperand(2).getImm();
    break;
  }

  // Finally remove the instruction.
  II++;
  MI.eraseFromParent();*/
}

bool PDPRegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected SPAdj value");

  MachineInstr &MI = *II;
  DebugLoc dl = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction &MF = *MBB.getParent();
  const PDPTargetMachine &TM = (const PDPTargetMachine &)MF.getTarget();
  const TargetInstrInfo &TII = *TM.getSubtargetImpl()->getInstrInfo();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFI = TM.getSubtargetImpl()->getFrameLowering();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  int Offset = MFI.getObjectOffset(FrameIndex);

  // Add one to the offset because SP points to an empty slot.
  Offset += MFI.getStackSize() - TFI->getOffsetOfLocalArea();
  // Fold incoming offset.
  auto &op = MI.getOperand(FIOperandNum);
  auto &offs = MI.getOperand(FIOperandNum + 1);

  Offset += offs.getImm();
  offs.ChangeToImmediate(0);

  if (op.isImm()) {
    llvm_unreachable("!!!");
    Offset += op.getImm();
  }
  else if (op.isFI()) {
    op.ChangeToImmediate(Offset);

    return false;
  }

  llvm_unreachable("PDPRegisterInfo::eliminateFrameIndex");

  /*// This is actually "load effective address" of the stack slot
  // instruction. We have only two-address instructions, thus we need to
  // expand it into move + add.
  if (MI.getOpcode() == PDP::FRMIDX) {
    Register DstReg = MI.getOperand(0).getReg();
    assert(DstReg != PDP::R29R28 && "Dest reg cannot be the frame pointer");

    // Copy the frame pointer.
    if (STI.hasMOVW()) {
      BuildMI(MBB, MI, dl, TII.get(PDP::MOVWRdRr), DstReg)
          .addReg(PDP::R29R28);
    } else {
      Register DstLoReg, DstHiReg;
      splitReg(DstReg, DstLoReg, DstHiReg);
      BuildMI(MBB, MI, dl, TII.get(PDP::MOVRdRr), DstLoReg)
          .addReg(PDP::R28);
      BuildMI(MBB, MI, dl, TII.get(PDP::MOVRdRr), DstHiReg)
          .addReg(PDP::R29);
    }

    assert(Offset > 0 && "Invalid offset");

    // We need to materialize the offset via an add instruction.
    unsigned Opcode;

    II++; // Skip over the FRMIDX instruction.

    // Generally, to load a frame address two add instructions are emitted that
    // could get folded into a single one:
    //  movw    r31:r30, r29:r28
    //  adiw    r31:r30, 29
    //  adiw    r31:r30, 16
    // to:
    //  movw    r31:r30, r29:r28
    //  adiw    r31:r30, 45
    if (II != MBB.end())
      foldFrameOffset(II, Offset, DstReg);

    // Select the best opcode based on DstReg and the offset size.
    switch (DstReg) {
    case PDP::R25R24:
    case PDP::R27R26:
    case PDP::R31R30: {
      if (isUInt<6>(Offset) && STI.hasADDSUBIW()) {
        Opcode = PDP::ADIWRdK;
        break;
      }
      [[fallthrough]];
    }
    default: {
      // This opcode will get expanded into a pair of subi/sbci.
      Opcode = PDP::SUBIWRdK;
      Offset = -Offset;
      break;
    }
    }

    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(Opcode), DstReg)
                            .addReg(DstReg, RegState::Kill)
                            .addImm(Offset);
    New->getOperand(3).setIsDead();

    MI.eraseFromParent(); // remove FRMIDX

    return false;
  }

  MI.getOperand(FIOperandNum).ChangeToRegister(PDP::R29R28, false);
  assert(isUInt<6>(Offset) && "Offset is out of range");
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
  return false;*/
}

Register PDPRegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const PDPMachineFunctionInfo *MFI = MF.getInfo<PDPMachineFunctionInfo>();

  return PDP::R6;

  /*if (MFI->getHasStacksCall())
    return PDP::R5;
  else
    return PDP::R6;*/

  /*const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  if (TFI->hasFP(MF)) {
    // The Y pointer register
    return PDP::R28;
  }

  return PDP::SP;*/
}

const TargetRegisterClass *
PDPRegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  // FIXME: Currently we're using PDP-gcc as reference, so we restrict
  // ptrs to Y and Z regs. Though PDP-gcc has buggy implementation
  // of memory constraint, so we can fix it and bit PDP-gcc here ;-)
  return &PDP::GPR16RegClass;
}

Register PDPRegisterInfo::getSuperReg(Register Reg) const {
  switch(Reg.id()){
    case PDP::R0l: return PDP::R0;
    case PDP::R1l: return PDP::R1;
    case PDP::R2l: return PDP::R2;
    case PDP::R3l: return PDP::R3;
    case PDP::R4l: return PDP::R4;
    case PDP::R5l: return PDP::R5;
    case PDP::R0: return PDP::R0;
    case PDP::R1: return PDP::R1;
    case PDP::R2: return PDP::R2;
    case PDP::R3: return PDP::R3;
    case PDP::R4: return PDP::R4;
    case PDP::R5: return PDP::R5;
    case PDP::R6: return PDP::R6;
    case PDP::R7: return PDP::R7;
    default: llvm_unreachable("PDPRegisterInfo::getSuperReg");
  }
}

bool PDPRegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {

  return true;

  /*if (this->getRegClass(PDP::PTRDISPREGSRegClassID)->hasSubClassEq(NewRC)) {
    return false;
  }*/

  return TargetRegisterInfo::shouldCoalesce(MI, SrcRC, SubReg, DstRC, DstSubReg,
                                            NewRC, LIS);
}

} // end of namespace llvm
