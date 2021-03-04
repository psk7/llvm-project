//===-- Z80RegisterInfo.cpp - Z80 Register Information --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Z80 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "Z80RegisterInfo.h"

#include "llvm/ADT/BitVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

#include "Z80.h"
#include "Z80InstrInfo.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#define GET_REGINFO_TARGET_DESC
#include "Z80GenRegisterInfo.inc"

namespace llvm {

Z80RegisterInfo::Z80RegisterInfo() : Z80GenRegisterInfo(0) {}

const uint16_t *
Z80RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  //const Z80MachineFunctionInfo *AFI = MF->getInfo<Z80MachineFunctionInfo>();

  /*return AFI->isInterruptOrSignalHandler()
              ? CSR_Interrupts_SaveList
              : CSR_Normal_SaveList;*/

  auto *t = MF->getFunction().getReturnType();

  if (t->isIntegerTy() && t->getIntegerBitWidth() == 32)
    return CSR_Normal_NoDE_SaveList;

  return CSR_Normal_SaveList;
}

const uint32_t *
Z80RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CC) const {
  //const Z80MachineFunctionInfo *AFI = MF.getInfo<Z80MachineFunctionInfo>();

  /*return AFI->isInterruptOrSignalHandler()
              ? CSR_Interrupts_RegMask
              : CSR_Normal_RegMask;*/

  auto *t = MF.getFunction().getReturnType();

  if (t->isIntegerTy() && t->getIntegerBitWidth() == 32)
    return CSR_Normal_NoDE_RegMask;

  return CSR_Normal_RegMask;
}

BitVector Z80RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  //Reserved.set(Z80::IX);
  Reserved.set(Z80::IY);

  //Reserved.set(Z80::XL);
  //Reserved.set(Z80::XH);
  Reserved.set(Z80::YL);
  Reserved.set(Z80::YH);

  Reserved.set(Z80::SP);

  return Reserved;
}

const TargetRegisterClass *
Z80RegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &MF) const {
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
  if (TRI->isTypeLegalForClass(*RC, MVT::i16)) {
    return &Z80::DREGSRegClass;
  }

  if (TRI->isTypeLegalForClass(*RC, MVT::i8)) {
    return &Z80::GPR8RegClass;
  }

  llvm_unreachable("Invalid register size");
}

/// Fold a frame offset shared between two add instructions into a single one.
static void foldFrameOffset(MachineBasicBlock::iterator &II, int &Offset,
                            Register DstReg) {
  MachineInstr &MI = *II;
//  int Opcode = MI.getOpcode();

  // Don't bother trying if the next instruction is not an add or a sub.
//  if (Opcode != Z80::ADIWRdK) {
    return;
//  }

  // Check that DstReg matches with next instruction, otherwise the instruction
  // is not related to stack address manipulation.
  if (DstReg != MI.getOperand(0).getReg()) {
    return;
  }

  // Add the offset in the next instruction to our offset.
  Offset += MI.getOperand(2).getImm();

  // Finally remove the instruction.
  II++;
  MI.eraseFromParent();
}

void Z80RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected SPAdj value");

  MachineInstr &MI = *II;
  DebugLoc dl = MI.getDebugLoc();
  MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction &MF = *MBB.getParent();
  const Z80TargetMachine &TM = (const Z80TargetMachine &)MF.getTarget();
  const TargetInstrInfo &TII = *TM.getSubtargetImpl()->getInstrInfo();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetFrameLowering *TFI = TM.getSubtargetImpl()->getFrameLowering();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  int Offset = MFI.getObjectOffset(FrameIndex);

  // Add one to the offset because SP points to an empty slot.
  Offset += MFI.getStackSize() - TFI->getOffsetOfLocalArea();
  // Fold incoming offset.
  Offset += MI.getOperand(FIOperandNum + 1).getImm();

  // This is actually "load effective address" of the stack slot
  // instruction. We have only two-address instructions, thus we need to
  // expand it into move + add.
  if (MI.getOpcode() == Z80::FRMIDX) {
    Register r = MI.getOperand(0).getReg();

    const auto &LDI16 = BuildMI(MBB, II, dl, TII.get(Z80::LDIWRdK), r);

    II++;

    assert(Offset >= 0 && "Invalid offset");

    MI.setDesc(TII.get(Z80::ADDW));
    MI.getOperand(FIOperandNum).ChangeToRegister(r, false);
    MI.getOperand(FIOperandNum + 1).ChangeToRegister(Z80::SP, false);
    MI.tieOperands(0, 1);

    if (II != MBB.end())
      foldFrameOffset(II, Offset, Z80::HL);

    LDI16.addImm(Offset);

    /*MI.setDesc(TII.get(Z80::MOVWRdRr));
    MI.getOperand(FIOperandNum).ChangeToRegister(Z80::R29R28, false);
    MI.RemoveOperand(2);*/

    /*// We need to materialize the offset via an add instruction.
    unsigned Opcode;
    Register DstReg = MI.getOperand(0).getReg();
    assert(DstReg != Z80::R29R28 && "Dest reg cannot be the frame pointer");

    II++; // Skip over the FRMIDX (and now MOVW) instruction.

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
    case Z80::R25R24:
    case Z80::R27R26:
    case Z80::R31R30: {
      if (isUInt<6>(Offset)) {
        Opcode = Z80::ADIWRdK;
        break;
      }
      LLVM_FALLTHROUGH;
    }
    default: {
      // This opcode will get expanded into a pair of subi/sbci.
      Opcode = Z80::SUBIWRdK;
      Offset = -Offset;
      break;
    }
    }

    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(Opcode), DstReg)
                            .addReg(DstReg, RegState::Kill)
                            .addImm(Offset);
    New->getOperand(3).setIsDead();*/

    return;
  }

  // If the offset is too big we have to adjust and restore the frame pointer
  // to materialize a valid load/store with displacement.
  //:TODO: consider using only one adiw/sbiw chain for more than one frame index
  if (Offset > 126) {
    BuildMI(MBB, II, dl, TII.get(Z80::LDIWRdK), Z80::IX).addImm(Offset);
    BuildMI(MBB, II, dl, TII.get(Z80::ADDW))
        .addReg(Z80::IX, RegState::Define)
        .addReg(Z80::IX)
        .addReg(Z80::SP);

    MI.getOperand(FIOperandNum).ChangeToRegister(Z80::IX, false, false, true);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(0);

    return;

    llvm_unreachable("Z80RegisterInfo::eliminateFrameIndex Offset > 126");
    /*unsigned AddOpc = Z80::ADIWRdK, SubOpc = Z80::SBIWRdK;
    int AddOffset = Offset - 63 + 1;

    // For huge offsets where adiw/sbiw cannot be used use a pair of subi/sbci.
    if ((Offset - 63 + 1) > 63) {
      AddOpc = Z80::SUBIWRdK;
      SubOpc = Z80::SUBIWRdK;
      AddOffset = -AddOffset;
    }

    // It is possible that the spiller places this frame instruction in between
    // a compare and branch, invalidating the contents of SREG set by the
    // compare instruction because of the add/sub pairs. Conservatively save and
    // restore SREG before and after each add/sub pair.
    BuildMI(MBB, II, dl, TII.get(Z80::INRdA), Z80::R0).addImm(0x3f);

    MachineInstr *New = BuildMI(MBB, II, dl, TII.get(AddOpc), Z80::R29R28)
                            .addReg(Z80::R29R28, RegState::Kill)
                            .addImm(AddOffset);
    New->getOperand(3).setIsDead();

    // Restore SREG.
    BuildMI(MBB, std::next(II), dl, TII.get(Z80::OUTARr))
        .addImm(0x3f)
        .addReg(Z80::R0, RegState::Kill);

    // No need to set SREG as dead here otherwise if the next instruction is a
    // cond branch it will be using a dead register.
    BuildMI(MBB, std::next(II), dl, TII.get(SubOpc), Z80::R29R28)
        .addReg(Z80::R29R28, RegState::Kill)
        .addImm(Offset - 63 + 1);

    Offset = 62;*/
  }

  MI.getOperand(FIOperandNum).ChangeToRegister(Z80::IY, false);
  assert(isInt<8>(Offset) && "Offset is out of range");
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
}

Register Z80RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
/*  const TargetFrameLowering *TFI = MF.getSubtarget().getFrameLowering();
  if (TFI->hasFP(MF)) {
    // The Y pointer register
    return Z80::R28;
  }

  return Z80::SP;*/
  llvm_unreachable("Z80RegisterInfo::getFrameRegister");
}

const TargetRegisterClass *
Z80RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  return &Z80::PTRREGSRegClass;
}

void Z80RegisterInfo::splitReg(Register Reg, Register &LoReg,
                               Register &HiReg) const {
  static Register SplittableDregs[] = {Z80::BC, Z80::DE, Z80::HL, Z80::IX,
                                       Z80::IY};

  bool f = false;

  for(Register r : SplittableDregs)
    f |= (r == Reg);

  assert(f && "can only split 16-bit registers");

  LoReg = getSubReg(Reg, Z80::sub_lo);
  HiReg = getSubReg(Reg, Z80::sub_hi);
}

bool Z80RegisterInfo::shouldCoalesce(MachineInstr *MI,
                                     const TargetRegisterClass *SrcRC,
                                     unsigned SubReg,
                                     const TargetRegisterClass *DstRC,
                                     unsigned DstSubReg,
                                     const TargetRegisterClass *NewRC,
                                     LiveIntervals &LIS) const {
  if(this->getRegClass(Z80::PTRREGSRegClassID)->hasSubClassEq(NewRC)) {
    return false;
  }

  return true;
}

static void CheckDisableRegs(MachineInstr &MI, Register Reg, bool &DisableHL,
                             bool &DisableXY, bool IsDef,
                             const VirtRegMap *VRM) {
  if (Z80::ADDW == MI.getOpcode()) {
    //MI.dump();
    DisableXY = true;

    /*if (VRM) {
      if (VRM->hasPhys(Reg)) {
        printReg(Reg).Print(dbgs());
        dbgs() << "->";
        printReg(VRM->getPhys(Reg)).Print(dbgs());
        dbgs() << "\r\n";
      }

      VRM->dump();
    }
*/
    return;
  }

  return;

  auto opc = MI.getOpcode();

  switch (opc) {
  case Z80::LDWPTR:
  case Z80::LDDWPTR:
//    if (IsDef)
//      DisableXY = true;
//    else
    if (!IsDef)
      DisableHL = true;
    break;

  case Z80::LDDPTR: {
    auto Offset = MI.getOperand(2).getImm();
    //if (Offset != 0 && !IsDef)
    if (!IsDef)
      DisableHL = true;
    break;
  }

  case Z80::STWPTR:{
    auto Op0 = MI.getOperand(0);
    if (Op0.isReg() && Op0.getReg() == Reg)
      DisableHL = true;
    if (MI.getOperand(1).getReg() == Reg)
      DisableXY = true;
    break;
  }

  case Z80::STDWPTR: {
    auto Op0 = MI.getOperand(0);
    if (Op0.isReg() && Op0.getReg() == Reg)
      DisableHL = true;
    if (MI.getOperand(2).getReg() == Reg)
      DisableXY = true;
    break;
  }
  }
}

bool Z80RegisterInfo::getRegAllocationHints(Register VirtReg,
                                            ArrayRef<MCPhysReg> Order,
                                            SmallVectorImpl<MCPhysReg> &Hints,
                                            const MachineFunction &MF,
                                            const VirtRegMap *VRM,
                                            const LiveRegMatrix *Matrix) const {

  //return TargetRegisterInfo::getRegAllocationHints(VirtReg, Order, Hints, MF, VRM);

  const MachineRegisterInfo &MRI = MF.getRegInfo();
  auto Hint = MRI.getRegAllocationHint(VirtReg);

  bool DisableHL = false;
  bool DisableXY = false;

  for (auto it = MRI.use_instr_begin(VirtReg); it != MRI.use_instr_end();
       it = std::next(it))
    CheckDisableRegs(*it, VirtReg, DisableHL, DisableXY, false, VRM);

  for (auto it = MRI.def_instr_begin(VirtReg); it != MRI.def_instr_end();
       it = std::next(it))
    CheckDisableRegs(*it, VirtReg, DisableHL, DisableXY, true, VRM);

  if (!DisableHL && !DisableXY)
    TargetRegisterInfo::getRegAllocationHints(VirtReg, Order, Hints, MF, VRM);

  if (DisableHL && DisableXY)
  {
    dumpReg(VirtReg, 0, MRI.getTargetRegisterInfo());
  }

  auto p = [=](auto const &x) {
    if (DisableHL && ((Z80::HL == x) || (Z80::H == x) || (Z80::L == x)))
      return true;

    if (DisableXY && ((Z80::IY == x) || (Z80::IX == x) || (Z80::XL == x) ||
                      (Z80::XH == x) || (Z80::YL == x) || (Z80::YH == x)))
      return true;

    return false;
  };

  for (const Register &o : Order) {
    if (!p(o))
      Hints.push_back(o);
  }

  return true;
}

void Z80RegisterInfo::updateRegAllocHint(Register Reg, Register NewReg,
                                         MachineFunction &MF) const {
  return;

  MachineRegisterInfo &MRI = MF.getRegInfo();

  auto Hint = MRI.getRegAllocationHint(Reg);
  auto NewHint = MRI.getRegAllocationHint(NewReg);

  if (Hint.first == 12345 && NewHint.first != 12345) {
    MRI.setRegAllocationHint(NewReg, 12345, Hint.second);
  }

  if (Hint.first != 12345 && NewHint.first == 12345) {
    MRI.setRegAllocationHint(Reg, 12345, NewHint.second);
  }

/*  errs() << "---\r\n";
  dumpReg(Reg);
  dumpReg(NewReg);
  errs() << "---\r\n";*/
}

} // end of namespace llvm
