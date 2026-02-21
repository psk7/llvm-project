//===-- PDPFrameLowering.cpp - PDP Frame Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the PDP implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "PDPFrameLowering.h"

#include "PDP.h"
#include "PDPInstrInfo.h"
#include "PDPMachineFunctionInfo.h"
#include "PDPTargetMachine.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

PDPFrameLowering::PDPFrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(2), -2, Align(2)) {}

bool PDPFrameLowering::canSimplifyCallFramePseudos(
    const MachineFunction &MF) const {
  // Always simplify call frame pseudo instructions, even when
  // hasReservedCallFrame is false.
  return true;
}

bool PDPFrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame memory in function prologue under the following
  // conditions:
  // - Y pointer is reserved to be the frame pointer.
  // - The function does not contain variable sized objects.

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return hasFP(MF) && !MFI.hasVarSizedObjects();
}

void PDPFrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  const PDPInstrInfo &TII = *STI.getInstrInfo();
  const PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  bool HasFP = hasFP(MF);

  // Early exit if the frame pointer is not needed in this function.
  if (!HasFP) {
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (
      (MBBI != MBB.end()) && MBBI->getFlag(MachineInstr::FrameSetup) &&
      (MBBI->getOpcode() == PDP::PUSH)) {
    ++MBBI;
  }

  if (!FrameSize) {
    return;
  }

  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(PDP::SUBREG), PDP::R6)
                         .addReg(PDP::R7)
                         .addImm(2)
                         .addImm(FrameSize)
                         .addImm(0)
                         .addReg(PDP::R6, RegState::Kill)
                         .setMIFlag(MachineInstr::FrameSetup);

  // The SREG implicit def is dead.
  MI->getOperand(6).setIsDead();
}

void PDPFrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();

  // Early exit if the frame pointer is not needed in this function except for
  // signal/interrupt handlers where special code generation is required.
  if (!hasFP(MF) && !AFI->isInterruptOrSignalHandler()) {
    return;
  }

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getDesc().isReturn() &&
         "Can only insert epilog into returning blocks");

  DebugLoc DL = MBBI->getDebugLoc();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  unsigned FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  const PDPInstrInfo &TII = *STI.getInstrInfo();

  // Early exit if there is no need to restore the frame pointer.
  if (!FrameSize && !MF.getFrameInfo().hasVarSizedObjects()) {
    return;
  }

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    int Opc = PI->getOpcode();

    if (Opc != PDP::POP && !PI->isTerminator()) {
      break;
    }

    --MBBI;
  }

  if (!FrameSize)
    return;

  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(PDP::ADDREG), PDP::R6)
                        .addReg(PDP::R7)
                        .addImm(2)
                        .addImm(FrameSize)
                        .addImm(0)
                        .addReg(PDP::R6, RegState::Kill)
                        .setMIFlag(MachineInstr::FrameSetup);

  // The SREG implicit def is dead.
  MI->getOperand(6).setIsDead();
}

// Return true if the specified function should have a dedicated frame
// pointer register. This is true if the function meets any of the following
// conditions:
//  - a register has been spilled
//  - has allocas
//  - input arguments are passed using the stack
//
// Notice that strictly this is not a frame pointer because it contains SP after
// frame allocation instead of having the original SP in function entry.
bool PDPFrameLowering::hasFPImpl(const MachineFunction &MF) const {
  const PDPMachineFunctionInfo *FuncInfo = MF.getInfo<PDPMachineFunctionInfo>();

  return (FuncInfo->getHasSpills() || FuncInfo->getHasAllocas() ||
          FuncInfo->getHasStackArgs() ||
          MF.getFrameInfo().hasVarSizedObjects());
}

bool PDPFrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  unsigned CalleeFrameSize = 0;
  DebugLoc DL = MBB.findDebugLoc(MI);
  MachineFunction &MF = *MBB.getParent();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  PDPMachineFunctionInfo *PDPFI = MF.getInfo<PDPMachineFunctionInfo>();

  for (const CalleeSavedInfo &I : llvm::reverse(CSI)) {
    Register Reg = I.getReg();
    bool IsNotLiveIn = !MBB.isLiveIn(Reg);

    // Check if Reg is a sub register of a 16-bit livein register, and then
    // add it to the livein list.
    if (IsNotLiveIn)
      for (const auto &LiveIn : MBB.liveins())
        if (STI.getRegisterInfo()->isSubRegister(LiveIn.PhysReg, Reg)) {
          IsNotLiveIn = false;
          MBB.addLiveIn(Reg);
          break;
        }

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 16 &&
           "Invalid register size");

    // Add the callee-saved register as live-in only if it is not already a
    // live-in register, this usually happens with arguments that are passed
    // through callee-saved registers.
    if (IsNotLiveIn) {
      MBB.addLiveIn(Reg);
    }

    // Do not kill the register when it is an input argument.
    BuildMI(MBB, MI, DL, TII.get(PDP::PUSH))
        .addReg(Reg, getKillRegState(IsNotLiveIn))
        .setMIFlag(MachineInstr::FrameSetup);
    CalleeFrameSize += 2;
  }

  PDPFI->setCalleeSavedFrameSize(CalleeFrameSize);

  return true;
}

bool PDPFrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  DebugLoc DL = MBB.findDebugLoc(MI);
  const MachineFunction &MF = *MBB.getParent();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (const CalleeSavedInfo &CCSI : CSI) {
    Register Reg = CCSI.getReg();

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 16 &&
           "Invalid register size");

    BuildMI(MBB, MI, DL, TII.get(PDP::POP), Reg);
  }

  return true;
}

/*/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions.
static void fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator StartMI,
                           const TargetInstrInfo &TII) {
  // Iterate through the BB until we hit a call instruction or we reach the end.
  for (MachineInstr &MI :
       llvm::make_early_inc_range(llvm::make_range(StartMI, MBB.end()))) {
    if (MI.isCall())
      break;

    unsigned Opcode = MI.getOpcode();

    // Only care of pseudo store instructions where SP is the base pointer.
    if (Opcode != PDP::STDSPQRr && Opcode != PDP::STDWSPQRr)
      continue;

    assert(MI.getOperand(0).getReg() == PDP::SP &&
           "SP is expected as base pointer");

    // Replace this instruction with a regular store. Use Y as the base
    // pointer since it is guaranteed to contain a copy of SP.
    unsigned STOpc =
        (Opcode == PDP::STDWSPQRr) ? PDP::STDWPtrQRr : PDP::STDPtrQRr;

    MI.setDesc(TII.get(STOpc));
    MI.getOperand(0).setReg(PDP::R31R30);
  }
}*/

MachineBasicBlock::iterator PDPFrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  const PDPInstrInfo &TII = *STI.getInstrInfo();

  if (hasReservedCallFrame(MF)) {
    return MBB.erase(MI);
  }

  DebugLoc DL = MI->getDebugLoc();
  unsigned int Opcode = MI->getOpcode();
  int Amount = TII.getFrameSize(*MI);

  if (Amount == 0) {
    return MBB.erase(MI);
  }

  assert(getStackAlign() == Align(2) && "Unsupported stack alignment");

  if (Opcode == TII.getCallFrameSetupOpcode()) {
    // Update the stack pointer.
    // In many cases this can be done far more efficiently by pushing the
    // relevant values directly to the stack. However, doing that correctly
    // (in the right order, possibly skipping some empty space for undef
    // values, etc) is tricky and thus left to be optimized in the future.
    //BuildMI(MBB, MI, DL, TII.get(PDP::SPREAD), PDP::R31R30).addReg(PDP::SP);

    /*MachineInstr *New =
        BuildMI(MBB, MI, DL, TII.get(PDP::SUBIWRdK), PDP::R31R30)
            .addReg(PDP::R31R30, RegState::Kill)
            .addImm(Amount);
    New->getOperand(3).setIsDead();*/

    //BuildMI(MBB, MI, DL, TII.get(PDP::SPWRITE), PDP::SP).addReg(PDP::R31R30);

    // Make sure the remaining stack stores are converted to real store
    // instructions.
    //fixStackStores(MBB, MI, TII);

    auto *Prev = MI->getPrevNode();

    MachineInstr *New = nullptr;

    if (Prev != nullptr && Prev->getOpcode() == PDP::ADD &&
        Prev->getOperand(0).getReg() == PDP::R7 &&
        Prev->getOperand(1).getImm() == 2 &&
        Prev->getOperand(4).getReg() == PDP::R6) {

      Amount =
          Prev->getOperand(2).getImm() + Prev->getOperand(3).getImm() - Amount;

      if (Amount == 0)
        Prev->eraseFromParent();
      else {
        Prev->getOperand(2).setImm(Amount);
        Prev->getOperand(3).setImm(0);
      }
    } else {
      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(PDP::SUB))
                .addReg(PDP::R7).addImm(2).addImm(Amount).addImm(0)
                .addReg(PDP::R6).addImm(0).addImm(0).addImm(0);
      New->getOperand(8).setIsDead();
    }
  } else {
    assert(Opcode == TII.getCallFrameDestroyOpcode());

    // Note that small stack changes could be implemented more efficiently
    // with a few pop instructions instead of the 8-9 instructions now
    // required.

    // Select the best opcode to adjust SP based on the offset size.
    unsigned AddOpcode;

    /*if (isUInt<6>(Amount) && STI.hasADDSUBIW()) {
      AddOpcode = PDP::ADIWRdK;
    } else {
      AddOpcode = PDP::SUBIWRdK;
      Amount = -Amount;
    }*/

    // Build the instruction sequence.
    //BuildMI(MBB, MI, DL, TII.get(PDP::SPREAD), PDP::R31R30).addReg(PDP::SP);

    /*MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(AddOpcode), PDP::R31R30)
                            .addReg(PDP::R31R30, RegState::Kill)
                            .addImm(Amount);
    New->getOperand(3).setIsDead();

    BuildMI(MBB, MI, DL, TII.get(PDP::SPWRITE), PDP::SP)
        .addReg(PDP::R31R30, RegState::Kill);*/

    MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(PDP::ADD))
                            .addReg(PDP::R7).addImm(2).addImm(Amount).addImm(0)
                            .addReg(PDP::R6).addImm(0).addImm(0).addImm(0);

    New->getOperand(8).setIsDead();
  }

  return MBB.erase(MI);
}

void PDPFrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  /*// If we have a frame pointer, the Y register needs to be saved as well.
  if (hasFP(MF)) {
    SavedRegs.set(PDP::R29);
    SavedRegs.set(PDP::R28);
  }*/
}
/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
struct PDPFrameAnalyzer : public MachineFunctionPass {
  static char ID;
  PDPFrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();

    for (MachineBasicBlock &BB : MF) {
      //BB.addLiveIn(PDP::R6);
      //BB.addLiveIn(PDP::R7);
    }

    // If there are no fixed frame indexes during this stage it means there
    // are allocas present in the function.
    if (MFI.getNumObjects() != MFI.getNumFixedObjects()) {
      // Check for the type of allocas present in the function. We only care
      // about fixed size allocas so do not give false positives if only
      // variable sized allocas are present.
      for (unsigned i = 0, e = MFI.getObjectIndexEnd(); i != e; ++i) {
        // Variable sized objects have size 0.
        if (MFI.getObjectSize(i)) {
          AFI->setHasAllocas(true);
          break;
        }
      }
    }

    // If there are fixed frame indexes present, scan the function to see if
    // they are really being used.
    if (MFI.getNumFixedObjects() == 0) {
      return false;
    }

    // Ok fixed frame indexes present, now scan the function to see if they
    // are really being used, otherwise we can ignore them.
    for (const MachineBasicBlock &BB : MF) {
      for (const MachineInstr &MI : BB) {
        int Opcode = MI.getOpcode();

        if (/*(Opcode != PDP::LDDRdPtrQ) && (Opcode != PDP::LDDWRdPtrQ) &&
            (Opcode != PDP::STDPtrQRr) && (Opcode != PDP::STDWPtrQRr) &&*/
            (Opcode != PDP::FRMIDX)) {
          continue;
        }

        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI()) {
            continue;
          }

          if (MFI.isFixedObjectIndex(MO.getIndex())) {
            AFI->setHasStackArgs(true);
            return false;
          }
        }
      }
    }

    return false;
  }

  StringRef getPassName() const override { return "PDP Frame Analyzer"; }
};

char PDPFrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createPDPFrameAnalyzerPass() { return new PDPFrameAnalyzer(); }

} // end of namespace llvm
