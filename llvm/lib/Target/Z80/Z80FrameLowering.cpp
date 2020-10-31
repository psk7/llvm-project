//===-- Z80FrameLowering.cpp - Z80 Frame Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the Z80 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "Z80FrameLowering.h"

#include "Z80.h"
#include "Z80InstrInfo.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/Function.h"

#include <vector>

namespace llvm {

Z80FrameLowering::Z80FrameLowering()
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(1), -2) {}

bool Z80FrameLowering::canSimplifyCallFramePseudos(
    const MachineFunction &MF) const {
  // Always simplify call frame pseudo instructions, even when
  // hasReservedCallFrame is false.
  return true;
}

bool Z80FrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  // Reserve call frame memory in function prologue under the following
  // conditions:
  // - Y pointer is reserved to be the frame pointer.
  // - The function does not contain variable sized objects.

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return hasFP(MF) && !MFI.hasVarSizedObjects();
}

void Z80FrameLowering::emitPrologue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = (MBBI != MBB.end()) ? MBBI->getDebugLoc() : DebugLoc();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();
  const Z80MachineFunctionInfo *AFI = MF.getInfo<Z80MachineFunctionInfo>();
  bool HasFP = hasFP(MF);

  // Interrupt handlers re-enable interrupts in function entry.
/*  if (AFI->isInterruptHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(Z80::BSETs))
        .addImm(0x07)
        .setMIFlag(MachineInstr::FrameSetup);
  }*/

  // Emit special prologue code to save R1, R0 and SREG in interrupt/signal
  // handlers before saving any other registers.
/*  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(Z80::PUSHWRr))
        .addReg(Z80::R1R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);

    BuildMI(MBB, MBBI, DL, TII.get(Z80::INRdA), Z80::R0)
        .addImm(0x3f)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(Z80::PUSHRr))
        .addReg(Z80::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
    BuildMI(MBB, MBBI, DL, TII.get(Z80::EORRdRr))
        .addReg(Z80::R0, RegState::Define)
        .addReg(Z80::R0, RegState::Kill)
        .addReg(Z80::R0, RegState::Kill)
        .setMIFlag(MachineInstr::FrameSetup);
  }*/

  // Early exit if the frame pointer is not needed in this function.
  if (!HasFP) {
    return;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  int FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (
      (MBBI != MBB.end()) && MBBI->getFlag(MachineInstr::FrameSetup) &&
      (MBBI->getOpcode() == Z80::PUSHRr)) {
    ++MBBI;
  }

  /*// Update Y with the new base value.
  BuildMI(MBB, MBBI, DL, TII.get(Z80::SPREAD), Z80::R29R28)
      .addReg(Z80::SP)
      .setMIFlag(MachineInstr::FrameSetup);*/

  // Mark the FramePtr as live-in in every block except the entry.
  for (MachineFunction::iterator I = std::next(MF.begin()), E = MF.end();
       I != E; ++I) {
    I->addLiveIn(Z80::IY);
  }

  // Reserve the necessary frame memory by doing FP -= <size>.
  /*unsigned Opcode = (isUInt<6>(FrameSize)) ? Z80::SBIWRdK : Z80::SUBIWRdK;*/

  BuildMI(MBB, MBBI, DL, TII.get(Z80::LDIWRdK), Z80::IY)
      //.addReg(Z80::R29R28, RegState::Kill)
      .addImm(-FrameSize)
      .setMIFlag(MachineInstr::FrameSetup);

  BuildMI(MBB, MBBI, DL, TII.get(Z80::ADDRdRr16), Z80::IY)
      .addReg(Z80::IY)
      .addReg(Z80::SP)
      .setMIFlag(MachineInstr::FrameSetup);

  if (!FrameSize) {
    return;
  }

  BuildMI(MBB, MBBI, DL, TII.get(Z80::LDSP), Z80::SP)
      .addReg(Z80::IY)
      .setMIFlag(MachineInstr::FrameSetup);

  /*MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Z80::LDIWRdK), Z80::HL)
      //.addReg(Z80::R29R28, RegState::Kill)
      .addImm(-FrameSize)
      .setMIFlag(MachineInstr::FrameSetup);*/


  // The SREG implicit def is dead.
  //MI->getOperand(3).setIsDead();

  // Write back R29R28 to SP and temporarily disable interrupts.
  //BuildMI(MBB, MBBI, DL, TII.get(Z80::SPWRITE), Z80::SP)
  //    .addReg(Z80::R29R28)
  //    .setMIFlag(MachineInstr::FrameSetup);
}

void Z80FrameLowering::emitEpilogue(MachineFunction &MF,
                                    MachineBasicBlock &MBB) const {
  const Z80MachineFunctionInfo *AFI = MF.getInfo<Z80MachineFunctionInfo>();

  // Early exit if the frame pointer is not needed in this function except for
  // signal/interrupt handlers where special code generation is required.
/*  if (!hasFP(MF) && !AFI->isInterruptOrSignalHandler()) {
    return;
  }*/

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  assert(MBBI->getDesc().isReturn() &&
         "Can only insert epilog into returning blocks");

  DebugLoc DL = MBBI->getDebugLoc();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  int FrameSize = MFI.getStackSize() - AFI->getCalleeSavedFrameSize();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();

  // Emit special epilogue code to restore R1, R0 and SREG in interrupt/signal
  // handlers at the very end of the function, just before reti.
/*  if (AFI->isInterruptOrSignalHandler()) {
    BuildMI(MBB, MBBI, DL, TII.get(Z80::POPRd), Z80::R0);
    BuildMI(MBB, MBBI, DL, TII.get(Z80::OUTARr))
        .addImm(0x3f)
        .addReg(Z80::R0, RegState::Kill);
    BuildMI(MBB, MBBI, DL, TII.get(Z80::POPWRd), Z80::R1R0);
  }*/

  // Early exit if there is no need to restore the frame pointer.
  if (!FrameSize) {
    return;
  }

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    int Opc = PI->getOpcode();

    if (Opc != Z80::POPRd && !PI->isTerminator()) {
      break;
    }

    --MBBI;
  }

  BuildMI(MBB, MBBI, DL, TII.get(Z80::LDIWRdK), Z80::IY)
      .addImm(FrameSize)
      .setMIFlag(MachineInstr::FrameDestroy);

  BuildMI(MBB, MBBI, DL, TII.get(Z80::ADDRdRr16), Z80::IY)
      .addReg(Z80::IY)
      .addReg(Z80::SP)
      .setMIFlag(MachineInstr::FrameDestroy);

  BuildMI(MBB, MBBI, DL, TII.get(Z80::LDSP), Z80::SP)
      .addReg(Z80::IY, RegState::Kill)
      .setMIFlag(MachineInstr::FrameDestroy);

  unsigned Opcode;

  // Select the optimal opcode depending on how big it is.
/*  if (isUInt<6>(FrameSize)) {
    Opcode = Z80::ADIWRdK;
  } else {
    Opcode = Z80::SUBIWRdK;
    FrameSize = -FrameSize;
  }*/

/*  // Restore the frame pointer by doing FP += <size>.
  MachineInstr *MI = BuildMI(MBB, MBBI, DL, TII.get(Opcode), Z80::R29R28)
                         .addReg(Z80::R29R28, RegState::Kill)
                         .addImm(FrameSize);
  // The SREG implicit def is dead.
  MI->getOperand(3).setIsDead();

  // Write back R29R28 to SP and temporarily disable interrupts.
  BuildMI(MBB, MBBI, DL, TII.get(Z80::SPWRITE), Z80::SP)
      .addReg(Z80::R29R28, RegState::Kill);*/
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
bool Z80FrameLowering::hasFP(const MachineFunction &MF) const {
  const Z80MachineFunctionInfo *FuncInfo = MF.getInfo<Z80MachineFunctionInfo>();

  return (FuncInfo->getHasSpills() || FuncInfo->getHasAllocas() ||
          FuncInfo->getHasStackArgs());
}

bool Z80FrameLowering::spillCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty()) {
    return false;
  }

  unsigned CalleeFrameSize = 0;
  DebugLoc DL = MBB.findDebugLoc(MI);
  MachineFunction &MF = *MBB.getParent();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  Z80MachineFunctionInfo *Z80FI = MF.getInfo<Z80MachineFunctionInfo>();

  for (unsigned i = CSI.size(); i != 0; --i) {
    unsigned Reg = CSI[i - 1].getReg();
    bool IsNotLiveIn = !MBB.isLiveIn(Reg);

    unsigned rs = TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg));

    assert(rs == 16 && "Invalid register size");

    // Add the callee-saved register as live-in only if it is not already a
    // live-in register, this usually happens with arguments that are passed
    // through callee-saved registers.
    if (IsNotLiveIn) {
      MBB.addLiveIn(Reg);
    }

    // Do not kill the register when it is an input argument.
    BuildMI(MBB, MI, DL, TII.get(Z80::PUSHRr))
        .addReg(Reg, getKillRegState(IsNotLiveIn))
        .setMIFlag(MachineInstr::FrameSetup);

    CalleeFrameSize += rs / 8;
  }

  Z80FI->setCalleeSavedFrameSize(CalleeFrameSize);

  return true;
}

bool Z80FrameLowering::restoreCalleeSavedRegisters(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
    MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {

  if (CSI.empty()) {
    return false;
  }

  DebugLoc DL = MBB.findDebugLoc(MI);
  const MachineFunction &MF = *MBB.getParent();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();

  for (const CalleeSavedInfo &CCSI : CSI) {
    unsigned Reg = CCSI.getReg();

    assert(TRI->getRegSizeInBits(*TRI->getMinimalPhysRegClass(Reg)) == 16 &&
           "Invalid register size");

    BuildMI(MBB, MI, DL, TII.get(Z80::POPRd), Reg);
  }

  return true;
}

/// Replace pseudo store instructions that pass arguments through the stack with
/// real instructions.
static void fixStackStores(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI,
                           const TargetInstrInfo &TII, Register FP) {

  // Iterate through the BB until we hit a call instruction or we reach the end.
  for (auto I = MI, E = MBB.end(); I != E && !I->isCall();) {
    MachineBasicBlock::iterator NextMI = std::next(I);
    MachineInstr &MI = *I;
    unsigned Opcode = I->getOpcode();

    // Only care of pseudo store instructions where SP is the base pointer.
    if (Opcode != Z80::STDSPQRr && Opcode != Z80::STDWSPQRr) {
      I = NextMI;
      continue;
    }

    auto reg = MI.getOperand(0).getReg();

    assert((reg == Z80::IX || reg == Z80::IY) &&
           "Invalid register, should be IX or IY!");

    // Replace this instruction with a regular store. Use Y as the base
    // pointer since it is guaranteed to contain a copy of SP.
    unsigned STOpc =
        (Opcode == Z80::STDWSPQRr) ? Z80::STDWPtrQRr : Z80::STDPtrQRr;

    MI.setDesc(TII.get(STOpc));
    MI.getOperand(0).setReg(FP);

    I = NextMI;
  }
}

MachineBasicBlock::iterator Z80FrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator MI) const {

  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();

  // There is nothing to insert when the call frame memory is allocated during
  // function entry. Delete the call frame pseudo and replace all pseudo stores
  // with real store instructions.
  if (hasReservedCallFrame(MF)) {
    fixStackStores(MBB, MI, TII, Z80::IY);
    return MBB.erase(MI);
  }

  DebugLoc DL = MI->getDebugLoc();
  unsigned int Opcode = MI->getOpcode();
  int Amount = TII.getFrameSize(*MI);

  // ADJCALLSTACKUP and ADJCALLSTACKDOWN are converted to adiw/subi
  // instructions to read and write the stack pointer in I/O space.
  if (Amount != 0) {
    assert(getStackAlign() == Align(1) && "Unsupported stack alignment");

    if (Opcode == TII.getCallFrameSetupOpcode()) {
      // Update the stack pointer.
      // In many cases this can be done far more efficiently by pushing the
      // relevant values directly to the stack. However, doing that correctly
      // (in the right order, possibly skipping some empty space for undef
      // values, etc) is tricky and thus left to be optimized in the future.

      BuildMI(MBB, MI, DL, TII.get(Z80::LDIWRdK), Z80::IX)
          .addImm(-Amount);

      BuildMI(MBB, MI, DL, TII.get(Z80::ADDRdRr16), Z80::IX)
          .addReg(Z80::IX)
          .addReg(Z80::SP, RegState::Kill);

      BuildMI(MBB, MI, DL, TII.get(Z80::LDSP), Z80::SP)
          .addReg(Z80::IX);

      /*BuildMI(MBB, MI, DL, TII.get(Z80::SPREAD), Z80::IX).addReg(Z80::SP);

      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(Z80::ADDWRdRr), Z80::IX)
                              .addReg(Z80::IX, RegState::Kill)
                              .addImm(-Amount);
      New->getOperand(3).setIsDead();

      BuildMI(MBB, MI, DL, TII.get(Z80::SPWRITE), Z80::SP)
          .addReg(Z80::R31R30, RegState::Kill);*/

      // Make sure the remaining stack stores are converted to real store
      // instructions.
      fixStackStores(MBB, MI, TII, Z80::IX);
    } else {
      assert(Opcode == TII.getCallFrameDestroyOpcode());

      // Note that small stack changes could be implemented more efficiently
      // with a few pop instructions instead of the 8-9 instructions now
      // required.

      // Select the best opcode to adjust SP based on the offset size.
      /*unsigned addOpcode;
      if (isUInt<6>(Amount)) {
        addOpcode = Z80::ADIWRdK;
      } else {
        addOpcode = Z80::SUBIWRdK;
        Amount = -Amount;
      }

      // Build the instruction sequence.
      BuildMI(MBB, MI, DL, TII.get(Z80::SPREAD), Z80::R31R30).addReg(Z80::SP);

      MachineInstr *New = BuildMI(MBB, MI, DL, TII.get(addOpcode), Z80::R31R30)
                              .addReg(Z80::R31R30, RegState::Kill)
                              .addImm(Amount);
      New->getOperand(3).setIsDead();

      BuildMI(MBB, MI, DL, TII.get(Z80::SPWRITE), Z80::SP)
          .addReg(Z80::R31R30, RegState::Kill);*/
    }
  }

  return MBB.erase(MI);
}

void Z80FrameLowering::determineCalleeSaves(MachineFunction &MF,
                                            BitVector &SavedRegs,
                                            RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the IY register needs to be saved as well.
  if (hasFP(MF)) {
    SavedRegs.set(Z80::IY);
  }
}
/// The frame analyzer pass.
///
/// Scans the function for allocas and used arguments
/// that are passed through the stack.
struct Z80FrameAnalyzer : public MachineFunctionPass {
  static char ID;
  Z80FrameAnalyzer() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    const MachineFrameInfo &MFI = MF.getFrameInfo();
    Z80MachineFunctionInfo *FuncInfo = MF.getInfo<Z80MachineFunctionInfo>();

    // If there are no fixed frame indexes during this stage it means there
    // are allocas present in the function.
    if (MFI.getNumObjects() != MFI.getNumFixedObjects()) {
      // Check for the type of allocas present in the function. We only care
      // about fixed size allocas so do not give false positives if only
      // variable sized allocas are present.
      for (unsigned i = 0, e = MFI.getObjectIndexEnd(); i != e; ++i) {
        // Variable sized objects have size 0.
        if (MFI.getObjectSize(i)) {
          FuncInfo->setHasAllocas(true);
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

        /*if ((Opcode != Z80::LDDRdPtrQ) && (Opcode != Z80::LDDWRdPtrQ) &&
            (Opcode != Z80::STDPtrQRr) && (Opcode != Z80::STDWPtrQRr)) {
          continue;
        }*/

        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isFI()) {
            continue;
          }

          if (MFI.isFixedObjectIndex(MO.getIndex())) {
            FuncInfo->setHasStackArgs(true);
            return false;
          }
        }
      }
    }

    return false;
  }

  StringRef getPassName() const override { return "Z80 Frame Analyzer"; }
};

char Z80FrameAnalyzer::ID = 0;

/// Creates instance of the frame analyzer pass.
FunctionPass *createZ80FrameAnalyzerPass() { return new Z80FrameAnalyzer(); }

/// Create the Dynalloca Stack Pointer Save/Restore pass.
/// Insert a copy of SP before allocating the dynamic stack memory and restore
/// it in function exit to restore the original SP state. This avoids the need
/// of reserving a register pair for a frame pointer.
struct Z80DynAllocaSR : public MachineFunctionPass {
  static char ID;
  Z80DynAllocaSR() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override {
    // Early exit when there are no variable sized objects in the function.
    if (!MF.getFrameInfo().hasVarSizedObjects()) {
      return false;
    }

    const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
    const TargetInstrInfo &TII = *STI.getInstrInfo();
    MachineBasicBlock &EntryMBB = MF.front();
    MachineBasicBlock::iterator MBBI = EntryMBB.begin();
    DebugLoc DL = EntryMBB.findDebugLoc(MBBI);

    Register SPCopy =
        MF.getRegInfo().createVirtualRegister(&Z80::DREGSRegClass);

    // Create a copy of SP in function entry before any dynallocas are
    // inserted.
/*    BuildMI(EntryMBB, MBBI, DL, TII.get(Z80::COPY), SPCopy).addReg(Z80::SP);*/

    // Restore SP in all exit basic blocks.
/*    for (MachineBasicBlock &MBB : MF) {
      // If last instruction is a return instruction, add a restore copy.
      if (!MBB.empty() && MBB.back().isReturn()) {
        MBBI = MBB.getLastNonDebugInstr();
        DL = MBBI->getDebugLoc();
        BuildMI(MBB, MBBI, DL, TII.get(Z80::COPY), Z80::SP)
            .addReg(SPCopy, RegState::Kill);
      }
    }*/

    return true;
  }

  StringRef getPassName() const override {
    return "Z80 dynalloca stack pointer save/restore";
  }
};

char Z80DynAllocaSR::ID = 0;

/// createZ80DynAllocaSRPass - returns an instance of the dynalloca stack
/// pointer save/restore pass.
FunctionPass *createZ80DynAllocaSRPass() { return new Z80DynAllocaSR(); }

} // end of namespace llvm

