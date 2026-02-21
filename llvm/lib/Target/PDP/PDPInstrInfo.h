//===-- PDPInstrInfo.h - PDP Instruction Information ------------*- C++ -*-===//
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

#ifndef LLVM_PDP_INSTR_INFO_H
#define LLVM_PDP_INSTR_INFO_H

#include "llvm/CodeGen/TargetInstrInfo.h"

#include <functional>
#include "PDPRegisterInfo.h"

#define GET_INSTRINFO_HEADER
#include "PDPGenInstrInfo.inc"
#undef GET_INSTRINFO_HEADER

namespace llvm {

class PDPSubtarget;

namespace PDPCC {

/// PDP specific condition codes.
/// These correspond to `PDP_*_COND` in `PDPInstrInfo.td`.
/// They must be kept in synch.
enum CondCodes {
  COND_EQ, //!< Equal
  COND_NE, //!< Not equal

  COND_GE, //!< Greater than or equal
  COND_GT, //!< Greater than
  COND_LT, //!< Less than
  COND_LE, //!< Less than or equal

  COND_SH, //!< Unsigned same or higher
  COND_LO, //!< Unsigned lower
  COND_SL, //!< Unsigned same or lower
  COND_HI, //!< Unsigned higher

  COND_MI, //!< Minus
  COND_PL, //!< Plus

  COND_INVALID
};

} // end of namespace PDPCC

namespace PDPII {

/// Specifies a target operand flag.
enum TOF {
  MO_NO_FLAG,

  /// On a symbol operand, this represents the lo part.
  MO_LO = (1 << 1),

  /// On a symbol operand, this represents the hi part.
  MO_HI = (1 << 2),

  /// On a symbol operand, this represents it has to be negated.
  MO_NEG = (1 << 3)
};

} // end of namespace PDPII

/// Utilities related to the PDP instruction set.
class PDPInstrInfo : public PDPGenInstrInfo {
public:
  explicit PDPInstrInfo(PDPSubtarget &STI);

  const PDPRegisterInfo &getRegisterInfo() const { return RI; }

  const MCInstrDesc &getBrCond(PDPCC::CondCodes CC) const;

  PDPCC::CondCodes getCondFromBranchOpc(unsigned Opc) const;

  PDPCC::CondCodes getOppositeCondition(PDPCC::CondCodes CC) const;

  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc, bool RenamableDest = false,
                   bool RenamableSrc = false) const override;

  void storeRegToStackSlot(
      MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg,
      bool isKill, int FrameIndex, const TargetRegisterClass *RC,
      const TargetRegisterInfo *TRI, Register VReg,
      MachineInstr::MIFlag Flags = MachineInstr::NoFlags) const override;

  void loadRegFromStackSlot(
      MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg,
      int FrameIndex, const TargetRegisterClass *RC,
      const TargetRegisterInfo *TRI, Register VReg,
      MachineInstr::MIFlag Flags = MachineInstr::NoFlags) const override;

  Register isLoadFromStackSlot(const MachineInstr &MI,
                               int &FrameIndex) const override;
  Register isStoreToStackSlot(const MachineInstr &MI,
                              int &FrameIndex) const override;

  // Branch analysis.
  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
                     MachineBasicBlock *&FBB,
                     SmallVectorImpl<MachineOperand> &Cond,
                     bool AllowModify = false) const override;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
                        MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond,
                        const DebugLoc &DL,
                        int *BytesAdded = nullptr) const override;

  unsigned removeBranch(MachineBasicBlock &MBB,
                        int *BytesRemoved = nullptr) const override;

  bool
  reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;

  MachineBasicBlock *getBranchDestBlock(const MachineInstr &MI) const override;

  bool isBranchOffsetInRange(unsigned BranchOpc,
                             int64_t BrOffset) const override;

  void insertIndirectBranch(MachineBasicBlock &MBB,
                            MachineBasicBlock &NewDestBB,
                            MachineBasicBlock &RestoreBB, const DebugLoc &DL,
                            int64_t BrOffset, RegScavenger *RS) const override;

  // bool isPredicated(const MachineInstr &MI) const override;
private:
  const PDPRegisterInfo RI;

protected:
  const PDPSubtarget &STI;
};

} // end namespace llvm

#endif // LLVM_PDP_INSTR_INFO_H
