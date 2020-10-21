//===-- Z80InstrInfo.h - Z80 Instruction Information ------------*- C++ -*-===//
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

#ifndef LLVM_Z80_INSTR_INFO_H
#define LLVM_Z80_INSTR_INFO_H

#include "llvm/CodeGen/TargetInstrInfo.h"

#include "Z80RegisterInfo.h"

#define GET_INSTRINFO_HEADER
#include "Z80GenInstrInfo.inc"
#undef GET_INSTRINFO_HEADER

namespace llvm {

namespace Z80CC {

/// Z80 specific condition codes.
/// These correspond to `Z80_*_COND` in `Z80InstrInfo.td`.
/// They must be kept in synch.
enum CondCodes {
  COND_EQ, //!< Equal
  COND_NE, //!< Not equal
  COND_GE, //!< Greater than or equal
  COND_LT, //!< Less than
  COND_SH, //!< Unsigned same or higher
  COND_LO, //!< Unsigned lower
  COND_MI, //!< Minus
  COND_PL, //!< Plus
  COND_INVALID
};

} // end of namespace Z80CC

namespace Z80II {

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

enum Prefix {
  NoPrfx = 0,
  CB = 1,
  ED = 2,
  DD = 3,
  FD = 4,
  DDCB = 5,
  FDCB = 6
};

unsigned getPrefixLength(const MCInstrDesc &MI);

bool hasCBPrefix(const MCInstrDesc &MI);
bool hasEDPrefix(const MCInstrDesc &MI);
bool hasDDPrefix(const MCInstrDesc &MI);
bool hasFDPrefix(const MCInstrDesc &MI);

} // end of namespace Z80II

/// Utilities related to the Z80 instruction set.
class Z80InstrInfo : public Z80GenInstrInfo {
public:
  explicit Z80InstrInfo();

  const Z80RegisterInfo &getRegisterInfo() const { return RI; }
  const MCInstrDesc &getBrCond(Z80CC::CondCodes CC) const;
  Z80CC::CondCodes getCondFromBranchOpc(unsigned Opc) const;
  Z80CC::CondCodes getOppositeCondition(Z80CC::CondCodes CC) const;
  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI,
                   const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg,
                   bool KillSrc) const override;
  void storeRegToStackSlot(MachineBasicBlock &MBB,
                           MachineBasicBlock::iterator MI, Register SrcReg,
                           bool isKill, int FrameIndex,
                           const TargetRegisterClass *RC,
                           const TargetRegisterInfo *TRI) const override;
  void loadRegFromStackSlot(MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MI, Register DestReg,
                            int FrameIndex, const TargetRegisterClass *RC,
                            const TargetRegisterInfo *TRI) const override;
  unsigned isLoadFromStackSlot(const MachineInstr &MI,
                               int &FrameIndex) const override;
  unsigned isStoreToStackSlot(const MachineInstr &MI,
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

  unsigned insertIndirectBranch(MachineBasicBlock &MBB,
                                MachineBasicBlock &NewDestBB,
                                const DebugLoc &DL,
                                int64_t BrOffset,
                                RegScavenger *RS) const override;
private:
  const Z80RegisterInfo RI;
};

} // end namespace llvm

#endif // LLVM_Z80_INSTR_INFO_H
