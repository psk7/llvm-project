//===-- Z80ExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
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

#include "Z80.h"
#include "Z80InstrInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define Z80_EXPAND_PSEUDO_NAME "Z80 pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual Z80 instructions.
class Z80ExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  Z80ExpandPseudo() : MachineFunctionPass(ID) {
    initializeZ80ExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return Z80_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const Z80RegisterInfo *TRI;
  const TargetInstrInfo *TII;

  /*/// The register to be used for temporary storage.
  const Register SCRATCH_REGISTER = Z80::R0;
  /// The register that will always contain zero.
  const Register ZERO_REGISTER = Z80::R1;
  /// The IO address of the status register.
  const unsigned SREG_ADDR = 0x3f;*/

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

  MachineRegisterInfo &getRegInfo(Block &MBB) { return MBB.getParent()->getRegInfo(); }

  bool expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI);
  bool expandLogic(unsigned Op, Block &MBB, BlockIt MBBI);
  bool expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI);
  bool isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const;

  /// Scavenges a free GPR8 register for use.
  Register scavengeGPR8(MachineInstr &MI, const TargetRegisterClass *RC);
};

char Z80ExpandPseudo::ID = 0;

bool Z80ExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);
    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;
  }

  return Modified;
}

bool Z80ExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  TRI = STI.getRegisterInfo();
  TII = STI.getInstrInfo();

  // We need to track liveness in order to use register scavenging.
  MF.getProperties().set(MachineFunctionProperties::Property::TracksLiveness);

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

bool Z80ExpandPseudo::
expandArith(unsigned OpLo, unsigned OpHi, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}
/*
bool Z80ExpandPseudo::
expandLogic(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, Op)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(3).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, Op)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

bool Z80ExpandPseudo::
  isLogicImmOpRedundant(unsigned Op, unsigned ImmVal) const {

  // ANDI Rd, 0xff is redundant.
  if (Op == Z80::ANDIRdK && ImmVal == 0xff)
    return true;

  // ORI Rd, 0x0 is redundant.
  if (Op == Z80::ORIRdK && ImmVal == 0x0)
    return true;

  return false;
}

bool Z80ExpandPseudo::
expandLogicImm(unsigned Op, Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (!isLogicImmOpRedundant(Op, Lo8)) {
    auto MIBLO = buildMI(MBB, MBBI, Op)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstLoReg, getKillRegState(SrcIsKill))
      .addImm(Lo8);

    // SREG is always implicitly dead
    MIBLO->getOperand(3).setIsDead();
  }

  if (!isLogicImmOpRedundant(Op, Hi8)) {
    auto MIBHI = buildMI(MBB, MBBI, Op)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstHiReg, getKillRegState(SrcIsKill))
      .addImm(Hi8);

    if (ImpIsDead)
      MIBHI->getOperand(3).setIsDead();
  }

  MI.eraseFromParent();
  return true;
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::ADDWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  auto MIBHI = buildMI(MBB, MBBI, Z80::ADDRdRr16)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(SrcReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  //MIBHI.addReg(Z80::SREG, RegState::ImplicitKill);

  MI.eraseFromParent();
  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::ADCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(Z80::ADCRdRr, Z80::ADCRdRr, MBB, MBBI);
}
*/
/*
template <>
bool Z80ExpandPseudo::expand<Z80::SUBIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, Z80::SUBIRdK)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, Z80::SBCIRdK)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill));

  switch (MI.getOperand(2).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(2).getGlobal();
    int64_t Offs = MI.getOperand(2).getOffset();
    unsigned TF = MI.getOperand(2).getTargetFlags();
    MIBLO.addGlobalAddress(GV, Offs, TF | Z80II::MO_NEG | Z80II::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | Z80II::MO_NEG | Z80II::MO_HI);
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(2).getImm();
    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::SBCWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandArith(Z80::SBCRdRr, Z80::SBCRdRr, MBB, MBBI);
}

template <>
bool Z80ExpandPseudo::expand<Z80::SBCIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  unsigned Imm = MI.getOperand(2).getImm();
  unsigned Lo8 = Imm & 0xff;
  unsigned Hi8 = (Imm >> 8) & 0xff;
  unsigned OpLo = Z80::SBCIRdK;
  unsigned OpHi = Z80::SBCIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(SrcIsKill))
    .addImm(Lo8);

  // SREG is always implicitly killed
  MIBLO->getOperand(4).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(SrcIsKill))
    .addImm(Hi8);

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::ANDWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(Z80::ANDRdRr, MBB, MBBI);
}

template <>
bool Z80ExpandPseudo::expand<Z80::ANDIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(Z80::ANDIRdK, MBB, MBBI);
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::ANDIWRdK_RESBIT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Imm = MI.getOperand(2).getImm();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(SrcReg == DstReg && "SrcReg and DstReg must be same");
  assert(Imm >= 0 && Imm <= 15 && "Wrong bit position");

  auto reg = Imm >= 0 && Imm <= 7 ? DstLoReg : DstHiReg;
  if (Imm > 7)
    Imm -= 8;

  buildMI(MBB, MBBI, Z80::RESBIT)
      .addReg(reg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(reg, getKillRegState(SrcIsKill))
      .addImm(Imm);

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::SETBITWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  auto bit = MI.getOperand(2).getImm();
  auto displ = MI.getOperand(1).getImm();

  assert(bit >= 0 && bit <= 15);

  if (bit > 8){
    bit -= 8;
    displ++;
  }

  MI.setDesc(TII->get(Z80::SETBITPTR));
  MI.getOperand(2).setImm(bit);
  MI.getOperand(1).setImm(displ);

  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::RESBITWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  auto bit = MI.getOperand(2).getImm();
  auto displ = MI.getOperand(1).getImm();

  assert(bit >= 0 && bit <= 15);

  if (bit > 8){
    bit -= 8;
    displ++;
  }

  MI.setDesc(TII->get(Z80::RESBITPTR));
  MI.getOperand(2).setImm(bit);
  MI.getOperand(1).setImm(displ);

  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::ORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(Z80::ORRdRr, MBB, MBBI);
}

template <>
bool Z80ExpandPseudo::expand<Z80::ORIWRdK>(Block &MBB, BlockIt MBBI) {
  return expandLogicImm(Z80::ORIRdK, MBB, MBBI);
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::ORIWRdK_SETBIT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned Imm = MI.getOperand(2).getImm();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(SrcReg == DstReg && "SrcReg and DstReg must be same");
  assert(Imm >= 0 && Imm <= 15 && "Wrong bit position");

  auto reg = Imm >= 0 && Imm <= 7 ? DstLoReg : DstHiReg;
  if (Imm > 7)
    Imm -= 8;

  buildMI(MBB, MBBI, Z80::SETBIT)
      .addReg(reg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(reg, getKillRegState(SrcIsKill))
      .addImm(Imm);

  MI.eraseFromParent();
  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::EORWRdRr>(Block &MBB, BlockIt MBBI) {
  return expandLogic(Z80::EORRdRr, MBB, MBBI);
}

template <>
bool Z80ExpandPseudo::expand<Z80::COMWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = Z80::COMRd;
  unsigned OpHi = Z80::COMRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  // SREG is always implicitly dead
  MIBLO->getOperand(2).setIsDead();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::CPWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(1).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  buildMI(MBB, MBBI, Z80::CLEARC);

  auto MIBHI = buildMI(MBB, MBBI, Z80::SBCRdRr16)
      .addReg(DstReg, RegState::Define | getDeadRegState(true))
      .addReg(DstReg, getKillRegState(DstIsKill))
      .addReg(SrcReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(4).setIsDead();

  // SREG is always implicitly killed
  //MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::CPCWRdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = Z80::CPCRdRr;
  unsigned OpHi = Z80::CPCRdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, getKillRegState(DstIsKill))
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, getKillRegState(DstIsKill))
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::LDIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();

  auto MIBLO = buildMI(MBB, MBBI, Z80::LDI16)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF | Z80II::MO_LO);
    break;
  }
  case MachineOperand::MO_BlockAddress: {
    const BlockAddress *BA = MI.getOperand(1).getBlockAddress();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.add(MachineOperand::CreateBA(BA, TF | Z80II::MO_LO));
    break;
  }
  case MachineOperand::MO_Immediate: {
    int Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDSWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcReg = MI.getOperand(0).getReg();

  auto op = SrcReg == Z80::HL ? Z80::LDSWRdKOp : Z80::LDSWRdKOpExt;
  MI.setDesc(TII->get(op));

  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDWRdPtr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Load low byte.
  auto MIBLO = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
                   .addReg(DstLoReg, RegState::Define)
                   .addReg(SrcReg)
                   .addImm(0);

  // Load high byte.
  auto MIBHI = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
    .addReg(DstHiReg, RegState::Define)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(1);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDDWRdPtrQ>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  auto Imm = MI.getOperand(2).getImm();
  bool SrcIsKill = MI.getOperand(1).isKill();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  bool NeedAcc = Z80::IX == DstReg || Z80::IY == DstReg;

  if (Imm != 0)
    assert(SrcReg == Z80::IX || SrcReg == Z80::IY && "Only IX or IY can has displacement");

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 126 is the limit here.
  assert(Imm <= 126 && "Offset is out of range");

  MachineInstrBuilder MIBLO, MIBHI;

  if (NeedAcc) {
    Register TmpReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);
    bool haspush = false;

    if (TmpReg == -1) {
      TmpReg = Z80::B;
      buildMI(MBB, MI, Z80::PUSHRr).addReg(Z80::BC);
      haspush = true;
    }

    // Load low byte.
    MIBLO = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
        .addReg(TmpReg, RegState::Define)
        .addReg(SrcReg)
        .addImm(Imm);

    buildMI(MBB, MI, Z80::LDRdRr8)
        .addReg(DstLoReg, RegState::Define)
        .addReg(TmpReg, RegState::Kill);

    // Load high byte.
    MIBHI = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
        .addReg(TmpReg, RegState::Define)
        .addReg(SrcReg, getKillRegState(SrcIsKill))
        .addImm(Imm + 1);

    buildMI(MBB, MI, Z80::LDRdRr8)
        .addReg(DstHiReg, RegState::Define)
        .addReg(TmpReg, RegState::Kill);

    if (haspush) {
      buildMI(MBB, MI, Z80::POPRd).addReg(Z80::BC);
    }
  } else {
    // Load low byte.
    MIBLO = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
                .addReg(DstLoReg, RegState::Define)
                .addReg(SrcReg)
                .addImm(Imm);

    // Load high byte.
    MIBHI = buildMI(MBB, MBBI, Z80::LDDRdPtrQ)
                .addReg(DstHiReg, RegState::Define)
                .addReg(SrcReg, getKillRegState(SrcIsKill))
                .addImm(Imm + 1);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

Register Z80ExpandPseudo::scavengeGPR8(MachineInstr &MI, const TargetRegisterClass *RC) {
  MachineBasicBlock &MBB = *MI.getParent();
  RegScavenger RS;

  RS.enterBasicBlock(MBB);
  RS.forward(MI);

  BitVector Candidates = TRI->getAllocatableSet (*MBB.getParent(), RC);

  // Exclude all the registers being used by the instruction.
  for (MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg() != 0 && !MO.isDef() &&
        !Register::isVirtualRegister(MO.getReg()))
      Candidates.reset(MO.getReg());
  }

  BitVector Available = RS.getRegsAvailable(RC);
  Available &= Candidates;

  signed Reg = Available.find_first();
  //assert(Reg != -1 && "ran out of registers");
  return Reg;
}

template <>
bool Z80ExpandPseudo::expand<Z80::STSWKRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register SrcReg = MI.getOperand(1).getReg();

  auto op = SrcReg == Z80::HL ? Z80::STSWKRrOp : Z80::STSWKRrOpExt;
  MI.setDesc(TII->get(op));

  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::STWPtrRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  auto MIBLO = buildMI(MBB, MBBI, Z80::STDPtrQRr)
    .addReg(DstReg)
    .addImm(0)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  auto MIBHI = buildMI(MBB, MBBI, Z80::STDPtrQRr)
    .addReg(DstReg)
    .addImm(1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::STWPtrPiRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = Z80::STPtrPiRr;
  unsigned OpHi = Z80::STPtrPiRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, RegState::Define)
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::STWPtrPdRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  unsigned Imm = MI.getOperand(3).getImm();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(2).isKill();
  unsigned OpLo = Z80::STPtrPdRr;
  unsigned OpHi = Z80::STPtrPdRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  assert(DstReg != SrcReg && "SrcReg and DstReg cannot be the same");

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstReg, RegState::Define)
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg, RegState::Kill)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .addImm(Imm);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}
*/
template <>
bool Z80ExpandPseudo::expand<Z80::STDWPtrQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  int Imm = MI.getOperand(1).getImm();
  bool DstIsKill = MI.getOperand(0).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
  Register TmpReg;
  auto SrcKillState = getKillRegState(SrcIsKill);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 126 && Imm >= -126 && "Offset is out of range");

  assert(DstReg == Z80::IX || DstReg == Z80::IY);

  if (MBBI != MBB.begin()) {
    auto pi = std::prev(MBBI);

    if (pi->getOpcode() == Z80::COPYREG) {
      if (pi->getOperand(0).getReg() == SrcReg) {
        if (pi->getOperand(1).getReg() != Z80::HL) {
          auto k = pi->getOperand(1).isKill();

          SrcReg = pi->getOperand(1).getReg();
          pi->getOperand(1).setIsKill(false);

          SrcIsKill = k;

          llvm_unreachable("UNTESTED CODE!!!! CHECK IT!!!");
        }
      }
    }
  }

  bool NeedAcc = Z80::IX == SrcReg || Z80::IY == SrcReg;

  bool haspush = false;

  if (NeedAcc) {
    TmpReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);

    if (TmpReg == -1) {
      TmpReg = Z80::B;
      buildMI(MBB, MI, Z80::PUSHRr).addReg(Z80::BC);
      haspush = true;
    }

    buildMI(MBB, MI, Z80::LDRdRr8)
        .addReg(TmpReg, RegState::Define)
        .addReg(SrcLoReg, SrcKillState);

    SrcLoReg = TmpReg;
    SrcKillState = getKillRegState(true);
  }

  auto MIBLO = buildMI(MBB, MBBI, Z80::STDPtrQRr)
    .addReg(DstReg)
    .addImm(Imm)
    .addReg(SrcLoReg, SrcKillState);

  if (NeedAcc) {
    SrcKillState = getKillRegState(SrcIsKill);

    buildMI(MBB, MI, Z80::LDRdRr8)
        .addReg(TmpReg, RegState::Define)
        .addReg(SrcHiReg, SrcKillState);

    SrcHiReg = TmpReg;
    SrcKillState = getKillRegState(true);
  }

  auto MIBHI = buildMI(MBB, MBBI, Z80::STDPtrQRr)
    .addReg(DstReg, getKillRegState(DstIsKill))
    .addImm(Imm + 1)
    .addReg(SrcHiReg, SrcKillState);

  if (haspush) {
    buildMI(MBB, MI, Z80::POPRd).addReg(Z80::BC);
  }

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}
/*
template <>
bool Z80ExpandPseudo::expand<Z80::INWRdA>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  unsigned Imm = MI.getOperand(1).getImm();
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = Z80::INRdA;
  unsigned OpHi = Z80::INRdA;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm);

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addImm(Imm + 1);

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::OUTWARr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  unsigned Imm = MI.getOperand(0).getImm();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  unsigned OpLo = Z80::OUTARr;
  unsigned OpHi = Z80::OUTARr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Since we add 1 to the Imm value for the high byte below, and 63 is the highest Imm value
  // allowed for the instruction, 62 is the limit here.
  assert(Imm <= 62 && "Address is out of range");

  // 16 bit I/O writes need the high byte first
  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addImm(Imm + 1)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addImm(Imm)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill));

  MIBLO.setMemRefs(MI.memoperands());
  MIBHI.setMemRefs(MI.memoperands());

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::PUSHWRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcLoReg, SrcHiReg;
  Register SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = Z80::PUSHRr;
  unsigned OpHi = Z80::PUSHRr;
  TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(SrcLoReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(SrcHiReg, getKillRegState(SrcIsKill))
    .setMIFlags(Flags);

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::POPWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  unsigned Flags = MI.getFlags();
  unsigned OpLo = Z80::POPRd;
  unsigned OpHi = Z80::POPRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  buildMI(MBB, MBBI, OpHi, DstHiReg).setMIFlags(Flags); // High
  buildMI(MBB, MBBI, OpLo, DstLoReg).setMIFlags(Flags); // Low

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::ROLBRd>(Block &MBB, BlockIt MBBI) {
  // In Z80, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShift, OpCarry;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  OpShift = Z80::ADDRdRr;
  OpCarry = Z80::ADCRdRr;

  // add r16, r16
  // adc r16, r1

  // Shift part
  buildMI(MBB, MBBI, OpShift)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(DstReg);

  // Add the carry bit
  auto MIB = buildMI(MBB, MBBI, OpCarry)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(ZERO_REGISTER);

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::RORBRd>(Block &MBB, BlockIt MBBI) {
  // In Z80, the rotate instructions behave quite unintuitively. They rotate
  // bits through the carry bit in SREG, effectively rotating over 9 bits,
  // instead of 8. This is useful when we are dealing with numbers over
  // multiple registers, but when we actually need to rotate stuff, we have
  // to explicitly add the carry bit.

  MachineInstr &MI = *MBBI;
  unsigned OpShiftOut, OpLoad, OpShiftIn, OpAdd;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  OpShiftOut = Z80::LSRRd;
  OpLoad = Z80::LDIRdK;
  OpShiftIn = Z80::RORRd;
  OpAdd = Z80::ORRdRr;

  // lsr r16
  // ldi r0, 0
  // ror r0
  // or r16, r17

  // Shift out
  buildMI(MBB, MBBI, OpShiftOut)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg);

  // Put 0 in temporary register
  buildMI(MBB, MBBI, OpLoad)
    .addReg(SCRATCH_REGISTER, RegState::Define | getDeadRegState(true))
    .addImm(0x00);

  // Shift in
  buildMI(MBB, MBBI, OpShiftIn)
    .addReg(SCRATCH_REGISTER, RegState::Define | getDeadRegState(true))
    .addReg(SCRATCH_REGISTER);

  // Add the results together using an or-instruction
  auto MIB = buildMI(MBB, MBBI, OpAdd)
    .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstReg)
    .addReg(SCRATCH_REGISTER);

  // SREG is always implicitly killed
  MIB->getOperand(2).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LSLWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = Z80::ADDRdRr; // ADD Rd, Rd <==> LSL Rd
  unsigned OpHi = Z80::ADCRdRr; // ADC Rd, Rd <==> ROL Rd
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // Low part
  buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg)
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  auto MIBHI = buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg)
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBHI->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  MIBHI->getOperand(4).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LSRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = Z80::RORRd;
  unsigned OpHi = Z80::LSRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::RORWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("RORW unimplemented");
  return false;
}

template <>
bool Z80ExpandPseudo::expand<Z80::ROLWRd>(Block &MBB, BlockIt MBBI) {
  llvm_unreachable("ROLW unimplemented");
  return false;
}

template <>
bool Z80ExpandPseudo::expand<Z80::ASRWRd>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  unsigned OpLo = Z80::RORRd;
  unsigned OpHi = Z80::ASRRd;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  // High part
  buildMI(MBB, MBBI, OpHi)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, getKillRegState(DstIsKill));

  auto MIBLO = buildMI(MBB, MBBI, OpLo)
    .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstLoReg, getKillRegState(DstIsKill));

  if (ImpIsDead)
    MIBLO->getOperand(2).setIsDead();

  // SREG is always implicitly killed
  MIBLO->getOperand(3).setIsKill();

  MI.eraseFromParent();
  return true;
}
*/
template <> bool Z80ExpandPseudo::expand<Z80::SEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  // sext R17:R16, R17
  // mov     r16, r17
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R13
  // mov     r16, r13
  // mov     r17, r13
  // lsl     r17
  // sbc     r17, r17
  // sext R17:R16, R16
  // mov     r17, r16
  // lsl     r17
  // sbc     r17, r17
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    auto MOV = buildMI(MBB, MBBI, Z80::LDRdRr8)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg);

    if (SrcReg == DstHiReg) {
      MOV->getOperand(1).setIsKill();
    }
  }

  if (SrcReg != Z80::A) {
    buildMI(MBB, MBBI, Z80::LDRdRr8)
        .addReg(Z80::A, RegState::Define)
        .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  buildMI(MBB, MBBI, Z80::ADDRdRr8) // LSL Rd <==> ADD Rd, Rr
      .addReg(Z80::A, RegState::Define)
      .addReg(Z80::A, RegState::Kill)
      .addReg(Z80::A, RegState::Kill);

  buildMI(MBB, MBBI, Z80::SBCRdRr8)
      .addReg(Z80::A, RegState::Define)
      .addReg(Z80::A, RegState::Kill)
      .addReg(Z80::A, RegState::Kill);

  buildMI(MBB, MBBI, Z80::LDRdRr8)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(Z80::A, RegState::Kill);

/*  if (SrcReg != DstHiReg) {
    buildMI(MBB, MBBI, Z80::LDRdRr8)
      .addReg(DstHiReg, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }*/

  /*buildMI(MBB, MBBI, Z80::ADDRdRr8) // LSL Rd <==> ADD Rd, Rr
    .addReg(DstHiReg, RegState::Define)
    .addReg(DstHiReg)
    .addReg(DstHiReg, RegState::Kill);

  auto SBC = buildMI(MBB, MBBI, Z80::SBCRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill)
    .addReg(DstHiReg, RegState::Kill);

  if (ImpIsDead)
    SBC->getOperand(3).setIsDead();

  // SREG is always implicitly killed
  SBC->getOperand(4).setIsKill();
*/
  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::ZEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    buildMI(MBB, MBBI, Z80::LDRdRr8)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  /*auto EOR = buildMI(MBB, MBBI, Z80::EORRdRr)
    .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
    .addReg(DstHiReg, RegState::Kill)
    .addReg(DstHiReg, RegState::Kill);*/

  buildMI(MBB, MBBI, Z80::LDIRdK)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addImm(0);

  //if (ImpIsDead)
  //  EOR->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::ROT16>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  assert(SrcReg == DstReg);

  Z80II::Rotation rot = (Z80II::Rotation)MI.getOperand(2).getImm();

  std::tuple<unsigned, unsigned, bool> ops;

  switch (rot) {
  default:
    llvm_unreachable("wrong rotation code");

  case llvm::Z80II::ROT_SRL:
    ops = {Z80::SRLRd, Z80::RRRd, true};
    break;

  case llvm::Z80II::ROT_SLA:
    ops = {Z80::RLRd, Z80::SLARd, false};
    break;

  case llvm::Z80II::ROT_SRA:
    ops = {Z80::SRARd, Z80::RRRd, true};
    break;
  }

  MachineInstrBuilder hi, lo;

  auto inv = std::get<2>(ops);

  auto ehi = [&](){
    hi = buildMI(MBB, MBBI, std::get<0>(ops))
        .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(DstHiReg, getKillRegState(SrcIsKill));
  };

  if (inv)
    ehi();

  lo = buildMI(MBB, MBBI, std::get<1>(ops))
           .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
           .addReg(DstLoReg, getKillRegState(SrcIsKill));

  if (!inv)
    ehi();

  if (ImpIsDead) {
    lo->getOperand(2).setIsDead();
    hi->getOperand(2).setIsDead();
  }

  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::SUBRdRr16>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  buildMI(MBB, MBBI, Z80::CLEARC);
  auto mi = buildMI(MBB, MBBI, Z80::SBCRdRr16)
                .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
                .addReg(DstReg, getKillRegState(DstIsKill))
                .addReg(SrcReg, getKillRegState(SrcIsKill));

  if (ImpIsDead)
    mi->getOperand(3).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::SETRESBITWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  MachineOperand Ptr = MI.getOperand(0);
  unsigned offset = MI.getOperand(1).getImm();
  unsigned vo = MI.getOperand(3).getImm();
  unsigned va = MI.getOperand(2).getImm() | vo;

  SmallVector<int, 16> ors, ands;

  for (auto i = 0; i < 16; i++)
    if ((va & (1 << i)) == 0)
      ands.push_back(i);

  for (auto i = 0; i < 16; i++)
    if ((vo & (1 << i)) != 0)
      ors.push_back(i);

  for (auto i : ands)
    if (i < 8)
      buildMI(MBB, MBBI, Z80::RESBITWPTR).add(Ptr).addImm(offset).addImm(i);
    else
      buildMI(MBB, MBBI, Z80::RESBITWPTR)
          .add(Ptr)
          .addImm(offset + 1)
          .addImm(i - 8);

  for (auto i : ors)
    if (i < 8)
      buildMI(MBB, MBBI, Z80::SETBITWPTR).add(Ptr).addImm(offset).addImm(i);
    else
      buildMI(MBB, MBBI, Z80::SETBITWPTR)
          .add(Ptr)
          .addImm(offset + 1)
          .addImm(i - 8);

  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::COPYREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();

  buildMI(MBB, MBBI, Z80::PUSHRr).addReg(SrcReg, getKillRegState(SrcIsKill));
  buildMI(MBB, MBBI, Z80::POPRd)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));

  MI.eraseFromParent();

  return true;
}

bool Z80ExpandPseudo::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)               \
  case Op:                       \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(Z80::ADDWRdRr);
//    EXPAND(Z80::ADCWRdRr);
//    EXPAND(Z80::SUBIWRdK);
//    EXPAND(Z80::SBCWRdRr);
//    EXPAND(Z80::SBCIWRdK);
//    EXPAND(Z80::ANDWRdRr);
//    EXPAND(Z80::ANDIWRdK);
    EXPAND(Z80::ANDIWRdK_RESBIT);
    EXPAND(Z80::SETBITWPTR);
    EXPAND(Z80::RESBITWPTR);
//    EXPAND(Z80::ORWRdRr);
//    EXPAND(Z80::ORIWRdK);
    EXPAND(Z80::ORIWRdK_SETBIT);
//    EXPAND(Z80::EORWRdRr);
//    EXPAND(Z80::COMWRd);
    EXPAND(Z80::CPWRdRr);
//    EXPAND(Z80::CPCWRdRr);
    EXPAND(Z80::LDIWRdK);
    EXPAND(Z80::LDSWRdK);
    EXPAND(Z80::LDWRdPtr);
  case Z80::LDDWRdYQ: //:FIXME: remove this once PR13375 gets fixed
    EXPAND(Z80::LDDWRdPtrQ);
    EXPAND(Z80::STSWKRr);
    EXPAND(Z80::STWPtrRr);
//    EXPAND(Z80::STWPtrPiRr);
//    EXPAND(Z80::STWPtrPdRr);
    EXPAND(Z80::STDWPtrQRr);
//    EXPAND(Z80::INWRdA);
//    EXPAND(Z80::OUTWARr);
//    EXPAND(Z80::PUSHWRr);
//    EXPAND(Z80::POPWRd);
//    EXPAND(Z80::ROLBRd);
//    EXPAND(Z80::RORBRd);
//    EXPAND(Z80::LSLWRd);
//    EXPAND(Z80::LSRWRd);
//    EXPAND(Z80::RORWRd);
//    EXPAND(Z80::ROLWRd);
//    EXPAND(Z80::ASRWRd);
    EXPAND(Z80::SEXT);
    EXPAND(Z80::ZEXT);
    EXPAND(Z80::ROT16);
    EXPAND(Z80::SUBRdRr16);
    EXPAND(Z80::SETRESBITWPTR);
    EXPAND(Z80::COPYREG);
    return true;
  }
#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(Z80ExpandPseudo, "z80-expand-pseudo",
                Z80_EXPAND_PSEUDO_NAME, false, false)
namespace llvm {

FunctionPass *createZ80ExpandPseudoPass() { return new Z80ExpandPseudo(); }

} // end of namespace llvm
