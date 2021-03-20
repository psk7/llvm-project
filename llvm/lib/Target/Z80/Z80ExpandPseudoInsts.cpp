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

  bool emitIndirectWordLoad(Block &MBB, BlockIt MBBI, MachineInstr &MI,
                            Register Dst, Register Ptr, int Displacement,
                            bool PtrIsKill, bool DstIsDead);

  bool emitIndirectWordStore(Block &MBB, BlockIt MBBI, MachineInstr &MI,
                            Register Data, Register Ptr, int Displacement,
                            bool PtrIsKill, bool DataIsKill);
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

template <>
bool Z80ExpandPseudo::expand<Z80::RESBITW>(Block &MBB, BlockIt MBBI) {
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

template <>
bool Z80ExpandPseudo::expand<Z80::SETBITW>(Block &MBB, BlockIt MBBI) {
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

template <>
bool Z80ExpandPseudo::expand<Z80::CPW>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(1).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  buildMI(MBB, MBBI, Z80::CLEARC);

  auto MIBHI = buildMI(MBB, MBBI, Z80::SBCW)
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

template <>
bool Z80ExpandPseudo::expand<Z80::LDIWRdK>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();

  auto MIBLO = buildMI(MBB, MBBI, Z80::LDWk)
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
bool Z80ExpandPseudo::expand<Z80::LDWMEM>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register SrcReg = MI.getOperand(0).getReg();

  auto op = SrcReg == Z80::HL ? Z80::LDSWRdKOp : Z80::LDSWRdKOpExt;
  MI.setDesc(TII->get(op));

  return true;
}

bool Z80ExpandPseudo::emitIndirectWordLoad(Block &MBB, BlockIt MBBI,
                                           MachineInstr &MI, Register Dst,
                                           Register Ptr, int Displacement,
                                           bool PtrIsKill, bool DstIsDead) {
  Register DstLoReg, DstHiReg;
  TRI->splitReg(Dst, DstLoReg, DstHiReg);

  auto DstRegState = RegState::Define | getDeadRegState(DstIsDead);

  if (Z80::HL == Ptr && 0 == Displacement) {
    if (Z80::HL != Dst && Z80::IX != Dst && Z80::IY != Dst) {
      buildMI(MBB, MBBI, Z80::LDPTR)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(Ptr)
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::INCW, Ptr).addReg(Ptr);

      buildMI(MBB, MBBI, Z80::LDPTR)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(Ptr, getKillRegState(PtrIsKill))
          .setMemRefs(MI.memoperands());

      if (!PtrIsKill) {
        buildMI(MBB, MBBI, Z80::DECW, Ptr).addReg(Ptr);
      }

      return true;
    }

    bool DstIsHL = Z80::HL == Dst;

    auto TmpReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);

    if (TmpReg == -1)
      return false;

    buildMI(MBB, MBBI, Z80::LDPTR, TmpReg)
        .addReg(Ptr)
        .setMemRefs(MI.memoperands());

    buildMI(MBB, MBBI, Z80::INCW, Ptr).addReg(Ptr);

    if (DstIsHL) {
      buildMI(MBB, MBBI, Z80::LDPTR)
          .addReg(DstHiReg, DstRegState)
          .addReg(Ptr, getKillRegState(PtrIsKill))
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::LD)
          .addReg(DstLoReg, DstRegState)
          .addReg(TmpReg, getKillRegState(true));
    } else {
      buildMI(MBB, MBBI, Z80::LD)
          .addReg(DstLoReg, DstRegState)
          .addReg(TmpReg, getKillRegState(true));

      buildMI(MBB, MBBI, Z80::LDPTR, TmpReg)
          .addReg(Ptr, getKillRegState(PtrIsKill))
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::LD)
          .addReg(DstHiReg, DstRegState)
          .addReg(TmpReg, getKillRegState(true));
    }

    if (!PtrIsKill) {
      buildMI(MBB, MBBI, Z80::DECW, Ptr).addReg(Ptr);
    }

    return true;
  }

  if (Z80::IX == Ptr || Z80::IY == Ptr) {
    if (Z80::IX != Dst && Z80::IY != Dst) {
      buildMI(MBB, MBBI, Z80::LDDPTR)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(Ptr)
          .addImm(Displacement)
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::LDDPTR)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
          .addReg(Ptr, getKillRegState(PtrIsKill))
          .addImm(Displacement + 1)
          .setMemRefs(MI.memoperands());
      return true;
    }

    auto TempPair = scavengeGPR8(MI, &Z80::BDREGSRegClass);
    bool UseAltBank = false;

    if (TempPair == -1) {
      UseAltBank = true;
      TempPair = Z80::BC;
      buildMI(MBB, MBBI, Z80::EXX);
    }

    Register TmpReg1, TmpReg2;

    TRI->splitReg(TempPair, TmpReg1, TmpReg2);

    buildMI(MBB, MBBI, Z80::LDDPTR, TmpReg1)
        .addReg(Ptr)
        .addImm(Displacement)
        .setMemRefs(MI.memoperands());

    buildMI(MBB, MBBI, Z80::LDDPTR, TmpReg2)
        .addReg(Ptr, getKillRegState(PtrIsKill))
        .addImm(Displacement + 1)
        .setMemRefs(MI.memoperands());

    buildMI(MBB, MBBI, Z80::COPYREGW)
        .addReg(Dst, DstRegState)
        .addReg(TempPair, getKillRegState(true));

    if (UseAltBank) {
      buildMI(MBB, MBBI, Z80::EXX);
    }

    return true;
  }

  return false;
}

bool Z80ExpandPseudo::emitIndirectWordStore(Block &MBB, BlockIt MBBI,
                                            MachineInstr &MI, Register Data,
                                            Register Ptr, int Displacement,
                                            bool PtrIsKill, bool DataIsKill) {

  Register DataLoReg, DataHiReg;
  TRI->splitReg(Data, DataLoReg, DataHiReg);

  auto PtrRegState = getKillRegState(PtrIsKill);
  auto DataRegState = getKillRegState(DataIsKill);

  if (Z80::HL == Ptr && Displacement == 0){
    if (Z80::HL != Data && Z80::IX != Data && Z80::IY != Data) {
      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr)
          .addReg(DataLoReg, DataRegState)
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::INCW, Ptr).addReg(Ptr);

      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr, PtrRegState)
          .addReg(DataHiReg, DataRegState)
          .setMemRefs(MI.memoperands());

      if(!PtrIsKill) {
        buildMI(MBB, MBBI, Z80::DECW, Ptr).addReg(Ptr, PtrRegState);
      }

      return true;
    }

    bool DataIsHL = Z80::HL == Data;

    auto TmpReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);

    if (TmpReg == -1)
      return false;

    if(DataIsHL) {
      buildMI(MBB, MBBI, Z80::LD, TmpReg).addReg(DataHiReg, DataRegState);

      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr)
          .addReg(DataLoReg, DataRegState)
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::INCW, Ptr).addReg(Ptr);

      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr, PtrRegState)
          .addReg(TmpReg, getKillRegState(true))
          .setMemRefs(MI.memoperands());
    } else {
      buildMI(MBB, MBBI, Z80::LD, TmpReg).addReg(DataLoReg, DataRegState);
      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr)
          .addReg(TmpReg, getKillRegState(true))
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::INCW, Ptr).addReg(Ptr);

      buildMI(MBB, MBBI, Z80::LD, TmpReg).addReg(DataHiReg, DataRegState);
      buildMI(MBB, MBBI, Z80::STPTR)
          .addReg(Ptr, PtrRegState)
          .addReg(TmpReg, getKillRegState(true))
          .setMemRefs(MI.memoperands());
    }

    if (!PtrIsKill) {
      buildMI(MBB, MBBI, Z80::DECW, Ptr).addReg(Ptr, PtrRegState);
    }

    return true;
  }

  if (Z80::IX == Ptr || Z80::IY == Ptr) {
    if (Z80::IX != Data && Z80::IY != Data) {
      buildMI(MBB, MBBI, Z80::STDPTR)
          .addReg(Ptr)
          .addImm(Displacement)
          .addReg(DataLoReg, DataRegState)
          .setMemRefs(MI.memoperands());

      buildMI(MBB, MBBI, Z80::STDPTR)
          .addReg(Ptr, PtrRegState)
          .addImm(Displacement + 1)
          .addReg(DataHiReg, DataRegState)
          .setMemRefs(MI.memoperands());

      return true;
    }

    auto TempPair = scavengeGPR8(MI, &Z80::BDREGSRegClass);
    bool UseAltBank = false;

    if (TempPair == -1) {
      UseAltBank = true;
      TempPair = Z80::BC;
      buildMI(MBB, MBBI, Z80::EXX);
    }

    buildMI(MBB, MBBI, Z80::COPYREGW, TempPair)
        .addReg(Data, DataRegState);

    Register TmpLoReg, TmpHiReg;
    TRI->splitReg(TempPair, TmpLoReg, TmpHiReg);

    buildMI(MBB, MBBI, Z80::STDPTR)
        .addReg(Ptr)
        .addImm(Displacement)
        .addReg(TmpLoReg, getKillRegState(true))
        .setMemRefs(MI.memoperands());

    buildMI(MBB, MBBI, Z80::STDPTR)
        .addReg(Ptr, PtrRegState)
        .addImm(Displacement + 1)
        .addReg(TmpHiReg, getKillRegState(true))
        .setMemRefs(MI.memoperands());

    if (UseAltBank) {
      buildMI(MBB, MBBI, Z80::EXX);
    }

    return true;
  }

  return false;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool DstIsDead = MI.getOperand(0).isDead();

  auto r = emitIndirectWordLoad(MBB, MBBI, MI, DstReg, SrcReg, 0, SrcIsKill,
                                DstIsDead);

  if (r)
    MI.eraseFromParent();

  return r;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDDWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  auto Displacement = MI.getOperand(2).getImm();
  bool SrcIsKill = MI.getOperand(1).isKill();

  if (Displacement != 0)
    assert(SrcReg == Z80::IX ||
           SrcReg == Z80::IY && "Only IX or IY can has displacement");

  assert(Displacement <= 126 && "Offset is out of range");

  auto r = emitIndirectWordLoad(MBB, MBBI, MI, DstReg, SrcReg, Displacement,
                                SrcIsKill, DstIsDead);

  if (r)
    MI.eraseFromParent();

  return r;
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
bool Z80ExpandPseudo::expand<Z80::STWMEM>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register SrcReg = MI.getOperand(1).getReg();

  auto op = SrcReg == Z80::HL ? Z80::STSWKRrOp : Z80::STSWKRrOpExt;
  MI.setDesc(TII->get(op));

  return true;
}

template <>
bool Z80ExpandPseudo::expand<Z80::STWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register PtrReg = MI.getOperand(0).getReg();
  Register DataReg = MI.getOperand(1).getReg();
  bool PtrIsKill = MI.getOperand(0).isKill();
  bool DataIsKill = MI.getOperand(1).isKill();

  auto r = emitIndirectWordStore(MBB, MBBI, MI, DataReg, PtrReg, 0, PtrIsKill,
                                 DataIsKill);

  if (r)
    MI.eraseFromParent();

  return r;
}

template <>
bool Z80ExpandPseudo::expand<Z80::STDWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register PtrReg = MI.getOperand(0).getReg();
  Register DataReg = MI.getOperand(2).getReg();
  int Displacement = MI.getOperand(1).getImm();
  bool PtrIsKill = MI.getOperand(0).isKill();
  bool DataIsKill = MI.getOperand(2).isKill();

  assert(Displacement <= 126 && Displacement >= -126 && "Offset is out of range");

  auto r = emitIndirectWordStore(MBB, MBBI, MI, DataReg, PtrReg, Displacement,
                                 PtrIsKill, DataIsKill);

  if (r)
    MI.eraseFromParent();

  return r;
}

template <>
bool Z80ExpandPseudo::expand<Z80::LDSTDWPTR>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register Res = MI.getOperand(0).getReg();
  Register Dst = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(3).getReg();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(3).isKill();
  bool ResIsDead = MI.getOperand(0).isDead();
  auto DstOffs = MI.getOperand(2).getImm();
  auto SrcOffs = MI.getOperand(4).getImm();

  if (Z80::IX != Src && Z80::IY != Src)
    return false;

  if (Z80::IX != Dst && Z80::IY != Dst)
    return false;

  if (Z80::IX != Res && Z80::IY != Res)
    return false;

  Register LoReg, HiReg;
  TRI->splitReg(Res, LoReg, HiReg);

  buildMI(MBB, MBBI, Z80::EXX);

  buildMI(MBB, MBBI, Z80::LDDPTR)
      .addReg(Z80::C, RegState::Define)
      .addReg(Src)
      .addImm(SrcOffs);

  buildMI(MBB, MBBI, Z80::LDDPTR)
      .addReg(Z80::B, RegState::Define)
      .addReg(Src, getKillRegState(SrcIsKill))
      .addImm(SrcOffs + 1);

  buildMI(MBB, MBBI, Z80::STDPTR)
      .addReg(Dst)
      .addImm(DstOffs)
      .addReg(Z80::C, getKillRegState(ResIsDead));

  buildMI(MBB, MBBI, Z80::STDPTR)
      .addReg(Dst, getKillRegState(DstIsKill))
      .addImm(DstOffs + 1)
      .addReg(Z80::B, getKillRegState(ResIsDead));

  if (!ResIsDead) {
    buildMI(MBB, MBBI, Z80::COPYREG)
        .addReg(LoReg, RegState::Define | getDeadRegState(ResIsDead))
        .addReg(Z80::C, getKillRegState(true));

    buildMI(MBB, MBBI, Z80::COPYREG)
        .addReg(HiReg, RegState::Define | getDeadRegState(ResIsDead))
        .addReg(Z80::B, getKillRegState(true));
  }

  buildMI(MBB, MBBI, Z80::EXX);

  MI.eraseFromParent();

  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::SEXT>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();
  bool ImpIsDead = MI.getOperand(2).isDead();
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    auto MOV = buildMI(MBB, MBBI, Z80::LD)
      .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(SrcReg);

    if (SrcReg == DstHiReg) {
      MOV->getOperand(1).setIsKill();
    }
  }

  if (SrcReg != Z80::A) {
    buildMI(MBB, MBBI, Z80::LD)
        .addReg(Z80::A, RegState::Define)
        .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  buildMI(MBB, MBBI, Z80::ADD) // LSL Rd <==> ADD Rd, Rr
      .addReg(Z80::A, RegState::Define)
      .addReg(Z80::A, RegState::Kill)
      .addReg(Z80::A, RegState::Kill);

  buildMI(MBB, MBBI, Z80::SBC8)
      .addReg(Z80::A, RegState::Define)
      .addReg(Z80::A, RegState::Kill)
      .addReg(Z80::A, RegState::Kill);

  buildMI(MBB, MBBI, Z80::LD)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(Z80::A, RegState::Kill);

/*  if (SrcReg != DstHiReg) {
    buildMI(MBB, MBBI, Z80::LDRdRr8)
      .addReg(DstHiReg, RegState::Define)
      .addReg(SrcReg, getKillRegState(SrcIsKill));
  }*/

  /*buildMI(MBB, MBBI, Z80::ADD) // LSL Rd <==> ADD Rd, Rr
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
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  if (SrcReg != DstLoReg) {
    buildMI(MBB, MBBI, Z80::COPYREG)
        .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcReg, getKillRegState(SrcIsKill));
  }

  buildMI(MBB, MBBI, Z80::LDk)
      .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
      .addImm(0);

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
    ops = {Z80::SRL, Z80::RR, true};
    break;

  case llvm::Z80II::ROT_SLA:
    ops = {Z80::RL, Z80::SLA, false};
    break;

  case llvm::Z80II::ROT_SRA:
    ops = {Z80::SRA, Z80::RR, true};
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

  if (!inv && ImpIsDead)
    hi->getOperand(2).setIsDead();

  if (inv && ImpIsDead)
    lo->getOperand(2).setIsDead();

  MI.eraseFromParent();
  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::SUBW>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(2).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool DstIsKill = MI.getOperand(1).isKill();
  bool SrcIsKill = MI.getOperand(2).isKill();
  bool ImpIsDead = MI.getOperand(3).isDead();

  buildMI(MBB, MBBI, Z80::CLEARC);
  auto mi = buildMI(MBB, MBBI, Z80::SBCW)
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

  assert(Z80::GPR8RegClass.contains(SrcReg, DstReg));

  bool SrcHasX = (Z80::XL == SrcReg) || (Z80::XH == SrcReg);
  bool DstHasX = (Z80::XL == DstReg) || (Z80::XH == DstReg);
  bool SrcHasY = (Z80::YL == SrcReg) || (Z80::YH == SrcReg);
  bool DstHasY = (Z80::YL == DstReg) || (Z80::YH == DstReg);
  bool SrcHasHL = (Z80::H == SrcReg) || (Z80::L == SrcReg);
  bool DstHasHL = (Z80::H == DstReg) || (Z80::L == DstReg);

  bool HasHL = SrcHasHL || DstHasHL;
  bool HasX = SrcHasX || DstHasX;
  bool HasY = SrcHasY || DstHasY;

  bool NeedIntermediateReg = (HasHL && (HasX || HasY)) || (HasX && HasY);

  if (!NeedIntermediateReg) {
    buildMI(MBB, MBBI, Z80::LD)
        .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcReg, getKillRegState(SrcIsKill))
        .copyImplicitOps(MI);

    MI.eraseFromParent();
    return true;
  }

  auto IntermediateReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);

  bool SavedBC = false;
  bool UsingAltAF = false;

  if (IntermediateReg == -1) {
    if (Z80::A != SrcReg) {
      IntermediateReg = Z80::A;
      UsingAltAF = true;
      buildMI(MBB, MBBI, Z80::EXAF);
    } else {
      SavedBC = true;
      IntermediateReg = Z80::B;
      buildMI(MBB, MBBI, Z80::PUSH).addReg(Z80::BC, getKillRegState(true));
    }
  }

  buildMI(MBB, MBBI, Z80::LD, IntermediateReg)
      .addReg(SrcReg, getKillRegState(SrcIsKill));

  buildMI(MBB, MBBI, Z80::LD)
      .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead))
      .addReg(IntermediateReg, getKillRegState(true))
      .copyImplicitOps(MI);

  if (UsingAltAF) {
    buildMI(MBB, MBBI, Z80::EXAF);
  } else if (SavedBC) {
    buildMI(MBB, MBBI, Z80::POP, Z80::BC);
  }

  MI.eraseFromParent();

  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::COPYREGW>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool SrcIsKill = MI.getOperand(1).isKill();

  assert(Z80::DREGSRegClass.contains(SrcReg, DstReg));

  bool SrcIsIX = Z80::IX == SrcReg;
  bool DstIsIX = Z80::IX == DstReg;
  bool SrcIsIY = Z80::IY == SrcReg;
  bool DstIsIY = Z80::IY == DstReg;
  bool SrcIsHL = Z80::HL == SrcReg;
  bool DstIsHL = Z80::HL == DstReg;

  bool HasHL = SrcIsHL || DstIsHL;
  bool HasIX = SrcIsIX || DstIsIX;
  bool HasIY = SrcIsIY || DstIsIY;

  bool CopyThroughStack = (HasHL && (HasIX || HasIY)) || (HasIX && HasIY);

  if (CopyThroughStack) {
    buildMI(MBB, MBBI, Z80::PUSH).addReg(SrcReg, getKillRegState(SrcIsKill));
    buildMI(MBB, MBBI, Z80::POP)
        .addReg(DstReg, RegState::Define | getDeadRegState(DstIsDead));
  } else {
    Register SrcLoReg, SrcHiReg, DstLoReg, DstHiReg;
    TRI->splitReg(SrcReg, SrcLoReg, SrcHiReg);
    TRI->splitReg(DstReg, DstLoReg, DstHiReg);

    buildMI(MBB, MBBI, Z80::LD)
        .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcLoReg, getKillRegState(SrcIsKill));

    buildMI(MBB, MBBI, Z80::LD)
        .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead))
        .addReg(SrcHiReg, getKillRegState(SrcIsKill));
  }

  MI.eraseFromParent();

  return true;
}

template <> bool Z80ExpandPseudo::expand<Z80::STDPTR_P>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register Reg = MI.getOperand(2).getReg();
  Register Ptr = MI.getOperand(0).getReg();
  bool RegIsKill = MI.getOperand(2).isKill();
  bool PtrIsKill = MI.getOperand(0).isKill();
  auto Offset = MI.getOperand(1).getImm();

  if (Z80::XL != Reg && Z80::XH != Reg && Z80::YL != Reg && Z80::YH != Reg) {
    MI.setDesc(TII->get(Z80::STDPTR));
    return true;
  }

  Register TempReg = scavengeGPR8(MI, &Z80::GPR8_NoHLRegClass);

  if (TempReg == -1)
    return false;

  buildMI(MBB, MBBI, Z80::LD, TempReg).addReg(Reg, getKillRegState(RegIsKill));
  buildMI(MBB, MBBI, Z80::STDPTR)
      .addReg(Ptr, getKillRegState(PtrIsKill))
      .addImm(Offset)
      .addReg(TempReg, getKillRegState(true));

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
    EXPAND(Z80::RESBITW);
    EXPAND(Z80::SETBITWPTR);
    EXPAND(Z80::RESBITWPTR);
    EXPAND(Z80::SETBITW);
    EXPAND(Z80::CPW);
    EXPAND(Z80::LDIWRdK);
    EXPAND(Z80::LDWMEM);
    //EXPAND(Z80::LDWPTR);
  //case Z80::LDDWRdYQ: //:FIXME: remove this once PR13375 gets fixed
    //EXPAND(Z80::LDDWPTR);
    EXPAND(Z80::STWMEM);
    //EXPAND(Z80::STWPTR);
    //EXPAND(Z80::STDWPTR);
    EXPAND(Z80::SEXT);
    EXPAND(Z80::ZEXT);
    EXPAND(Z80::ROT16);
    EXPAND(Z80::SUBW);
    EXPAND(Z80::SETRESBITWPTR);
    //EXPAND(Z80::COPYREG);
    //EXPAND(Z80::COPYREGW);
    //EXPAND(Z80::LDSTDWPTR);
    //EXPAND(Z80::STDPTR_P);
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
