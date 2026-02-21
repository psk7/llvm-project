//===-- PDPExpandPseudoInsts.cpp - Expand pseudo instructions -------------===//
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

#include "MCTargetDesc/PDPMCCodeEmitter.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "PDP.h"
#include "PDPInstrInfo.h"
#include "PDPTargetMachine.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

using namespace llvm;

#define PDP_EXPAND_PSEUDO_NAME "PDP pseudo instruction expansion pass"

namespace {

/// Expands "placeholder" instructions marked as pseudo into
/// actual PDP instructions.
class PDPExpandPseudo : public MachineFunctionPass {
public:
  static char ID;

  PDPExpandPseudo() : MachineFunctionPass(ID) {
    initializePDPExpandPseudoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override { return PDP_EXPAND_PSEUDO_NAME; }

private:
  typedef MachineBasicBlock Block;
  typedef Block::iterator BlockIt;

  const PDPRegisterInfo *TRI;
  const PDPInstrInfo *TII;

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

  MachineRegisterInfo &getRegInfo(Block &MBB) {
    return MBB.getParent()->getRegInfo();
  }

  template <typename Func> bool expandAtomic(Block &MBB, BlockIt MBBI, Func f);

  template <typename Func>
  bool expandAtomicBinaryOp(unsigned Opcode, Block &MBB, BlockIt MBBI, Func f);

  bool expandTwoOpWithResult(Block &MBB, BlockIt MBBI, unsigned OpCode);
  bool expandTwoOpWithoutResult(Block &MBB, BlockIt MBBI, unsigned OpCode);

  bool expandOneOp(Block &MBB, BlockIt MBBI, unsigned OpCode);

  bool expandCondBr(Block &MBB, BlockIt MBBI, unsigned OpCode, unsigned OpCodeBit,
                    unsigned OpCodeTst);

  bool expandEMT(Block &MBB, BlockIt MBBI);
};

char PDPExpandPseudo::ID = 0;

bool IsImmediate(const MachineInstr &MI, unsigned OpNo, int &Val) {
  if (MI.getOperand(OpNo).getReg() != PDP::R7)
    return false;

  if (!MI.getOperand(OpNo +1).isImm() || MI.getOperand(OpNo +1).getImm() != 2)
    return false;

  Val = MI.getOperand(OpNo + 2).getImm() + MI.getOperand(OpNo + 3).getImm();

  return true;
}

bool PDPExpandPseudo::expandMBB(MachineBasicBlock &MBB) {
  bool Modified = false;

  BlockIt MBBI = MBB.begin(), E = MBB.end();
  while (MBBI != E) {
    BlockIt NMBBI = std::next(MBBI);

    const auto &Desc = TII->get(MBBI->getOpcode());

    PDPMCCodeEmitter::forEachArg(Desc, [&MBBI, &Modified](unsigned N)->void {
      if (MBBI->getOperand(N + 1).getImm() != 6)
        return;

      if (!MBBI->getOperand(N + 2).isImm() || !MBBI->getOperand(N + 3).isImm())
        return;

      int Val =
          MBBI->getOperand(N + 2).getImm() + MBBI->getOperand(N + 3).getImm();

      if (Val == 0) {
        MBBI->getOperand(N + 1).setImm(1);
        Modified = true;
      }
    });

    Modified |= expandMI(MBB, MBBI);
    MBBI = NMBBI;

  }

  return Modified;
}

bool PDPExpandPseudo::runOnMachineFunction(MachineFunction &MF) {
  bool Modified = false;

  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();
  TRI = STI.getRegisterInfo();
  TII = dyn_cast<PDPInstrInfo>(STI.getInstrInfo());

  for (Block &MBB : MF) {
    bool ContinueExpanding = true;
    unsigned ExpandCount = 0;

    // Continue expanding the block until all pseudos are expanded.
    do {
      assert(ExpandCount < 10 && "pseudo expand limit reached");
      (void)ExpandCount;

      bool BlockModified = expandMBB(MBB);
      Modified |= BlockModified;
      ExpandCount++;

      ContinueExpanding = BlockModified;
    } while (ContinueExpanding);
  }

  return Modified;
}

static void createUniop(const MachineInstrBuilder &MIB, MachineOperand &Op,
                        bool IsDirect, long offset) {
  auto OpType = Op.getType();

  if (Op.isReg()) {
    bool IsKill = Op.isKill();
    bool IsDead = Op.isDead();
    bool IsDef = Op.isDef();

    MIB.addReg(Op.getReg(), getDefRegState(IsDef) | getKillRegState(IsKill) |
                                getDeadRegState(IsDead));

    if (IsDirect)
      MIB.addImm(0).addImm(0).addImm(0);
    else {
      if (offset == 0)
        MIB.addImm(1).addImm(0).addImm(0);
      else
        MIB.addImm(6).addImm(offset).addImm(0);
    }
  } else if (Op.isImm()) {
    MIB.addReg(PDP::R7);

    if (IsDirect)
      MIB.addImm(2).addImm(Op.getImm()).addImm(0);
    else
      MIB.addImm(3).addImm(Op.getImm()).addImm(0);
  } else if (OpType == MachineOperand::MO_GlobalAddress) {
    const GlobalValue *GV = Op.getGlobal();
    unsigned TF = Op.getTargetFlags();

    if (IsDirect)
      MIB.addReg(PDP::R7).addImm(2).addGlobalAddress(GV, Op.getOffset(), TF).addImm(0);
    else
      MIB.addReg(PDP::R7).addImm(3).addGlobalAddress(GV, Op.getOffset(), TF).addImm(0);
  } else
    llvm_unreachable("Unknown operand type!");
}

static void createUniopDirect(const MachineInstrBuilder &M, MachineOperand &Op,
                              long offset = 0) {
  createUniop(M, Op, true, offset);
}

static void createUniopIndirect(const MachineInstrBuilder &M,
                                MachineOperand &Op, long offset = 0) {
  createUniop(M, Op, false, offset);
}

static void createUniopRegister(const MachineInstrBuilder &M,
                                Register Register) {
  M.addReg(Register);
  M.addImm(0);
  M.addImm(0);
  M.addImm(0);
}

/*template <>
bool PDPExpandPseudo::expand<PDP::LDIWRdK>(Block &MBB, BlockIt MBBI) {
  return false;

  MachineInstr &MI = *MBBI;
  Register DstLoReg, DstHiReg;
  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();
  unsigned OpLo = PDP::LDIRdK;
  unsigned OpHi = PDP::LDIRdK;
  TRI->splitReg(DstReg, DstLoReg, DstHiReg);

  auto MIBLO =
      buildMI(MBB, MBBI, OpLo)
          .addReg(DstLoReg, RegState::Define | getDeadRegState(DstIsDead));

  auto MIBHI =
      buildMI(MBB, MBBI, OpHi)
          .addReg(DstHiReg, RegState::Define | getDeadRegState(DstIsDead));

  switch (MI.getOperand(1).getType()) {
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MI.getOperand(1).getGlobal();
    int64_t Offs = MI.getOperand(1).getOffset();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.addGlobalAddress(GV, Offs, TF | PDPII::MO_LO);
    MIBHI.addGlobalAddress(GV, Offs, TF | PDPII::MO_HI);
    break;
  }
  case MachineOperand::MO_BlockAddress: {
    const BlockAddress *BA = MI.getOperand(1).getBlockAddress();
    unsigned TF = MI.getOperand(1).getTargetFlags();

    MIBLO.add(MachineOperand::CreateBA(BA, TF | PDPII::MO_LO));
    MIBHI.add(MachineOperand::CreateBA(BA, TF | PDPII::MO_HI));
    break;
  }
  case MachineOperand::MO_Immediate: {
    unsigned Imm = MI.getOperand(1).getImm();

    MIBLO.addImm(Imm & 0xff);
    MIBHI.addImm((Imm >> 8) & 0xff);
    break;
  }
  default:
    llvm_unreachable("Unknown operand type!");
  }

  MI.eraseFromParent();
  return true;
}*/

template <typename Func>
bool PDPExpandPseudo::expandAtomicBinaryOp(unsigned Opcode, Block &MBB,
                                           BlockIt MBBI, Func f) {
  return expandAtomic(MBB, MBBI, [&](MachineInstr &MI) {
    auto Op1 = MI.getOperand(0);
    auto Op2 = MI.getOperand(1);

    MachineInstr &NewInst =
        *buildMI(MBB, MBBI, Opcode).add(Op1).add(Op2).getInstr();
    f(NewInst);
  });
}

/*bool PDPExpandPseudo::expandAtomicBinaryOp(unsigned Opcode, Block &MBB,
                                           BlockIt MBBI) {
  return expandAtomicBinaryOp(Opcode, MBB, MBBI, [](MachineInstr &MI) {});
}*/

/*template <> bool PDPExpandPseudo::expand<PDP::LDST_>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  auto M = buildMI(MBB, MBBI, PDP::MOV);

  createUniopIndirect(M, MI.getOperand(0));
  createUniopIndirect(M, MI.getOperand(1));

  MI.eraseFromParent();

  return true;
}*/

/*template <>
bool PDPExpandPseudo::expand<PDP::STDWSPQRr>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  const MachineFunction &MF = *MBB.getParent();
  const PDPSubtarget &STI = MF.getSubtarget<PDPSubtarget>();

  assert(MI.getOperand(0).getReg() == PDP::SP &&
         "SP is expected as base pointer");

  assert(STI.getFrameLowering()->hasReservedCallFrame(MF) &&
         "unexpected STDWSPQRr pseudo instruction");
  (void)STI;

  MI.setDesc(TII->get(PDP::STDWPtrQRr));
  MI.getOperand(0).setReg(PDP::R29R28);

  return true;
}*/


template <> bool PDPExpandPseudo::expand<PDP::PUSH>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();

  auto mov = buildMI(MBB, MBBI, PDP::MOV)
      .addReg(SrcReg, getKillRegState(SrcIsKill)).addImm(0).addImm(0).addImm(0)
      .addReg(PDP::R6).addImm(4).addImm(0).addImm(0);

  mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template <> bool PDPExpandPseudo::expand<PDP::POP>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  Register DstReg = MI.getOperand(0).getReg();
  bool DstIsDead = MI.getOperand(0).isDead();

  auto mov = buildMI(MBB, MBBI, PDP::MOV)
      .addReg(PDP::R6).addImm(2).addImm(0).addImm(0)
      .addReg(DstReg, getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0);

  mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::L__>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(1).isDead();
  bool DstIsDead = MI.getOperand(0).isDead();
  bool ImpIsDead = MI.getOperand(5).isDead();
  const auto DstReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::MOV);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  mov.addReg(DstReg, getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0);

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::S__>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool ImpIsDead = MI.getOperand(5).isDead();

  auto mov = buildMI(MBB, MBBI, PDP::MOV);
  mov->addOperand(MI.getOperand(0));

  mov.addImm(0).addImm(0).addImm(0);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::SB_>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  auto SrcReg = MI.getOperand(0).getReg();
  bool SrcIsKill = MI.getOperand(0).isKill();
  bool ImpIsDead = MI.getOperand(5).isDead();

  auto mov = buildMI(MBB, MBBI, PDP::MOVB)
    .addReg(TRI->getSuperReg(SrcReg), getKillRegState(SrcIsKill));

  mov.addImm(0).addImm(0).addImm(0);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::LB_>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(1).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();
  const bool ImpIsDead = MI.getOperand(5).isDead();
  const auto DstReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::MOVB);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  mov.addReg(TRI->getSuperReg(DstReg), getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0);

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

/*template<>
bool PDPExpandPseudo::expand<PDP::CMPREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(0).isKill();
  bool ImpIsDead = MI.getOperand(5).isDead();
  const auto DstReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::CMP)
    .addReg(DstReg, getKillRegState(SrcIsKill))
    .addImm(0).addImm(0).addImm(0);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::CMPBREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(0).isKill();
  bool ImpIsDead = MI.getOperand(5).isDead();
  const auto DstReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::CMPB)
    .addReg(DstReg, getKillRegState(SrcIsKill))
    .addImm(0).addImm(0).addImm(0);

  mov->addOperand(MI.getOperand(1));
  mov->addOperand(MI.getOperand(2));
  mov->addOperand(MI.getOperand(3));
  mov->addOperand(MI.getOperand(4));

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}*/

template<>
bool PDPExpandPseudo::expand<PDP::TSTREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(0).isKill();
  bool ImpIsDead = MI.getOperand(1).isDead();
  const auto SrcReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::TST)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(0).addImm(0).addImm(0);

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(4).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::TSTBREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool SrcIsKill = MI.getOperand(0).isKill();
  bool ImpIsDead = MI.getOperand(1).isDead();
  const auto SrcReg = MI.getOperand(0).getReg();

  auto mov = buildMI(MBB, MBBI, PDP::TSTB)
    .addReg(SrcReg, getKillRegState(SrcIsKill))
    .addImm(0).addImm(0).addImm(0);

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(4).setIsDead();

  MI.eraseFromParent();

  return true;
}

bool PDPExpandPseudo::expandOneOp(Block &MBB, BlockIt MBBI, const unsigned OpCode) {
  MachineInstr &MI = *MBBI;

//  MI.dumpr(getRegInfo(MBB));

  const bool HasArgs = MI.getNumOperands() - MI.getNumImplicitOperands() > 1;

  const bool ImpIsDead = MI.getOperand(HasArgs ? 2 : 1).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();
  auto DstReg = MI.getOperand(HasArgs ? 1 : 0).getReg();

  if (PDP::GPR16lRegClass.contains(DstReg))
    DstReg = TRI->getSuperReg(DstReg);

  auto &Op = buildMI(MBB, MBBI, OpCode)
      .addReg(DstReg, getKillRegState(DstIsDead)) // <-- НЕ ошибка
      .addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    Op->getOperand(4).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

//  Op->dumpr(getRegInfo(MBB));

  return true;
}

bool PDPExpandPseudo::expandTwoOpWithResult(Block &MBB, BlockIt MBBI, const unsigned OpCode) {
  MachineInstr &MI = *MBBI;

//  MI.dumpr(getRegInfo(MBB));

  const bool ImpIsDead = MI.getOperand(6).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();

  const auto SrcReg = MI.getOperand(1).getReg();
  auto DstReg = MI.getOperand(0).getReg();

  if (PDP::GPR16lRegClass.contains(DstReg))
    DstReg = TRI->getSuperReg(DstReg);

  const auto Op = buildMI(MBB, MBBI, OpCode)
      .addReg(SrcReg, getKillRegState(DstIsDead)); // <-- НЕ ошибка
  Op->addOperand(MI.getOperand(2));
  Op->addOperand(MI.getOperand(3));
  Op->addOperand(MI.getOperand(4));
  Op->addOperand(MI.getOperand(5));  // $rd

  // ReSharper disable once CppExpressionWithoutSideEffects
  Op.addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  Op->getOperand(4).setReg(DstReg);
  Op->getOperand(4).setIsKill(DstIsDead);

  if (ImpIsDead)
    Op->getOperand(8).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

//  Op->dumpr(getRegInfo(MBB));

  return true;
}

bool PDPExpandPseudo::expandTwoOpWithoutResult(Block &MBB, BlockIt MBBI, const unsigned OpCode) {
  MachineInstr &MI = *MBBI;

  const bool ImpIsDead = MI.getOperand(6).isDead();
  const bool Src1IsKill = MI.getOperand(0).isKill();

  const auto Src2Reg = MI.getOperand(1).getReg();
  auto Src1Reg = MI.getOperand(0).getReg();

  if (PDP::GPR16lRegClass.contains(Src1Reg))
    Src1Reg = TRI->getSuperReg(Src1Reg);

  const auto Op = buildMI(MBB, MBBI, OpCode)
      .addReg(Src2Reg, getKillRegState(Src1IsKill)); // <-- НЕ ошибка
  Op->addOperand(MI.getOperand(2));
  Op->addOperand(MI.getOperand(3));
  Op->addOperand(MI.getOperand(4));
  Op->addOperand(MI.getOperand(5));  // $rd

  // ReSharper disable once CppExpressionWithoutSideEffects
  Op.addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  Op->getOperand(4).setReg(Src1Reg);
  Op->getOperand(4).setIsKill(Src1IsKill);

  if (ImpIsDead)
    Op->getOperand(8).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::XORREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

//  MI.dumpr(getRegInfo(MBB));

  const bool ImpIsDead = MI.getOperand(3).isDead();

  const auto SrcReg = MI.getOperand(1).getReg();
  const auto SrcIsKill = MI.getOperand(1).isKill();

  const auto DstReg = MI.getOperand(0).getReg();
  const auto DstIsDead = MI.getOperand(0).isDead();

  const auto Op = buildMI(MBB, MBBI, PDP::XOR)
      .addReg(SrcReg, getKillRegState(SrcIsKill))
      .addReg(DstReg, getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    Op->getOperand(5).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

//  Op->dumpr(getRegInfo(MBB));

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::SXTREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  const bool ImpIsDead = MI.getOperand(2).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();
  auto DstReg = MI.getOperand(0).getReg();

  const auto Op = buildMI(MBB, MBBI, PDP::MOVB)
      .addReg(DstReg, getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0)
      .addReg(DstReg, getKillRegState(DstIsDead)).addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    Op->getOperand(8).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::ADDREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::ADD);
}

template<>
bool PDPExpandPseudo::expand<PDP::SUBREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::SUB);
}

  template<>
  bool PDPExpandPseudo::expand<PDP::BICREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::BIC);
}

  template<>
  bool PDPExpandPseudo::expand<PDP::BICBREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::BICB);
}

template<>
bool PDPExpandPseudo::expand<PDP::BISREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::BIS);
}

template<>
bool PDPExpandPseudo::expand<PDP::BISBREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithResult(MBB, MBBI, PDP::BISB);
}

template<>
bool PDPExpandPseudo::expand<PDP::BITREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithoutResult(MBB, MBBI, PDP::BIT);
}

template<>
bool PDPExpandPseudo::expand<PDP::BITBREG>(Block &MBB, BlockIt MBBI) {
  return expandTwoOpWithoutResult(MBB, MBBI, PDP::BITB);
}

template<>
bool PDPExpandPseudo::expand<PDP::ADCREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ADC);
}

template<>
bool PDPExpandPseudo::expand<PDP::ADCBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ADCB);
}

template<>
bool PDPExpandPseudo::expand<PDP::CLRREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::CLR);
}

  template<>
bool PDPExpandPseudo::expand<PDP::CLRBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::CLRB);
}

template<>
bool PDPExpandPseudo::expand<PDP::INCREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::INC);
}

template<>
bool PDPExpandPseudo::expand<PDP::INCBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::INCB);
}

template<>
bool PDPExpandPseudo::expand<PDP::DECREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::DEC);
}

template<>
bool PDPExpandPseudo::expand<PDP::DECBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::DECB);
}

template<>
bool PDPExpandPseudo::expand<PDP::ASLREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ASL);
}

template<>
bool PDPExpandPseudo::expand<PDP::ASLBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ASLB);
}

template<>
bool PDPExpandPseudo::expand<PDP::ASRREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ASR);
}

template<>
bool PDPExpandPseudo::expand<PDP::ASRBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::ASRB);
}

template<>
bool PDPExpandPseudo::expand<PDP::COMREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::COM);
}

template<>
bool PDPExpandPseudo::expand<PDP::COMBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::COMB);
}

template<>
bool PDPExpandPseudo::expand<PDP::NEGREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::NEG);
}

template<>
bool PDPExpandPseudo::expand<PDP::NEGBREG>(Block &MBB, BlockIt MBBI) {
  return expandOneOp(MBB, MBBI, PDP::NEGB);
}

template<>
bool PDPExpandPseudo::expand<PDP::SWABREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool ImpIsDead = MI.getOperand(2).isDead();
  bool DstIsDead = MI.getOperand(0).isDead();

  auto mov = buildMI(MBB, MBBI, PDP::SWAB)
    .addReg(MI.getOperand(1).getReg(), getKillRegState(DstIsDead))  // НЕ ошибка
    .addImm(0).addImm(0).addImm(0);

  for (const auto mo : MI.memoperands())
    mov->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    mov->getOperand(4).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::LU_>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool ImpIsDead = MI.getOperand(6).isDead();
  bool DstIsDead = MI.getOperand(0).isDead();

  const auto reg = MI.getOperand(1).getReg(); //TRI->getSubReg(MI.getOperand(1).getReg(), PDP::sub_lo);

  auto clrb = buildMI(MBB, MBBI, PDP::CLRB)
    .addReg(reg).addImm(0).addImm(0).addImm(0);

  clrb->getOperand(4).setIsDead();

  auto bisb = buildMI(MBB, MBBI, PDP::BISB);

  bisb->addOperand(MI.getOperand(2));
  bisb->addOperand(MI.getOperand(3));
  bisb->addOperand(MI.getOperand(4));
  bisb->addOperand(MI.getOperand(5));

  bisb.addReg(reg, getKillRegState(DstIsDead))  // НЕ ошибка
    .addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    bisb->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    bisb->getOperand(8).setIsDead();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::LSRREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  const bool HasArgs = MI.getNumOperands() - MI.getNumImplicitOperands() > 1;

  const bool ImpIsDead = MI.getOperand(HasArgs ? 2 : 1).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();
  auto DstReg = MI.getOperand(HasArgs ? 1 : 0).getReg();

  if (PDP::GPR16lRegClass.contains(DstReg))
    DstReg = TRI->getSuperReg(DstReg);

  buildMI(MBB, MBBI, PDP::CLC);

  auto &Op = buildMI(MBB, MBBI, PDP::ROR)
      .addReg(DstReg, getKillRegState(DstIsDead)) // <-- НЕ ошибка
      .addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    Op->getOperand(4).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::LSRBREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  const bool HasArgs = MI.getNumOperands() - MI.getNumImplicitOperands() > 1;

  const bool ImpIsDead = MI.getOperand(HasArgs ? 2 : 1).isDead();
  const bool DstIsDead = MI.getOperand(0).isDead();
  auto DstReg = MI.getOperand(HasArgs ? 1 : 0).getReg();

  if (PDP::GPR16lRegClass.contains(DstReg))
    DstReg = TRI->getSuperReg(DstReg);

  buildMI(MBB, MBBI, PDP::CLC);

  auto &Op = buildMI(MBB, MBBI, PDP::ROR)
      .addReg(DstReg, getKillRegState(DstIsDead)) // <-- НЕ ошибка
      .addImm(0).addImm(0).addImm(0);

  for (auto *const mo : MI.memoperands())
    Op->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    Op->getOperand(4).setIsDead();

  Op.setMIFlags(MI.getFlags());

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::RET>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  buildMI(MBB, MBBI, PDP::RTS).addReg(PDP::R7);

  MI.eraseFromParent();

  return true;
}

bool PDPExpandPseudo::expandCondBr(Block &MBB, BlockIt MBBI, unsigned OpCode,
  unsigned OpCodeBit, unsigned OpCodeTst) {
  MachineInstr &MI = *MBBI;

  auto IsBitOp = MI.getOperand(10).getImm() == 1;

  const auto lr = TRI->getSuperReg(MI.getOperand(0).getReg());
  const auto rr = TRI->getSuperReg(MI.getOperand(4).getReg());

  PDPCC::CondCodes CC = (PDPCC::CondCodes)MI.getOperand(8).getImm();

  bool UseTst = false;
  int Imm = 0;

  if (IsImmediate(MI, 4, Imm) && Imm == -1 && CC == PDPCC::COND_GT) {
    UseTst = true;
    CC = PDPCC::COND_PL;
  }

  if (IsImmediate(MI, 4, Imm) && Imm == 0 && CC == PDPCC::COND_GE) {
    UseTst = true;
    CC = PDPCC::COND_PL;
  }

  if (IsImmediate(MI, 4, Imm) && Imm == 0 && CC == PDPCC::COND_EQ) {
    UseTst = true;
  }

  if (IsImmediate(MI, 4, Imm) && Imm == 0 && CC == PDPCC::COND_NE) {
    UseTst = true;
  }

  if (IsImmediate(MI, 4, Imm) && Imm == 0 && CC == PDPCC::COND_LT) {
    UseTst = true;
    CC = PDPCC::COND_MI;
  }

  if (UseTst) {
    auto tst = buildMI(MBB, MBBI, OpCodeTst);
    tst.addReg(lr);
    tst->addOperand(MI.getOperand(1));
    tst->addOperand(MI.getOperand(2));
    tst->addOperand(MI.getOperand(3));
  } else {
    auto cmp = buildMI(MBB, MBBI, IsBitOp ? OpCodeBit : OpCode);
    cmp.addReg(lr);
    cmp->addOperand(MI.getOperand(1));
    cmp->addOperand(MI.getOperand(2));
    cmp->addOperand(MI.getOperand(3));
    cmp.addReg(rr);
    cmp->addOperand(MI.getOperand(5));
    cmp->addOperand(MI.getOperand(6));
    cmp->addOperand(MI.getOperand(7));
  }

  auto br = BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->getBrCond(CC))
    .addMBB(MI.getOperand(9).getMBB());

  br->getOperand(1).setIsKill(true);

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::CBR>(Block &MBB, BlockIt MBBI) {
  return expandCondBr(MBB, MBBI, PDP::CMP, PDP::BIT, PDP::TST);
}

template<>
bool PDPExpandPseudo::expand<PDP::CBRB>(Block &MBB, BlockIt MBBI) {
  return expandCondBr(MBB, MBBI, PDP::CMPB, PDP::BITB, PDP::TSTB);
}

bool PDPExpandPseudo::expandEMT(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  auto def = MI.getOperand(0).isReg() && getDefRegState(MI.getOperand(0).getReg());

  MI.setDesc(TII->get(PDP::EMT));

  if (def)
    MI.removeOperand(0);

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::SB_PD>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  auto ImpIsDead = MI.getOperand(7).isDead();

  MI.dump();

  auto sb = buildMI(MBB, MBBI, PDP::MOVB);
  sb->addOperand(MI.getOperand(1));
  sb->addOperand(MI.getOperand(2));
  sb->addOperand(MI.getOperand(3));
  sb->addOperand(MI.getOperand(4));

  sb->addOperand(MI.getOperand(5));
  sb->addOperand(MI.getOperand(6));
  sb.addImm(0);
  sb.addImm(0);

  for (auto *const mo : MI.memoperands())
    sb->addMemOperand(*MBB.getParent(), mo);

  if (ImpIsDead)
    sb->getOperand(8).setIsDead();

  sb->dump();

  MI.eraseFromParent();

  return true;
}

template<>
bool PDPExpandPseudo::expand<PDP::MFPSREG>(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;

  bool IsDead = MI.getOperand(0).isDead();
  auto Reg = MI.getOperand(0).getReg();

  BuildMI(MBB, MBBI, MBBI->getDebugLoc(), TII->get(PDP::MFPS))
      .addReg(Reg, getKillRegState(IsDead))
      .addImm(0)
      .addImm(0)
      .addImm(0);

  MI.eraseFromParent();

  return true;
}

bool PDPExpandPseudo::expandMI(Block &MBB, BlockIt MBBI) {
  MachineInstr &MI = *MBBI;
  int Opcode = MBBI->getOpcode();

#define EXPAND(Op)                                                             \
  case Op:                                                                     \
    return expand<Op>(MBB, MI)

  switch (Opcode) {
    EXPAND(PDP::L__);
    EXPAND(PDP::S__);
    EXPAND(PDP::LB_);
    EXPAND(PDP::SB_);
    EXPAND(PDP::LU_);
    //EXPAND(PDP::CMPREG);
    //EXPAND(PDP::CMPBREG);
    EXPAND(PDP::TSTREG);
    EXPAND(PDP::TSTBREG);
    EXPAND(PDP::ADDREG);
    EXPAND(PDP::ADCREG);
    EXPAND(PDP::ADCBREG);
    EXPAND(PDP::SUBREG);
    EXPAND(PDP::BICREG);
    EXPAND(PDP::BICBREG);
    EXPAND(PDP::BISREG);
    EXPAND(PDP::BISBREG);
    EXPAND(PDP::BITREG);
    EXPAND(PDP::BITBREG);
    EXPAND(PDP::SWABREG);
    EXPAND(PDP::CLRREG);
    EXPAND(PDP::CLRBREG);
    EXPAND(PDP::INCREG);
    EXPAND(PDP::INCBREG);
    EXPAND(PDP::DECREG);
    EXPAND(PDP::DECBREG);
    EXPAND(PDP::ASLREG);
    EXPAND(PDP::ASLBREG);
    EXPAND(PDP::ASRREG);
    EXPAND(PDP::ASRBREG);
//    EXPAND(PDP::LDST_);
    EXPAND(PDP::PUSH);
    EXPAND(PDP::POP);
    EXPAND(PDP::SXTREG);
    EXPAND(PDP::XORREG);
    EXPAND(PDP::LSRREG);
    EXPAND(PDP::LSRBREG);
    EXPAND(PDP::COMREG);
    EXPAND(PDP::COMBREG);
    EXPAND(PDP::NEGREG);
    EXPAND(PDP::NEGBREG);
    EXPAND(PDP::RET);

    EXPAND(PDP::CBR);
    EXPAND(PDP::CBRB);

    EXPAND(PDP::SB_PD);

    EXPAND(PDP::MFPSREG);
  }
#undef EXPAND
  return false;
}

} // end of anonymous namespace

INITIALIZE_PASS(PDPExpandPseudo, "PDP-expand-pseudo", PDP_EXPAND_PSEUDO_NAME,
                false, false)
namespace llvm {

FunctionPass *createPDPExpandPseudoPass() { return new PDPExpandPseudo(); }

} // end of namespace llvm
