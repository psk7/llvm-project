//===-- Z80MCCodeEmitter.cpp - Convert Z80 Code to Machine Code -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Z80MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "Z80MCCodeEmitter.h"

#include "Z80InstrInfo.h"
#include "MCTargetDesc/Z80MCExpr.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/EndianStream.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "Z80GenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

namespace Z80II {

template <class T>
inline bool isImplicitDefOrKill(const T &Op);

template <>
inline bool isImplicitDefOrKill(const MCOperand &Op){
  return false;
}

template <>
inline bool isImplicitDefOrKill(const MachineOperand &Op){
  return Op.isReg() && Op.isImplicit() && (Op.isDef() || Op.isKill());
}

template <class T, class I>
InstPrefixInfo::InstPrefixInfo(const T &B, const T &E, const MCInstrDesc &MD,
                               const I &Inst) {
  HasXExtension = false;
  HasYExtension = false;
  HasHL = false;
  Displacement = 0;
  HasDisplacement = false;
  InstrSize = 0;
  PureSize = 0;

  if(MD.isPseudo())
    return;

  PureSize = ((MD.TSFlags >> 4) & 0x3) + 1;

  for (auto i = B; i != E; ++i) {
    auto &Op = *i;
    if (isImplicitDefOrKill(Op) || !Op.isReg())
      continue;
    unsigned reg = Op.getReg();
    HasXExtension |= ((reg == Z80::IX) | (reg == Z80::XL) | (reg == Z80::XH));
    HasYExtension |= ((reg == Z80::IY) | (reg == Z80::YL) | (reg == Z80::YH));
    HasHL |= (reg == Z80::HL);

    auto d = (reg == Z80::IX || reg == Z80::IY) && (MD.TSFlags & (1 << 3));
    HasDisplacement |= d;

    if (d) {
      ++i;
      assert(i != E);
      auto &Opi = *i;
      if (Opi.isImm()) {
        Displacement = Opi.getImm();
      }
    }
  }

  if (!MD.isPseudo())
  if (!MD.isPseudo())
    if (HasHL && (HasXExtension || HasYExtension)) {
      // Inst.dump();
      report_fatal_error(
          "Unable to mix IX, IY and HL registers in same instruction");
    }

  Z80II::Prefix Prefixes = static_cast<Z80II::Prefix>(MD.TSFlags & 7);

  HasCB = Prefixes == CB || Prefixes == DDCB || Prefixes == FDCB;
  HasED = Prefixes == ED;
  HasDD = Prefixes == DD || Prefixes == DDCB || HasXExtension;
  HasFD = Prefixes == FD || Prefixes == FDCB || HasYExtension;

  if (!MD.isPseudo())
    if (HasDD && HasFD)
      report_fatal_error(
          "Unable to mix IX and IY registers in same instruction");

  InstrSize = PureSize;
  if (HasCB)
    InstrSize += 1;
  if (HasED)
    InstrSize += 1;
  if (HasDD)
    InstrSize += 1;
  if (HasFD)
    InstrSize += 1;

  if (HasDisplacement)
    InstrSize += 1;
}

InstPrefixInfo::InstPrefixInfo(const MachineInstr &MI)
    : InstPrefixInfo(MI.operands_begin(), MI.operands_end(), MI.getDesc(), MI) {
}

InstPrefixInfo::InstPrefixInfo(const MCInst &MI, const MCInstrInfo &MII)
    : InstPrefixInfo(MI.begin(), MI.end(), MII.get(MI.getOpcode()), MI) {}

}
/*

/// Performs a post-encoding step on a `LD` or `ST` instruction.
///
/// The encoding of the LD/ST family of instructions is inconsistent w.r.t
/// the pointer register and the addressing mode.
///
/// The permutations of the format are as followed:
/// ld Rd, X    `1001 000d dddd 1100`
/// ld Rd, X+   `1001 000d dddd 1101`
/// ld Rd, -X   `1001 000d dddd 1110`
///
/// ld Rd, Y    `1000 000d dddd 1000`
/// ld Rd, Y+   `1001 000d dddd 1001`
/// ld Rd, -Y   `1001 000d dddd 1010`
///
/// ld Rd, Z    `1000 000d dddd 0000`
/// ld Rd, Z+   `1001 000d dddd 0001`
/// ld Rd, -Z   `1001 000d dddd 0010`
///                 ^
///                 |
/// Note this one inconsistent bit - it is 1 sometimes and 0 at other times.
/// There is no logical pattern. Looking at a truth table, the following
/// formula can be derived to fit the pattern:
//
/// ```
/// inconsistent_bit = is_predec OR is_postinc OR is_reg_x
/// ```
//
/// We manually set this bit in this post encoder method.
unsigned
Z80MCCodeEmitter::loadStorePostEncoder(const MCInst &MI, unsigned EncodedValue,
                                       const MCSubtargetInfo &STI) const {

  assert(MI.getOperand(0).isReg() && MI.getOperand(1).isReg() &&
         "the load/store operands must be registers");

  unsigned Opcode = MI.getOpcode();

  // check whether either of the registers are the X pointer register.
  bool IsRegX = MI.getOperand(0).getReg() == Z80::R27R26 ||
                  MI.getOperand(1).getReg() == Z80::R27R26;

  bool IsPredec = Opcode == Z80::LDRdPtrPd || Opcode == Z80::STPtrPdRr;
  bool IsPostinc = Opcode == Z80::LDRdPtrPi || Opcode == Z80::STPtrPiRr;

  // Check if we need to set the inconsistent bit
  if (IsRegX || IsPredec || IsPostinc) {
    EncodedValue |= (1 << 12);
  }

  return EncodedValue;
}
*/
template <Z80::Fixups Fixup>
unsigned
Z80MCCodeEmitter::encodeRelCondBrTarget(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    Fixups.push_back(MCFixup::create(1, MO.getExpr(),
                     MCFixupKind(Fixup), MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  return MO.getImm();
}
/*
unsigned Z80MCCodeEmitter::encodeLDSTPtrReg(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  // The operand should be a pointer register.
  assert(MO.isReg());

  switch (MO.getReg()) {
  case Z80::R27R26: return 0x03; // X: 0b11
  case Z80::R29R28: return 0x02; // Y: 0b10
  case Z80::R31R30: return 0x00; // Z: 0b00
  default:
    llvm_unreachable("invalid pointer register");
  }
}
*/

/// Encodes a `memri` operand.
/// The operand is 8-bits.
unsigned Z80MCCodeEmitter::encodeMemri(const MCInst &MI, unsigned OpNo,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  auto RegOp = MI.getOperand(OpNo);
  auto OffsetOp = MI.getOperand(OpNo + 1);

  assert(RegOp.isReg() && "Expected register operand");

  if (OffsetOp.isImm()) {
    return OffsetOp.getImm();
  } else if (OffsetOp.isExpr()) {
    return 0;
  } else {
    llvm_unreachable("invalid value for offset");
  }

/*  uint8_t RegBit = 0;

  switch (RegOp.getReg()) {
  default:
    llvm_unreachable("Expected either Y or Z register");
  case Z80::R31R30:
    RegBit = 0;
    break; // Z register
  case Z80::R29R28:
    RegBit = 1;
    break; // Y register
  }

  int8_t OffsetBits;

  if (OffsetOp.isImm()) {
    OffsetBits = OffsetOp.getImm();
  } else if (OffsetOp.isExpr()) {
    OffsetBits = 0;
    Fixups.push_back(MCFixup::create(0, OffsetOp.getExpr(),
                     MCFixupKind(Z80::fixup_6), MI.getLoc()));
  } else {
    llvm_unreachable("invalid value for offset");
  }

  return (RegBit << 6) | OffsetBits;*/
}

/*
unsigned Z80MCCodeEmitter::encodeComplement(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  // The operand should be an immediate.
  assert(MI.getOperand(OpNo).isImm());

  auto Imm = MI.getOperand(OpNo).getImm();
  return (~0) - Imm;
}
*/
template <Z80::Fixups Fixup, unsigned Offset>
unsigned Z80MCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo,
                                     SmallVectorImpl<MCFixup> &Fixups,
                                     const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    if (isa<Z80MCExpr>(MO.getExpr())) {
      // If the expression is already an Z80MCExpr (i.e. a lo8(symbol),
      // we shouldn't perform any more fixups. Without this check, we would
      // instead create a fixup to the symbol named 'lo8(symbol)' which
      // is not correct.
      return getExprOpValue(MO.getExpr(), Fixups, STI);
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Fixup);
    Fixups.push_back(MCFixup::create(Offset, MO.getExpr(), FixupKind, MI.getLoc()));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned Z80MCCodeEmitter::encodeCallTarget(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    MCFixupKind FixupKind = static_cast<MCFixupKind>(Z80::fixup_call);
    Fixups.push_back(MCFixup::create(1, MO.getExpr(), FixupKind, MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  return MO.getImm();
}

unsigned Z80MCCodeEmitter::getExprOpValue(const MCExpr *Expr,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {

  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr *>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  if (Kind == MCExpr::Target) {
    llvm_unreachable("Z80MCCodeEmitter::getExprOpValue Kind == MCExpr::Target");
    /*Z80MCExpr const *Z80Expr = cast<Z80MCExpr>(Expr);
    int64_t Result;
    if (Z80Expr->evaluateAsConstant(Result)) {
      return Result;
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Z80Expr->getFixupKind());
    Fixups.push_back(MCFixup::create(0, Z80Expr, FixupKind));
    return 0;*/
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}

unsigned Z80MCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                             const MCOperand &MO,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  if (MO.isReg()) return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());
  if (MO.isImm()) return static_cast<unsigned>(MO.getImm());

  if (MO.isFPImm())
    return static_cast<unsigned>(APFloat(MO.getFPImm())
                                     .bitcastToAPInt()
                                     .getHiBits(32)
                                     .getLimitedValue());

  // MO must be an Expr.
  assert(MO.isExpr());

  return getExprOpValue(MO.getExpr(), Fixups, STI);
}

/*void Z80MCCodeEmitter::emitInstruction(uint64_t Val, unsigned Size,
                                       const MCSubtargetInfo &STI,
                                       raw_ostream &OS) const {



  for (int64_t i = 0; i < Size; ++i) {
    uint8_t Word = (Val >> (i * 8)) & 0xFF;
    OS << Word;
  }
}*/

void Z80MCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  Z80II::InstPrefixInfo P(MI, MCII);

  // Get byte count of instruction
  unsigned Size = P.getSize();

#if DEBUG
  if (Size == 0)
    MI.dump();
#endif

  assert(Size > 0 && "Instruction size cannot be zero");

  if (P.hasFD())
    OS << (unsigned char)0xFD;

  if (P.hasDD())
    OS << (unsigned char)0xDD;

  if (P.hasCB())
    OS << (unsigned char)0xCB;

  if (P.hasED())
    OS << (unsigned char)0xED;

  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);

  if (P.hasDisplacement() && P.hasCB())
    OS << (unsigned char)P.getDisplacement();

  auto ps = P.getPureSize();

  for (int64_t i = 0; i < ps; ++i) {
    uint8_t Word = (BinaryOpCode >> (i * 8)) & 0xFF;
    OS << Word;
  }

  if (P.hasDisplacement() && !P.hasCB()) {
    OS << (unsigned char)P.getDisplacement();
  }
}

MCCodeEmitter *createZ80MCCodeEmitter(const MCInstrInfo &MCII,
                                      const MCRegisterInfo &MRI,
                                      MCContext &Ctx) {
  return new Z80MCCodeEmitter(MCII, Ctx);
}

#include "Z80GenMCCodeEmitter.inc"

} // end of namespace llvm
