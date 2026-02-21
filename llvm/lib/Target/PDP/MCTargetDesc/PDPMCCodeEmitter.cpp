//===-- PDPMCCodeEmitter.cpp - Convert PDP Code to Machine Code -----------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PDPMCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "PDPMCCodeEmitter.h"

#include "MCTargetDesc/PDPMCExpr.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"

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

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "PDPGenInstrInfo.inc"
#include "PDPInstrInfo.h"
#undef GET_INSTRMAP_INFO

namespace llvm {

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
PDPMCCodeEmitter::loadStorePostEncoder(const MCInst &MI, unsigned EncodedValue,
                                       const MCSubtargetInfo &STI) const {

  llvm_unreachable("PDPMCCodeEmitter::loadStorePostEncoder");

  /*assert(MI.getOperand(0).isReg() && MI.getOperand(1).isReg() &&
         "the load/store operands must be registers");

  unsigned Opcode = MI.getOpcode();

  // Get the index of the pointer register operand.
  unsigned Idx = 0;
  if (Opcode == PDP::LDRdPtrPd || Opcode == PDP::LDRdPtrPi /*||
      Opcode == PDP::LDRdPtr#1#)
    Idx = 1;

  // Check if we need to set the inconsistent bit
  bool IsPredec = Opcode == PDP::LDRdPtrPd || Opcode == PDP::STPtrPdRr;
  bool IsPostinc = Opcode == PDP::LDRdPtrPi || Opcode == PDP::STPtrPiRr;
  if (MI.getOperand(Idx).getReg() == PDP::R27R26 || IsPredec || IsPostinc)
    EncodedValue |= (1 << 12);

  // Encode the pointer register.
  switch (MI.getOperand(Idx).getReg()) {
  case PDP::R27R26:
    EncodedValue |= 0xc;
    break;
  case PDP::R29R28:
    EncodedValue |= 0x8;
    break;
  case PDP::R31R30:
    break;
  default:
    llvm_unreachable("invalid pointer register");
    break;
  }

  return EncodedValue;*/
}

template <PDP::Fixups Fixup>
unsigned
PDPMCCodeEmitter::encodeRelCondBrTarget(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
  const MCOperand &MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    Fixups.push_back(
        MCFixup::create(0, MO.getExpr(), MCFixupKind(Fixup), MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  int64_t Res = 0;

  if (!MO.evaluateAsConstantImm(Res)) {
    llvm_unreachable("encodeRelCondBrTarget");
  }

  Res -= 2;
  Res /= 2;

  if (MI.getOpcode() == PDP::SOB) {
    assert(Res <= 0);
    assert(Res >= -63);

    return -Res;
  }

  assert(Res <= 254);
  assert(Res >= -256);

  return Res;

  // Take the size of the current instruction away.
  // With labels, this is implicitly done.
  auto target = MO.getImm();
  PDP::fixups::adjustBranchTarget(target);
  return target;
}

  unsigned PDPMCCodeEmitter::encodeUniop(const MCInst &MI, unsigned OpNo,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  const auto Reg = MI.getOperand(OpNo).getReg();
  const auto Mode = MI.getOperand(OpNo + 1).getImm();

  const auto RegVal = Ctx.getRegisterInfo()->getEncodingValue(Reg);

  return (Mode & 7) << 3 | (RegVal & 7);
}

unsigned PDPMCCodeEmitter::encodeComplement(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  // The operand should be an immediate.
  assert(MI.getOperand(OpNo).isImm());

  auto Imm = MI.getOperand(OpNo).getImm();
  return (~0) - Imm;
}

template <PDP::Fixups Fixup, unsigned Offset>
unsigned PDPMCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo,
                                     SmallVectorImpl<MCFixup> &Fixups,
                                     const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    if (isa<PDPMCExpr>(MO.getExpr())) {
      // If the expression is already an PDPMCExpr (i.e. a lo8(symbol),
      // we shouldn't perform any more fixups. Without this check, we would
      // instead create a fixup to the symbol named 'lo8(symbol)' which
      // is not correct.
      return getExprOpValue(MO.getExpr(), Fixups, STI);
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Fixup);
    Fixups.push_back(
        MCFixup::create(Offset, MO.getExpr(), FixupKind, MI.getLoc()));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned PDPMCCodeEmitter::encodeCallTarget(const MCInst &MI, unsigned OpNo,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {
  llvm_unreachable("PDPMCCodeEmitter::encodeCallTarget");

  /*auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    MCFixupKind FixupKind = static_cast<MCFixupKind>(PDP::fixup_call);
    Fixups.push_back(MCFixup::create(0, MO.getExpr(), FixupKind, MI.getLoc()));
    return 0;
  }

  assert(MO.isImm());

  auto Target = MO.getImm();
  PDP::fixups::adjustBranchTarget(Target);
  return Target;*/
}

unsigned PDPMCCodeEmitter::getExprOpValue(const MCExpr *Expr,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {

  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr *>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  if (Kind == MCExpr::Target) {
    PDPMCExpr const *PDPExpr = cast<PDPMCExpr>(Expr);
    int64_t Result;
    if (PDPExpr->evaluateAsConstant(Result)) {
      return Result;
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(PDPExpr->getFixupKind());
    Fixups.push_back(MCFixup::create(0, PDPExpr, FixupKind));
    return 0;
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}

unsigned PDPMCCodeEmitter::getMachineOpValue(const MCInst &MI,
                                             const MCOperand &MO,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI) const {
  if (MO.isReg())
    return Ctx.getRegisterInfo()->getEncodingValue(MO.getReg());
  if (MO.isImm())
    return static_cast<unsigned>(MO.getImm());

  if (MO.isDFPImm())
    return static_cast<unsigned>(bit_cast<double>(MO.getDFPImm()));

  // MO must be an Expr.
  assert(MO.isExpr());

  return getExprOpValue(MO.getExpr(), Fixups, STI);
}

void PDPMCCodeEmitter::appendArgument(const MCInst &MI,
                                      SmallVectorImpl<char> &CB,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI, const unsigned OpNum, unsigned &Offset) const {
  auto reg = MI.getOperand(OpNum).getReg();
  auto mode = MI.getOperand(OpNum + 1).getImm();
  auto offset_or_immediate = MI.getOperand(OpNum + 2);
  auto disp = MI.getOperand(OpNum + 3).getImm();

  uint16_t word = 0;

  if (mode == 2 && reg == PDP::R7) {
    // Аргумент - число
    if (offset_or_immediate.isImm()) {
      word = offset_or_immediate.getImm() + disp;
    } else if (offset_or_immediate.isExpr()) {
      assert(disp == 0 && "disp == 0");

      constexpr auto FixupKind = static_cast<MCFixupKind>(PDP::fixup_16);
      Fixups.push_back(MCFixup::create(Offset, offset_or_immediate.getExpr(), FixupKind, MI.getLoc()));
      Offset += 2;
    } else
      llvm_unreachable("mode == 2");

    support::endian::write(CB, word, endianness::little);
  }
  else if (mode == 3 && reg == PDP::R7) {
    // Аргумент - адрес
    if (offset_or_immediate.isImm()) {
      word = offset_or_immediate.getImm() + disp;
    } else if (offset_or_immediate.isExpr()) {
      assert(disp == 0 && "disp == 0");

      constexpr auto FixupKind = static_cast<MCFixupKind>(PDP::fixup_16);
      Fixups.push_back(MCFixup::create(Offset, offset_or_immediate.getExpr(), FixupKind, MI.getLoc()));
      Offset += 2;
    } else
      llvm_unreachable("mode == 3");

    support::endian::write(CB, word, endianness::little);
  }
  else if (mode == 6 || mode == 7) {
    // Аргумент - смещение
    assert(offset_or_immediate.isImm() && "offset_or_immediate.isImm()");

    word = offset_or_immediate.getImm() + disp;
    Offset += 2;

    support::endian::write(CB, word, endianness::little);
  }
}

void PDPMCCodeEmitter::encodeInstruction(const MCInst &MI,
                                         SmallVectorImpl<char> &CB,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());

  // Get byte count of instruction
  const unsigned Size = Desc.getSize();

  assert(Size > 0 && "Instruction size cannot be zero");

  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);

  uint16_t Word = BinaryOpCode & 0xFFFF;
  support::endian::write(CB, Word, endianness::little);

  unsigned Offset = 2;

  forEachArg(Desc, [this, &MI, &CB, &Fixups, &STI, &Offset](unsigned N) -> void {
    appendArgument(MI, CB, Fixups, STI, N, Offset);
  });
}

void PDPMCCodeEmitter::forEachArg(
    const MCInstrDesc &D, function_ref<void(unsigned N)> F) {
  switch (D.TSFlags & 7) {
  case 1: // Инструкции с двумя аргументами
    F(0);
    F(4);
    break;

  case 2: // Инструкция с одним аргументом
    F(0);
    break;

  case 3:
    F(1);
    break;

  default:
    break;
  }
}

MCCodeEmitter *createPDPMCCodeEmitter(const MCInstrInfo &MCII, MCContext &Ctx) {
  return new PDPMCCodeEmitter(MCII, Ctx);
}

#include "PDPGenMCCodeEmitter.inc"

} // end of namespace llvm
