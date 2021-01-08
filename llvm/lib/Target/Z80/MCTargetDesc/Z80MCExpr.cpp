//===-- Z80MCExpr.cpp - Z80 specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Z80MCExpr.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char * const Spelling;
  Z80MCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"lo8", Z80MCExpr::VK_Z80_LO8},       {"hi8", Z80MCExpr::VK_Z80_HI8},
    {"hh8", Z80MCExpr::VK_Z80_HH8}, // synonym with hlo8
    {"hlo8", Z80MCExpr::VK_Z80_HH8},      {"hhi8", Z80MCExpr::VK_Z80_HHI8},

    {"pm_lo8", Z80MCExpr::VK_Z80_PM_LO8}, {"pm_hi8", Z80MCExpr::VK_Z80_PM_HI8},
    {"pm_hh8", Z80MCExpr::VK_Z80_PM_HH8},

    {"lo8_gs", Z80MCExpr::VK_Z80_LO8_GS}, {"hi8_gs", Z80MCExpr::VK_Z80_HI8_GS},
    {"gs", Z80MCExpr::VK_Z80_GS},
};

} // end of anonymous namespace

const Z80MCExpr *Z80MCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) Z80MCExpr(Kind, Expr, Negated);
}

void Z80MCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_Z80_None);

  if (isNegated())
    OS << '-';

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool Z80MCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool isRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool Z80MCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout) return false;

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t Z80MCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated)
    Value *= -1;

  switch (Kind) {
  case Z80MCExpr::VK_Z80_LO8:
    Value &= 0xff;
    break;
  case Z80MCExpr::VK_Z80_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case Z80MCExpr::VK_Z80_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case Z80MCExpr::VK_Z80_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case Z80MCExpr::VK_Z80_PM_LO8:
  case Z80MCExpr::VK_Z80_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case Z80MCExpr::VK_Z80_PM_HI8:
  case Z80MCExpr::VK_Z80_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
  case Z80MCExpr::VK_Z80_PM_HH8:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case Z80MCExpr::VK_Z80_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case Z80MCExpr::VK_Z80_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;
}

Z80::Fixups Z80MCExpr::getFixupKind() const {
  Z80::Fixups Kind = Z80::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_Z80_LO8:
    Kind = isNegated() ? Z80::fixup_lo8_ldi_neg : Z80::fixup_lo8_ldi;
    break;
  case VK_Z80_HI8:
    Kind = isNegated() ? Z80::fixup_hi8_ldi_neg : Z80::fixup_hi8_ldi;
    break;
  case VK_Z80_HH8:
    Kind = isNegated() ? Z80::fixup_hh8_ldi_neg : Z80::fixup_hh8_ldi;
    break;
  case VK_Z80_HHI8:
    Kind = isNegated() ? Z80::fixup_ms8_ldi_neg : Z80::fixup_ms8_ldi;
    break;

  case VK_Z80_PM_LO8:
    Kind = isNegated() ? Z80::fixup_lo8_ldi_pm_neg : Z80::fixup_lo8_ldi_pm;
    break;
  case VK_Z80_PM_HI8:
    Kind = isNegated() ? Z80::fixup_hi8_ldi_pm_neg : Z80::fixup_hi8_ldi_pm;
    break;
  case VK_Z80_PM_HH8:
    Kind = isNegated() ? Z80::fixup_hh8_ldi_pm_neg : Z80::fixup_hh8_ldi_pm;
    break;
  case VK_Z80_GS:
    Kind = Z80::fixup_16_pm;
    break;
  case VK_Z80_LO8_GS:
    Kind = Z80::fixup_lo8_ldi_gs;
    break;
  case VK_Z80_HI8_GS:
    Kind = Z80::fixup_hi8_ldi_gs;
    break;

  case VK_Z80_None:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void Z80MCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *Z80MCExpr::getName() const {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [this](ModifierEntry const &Mod) { return Mod.VariantKind == Kind; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

Z80MCExpr::VariantKind Z80MCExpr::getKindByName(StringRef Name) {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [&Name](ModifierEntry const &Mod) { return Mod.Spelling == Name; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_Z80_None;
}

} // end of namespace llvm

