//===-- PDPMCExpr.cpp - PDP specific MC expression classes ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "PDPMCExpr.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char *const Spelling;
  PDPMCExpr::VariantKind VariantKind;
} /*ModifierNames[] = {
//    {"lo8", PDPMCExpr::VK_PDP_LO8},       {"hi8", PDPMCExpr::VK_PDP_HI8},
//    {"hh8", PDPMCExpr::VK_PDP_HH8}, // synonym with hlo8
//    {"hlo8", PDPMCExpr::VK_PDP_HH8},      {"hhi8", PDPMCExpr::VK_PDP_HHI8},

//    {"pm", PDPMCExpr::VK_PDP_PM},         {"pm_lo8", PDPMCExpr::VK_PDP_PM_LO8},
//    {"pm_hi8", PDPMCExpr::VK_PDP_PM_HI8}, {"pm_hh8", PDPMCExpr::VK_PDP_PM_HH8},

//    {"lo8_gs", PDPMCExpr::VK_PDP_LO8_GS}, {"hi8_gs", PDPMCExpr::VK_PDP_HI8_GS},
//    {"gs", PDPMCExpr::VK_PDP_GS},
}*/;

} // end of anonymous namespace

const PDPMCExpr *PDPMCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) PDPMCExpr(Kind, Expr, Negated);
}

void PDPMCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_PDP_None);
  OS << getName() << '(';
  if (isNegated())
    OS << '-' << '(';
  getSubExpr()->print(OS, MAI);
  if (isNegated())
    OS << ')';
  OS << ')';
}

bool PDPMCExpr::evaluateAsConstant(int64_t &Result) const {
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

bool PDPMCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAssembler *Asm,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool isRelocatable = SubExpr->evaluateAsRelocatable(Value, Asm, Fixup);

  if (!isRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Asm || !Asm->hasLayout())
      return false;

    MCContext &Context = Asm->getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None)
      return false;
    /*if (Kind == VK_PDP_PM) {
      Modifier = MCSymbolRefExpr::VK_PDP_PM;
    }*/

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t PDPMCExpr::evaluateAsInt64(int64_t Value) const {
  llvm_unreachable("PDPMCExpr::evaluateAsInt64");

  /*if (Negated)
    Value *= -1;

  switch (Kind) {
  case PDPMCExpr::VK_PDP_LO8:
    Value &= 0xff;
    break;
  case PDPMCExpr::VK_PDP_HI8:
    Value &= 0xff00;
    Value >>= 8;
    break;
  case PDPMCExpr::VK_PDP_HH8:
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case PDPMCExpr::VK_PDP_HHI8:
    Value &= 0xff000000;
    Value >>= 24;
    break;
  case PDPMCExpr::VK_PDP_PM_LO8:
  case PDPMCExpr::VK_PDP_LO8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff;
    break;
  case PDPMCExpr::VK_PDP_PM_HI8:
  case PDPMCExpr::VK_PDP_HI8_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff00;
    Value >>= 8;
    break;
  case PDPMCExpr::VK_PDP_PM_HH8:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    Value &= 0xff0000;
    Value >>= 16;
    break;
  case PDPMCExpr::VK_PDP_PM:
  case PDPMCExpr::VK_PDP_GS:
    Value >>= 1; // Program memory addresses must always be shifted by one.
    break;

  case PDPMCExpr::VK_PDP_None:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value) & 0xff;*/
}

PDP::Fixups PDPMCExpr::getFixupKind() const {
  PDP::Fixups Kind = PDP::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  /*case VK_PDP_LO8:
    Kind = isNegated() ? PDP::fixup_lo8_ldi_neg : PDP::fixup_lo8_ldi;
    break;
  case VK_PDP_HI8:
    Kind = isNegated() ? PDP::fixup_hi8_ldi_neg : PDP::fixup_hi8_ldi;
    break;
  case VK_PDP_HH8:
    Kind = isNegated() ? PDP::fixup_hh8_ldi_neg : PDP::fixup_hh8_ldi;
    break;
  case VK_PDP_HHI8:
    Kind = isNegated() ? PDP::fixup_ms8_ldi_neg : PDP::fixup_ms8_ldi;
    break;

  case VK_PDP_PM_LO8:
    Kind = isNegated() ? PDP::fixup_lo8_ldi_pm_neg : PDP::fixup_lo8_ldi_pm;
    break;
  case VK_PDP_PM_HI8:
    Kind = isNegated() ? PDP::fixup_hi8_ldi_pm_neg : PDP::fixup_hi8_ldi_pm;
    break;
  case VK_PDP_PM_HH8:
    Kind = isNegated() ? PDP::fixup_hh8_ldi_pm_neg : PDP::fixup_hh8_ldi_pm;
    break;
  case VK_PDP_PM:
  case VK_PDP_GS:
    Kind = PDP::fixup_16_pm;
    break;
  case VK_PDP_LO8_GS:
    Kind = PDP::fixup_lo8_ldi_gs;
    break;
  case VK_PDP_HI8_GS:
    Kind = PDP::fixup_hi8_ldi_gs;
    break;*/

  case VK_PDP_None:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void PDPMCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *PDPMCExpr::getName() const {
  return nullptr;

  /*const auto &Modifier =
      llvm::find_if(ModifierNames, [this](ModifierEntry const &Mod) {
        return Mod.VariantKind == Kind;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;*/
}

PDPMCExpr::VariantKind PDPMCExpr::getKindByName(StringRef Name) {
  /*const auto &Modifier =
      llvm::find_if(ModifierNames, [&Name](ModifierEntry const &Mod) {
        return Mod.Spelling == Name;
      });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }*/
  return VK_PDP_None;
}

} // end of namespace llvm
