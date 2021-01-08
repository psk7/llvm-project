//===--------- Z80MCELFStreamer.cpp - Z80 subclass of MCELFStreamer -------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#define DEBUG_TYPE "Z80mcelfstreamer"

#include "MCTargetDesc/Z80MCELFStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCObjectWriter.h"

using namespace llvm;

void Z80MCELFStreamer::emitValueForModiferKind(
    const MCSymbol *Sym, unsigned SizeInBytes, SMLoc Loc,
    Z80MCExpr::VariantKind ModifierKind) {
  MCSymbolRefExpr::VariantKind Kind = MCSymbolRefExpr::VK_Z80_NONE;
  if (ModifierKind == Z80MCExpr::VK_Z80_None) {
    Kind = MCSymbolRefExpr::VK_Z80_DIFF8;
    if (SizeInBytes == SIZE_LONG)
      Kind = MCSymbolRefExpr::VK_Z80_DIFF32;
    else if (SizeInBytes == SIZE_WORD)
      Kind = MCSymbolRefExpr::VK_Z80_DIFF16;
  } else if (ModifierKind == Z80MCExpr::VK_Z80_LO8)
    Kind = MCSymbolRefExpr::VK_Z80_LO8;
  else if (ModifierKind == Z80MCExpr::VK_Z80_HI8)
    Kind = MCSymbolRefExpr::VK_Z80_HI8;
  else if (ModifierKind == Z80MCExpr::VK_Z80_HH8)
    Kind = MCSymbolRefExpr::VK_Z80_HLO8;
  MCELFStreamer::emitValue(MCSymbolRefExpr::create(Sym, Kind, getContext()),
                           SizeInBytes, Loc);
}

namespace llvm {
MCStreamer *createZ80ELFStreamer(Triple const &TT, MCContext &Context,
                                 std::unique_ptr<MCAsmBackend> MAB,
                                 std::unique_ptr<MCObjectWriter> OW,
                                 std::unique_ptr<MCCodeEmitter> CE) {
  return new Z80MCELFStreamer(Context, std::move(MAB), std::move(OW),
                              std::move(CE));
}

} // end namespace llvm
