//===-- Z80TargetStreamer.cpp - Z80 Target Streamer Methods ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides Z80 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "Z80TargetStreamer.h"

#include "llvm/MC/MCContext.h"

namespace llvm {

Z80TargetStreamer::Z80TargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

Z80TargetAsmStreamer::Z80TargetAsmStreamer(MCStreamer &S)
    : Z80TargetStreamer(S) {}

void Z80TargetStreamer::finish() {
/*  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  MCSymbol *DoCopyData = Context.getOrCreateSymbol("__do_copy_data");
  MCSymbol *DoClearBss = Context.getOrCreateSymbol("__do_clear_bss");

  // FIXME: We can disable __do_copy_data if there are no static RAM variables.

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("copy all variables from program memory to RAM on startup");
  OS.emitSymbolAttribute(DoCopyData, MCSA_Global);

  OS.emitRawComment(" Declaring this symbol tells the CRT that it should");
  OS.emitRawComment("clear the zeroed data section on startup");
  OS.emitSymbolAttribute(DoClearBss, MCSA_Global);*/
}

void Z80TargetStreamer::changeSection(const MCSection *CurSection,
                                      MCSection *Section,
                                      const MCExpr *SubSection,
                                      raw_ostream &OS) {
  /*MCTargetStreamer::changeSection(CurSection, Section, SubSection, OS);*/
}

} // end namespace llvm

