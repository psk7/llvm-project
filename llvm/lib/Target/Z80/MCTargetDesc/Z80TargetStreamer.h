//===-- Z80TargetStreamer.h - Z80 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_TARGET_STREAMER_H
#define LLVM_Z80_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic Z80 target output stream.
class Z80TargetStreamer : public MCTargetStreamer {
public:
  explicit Z80TargetStreamer(MCStreamer &S);

  void finish() override;
  void changeSection(const MCSection *CurSection, MCSection *Section,
                     const MCExpr *SubSection, raw_ostream &OS) override;
};

/// A target streamer for textual Z80 assembly code.
class Z80TargetAsmStreamer : public Z80TargetStreamer {
public:
  explicit Z80TargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_Z80_TARGET_STREAMER_H
