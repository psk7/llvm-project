//===----- Z80ELFStreamer.h - Z80 Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_ELF_STREAMER_H
#define LLVM_Z80_ELF_STREAMER_H

#include "Z80TargetStreamer.h"

namespace llvm {

/// A target streamer for an Z80 ELF object file.
class Z80ELFStreamer : public Z80TargetStreamer {
public:
  Z80ELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

  MCELFStreamer &getStreamer() {
    return static_cast<MCELFStreamer &>(Streamer);
  }
};

} // end namespace llvm

#endif
