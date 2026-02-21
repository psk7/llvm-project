//===-- PDPTargetStreamer.h - PDP Target Streamer --------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_TARGET_STREAMER_H
#define LLVM_PDP_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic PDP target output stream.
class PDPTargetStreamer : public MCTargetStreamer {
public:
  explicit PDPTargetStreamer(MCStreamer &S);
};

/// A target streamer for textual PDP assembly code.
class PDPTargetAsmStreamer : public PDPTargetStreamer {
public:
  explicit PDPTargetAsmStreamer(MCStreamer &S);
};

} // end namespace llvm

#endif // LLVM_PDP_TARGET_STREAMER_H
