//===-- PDPTargetStreamer.cpp - PDP Target Streamer Methods ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides PDP specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "PDPTargetStreamer.h"

#include "llvm/MC/MCContext.h"

namespace llvm {

PDPTargetStreamer::PDPTargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

PDPTargetAsmStreamer::PDPTargetAsmStreamer(MCStreamer &S)
    : PDPTargetStreamer(S) {}

} // end namespace llvm
