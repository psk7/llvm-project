//===-- PDPSubtarget.cpp - PDP Subtarget Information ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PDP specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "PDPSubtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/TargetRegistry.h"

#include "PDP.h"
#include "PDPTargetMachine.h"

#define DEBUG_TYPE "PDP-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "PDPGenSubtargetInfo.inc"

namespace llvm {

PDPSubtarget::PDPSubtarget(const Triple &TT, const std::string &CPU,
                           const std::string &FS, const PDPTargetMachine &TM)
    : PDPGenSubtargetInfo(TT, CPU, /*TuneCPU*/ CPU, FS), InstrInfo(*this),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
}

PDPSubtarget &
PDPSubtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                              const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /*TuneCPU*/ CPU, FS);
  return *this;
}

} // end of namespace llvm
