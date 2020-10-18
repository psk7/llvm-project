//===-- Z80TargetInfo.cpp - Z80 Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/Z80TargetInfo.h"
#include "llvm/Support/TargetRegistry.h"

namespace llvm {
Target &getTheZ80Target() {
  static Target TheZ80Target;
  return TheZ80Target;
}
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeZ80TargetInfo() {
  llvm::RegisterTarget<llvm::Triple::z80> X(llvm::getTheZ80Target(), "z80",
                                            "Z80", "Z80");
}


