//===-- PDPTargetInfo.cpp - PDP Target Implementation ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/PDPTargetInfo.h"
#include "llvm/MC/TargetRegistry.h"
namespace llvm {
Target &getThePDPTarget() {
  static Target ThePDPTarget;
  return ThePDPTarget;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPTargetInfo() {
  llvm::RegisterTarget<llvm::Triple::pdp11> X(llvm::getThePDPTarget(), "pdp11",
                                            "PDP11", "PDP11");
}
