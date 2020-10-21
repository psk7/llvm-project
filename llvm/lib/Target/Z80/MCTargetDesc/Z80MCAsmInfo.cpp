//===-- Z80MCAsmInfo.cpp - Z80 asm properties -----------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the Z80MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "Z80MCAsmInfo.h"

#include "llvm/ADT/Triple.h"

namespace llvm {

Z80MCAsmInfo::Z80MCAsmInfo(const Triple &TT, const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 2;
  CommentString = ";";
  PrivateGlobalPrefix = ".L";
  PrivateLabelPrefix = ".L";
  UsesELFSectionDirectiveForBSS = true;
  SupportsDebugInformation = false;
  DollarIsPC = true;
  HasSingleParameterDotFile = false;
  HasDotTypeDotSizeDirective = false;
}

bool Z80MCAsmInfo::shouldOmitSectionDirective(StringRef SectionName) const {
  return true;
}

} // end of namespace llvm
