//===-- Z80MCAsmInfo.h - Z80 asm properties ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the Z80MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_ASM_INFO_H
#define LLVM_Z80_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of Z80 assembly files.
class Z80MCAsmInfo : public MCAsmInfo {
public:
  explicit Z80MCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
  bool shouldOmitSectionDirective(StringRef SectionName) const override;
};

} // end namespace llvm

#endif // LLVM_Z80_ASM_INFO_H
