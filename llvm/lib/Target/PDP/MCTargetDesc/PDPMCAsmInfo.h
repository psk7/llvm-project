//===-- PDPMCAsmInfo.h - PDP asm properties ---------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the PDPMCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_ASM_INFO_H
#define LLVM_PDP_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {

class Triple;

/// Specifies the format of PDP assembly files.
class PDPMCAsmInfo : public MCAsmInfo {
public:
  explicit PDPMCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_PDP_ASM_INFO_H
