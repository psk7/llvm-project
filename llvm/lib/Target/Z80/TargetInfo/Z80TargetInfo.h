//===-- Z80TargetInfo.h - Z80 Target Implementation -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_Z80_TARGET_INFO_H
#define LLVM_Z80_TARGET_INFO_H

namespace llvm {
class Target;

Target &getTheZ80Target();
} // namespace llvm

#endif // LLVM_Z80_TARGET_INFO_H
