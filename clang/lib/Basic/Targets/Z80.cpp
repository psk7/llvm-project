//===--- Z80.cpp - Implement Z80 target feature support -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements Z80 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "Z80.h"
#include "clang/Basic/MacroBuilder.h"

using namespace clang;
using namespace clang::targets;

bool Z80TargetInfo::isValidCPUName(StringRef Name) const {
  return Name == "Z80" || Name == "z80";
}

void Z80TargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  Values.push_back("z80");
}

void Z80TargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("Z80");
  Builder.defineMacro("__Z80");
  Builder.defineMacro("__Z80__");
}

bool Z80TargetInfo::setCPU(const std::string &Name) {
  bool isValid = isValidCPUName(Name);
  if (isValid)
    CPU = Name;
  return isValid;
}

bool Z80TargetInfo::hasExtIntType() const {
  return true;
}
