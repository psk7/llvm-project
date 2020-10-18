//===--- Z80.cpp - Z80 ToolChain Implementations ----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Z80.h"
#include "CommonArgs.h"
#include "InputInfo.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/Options.h"
#include "llvm/ADT/Optional.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/FileSystem.h"

using namespace clang::driver;
using namespace clang::driver::toolchains;
using namespace clang::driver::tools;
using namespace clang;
using namespace llvm::opt;

namespace {

} // end anonymous namespace

/// Z80 Toolchain
Z80ToolChain::Z80ToolChain(const Driver &D, const llvm::Triple &Triple,
                           const ArgList &Args)
    : ToolChain(D, Triple, Args) {

}

Tool *Z80ToolChain::buildLinker() const { return nullptr; }

Tool *Z80ToolChain::buildAssembler() const {
  return ToolChain::buildAssembler();

}

bool Z80ToolChain::supportsDebugInfoOption(const llvm::opt::Arg *arg) const {
  return false;
}

bool Z80ToolChain::isThreadModelSupported(const StringRef Model) const {
  return false;
}
