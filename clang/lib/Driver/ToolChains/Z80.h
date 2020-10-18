//===--- Z80.h - Z80 Tool and ToolChain Implementations ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_Z80_H
#define LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_Z80_H

#include "Gnu.h"
#include "InputInfo.h"
#include "clang/Driver/ToolChain.h"
#include "clang/Driver/Tool.h"

namespace clang {
namespace driver {
namespace toolchains {

class LLVM_LIBRARY_VISIBILITY Z80ToolChain : public ToolChain {
public:
  Z80ToolChain(const Driver &D, const llvm::Triple &Triple,
               const llvm::opt::ArgList &Args);

protected:
public:
  bool isPICDefault() const override { return false; }
  bool isPIEDefault() const override { return false; }
  bool isPICDefaultForced() const override { return false; }
  bool supportsDebugInfoOption(const llvm::opt::Arg *arg) const override;
  bool isThreadModelSupported(const StringRef Model) const override;

protected:
  Tool *buildAssembler() const override;
  Tool *buildLinker() const override;

private:
};

} // end namespace toolchains

} // end namespace driver
} // end namespace clang

#endif // LLVM_CLANG_LIB_DRIVER_TOOLCHAINS_Z80_H
