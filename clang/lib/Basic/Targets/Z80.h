//===--- Z80.h - Declare Z80 target feature support -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares Z80 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_Z80_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_Z80_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Support/Compiler.h"

namespace clang {
namespace targets {

// Z80 Target
class LLVM_LIBRARY_VISIBILITY Z80TargetInfo : public TargetInfo {
public:
  Z80TargetInfo(const llvm::Triple &Triple, const TargetOptions &)
      : TargetInfo(Triple) {
    TLSSupported = false;
    PointerWidth = 16;
    PointerAlign = 8;
    IntWidth = 16;
    IntAlign = 8;
    LongWidth = 16;
    LongAlign = 8;
    LongLongWidth = 32;
    LongLongAlign = 8;
    SuitableAlign = 8;
    DefaultAlignForAttributeAligned = 8;
    HalfWidth = 16;
    HalfAlign = 8;
    FloatWidth = 32;
    FloatAlign = 8;
    DoubleWidth = 32;
    DoubleAlign = 8;
    DoubleFormat = &llvm::APFloat::IEEEsingle();
    LongDoubleWidth = 32;
    LongDoubleAlign = 8;
    LongDoubleFormat = &llvm::APFloat::IEEEsingle();
    SizeType = UnsignedInt;
    PtrDiffType = SignedInt;
    IntPtrType = SignedInt;
    Char16Type = UnsignedInt;
    WIntType = SignedInt;
    Char32Type = UnsignedLong;
    SigAtomicType = SignedChar;
    resetDataLayout("e-P1-p:16:8-i8:8-i16:8-i32:8-n8-a:8");
  }

  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override;

  ArrayRef<Builtin::Info> getTargetBuiltins() const override { return None; }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  const char *getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override {
    static const char *const GCCRegNames[] = { "" };
    return llvm::makeArrayRef(GCCRegNames);
  }

  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override {
    return None;
  }

  ArrayRef<TargetInfo::AddlRegName> getGCCAddlRegNames() const override {
    return TargetInfo::getGCCAddlRegNames();
  }

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override {
    // There aren't any multi-character Z80 specific constraints.
    if (StringRef(Name).size() > 1)
      return false;

    switch (*Name) {
    default:
      return false;
    case 'a': // Simple upper registers
    case 'b': // Base pointer registers pairs
    case 'd': // Upper register
    case 'l': // Lower registers
    case 'e': // Pointer register pairs
    case 'q': // Stack pointer register
    case 'r': // Any register
    case 'w': // Special upper register pairs
    case 't': // Temporary register
    case 'x':
    case 'X': // Pointer register pair X
    case 'y':
    case 'Y': // Pointer register pair Y
    case 'z':
    case 'Z': // Pointer register pair Z
      Info.setAllowsRegister();
      return true;
    case 'I': // 6-bit positive integer constant
      Info.setRequiresImmediate(0, 63);
      return true;
    case 'J': // 6-bit negative integer constant
      Info.setRequiresImmediate(-63, 0);
      return true;
    case 'K': // Integer constant (Range: 2)
      Info.setRequiresImmediate(2);
      return true;
    case 'L': // Integer constant (Range: 0)
      Info.setRequiresImmediate(0);
      return true;
    case 'M': // 8-bit integer constant
      Info.setRequiresImmediate(0, 0xff);
      return true;
    case 'N': // Integer constant (Range: -1)
      Info.setRequiresImmediate(-1);
      return true;
    case 'O': // Integer constant (Range: 8, 16, 24)
      Info.setRequiresImmediate({8, 16, 24});
      return true;
    case 'P': // Integer constant (Range: 1)
      Info.setRequiresImmediate(1);
      return true;
    case 'R': // Integer constant (Range: -6 to 5)
      Info.setRequiresImmediate(-6, 5);
      return true;
    case 'G': // Floating point constant
    case 'Q': // A memory address based on Y or Z pointer with displacement.
      return true;
    }

    return false;
  }

  IntType getIntTypeByWidth(unsigned BitWidth, bool IsSigned) const final {
    // Z80 prefers int for 16-bit integers.
    return BitWidth == 16 ? (IsSigned ? SignedInt : UnsignedInt)
                          : TargetInfo::getIntTypeByWidth(BitWidth, IsSigned);
  }

  IntType getLeastIntTypeByWidth(unsigned BitWidth, bool IsSigned) const final {
    // Z80 uses int for int_least16_t and int_fast16_t.
    return BitWidth == 16
               ? (IsSigned ? SignedInt : UnsignedInt)
               : TargetInfo::getLeastIntTypeByWidth(BitWidth, IsSigned);
  }

  bool isValidCPUName(StringRef Name) const override;
  void fillValidCPUList(SmallVectorImpl<StringRef> &Values) const override;

  bool setCPU(const std::string &Name) override;

protected:
  std::string CPU;
};

} // namespace targets
} // namespace clang

#endif // LLVM_CLANG_LIB_BASIC_TARGETS_Z80_H
