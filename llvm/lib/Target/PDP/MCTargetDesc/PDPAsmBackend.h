//===-- PDPAsmBackend.h - PDP Asm Backend  --------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// \file The PDP assembly backend implementation.
//
//===----------------------------------------------------------------------===//
//

#ifndef LLVM_PDP_ASM_BACKEND_H
#define LLVM_PDP_ASM_BACKEND_H

#include "MCTargetDesc/PDPFixupKinds.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/TargetParser/Triple.h"

namespace llvm {

class MCAssembler;
class MCContext;
struct MCFixupKindInfo;

/// Utilities for manipulating generated PDP machine code.
class PDPAsmBackend : public MCAsmBackend {
public:
  PDPAsmBackend(Triple::OSType OSType)
      : MCAsmBackend(llvm::endianness::little), OSType(OSType) {}

  void adjustFixupValue(const MCFixup &Fixup, const MCValue &Target,
                        uint64_t &Value, MCContext *Ctx = nullptr) const;

  std::unique_ptr<MCObjectTargetWriter>
  createObjectTargetWriter() const override;

  void applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                  const MCValue &Target, MutableArrayRef<char> Data,
                  uint64_t Value, bool IsResolved,
                  const MCSubtargetInfo *STI) const override;

  std::optional<MCFixupKind> getFixupKind(StringRef Name) const override;
  const MCFixupKindInfo &getFixupKindInfo(MCFixupKind Kind) const override;

  unsigned getNumFixupKinds() const override {
    return PDP::NumTargetFixupKinds;
  }

  bool writeNopData(raw_ostream &OS, uint64_t Count,
                    const MCSubtargetInfo *STI) const override;

  bool shouldForceRelocation(const MCAssembler &Asm, const MCFixup &Fixup,
                             const MCValue &Target, const uint64_t Value,
                             const MCSubtargetInfo *STI) override;

private:
  Triple::OSType OSType;
};

} // end namespace llvm

#endif // LLVM_PDP_ASM_BACKEND_H
