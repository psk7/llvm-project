//===-- PDPAsmBackend.cpp - PDP Asm Backend  ------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the PDPAsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PDPAsmBackend.h"
#include "MCTargetDesc/PDPFixupKinds.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

namespace adjust {

using namespace llvm;

static void unsigned_width(unsigned Width, uint64_t Value,
                           std::string Description, const MCFixup &Fixup,
                           MCContext *Ctx) {
  if (!isUIntN(Width, Value)) {
    std::string Diagnostic = "out of range " + Description;

    int64_t Max = maxUIntN(Width);

    Diagnostic +=
        " (expected an integer in the range 0 to " + std::to_string(Max) + ")";

    Ctx->reportError(Fixup.getLoc(), Diagnostic);
  }
}

} // namespace adjust

namespace llvm {

// Prepare value for the target space for it
void PDPAsmBackend::adjustFixupValue(const MCFixup &Fixup,
                                     const MCValue &Target, uint64_t &Value,
                                     MCContext *Ctx) const {
  // The size of the fixup in bits.
  uint64_t Size = PDPAsmBackend::getFixupKindInfo(Fixup.getKind()).TargetSize;

  unsigned Kind = Fixup.getKind();

  switch (Kind) {
    default:
      llvm_unreachable("unhandled fixup");

    case PDP::fixup_16:
      adjust::unsigned_width(16, Value, std::string("fixup 16"), Fixup, Ctx);

      Value &= 0xffff;
      break;

  case PDP::fixup_8_pcrel: {
    int64_t so = Value - 2;

    assert(so <= 254);
    assert(so >= -256);

    Value = (so / 2) & 0xff;
  }
    break;

  case PDP::fixup_sob_pcrel: {
    int64_t so = Value - 2;

    assert(so <= 0);
    assert(so >= -126);

    Value = (-so / 2) & 0xff;
  }
    break;

    // Fixups which do not require adjustments.
    case FK_Data_1:
    case FK_Data_2:
    case FK_Data_4:
    case FK_Data_8:
      break;

    case FK_GPRel_4:
      llvm_unreachable("don't know how to adjust this fixup");
      break;
  }
}

std::unique_ptr<MCObjectTargetWriter>
PDPAsmBackend::createObjectTargetWriter() const {
  return createPDPELFObjectWriter(MCELFObjectTargetWriter::getOSABI(OSType));
}

void PDPAsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  if (Fixup.getKind() >= FirstLiteralRelocationKind)
    return;
  adjustFixupValue(Fixup, Target, Value, &Asm.getContext());
  if (Value == 0)
    return; // Doesn't change encoding.

  MCFixupKindInfo Info = getFixupKindInfo(Fixup.getKind());

  // The number of bits in the fixup mask
  auto NumBits = Info.TargetSize + Info.TargetOffset;
  auto NumBytes = (NumBits / 8) + ((NumBits % 8) == 0 ? 0 : 1);

  // Shift the value into position.
  Value <<= Info.TargetOffset;

  unsigned Offset = Fixup.getOffset();
  assert(Offset + NumBytes <= Data.size() && "Invalid fixup offset!");

  // For each byte of the fragment that the fixup touches, mask in the
  // bits from the fixup value.
  for (unsigned i = 0; i < NumBytes; ++i) {
    uint8_t mask = (((Value >> (i * 8)) & 0xff));
    Data[Offset + i] |= mask;
  }
}

std::optional<MCFixupKind> PDPAsmBackend::getFixupKind(StringRef Name) const {

  llvm_unreachable("PDPAsmBackend::getFixupKind");

  /*unsigned Type;
  Type = llvm::StringSwitch<unsigned>(Name)
#define ELF_RELOC(X, Y) .Case(#X, Y)
#include "llvm/BinaryFormat/ELFRelocs/PDP.def"
#undef ELF_RELOC
             .Case("BFD_RELOC_NONE", ELF::R_PDP_NONE)
             .Case("BFD_RELOC_16", ELF::R_PDP_16)
             .Case("BFD_RELOC_32", ELF::R_PDP_32)
             .Default(-1u);
  if (Type != -1u)
    return static_cast<MCFixupKind>(FirstLiteralRelocationKind + Type);
  return std::nullopt;*/
}

MCFixupKindInfo const &PDPAsmBackend::getFixupKindInfo(MCFixupKind Kind) const {

  // NOTE: Many PDP fixups work on sets of non-contignous bits. We work around
  // this by saying that the fixup is the size of the entire instruction.
  const static MCFixupKindInfo Infos[PDP::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // PDPFixupKinds.h.
      //
      // name                    offset  bits  flags
      {"fixup_16", 0, 16, 0},

      {"fixup_sob_pcrel", 0, 6, MCFixupKindInfo::FKF_IsPCRel},
      {"fixup_6_pcrel", 0, 6, MCFixupKindInfo::FKF_IsPCRel},
      {"fixup_8_pcrel", 0, 8, MCFixupKindInfo::FKF_IsPCRel},
      {"fixup_16_pcrel", 0, 16, MCFixupKindInfo::FKF_IsPCRel},
  };

  // Fixup kinds from .reloc directive are like R_PDP_NONE. They do not require
  // any extra processing.
  if (Kind >= FirstLiteralRelocationKind)
    return MCAsmBackend::getFixupKindInfo(FK_NONE);

  if (Kind < FirstTargetFixupKind)
    return MCAsmBackend::getFixupKindInfo(Kind);

  assert(unsigned(Kind - FirstTargetFixupKind) < getNumFixupKinds() &&
         "Invalid kind!");

  return Infos[Kind - FirstTargetFixupKind];
}

bool PDPAsmBackend::writeNopData(raw_ostream &OS, uint64_t Count,
                                 const MCSubtargetInfo *STI) const {
  // If the count is not 2-byte aligned, we must be writing data into the text
  // section (otherwise we have unaligned instructions, and thus have far
  // bigger problems), so just write zeros instead.
  assert((Count % 2) == 0 && "NOP instructions must be 2 bytes");

  OS.write_zeros(Count);
  return true;
}

bool PDPAsmBackend::shouldForceRelocation(const MCAssembler &Asm,
                                          const MCFixup &Fixup,
                                          const MCValue &Target,
                                          const uint64_t Value,
                                          const MCSubtargetInfo *STI) {
  switch ((unsigned)Fixup.getKind()) {
  default:
    return Fixup.getKind() >= FirstLiteralRelocationKind;

  case PDP::fixup_6_pcrel:
  case PDP::fixup_8_pcrel:
  case PDP::fixup_16_pcrel:
  case PDP::fixup_sob_pcrel:
    return false;
  }
}

MCAsmBackend *createPDPAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new PDPAsmBackend(STI.getTargetTriple().getOS());
}

} // end of namespace llvm
