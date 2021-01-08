//===-- Z80ELFObjectWriter.cpp - Z80 ELF Writer ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Z80FixupKinds.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes Z80 machine code into an ELF32 object file.
class Z80ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  Z80ELFObjectWriter(uint8_t OSABI);

  virtual ~Z80ELFObjectWriter() {}

  unsigned getRelocType(MCContext &Ctx,
                        const MCValue &Target,
                        const MCFixup &Fixup,
                        bool IsPCRel) const override;
};

Z80ELFObjectWriter::Z80ELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_NONE, true) {}

unsigned Z80ELFObjectWriter::getRelocType(MCContext &Ctx,
                                          const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
  llvm_unreachable("Z80ELFObjectWriter::getRelocType");

  /*MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();
  switch ((unsigned) Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_Z80_8;
    case MCSymbolRefExpr::VK_Z80_DIFF8:
      return ELF::R_Z80_DIFF8;
    case MCSymbolRefExpr::VK_Z80_LO8:
      return ELF::R_Z80_8_LO8;
    case MCSymbolRefExpr::VK_Z80_HI8:
      return ELF::R_Z80_8_HI8;
    case MCSymbolRefExpr::VK_Z80_HLO8:
      return ELF::R_Z80_8_HLO8;
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_Z80_32;
    case MCSymbolRefExpr::VK_Z80_DIFF32:
      return ELF::R_Z80_DIFF32;
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_Z80_16;
    case MCSymbolRefExpr::VK_Z80_NONE:
      return ELF::R_Z80_16_PM;
    case MCSymbolRefExpr::VK_Z80_DIFF16:
      return ELF::R_Z80_DIFF16;
    }
  case Z80::fixup_32:
    return ELF::R_Z80_32;
  case Z80::fixup_8_pcrel:
    return ELF::R_Z80_8_PCREL;
  case Z80::fixup_16:
    return ELF::R_Z80_16;
  case Z80::fixup_16_pm:
    return ELF::R_Z80_16_PM;
  case Z80::fixup_lo8_ldi:
    return ELF::R_Z80_LO8_LDI;
  case Z80::fixup_hi8_ldi:
    return ELF::R_Z80_HI8_LDI;
  case Z80::fixup_hh8_ldi:
    return ELF::R_Z80_HH8_LDI;
  case Z80::fixup_lo8_ldi_neg:
    return ELF::R_Z80_LO8_LDI_NEG;
  case Z80::fixup_hi8_ldi_neg:
    return ELF::R_Z80_HI8_LDI_NEG;
  case Z80::fixup_hh8_ldi_neg:
    return ELF::R_Z80_HH8_LDI_NEG;
  case Z80::fixup_lo8_ldi_pm:
    return ELF::R_Z80_LO8_LDI_PM;
  case Z80::fixup_hi8_ldi_pm:
    return ELF::R_Z80_HI8_LDI_PM;
  case Z80::fixup_hh8_ldi_pm:
    return ELF::R_Z80_HH8_LDI_PM;
  case Z80::fixup_lo8_ldi_pm_neg:
    return ELF::R_Z80_LO8_LDI_PM_NEG;
  case Z80::fixup_hi8_ldi_pm_neg:
    return ELF::R_Z80_HI8_LDI_PM_NEG;
  case Z80::fixup_hh8_ldi_pm_neg:
    return ELF::R_Z80_HH8_LDI_PM_NEG;
  case Z80::fixup_call:
    return ELF::R_Z80_CALL;
  case Z80::fixup_ldi:
    return ELF::R_Z80_LDI;
  case Z80::fixup_6:
    return ELF::R_Z80_6;
  case Z80::fixup_6_adiw:
    return ELF::R_Z80_6_ADIW;
  case Z80::fixup_ms8_ldi:
    return ELF::R_Z80_MS8_LDI;
  case Z80::fixup_ms8_ldi_neg:
    return ELF::R_Z80_MS8_LDI_NEG;
  case Z80::fixup_lo8_ldi_gs:
    return ELF::R_Z80_LO8_LDI_GS;
  case Z80::fixup_hi8_ldi_gs:
    return ELF::R_Z80_HI8_LDI_GS;
  case Z80::fixup_8:
    return ELF::R_Z80_8;
  case Z80::fixup_8_lo8:
    return ELF::R_Z80_8_LO8;
  case Z80::fixup_8_hi8:
    return ELF::R_Z80_8_HI8;
  case Z80::fixup_8_hlo8:
    return ELF::R_Z80_8_HLO8;
  case Z80::fixup_diff8:
    return ELF::R_Z80_DIFF8;
  case Z80::fixup_diff16:
    return ELF::R_Z80_DIFF16;
  case Z80::fixup_diff32:
    return ELF::R_Z80_DIFF32;
  case Z80::fixup_lds_sts_16:
    return ELF::R_Z80_LDS_STS_16;
  case Z80::fixup_port6:
    return ELF::R_Z80_PORT6;
  case Z80::fixup_port5:
    return ELF::R_Z80_PORT5;
  default:
    llvm_unreachable("invalid fixup kind!");
  }*/
}

std::unique_ptr<MCObjectTargetWriter> createZ80ELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<Z80ELFObjectWriter>(OSABI);
}

} // end of namespace llvm

