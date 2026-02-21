//===-- PDPELFObjectWriter.cpp - PDP ELF Writer ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PDPFixupKinds.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes PDP machine code into an ELF32 object file.
class PDPELFObjectWriter : public MCELFObjectTargetWriter {
public:
  PDPELFObjectWriter(uint8_t OSABI);

  virtual ~PDPELFObjectWriter() = default;

  unsigned getRelocType(MCContext &Ctx, const MCValue &Target,
                        const MCFixup &Fixup, bool IsPCRel) const override;
};

PDPELFObjectWriter::PDPELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_PDP11, true) {}

unsigned PDPELFObjectWriter::getRelocType(MCContext &Ctx, const MCValue &Target,
                                          const MCFixup &Fixup,
                                          bool IsPCRel) const {
  const unsigned Kind = Fixup.getTargetKind();
  if (Kind >= FirstLiteralRelocationKind)
    return Kind - FirstLiteralRelocationKind;

  MCSymbolRefExpr::VariantKind Modifier = Target.getAccessVariant();

  switch ((unsigned)Fixup.getKind()) {
  case FK_Data_1:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    /*case MCSymbolRefExpr::VK_None:
      return ELF::R_PDP_8;*/
    /*
    case MCSymbolRefExpr::VK_PDP_DIFF8:
      return ELF::R_PDP_DIFF8;
    case MCSymbolRefExpr::VK_PDP_LO8:
      return ELF::R_PDP_8_LO8;
    case MCSymbolRefExpr::VK_PDP_HI8:
      return ELF::R_PDP_8_HI8;
    case MCSymbolRefExpr::VK_PDP_HLO8:
      return ELF::R_PDP_8_HLO8;
    */
    }
  case FK_Data_4:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");

    case MCSymbolRefExpr::VK_None:
        return ELF::R_PDP11_16_0; // !!
    /*case MCSymbolRefExpr::VK_None:
      return ELF::R_PDP_32;*/
    /*case MCSymbolRefExpr::VK_PDP_DIFF32:
      return ELF::R_PDP_DIFF32;*/
    }
  case FK_Data_2:
    switch (Modifier) {
    default:
      llvm_unreachable("Unsupported Modifier");
    case MCSymbolRefExpr::VK_None:
      return ELF::R_PDP11_16_0;
    /*case MCSymbolRefExpr::VK_PDP_NONE:
    case MCSymbolRefExpr::VK_PDP_PM:
      return ELF::R_PDP_16_PM;
    case MCSymbolRefExpr::VK_PDP_DIFF16:
      return ELF::R_PDP_DIFF16;*/
    }
  /*case PDP::fixup_32:
    return ELF::R_PDP_32;*/
  case PDP::fixup_6_pcrel:
    return ELF::R_PDP11_6_PCREL;
  case PDP::fixup_8_pcrel:
    return ELF::R_PDP11_8_PCREL;
  case PDP::fixup_sob_pcrel:
    return ELF::R_PDP11_8_PCREL;
  case PDP::fixup_16_pcrel:
    return ELF::R_PDP11_16_PCREL;
  case PDP::fixup_16:
    return ELF::R_PDP11_16_0;
  
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createPDPELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<PDPELFObjectWriter>(OSABI);
}

} // end of namespace llvm
