#include "PDPELFStreamer.h"
#include "PDPMCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/TargetParser/SubtargetFeature.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;

/*  // Set architecture
  if (Features[PDP::ELFArchPDP1])
    EFlags |= ELF::EF_PDP_ARCH_PDP1;
  else if (Features[PDP::ELFArchPDP2])
    EFlags |= ELF::EF_PDP_ARCH_PDP2;
  else if (Features[PDP::ELFArchPDP25])
    EFlags |= ELF::EF_PDP_ARCH_PDP25;
  else if (Features[PDP::ELFArchPDP3])
    EFlags |= ELF::EF_PDP_ARCH_PDP3;
  else if (Features[PDP::ELFArchPDP31])
    EFlags |= ELF::EF_PDP_ARCH_PDP31;
  else if (Features[PDP::ELFArchPDP35])
    EFlags |= ELF::EF_PDP_ARCH_PDP35;
  else if (Features[PDP::ELFArchPDP4])
    EFlags |= ELF::EF_PDP_ARCH_PDP4;
  else if (Features[PDP::ELFArchPDP5])
    EFlags |= ELF::EF_PDP_ARCH_PDP5;
  else if (Features[PDP::ELFArchPDP51])
    EFlags |= ELF::EF_PDP_ARCH_PDP51;
  else if (Features[PDP::ELFArchPDP6])
    EFlags |= ELF::EF_PDP_ARCH_PDP6;
  else if (Features[PDP::ELFArchTiny])
    EFlags |= ELF::EF_PDP_ARCH_PDPTINY;
  else if (Features[PDP::ELFArchXMEGA1])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA1;
  else if (Features[PDP::ELFArchXMEGA2])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA2;
  else if (Features[PDP::ELFArchXMEGA3])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA3;
  else if (Features[PDP::ELFArchXMEGA4])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA4;
  else if (Features[PDP::ELFArchXMEGA5])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA5;
  else if (Features[PDP::ELFArchXMEGA6])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA6;
  else if (Features[PDP::ELFArchXMEGA7])
    EFlags |= ELF::EF_PDP_ARCH_XMEGA7;*/

  return EFlags;
}

PDPELFStreamer::PDPELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI)
    : PDPTargetStreamer(S) {
  ELFObjectWriter &W = getStreamer().getWriter();
  unsigned EFlags = W.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());
  //EFlags |= ELF::EF_PDP_LINKRELAX_PREPARED;

  W.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
