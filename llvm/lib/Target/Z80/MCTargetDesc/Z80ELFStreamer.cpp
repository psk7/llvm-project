#include "Z80ELFStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/SubtargetFeature.h"
#include "llvm/Support/FormattedStream.h"

#include "Z80MCTargetDesc.h"

namespace llvm {

static unsigned getEFlagsForFeatureSet(const FeatureBitset &Features) {
  unsigned EFlags = 0;

  // Set architecture
/*  if (Features[Z80::ELFArchZ801])
    EFlags |= ELF::EF_Z80_ARCH_Z801;
  else if (Features[Z80::ELFArchZ802])
    EFlags |= ELF::EF_Z80_ARCH_Z802;
  else if (Features[Z80::ELFArchZ8025])
    EFlags |= ELF::EF_Z80_ARCH_Z8025;
  else if (Features[Z80::ELFArchZ803])
    EFlags |= ELF::EF_Z80_ARCH_Z803;
  else if (Features[Z80::ELFArchZ8031])
    EFlags |= ELF::EF_Z80_ARCH_Z8031;
  else if (Features[Z80::ELFArchZ8035])
    EFlags |= ELF::EF_Z80_ARCH_Z8035;
  else if (Features[Z80::ELFArchZ804])
    EFlags |= ELF::EF_Z80_ARCH_Z804;
  else if (Features[Z80::ELFArchZ805])
    EFlags |= ELF::EF_Z80_ARCH_Z805;
  else if (Features[Z80::ELFArchZ8051])
    EFlags |= ELF::EF_Z80_ARCH_Z8051;
  else if (Features[Z80::ELFArchZ806])
    EFlags |= ELF::EF_Z80_ARCH_Z806;
  else if (Features[Z80::ELFArchTiny])
    EFlags |= ELF::EF_Z80_ARCH_Z80TINY;
  else if (Features[Z80::ELFArchXMEGA1])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA1;
  else if (Features[Z80::ELFArchXMEGA2])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA2;
  else if (Features[Z80::ELFArchXMEGA3])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA3;
  else if (Features[Z80::ELFArchXMEGA4])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA4;
  else if (Features[Z80::ELFArchXMEGA5])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA5;
  else if (Features[Z80::ELFArchXMEGA6])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA6;
  else if (Features[Z80::ELFArchXMEGA7])
    EFlags |= ELF::EF_Z80_ARCH_XMEGA7;*/

  return EFlags;
}

Z80ELFStreamer::Z80ELFStreamer(MCStreamer &S,
                               const MCSubtargetInfo &STI)
    : Z80TargetStreamer(S) {

  MCAssembler &MCA = getStreamer().getAssembler();
  unsigned EFlags = MCA.getELFHeaderEFlags();

  EFlags |= getEFlagsForFeatureSet(STI.getFeatureBits());

  MCA.setELFHeaderEFlags(EFlags);
}

} // end namespace llvm
