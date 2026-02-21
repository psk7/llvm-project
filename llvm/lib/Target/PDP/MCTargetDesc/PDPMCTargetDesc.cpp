//===-- PDPMCTargetDesc.cpp - PDP Target Descriptions ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides PDP specific target descriptions.
//
//===----------------------------------------------------------------------===//

#include "PDPMCTargetDesc.h"
#include "PDPELFStreamer.h"
#include "PDPInstPrinter.h"
#include "PDPMCAsmInfo.h"
#include "PDPTargetStreamer.h"
#include "TargetInfo/PDPTargetInfo.h"

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "PDPGenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "PDPGenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "PDPGenRegisterInfo.inc"

using namespace llvm;

MCInstrInfo *llvm::createPDPMCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitPDPMCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createPDPMCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitPDPMCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *createPDPMCSubtargetInfo(const Triple &TT,
                                                 StringRef CPU, StringRef FS) {
  return createPDPMCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createPDPMCInstPrinter(const Triple &T,
                                             unsigned SyntaxVariant,
                                             const MCAsmInfo &MAI,
                                             const MCInstrInfo &MII,
                                             const MCRegisterInfo &MRI) {
  if (SyntaxVariant == 0) {
    return new PDPInstPrinter(MAI, MII, MRI);
  }

  return nullptr;
}

static MCStreamer *createMCStreamer(const Triple &T, MCContext &Context,
                                    std::unique_ptr<MCAsmBackend> &&MAB,
                                    std::unique_ptr<MCObjectWriter> &&OW,
                                    std::unique_ptr<MCCodeEmitter> &&Emitter) {
  return createELFStreamer(Context, std::move(MAB), std::move(OW),
                           std::move(Emitter));
}

static MCTargetStreamer *
createPDPObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new PDPELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint) {
  return new PDPTargetAsmStreamer(S);
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPTargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<PDPMCAsmInfo> X(getThePDPTarget());

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getThePDPTarget(), createPDPMCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getThePDPTarget(), createPDPMCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getThePDPTarget(),
                                          createPDPMCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getThePDPTarget(),
                                        createPDPMCInstPrinter);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getThePDPTarget(),
                                        createPDPMCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getThePDPTarget(), createMCStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(getThePDPTarget(),
                                               createPDPObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getThePDPTarget(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getThePDPTarget(), createPDPAsmBackend);
}
