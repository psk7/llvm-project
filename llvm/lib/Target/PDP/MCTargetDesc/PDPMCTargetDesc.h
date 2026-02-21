//===-- PDPMCTargetDesc.h - PDP Target Descriptions -------------*- C++ -*-===//
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

#ifndef LLVM_PDP_MCTARGET_DESC_H
#define LLVM_PDP_MCTARGET_DESC_H

#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class Target;

MCInstrInfo *createPDPMCInstrInfo();

/// Creates a machine code emitter for PDP.
MCCodeEmitter *createPDPMCCodeEmitter(const MCInstrInfo &MCII,
                                      MCContext &Ctx);

/// Creates an assembly backend for PDP.
MCAsmBackend *createPDPAsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for PDP.
std::unique_ptr<MCObjectTargetWriter> createPDPELFObjectWriter(uint8_t OSABI);

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "PDPGenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#define GET_INSTRINFO_MC_HELPER_DECLS
#include "PDPGenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "PDPGenSubtargetInfo.inc"

#endif // LLVM_PDP_MCTARGET_DESC_H
