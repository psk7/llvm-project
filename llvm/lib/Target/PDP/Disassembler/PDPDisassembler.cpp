//===- PDPDisassembler.cpp - Disassembler for PDP ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the PDP Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/PDPMCCodeEmitter.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "TargetInfo/PDPTargetInfo.h"

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/STLExtras.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "PDP-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for PDP.
class PDPDisassembler : public MCDisassembler {
  std::unique_ptr<MCInstrInfo const> const MCII;

public:
  PDPDisassembler(const MCSubtargetInfo &STI, MCContext &Ctx, MCInstrInfo const *MCII)
      : MCDisassembler(STI, Ctx), MCII(MCII) {}
  virtual ~PDPDisassembler() = default;

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;

  void addOperand(MCInst &Instr, uint64_t &Size,
                  ArrayRef<uint8_t> Bytes, unsigned &Offset,
                  raw_ostream &CStream, unsigned OpNum) const;
};
} // namespace

static MCDisassembler *createPDPDisassembler(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             MCContext &Ctx) {
  return new PDPDisassembler(STI, Ctx, T.createMCInstrInfo());
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPDisassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getThePDPTarget(),
                                         createPDPDisassembler);
}

static const uint16_t GPRDecoderTable[] = {
    PDP::R0,  PDP::R1,  PDP::R2,  PDP::R3,  PDP::R4,  PDP::R5,  PDP::R6, PDP::R7,
};

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address,
                                            const MCDisassembler *Decoder) {
  if (RegNo > 31)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeGPR16RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address,
                                            const MCDisassembler *Decoder) {
  if (RegNo > 7)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address,
                                           const MCDisassembler *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo + 16];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder);

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder);

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder);

static DecodeStatus decodeRelCondBrTarget(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder);

static DecodeStatus decodeSobTarget(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder);

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

static DecodeStatus decodeUniop(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder);

static DecodeStatus decodeFBRk(MCInst &Inst, unsigned Insn, uint64_t Address,
                               const MCDisassembler *Decoder);

static DecodeStatus decodeCondBranch(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder);

static DecodeStatus decodeLoadStore(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder);

#include "PDPGenDisassemblerTables.inc"

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  Inst.addOperand(MCOperand::createImm(addr));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(addr));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn, uint64_t Address,
                                 const MCDisassembler *Decoder) {
  unsigned addr = fieldFromInstruction(Insn, 3, 5);
  unsigned b = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createImm(addr));
  Inst.addOperand(MCOperand::createImm(b));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder) {
  // Call targets need to be shifted left by one so this needs a custom
  // decoder.
  Inst.addOperand(MCOperand::createImm(Field << 1));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn, uint64_t Address,
                              const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  llvm_unreachable("decodeFLPMX");

  /*if (decodeFRd(Inst, Insn, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(PDP::R31R30));
  return MCDisassembler::Success;*/
}

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 3) + 16;
  unsigned r = fieldFromInstruction(Insn, 0, 3) + 16;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned r = fieldFromInstruction(Insn, 4, 4) * 2;
  unsigned d = fieldFromInstruction(Insn, 0, 4) * 2;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 2) * 2 + 24; // starts at r24:r25
  unsigned k = 0;
  k |= fieldFromInstruction(Insn, 0, 4);
  k |= fieldFromInstruction(Insn, 6, 2) << 4;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(k));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  unsigned rd = fieldFromInstruction(Insn, 4, 4) + 16;
  unsigned rr = fieldFromInstruction(Insn, 0, 4) + 16;
  if (DecodeGPR8RegisterClass(Inst, rd, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, rr, Address, Decoder) ==
      MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeUniop(MCInst &Inst, unsigned Insn, uint64_t Address,
                                const MCDisassembler *Decoder) {
  unsigned Reg = GPRDecoderTable[Insn & 0x7];
  unsigned Mode = (Insn >> 3) & 0x7;

  Inst.addOperand(MCOperand::createReg(Reg));
  Inst.addOperand(MCOperand::createImm(Mode));
  Inst.addOperand(MCOperand::createImm(0));
  Inst.addOperand(MCOperand::createImm(0));

  return MCDisassembler::Success;
}

static DecodeStatus decodeFBRk(MCInst &Inst, unsigned Insn, uint64_t Address,
                               const MCDisassembler *Decoder) {
  llvm_unreachable("decodeFBRk");

  /*// Decode the opcode.
  switch (Insn & 0xf000) {
  case 0xc000:
    Inst.setOpcode(PDP::RJMPk);
    break;
  default: // Unknown relative branch instruction.
    return MCDisassembler::Fail;
  }
  // Decode the relative offset.
  int16_t Offset = ((int16_t)((Insn & 0xfff) << 4)) >> 3;
  Inst.addOperand(MCOperand::createImm(Offset));
  return MCDisassembler::Success;*/
}

static DecodeStatus decodeRelCondBrTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder) {
  int64_t v = (char)Insn;
  
  Inst.addOperand(MCOperand::createImm(v * 2 + 2));

  return MCDisassembler::Success;
}

static DecodeStatus decodeSobTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder) {
  int64_t v = (char)Insn;

  Inst.addOperand(MCOperand::createImm(-v * 2 + 2));

  return MCDisassembler::Success;
}

static DecodeStatus decodeCondBranch(MCInst &Inst, unsigned Insn,
                                     uint64_t Address,
                                     const MCDisassembler *Decoder) {
  llvm_unreachable("decodeCondBranch");

  /*
  // These 8 instructions are not defined as aliases of BRBS/BRBC.
  DenseMap<unsigned, unsigned> brInsts = {
      {0x000, PDP::BRLOk}, {0x400, PDP::BRSHk}, {0x001, PDP::BREQk},
      {0x401, PDP::BRNEk}, {0x002, PDP::BRMIk}, {0x402, PDP::BRPLk},
      {0x004, PDP::BRLTk}, {0x404, PDP::BRGEk}};

  // Get the relative offset.
  int16_t Offset = ((int16_t)((Insn & 0x3f8) << 6)) >> 8;

  // Search the instruction pattern.
  auto NotAlias = [&Insn](const std::pair<unsigned, unsigned> &I) {
    return (Insn & 0x407) != I.first;
  };
  llvm::partition(brInsts, NotAlias);
  auto It = llvm::partition_point(brInsts, NotAlias);

  // Decode the instruction.
  if (It != brInsts.end()) {
    // This instruction is not an alias of BRBC/BRBS.
    Inst.setOpcode(It->second);
    Inst.addOperand(MCOperand::createImm(Offset));
  } else {
    // Fall back to an ordinary BRBS/BRBC.
    Inst.setOpcode(Insn & 0x400 ? PDP::BRBCsk : PDP::BRBSsk);
    Inst.addOperand(MCOperand::createImm(Insn & 7));
    Inst.addOperand(MCOperand::createImm(Offset));
  }

  return MCDisassembler::Success;
*/
}

static DecodeStatus decodeLoadStore(MCInst &Inst, unsigned Insn,
                                    uint64_t Address,
                                    const MCDisassembler *Decoder) {
  llvm_unreachable("decodeLoadStore");

  /*
  // Get the register will be loaded or stored.
  unsigned RegVal = GPRDecoderTable[(Insn >> 4) & 0x1f];

  // Decode LDD/STD with offset less than 8.
  if ((Insn & 0xf000) == 0x8000) {
    unsigned RegBase = (Insn & 0x8) ? PDP::R29R28 : PDP::R31R30;
    unsigned Offset = Insn & 7; // We need not consider offset > 7.
    if ((Insn & 0x200) == 0) {  // Decode LDD.
      Inst.setOpcode(PDP::LDDRdPtrQ);
      Inst.addOperand(MCOperand::createReg(RegVal));
      Inst.addOperand(MCOperand::createReg(RegBase));
      Inst.addOperand(MCOperand::createImm(Offset));
    } else { // Decode STD.
      Inst.setOpcode(PDP::STDPtrQRr);
      Inst.addOperand(MCOperand::createReg(RegBase));
      Inst.addOperand(MCOperand::createImm(Offset));
      Inst.addOperand(MCOperand::createReg(RegVal));
    }
    return MCDisassembler::Success;
  }

  // Decode the following 14 instructions. Bit 9 indicates load(0) or store(1),
  // bits 8~4 indicate the value register, bits 3-2 indicate the base address
  // register (11-X, 10-Y, 00-Z), bits 1~0 indicate the mode (00-basic,
  // 01-postinc, 10-predec).
  // ST X,  Rr : 1001 001r rrrr 1100
  // ST X+, Rr : 1001 001r rrrr 1101
  // ST -X, Rr : 1001 001r rrrr 1110
  // ST Y+, Rr : 1001 001r rrrr 1001
  // ST -Y, Rr : 1001 001r rrrr 1010
  // ST Z+, Rr : 1001 001r rrrr 0001
  // ST -Z, Rr : 1001 001r rrrr 0010
  // LD Rd, X  : 1001 000d dddd 1100
  // LD Rd, X+ : 1001 000d dddd 1101
  // LD Rd, -X : 1001 000d dddd 1110
  // LD Rd, Y+ : 1001 000d dddd 1001
  // LD Rd, -Y : 1001 000d dddd 1010
  // LD Rd, Z+ : 1001 000d dddd 0001
  // LD Rd, -Z : 1001 000d dddd 0010
  if ((Insn & 0xfc00) != 0x9000 || (Insn & 0xf) == 0)
    return MCDisassembler::Fail;

  // Get the base address register.
  unsigned RegBase;
  switch (Insn & 0xc) {
  case 0xc:
    RegBase = PDP::R27R26;
    break;
  case 0x8:
    RegBase = PDP::R29R28;
    break;
  case 0x0:
    RegBase = PDP::R31R30;
    break;
  default:
    return MCDisassembler::Fail;
  }

  // Set the opcode.
  switch (Insn & 0x203) {
  case 0x200:
    Inst.setOpcode(PDP::STPtrRr);
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegVal));
    return MCDisassembler::Success;
  case 0x201:
    Inst.setOpcode(PDP::STPtrPiRr);
    break;
  case 0x202:
    Inst.setOpcode(PDP::STPtrPdRr);
    break;
  case 0:
    Inst.setOpcode(PDP::LDRdPtr);
    Inst.addOperand(MCOperand::createReg(RegVal));
    Inst.addOperand(MCOperand::createReg(RegBase));
    return MCDisassembler::Success;
  case 1:
    Inst.setOpcode(PDP::LDRdPtrPi);
    break;
  case 2:
    Inst.setOpcode(PDP::LDRdPtrPd);
    break;
  default:
    return MCDisassembler::Fail;
  }

  // Build postinc/predec machine instructions.
  if ((Insn & 0x200) == 0) { // This is a load instruction.
    Inst.addOperand(MCOperand::createReg(RegVal));
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegBase));
  } else { // This is a store instruction.
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegBase));
    Inst.addOperand(MCOperand::createReg(RegVal));
    // STPtrPiRr and STPtrPdRr have an extra immediate operand.
    Inst.addOperand(MCOperand::createImm(1));
  }

  return MCDisassembler::Success;
*/
}

static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[0] << 0) | (Bytes[1] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Address,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 4;
  Insn =
      (Bytes[0] << 16) | (Bytes[1] << 24) | (Bytes[2] << 0) | (Bytes[3] << 8);

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {
  switch (Size) {
  case 2:
    return DecoderTable16;
//  case 4:
//    return DecoderTable32;
  default:
    llvm_unreachable("instructions must be 16 or 32-bits");
  }
}

void PDPDisassembler::addOperand(MCInst &Instr, uint64_t &Size,
                                 ArrayRef<uint8_t> Bytes,
                                 unsigned &Offset,
                                 raw_ostream &CStream, unsigned OpNum) const {

  const auto Reg = Instr.getOperand(OpNum).getReg();
  const auto Mode = Instr.getOperand(OpNum + 1).getImm();

  if (Mode == 2 && Reg == PDP::R7) {
    Size += 2;

    const uint16_t Imm = (Bytes[Offset] << 0) | (Bytes[Offset + 1] << 8);
    Instr.getOperand(OpNum + 2).setImm(Imm);

    Offset += 2;
  }
  else if (Mode == 3 && Reg == PDP::R7) {
    Size += 2;

    const uint16_t Imm = (Bytes[Offset] << 0) | (Bytes[Offset + 1] << 8);
    Instr.getOperand(OpNum + 2).setImm(Imm);

    Offset += 2;
  }
  else if (Mode == 6) {
    Size += 2;

    const uint16_t Imm = (Bytes[Offset] << 0) | (Bytes[Offset + 1] << 8);
    Instr.getOperand(OpNum + 2).setImm(Imm);

    Offset += 2;
  }
  else if (Mode == 7) {
    Size += 2;

    const uint16_t Imm = (Bytes[Offset] << 0) | (Bytes[Offset + 1] << 8);
    Instr.getOperand(OpNum + 2).setImm(Imm);

    Offset += 2;
  }
}

DecodeStatus PDPDisassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;

  DecodeStatus Result;

  // Try decode a 16-bit instruction.
  {
    Result = readInstruction16(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail)
      return MCDisassembler::Fail;

    // Try to decode PDPTiny instructions.
    /*if (STI.hasFeature(PDP::FeatureTinyEncoding)) {
      Result = decodeInstruction(DecoderTablePDPTiny16, Instr, Insn, Address,
                                 this, STI);
      if (Result != MCDisassembler::Fail)
        return Result;
    }*/

    // Try to auto-decode a 16-bit instruction.
    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn, Address,
                               this, STI);
    if (Result == MCDisassembler::Fail)
      return Result;

    const MCInstrDesc &Desc = MCII->get(Instr.getOpcode());
    unsigned Offset = 2;

    PDPMCCodeEmitter::forEachArg(
        Desc, [this, &Instr, &Size, &Bytes, &Offset, &CStream](const unsigned N) -> void {
          addOperand(Instr, Size, Bytes, Offset, CStream, N);
        });

    return MCDisassembler::Success;
  }

  return MCDisassembler::Fail;

  /*// Try decode a 32-bit instruction.
  {
    Result = readInstruction32(Bytes, Address, Size, Insn);

    if (Result == MCDisassembler::Fail)
      return MCDisassembler::Fail;

    Result = decodeInstruction(getDecoderTable(Size), Instr, Insn, Address,
                               this, STI);

    if (Result != MCDisassembler::Fail) {
      return Result;
    }

    return MCDisassembler::Fail;
  }*/
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const MCDisassembler *Decoder);
