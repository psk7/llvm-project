//===- Z80Disassembler.cpp - Disassembler for Z80 ---------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the Z80 Disassembler.
//
//===----------------------------------------------------------------------===//

#include "Z80.h"
#include "Z80RegisterInfo.h"
#include "Z80Subtarget.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "TargetInfo/Z80TargetInfo.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCFixedLenDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/Support/TargetRegistry.h"

using namespace llvm;

#define DEBUG_TYPE "z80-disassembler"

typedef MCDisassembler::DecodeStatus DecodeStatus;

namespace {

/// A disassembler class for Z80.
class Z80Disassembler : public MCDisassembler {
public:
  Z80Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  virtual ~Z80Disassembler() {}

  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;
};
}

static MCDisassembler *createZ80Disassembler(const Target &T,
                                             const MCSubtargetInfo &STI,
                                             MCContext &Ctx) {
  return new Z80Disassembler(STI, Ctx);
}


extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeZ80Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheZ80Target(),
                                         createZ80Disassembler);
}

static const uint16_t GPRDecoderTable[] = {
  Z80::B, Z80::C, Z80::D, Z80::E,
  Z80::H, Z80::L, Z80::A, Z80::A
};

static DecodeStatus DecodeGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                            uint64_t Address, const void *Decoder) {
  if (RegNo > 7)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeLD8RegisterClass(MCInst &Inst, unsigned RegNo,
                                           uint64_t Address, const void *Decoder) {
  llvm_unreachable("DecodeLD8RegisterClass");
  /*
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo+16];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
*/
}

static DecodeStatus DecodePTRREGSRegisterClass(MCInst &Inst, unsigned RegNo,
                                               uint64_t Address, const void *Decoder) {
  // Note: this function must be defined but does not seem to be called.
  assert(false && "unimplemented: PTRREGS register class");
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder);

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Insn,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus DecodeImm(MCInst &Inst, unsigned Insn,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder);

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder);

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeFormB(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeFormC_ED(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeBDREGSRegisterClass(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeBGPR8RegisterClass(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeREGSTOSTACKRegisterClass(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

static DecodeStatus DecodeDREGSRegisterClass(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder);

#include "Z80GenDisassemblerTables.inc"

static DecodeStatus DecodeBDREGSRegisterClass(MCInst &Inst, unsigned Insn,
                                              uint64_t Address,
                                              const void *Decoder) {
  unsigned Register;

  switch(Insn) {
  case 0:
    Register = Z80::BC;
    break;
  case 1:
    Register = Z80::DE;
    break;
  case 2:
    Register = Z80::HL;
    break;
  case 3:
    Register = Z80::SP;
    break;

  default:
    return MCDisassembler::Fail;
  }

  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeBGPR8RegisterClass(MCInst &Inst, unsigned RegNo,
                                             uint64_t Address,
                                             const void *Decoder) {
  if (RegNo > 7)
    return MCDisassembler::Fail;

  unsigned Register = GPRDecoderTable[RegNo];
  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeREGSTOSTACKRegisterClass(MCInst &Inst, unsigned Insn,
                                                   uint64_t Address,
                                                   const void *Decoder) {
  unsigned Register;

  switch(Insn) {
  case 0:
    Register = Z80::BC;
    break;
  case 1:
    Register = Z80::DE;
    break;
  case 2:
    Register = Z80::HL;
    break;
  case 3:
    Register = Z80::SP;
    break;

  default:
    return MCDisassembler::Fail;
  }

  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeDREGSRegisterClass(MCInst &Inst, unsigned Insn,
                                             uint64_t Address,
                                             const void *Decoder) {
  unsigned Register;

  switch(Insn) {
  case 0: 
    Register = Z80::BC;
    break;
  case 1:
    Register = Z80::DE;
    break;
  case 2:
    Register = Z80::HL;
    break;
  case 3:
    Register = Z80::SP;
    break;

  default:
    return MCDisassembler::Fail;
  }

  Inst.addOperand(MCOperand::createReg(Register));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFormB(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned reg = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createReg(Z80::ACC));
  Inst.addOperand(MCOperand::createReg(Z80::ACC));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus DecodeFormC_ED(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned reg = fieldFromInstruction(Insn, 12, 2);
  Inst.addOperand(MCOperand::createReg(Z80::HL));
  Inst.addOperand(MCOperand::createReg(Z80::HL));
  if (DecodeBDREGSRegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOARr(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  Inst.addOperand(MCOperand::createImm(addr));
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIORdA(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = 0;
  addr |= fieldFromInstruction(Insn, 0, 4);
  addr |= fieldFromInstruction(Insn, 9, 2) << 4;
  unsigned reg = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, reg, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(addr));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFIOBIT(MCInst &Inst, unsigned Insn,
                                 uint64_t Address, const void *Decoder) {
  unsigned addr = fieldFromInstruction(Insn, 3, 5);
  unsigned b = fieldFromInstruction(Insn, 0, 3);
  Inst.addOperand(MCOperand::createImm(addr));
  Inst.addOperand(MCOperand::createImm(b));
  return MCDisassembler::Success;
}

static DecodeStatus decodeCallTarget(MCInst &Inst, unsigned Field,
                                     uint64_t Address, const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(Field));
  return MCDisassembler::Success;
}
static DecodeStatus DecodeImm(MCInst &Inst, unsigned Field,
                                     uint64_t Address, const void *Decoder) {
  Inst.addOperand(MCOperand::createImm(Field));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFRd(MCInst &Inst, unsigned Insn,
                              uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 5);
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFLPMX(MCInst &Inst, unsigned Insn,
                                uint64_t Address, const void *Decoder) {
  llvm_unreachable("decodeFLPMX");
  /*
  if (decodeFRd(Inst, Insn, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createReg(Z80::R31R30));
  return MCDisassembler::Success;
*/
}

static DecodeStatus decodeFFMULRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 3) + 16;
  unsigned r = fieldFromInstruction(Insn, 0, 3) + 16;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMOVWRdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned r = fieldFromInstruction(Insn, 4, 4) * 2;
  unsigned d = fieldFromInstruction(Insn, 0, 4) * 2;
  if (DecodeGPR8RegisterClass(Inst, r, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus decodeFWRdK(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned d = fieldFromInstruction(Insn, 4, 2) * 2 + 24; // starts at r24:r25
  unsigned k = 0;
  k |= fieldFromInstruction(Insn, 0, 4);
  k |= fieldFromInstruction(Insn, 6, 2) << 4;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, d, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  Inst.addOperand(MCOperand::createImm(k));
  return MCDisassembler::Success;
}

static DecodeStatus decodeFMUL2RdRr(MCInst &Inst, unsigned Insn,
                                    uint64_t Address, const void *Decoder) {
  unsigned rd = fieldFromInstruction(Insn, 4, 4) + 16;
  unsigned rr = fieldFromInstruction(Insn, 0, 4) + 16;
  if (DecodeGPR8RegisterClass(Inst, rd, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  if (DecodeGPR8RegisterClass(Inst, rr, Address, Decoder) == MCDisassembler::Fail)
    return MCDisassembler::Fail;
  return MCDisassembler::Success;
}

static DecodeStatus readInstruction8(ArrayRef<uint8_t> Bytes, uint64_t Offset,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 1) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 1;
  Insn = Bytes[Offset];

  return MCDisassembler::Success;
}
static DecodeStatus readInstruction16(ArrayRef<uint8_t> Bytes, uint64_t Offset,
                                      uint64_t &Size, uint32_t &Insn) {
  if (Bytes.size() < 2) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 2;
  Insn = (Bytes[Offset] << 0) | (Bytes[Offset+1] << 8);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction24(ArrayRef<uint8_t> Bytes, uint64_t Offset,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 3) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 3;
  Insn = (Bytes[Offset] << 0) | (Bytes[Offset+1] << 8) | (Bytes[Offset+2] << 16);

  return MCDisassembler::Success;
}

static DecodeStatus readInstruction32(ArrayRef<uint8_t> Bytes, uint64_t Offset,
                                      uint64_t &Size, uint32_t &Insn) {

  if (Bytes.size() < 4) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  Size = 4;
  Insn = (Bytes[Offset] << 0) | (Bytes[Offset + 1] << 8) |
         (Bytes[Offset + 2] << 16) | (Bytes[Offset + 3] << 24);

  return MCDisassembler::Success;
}

static const uint8_t *getDecoderTable(uint64_t Size) {
  switch (Size) {
    case 1: return DecoderTable8;
    case 2: return DecoderTable16;
    case 3: return DecoderTable24;
    case 4: return DecoderTable32;
    default: llvm_unreachable("Wrong instruction size");
  }
}

static DecodeStatus AdjustPrefix(DecodeStatus Status, MCInst &Instr,
                                 uint64_t &Size, uint8_t Prefix) {

  bool HasDD = Prefix == 0xDD;
  bool HasFD = Prefix == 0xFD;

  if (HasDD || HasFD) {
    ++Size;

    switch (Instr.getNumOperands()) {
    case 2:
      auto &op0 = Instr.getOperand(1);

      switch (op0.getReg()) {
      case Z80::HL:
        if (HasDD)
          op0.setReg(Z80::IX);
        else if (HasFD)
          op0.setReg(Z80::IY);
        break;

      case Z80::H:
        if (HasDD)
          op0.setReg(Z80::XH);
        else if (HasFD)
          op0.setReg(Z80::YH);
        break;

      case Z80::L:
        if (HasDD)
          op0.setReg(Z80::XL);
        else if (HasFD)
          op0.setReg(Z80::YL);
        break;
      }

      break;
    }
  }

  return Status;
}

DecodeStatus Z80Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                             ArrayRef<uint8_t> Bytes,
                                             uint64_t Address,
                                             raw_ostream &CStream) const {
  uint32_t Insn;
  uint8_t Prefix = 0;
  uint64_t Offset = 0;

  DecodeStatus Result;

  auto &b = Bytes[Offset];

  if (Bytes.size() > 0 && ((b == 0xFD) || (b == 0xDD)))
    Prefix = b;

  if (Prefix != 0)
    ++Offset;

  // Try decode a 16-bit instruction.
  Result = readInstruction8(Bytes, Offset, Size, Insn);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  Result =
      decodeInstruction(getDecoderTable(Size), Instr, Insn, Address, this, STI);

  if (Result != MCDisassembler::Fail)
    return AdjustPrefix(Result, Instr, Size, Prefix);

  // Try decode a 16-bit instruction.
  Result = readInstruction16(Bytes, Offset, Size, Insn);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  Result =
      decodeInstruction(getDecoderTable(Size), Instr, Insn, Address, this, STI);

  if (Result != MCDisassembler::Fail)
    return AdjustPrefix(Result, Instr, Size, Prefix);

  // Try decode a 24-bit instruction.
  Result = readInstruction24(Bytes, Offset, Size, Insn);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  // Try to auto-decode a 24-bit instruction.
  Result =
      decodeInstruction(getDecoderTable(Size), Instr, Insn, Address, this, STI);

  if (Result != MCDisassembler::Fail)
    return AdjustPrefix(Result, Instr, Size, Prefix);

  // Try decode a 32-bit instruction.
  Result = readInstruction32(Bytes, Offset, Size, Insn);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  Result =
      decodeInstruction(getDecoderTable(Size), Instr, Insn, Address, this, STI);

  if (Result == MCDisassembler::Fail)
    return MCDisassembler::Fail;

  return AdjustPrefix(Result, Instr, Size, Prefix);
}

typedef DecodeStatus (*DecodeFunc)(MCInst &MI, unsigned insn, uint64_t Address,
                                   const void *Decoder);

