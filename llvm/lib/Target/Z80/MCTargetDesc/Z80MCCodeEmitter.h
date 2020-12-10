//===-- Z80MCCodeEmitter.h - Convert Z80 Code to Machine Code -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the Z80MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//
//

#ifndef LLVM_Z80_CODE_EMITTER_H
#define LLVM_Z80_CODE_EMITTER_H

//#include "Z80FixupKinds.h"

#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/Support/DataTypes.h"

#define GET_INSTRINFO_OPERAND_TYPES_ENUM
#include "Z80GenInstrInfo.inc"

namespace llvm {

class MCContext;
class MCExpr;
class MCFixup;
class MCInst;
class MCInstrInfo;
class MCOperand;
class MCSubtargetInfo;
class raw_ostream;
class MCInstrDesc;
class MachineInstr;

namespace Z80II {

enum Prefix {
  NoPrfx = 0,
  CB = 1,
  ED = 2,
  DD = 3,
  FD = 4,
  DDCB = 5,
  FDCB = 6
};

class InstPrefixInfo {
private:
  bool HasHL, HasIX, HasIY;
  bool HasCB, HasED, HasDD, HasFD;
  bool HasDisplacement;
  unsigned InstrSize;
  unsigned Displacement;

  template <class T, class I>
  InstPrefixInfo(const T &B, const T&E, const MCInstrDesc &MD, const I &Inst);

public:
  InstPrefixInfo(const MCInst &MI, const MCInstrInfo &MII);
  InstPrefixInfo(const MachineInstr &MI);

  bool hasCB() const { return HasCB; };
  bool hasED() const { return HasED; };
  bool hasDD() const { return HasDD; };
  bool hasFD() const { return HasFD; };
  bool hasDisplacement() const { return HasDisplacement; };
  unsigned getDisplacement() const { return Displacement; };

  unsigned getSize() const { return InstrSize; };
};

} // end of namespace Z80II


/// Writes Z80 machine code to a stream.
class Z80MCCodeEmitter : public MCCodeEmitter {
public:
  Z80MCCodeEmitter(const MCInstrInfo &MCII, MCContext &Ctx)
      : MCII(MCII), Ctx(Ctx) {}

private:
  /// Finishes up encoding an LD/ST instruction.
  /// The purpose of this function is to set an bit in the instruction
  /// which follows no logical pattern. See the implementation for details.
  unsigned loadStorePostEncoder(const MCInst &MI, unsigned EncodedValue,
                                const MCSubtargetInfo &STI) const;

/*  /// Gets the encoding for a conditional branch target.
  template <Z80::Fixups Fixup>
  unsigned encodeRelCondBrTarget(const MCInst &MI, unsigned OpNo,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;*/

  /// Encodes the `PTRREGS` operand to a load or store instruction.
  unsigned encodeLDSTPtrReg(const MCInst &MI, unsigned OpNo,
                            SmallVectorImpl<MCFixup> &Fixups,
                            const MCSubtargetInfo &STI) const;

  /// Encodes a `register+immediate` operand for `LDD`/`STD`.
  unsigned encodeMemri(const MCInst &MI, unsigned OpNo,
                       SmallVectorImpl<MCFixup> &Fixups,
                       const MCSubtargetInfo &STI) const;

  /// Takes the complement of a number (~0 - val).
  unsigned encodeComplement(const MCInst &MI, unsigned OpNo,
                            SmallVectorImpl<MCFixup> &Fixups,
                            const MCSubtargetInfo &STI) const;

  /*/// Encodes an immediate value with a given fixup.
  /// \tparam Offset The offset into the instruction for the fixup.
  template <Z80::Fixups Fixup, unsigned Offset>
  unsigned encodeImm(const MCInst &MI, unsigned OpNo,
                     SmallVectorImpl<MCFixup> &Fixups,
                     const MCSubtargetInfo &STI) const;*/

  /// Gets the encoding of the target for the `CALL k` instruction.
  unsigned encodeCallTarget(const MCInst &MI, unsigned OpNo,
                            SmallVectorImpl<MCFixup> &Fixups,
                            const MCSubtargetInfo &STI) const;

  /// TableGen'ed function to get the binary encoding for an instruction.
  uint64_t getBinaryCodeForInstr(const MCInst &MI,
                                 SmallVectorImpl<MCFixup> &Fixups,
                                 const MCSubtargetInfo &STI) const;

  unsigned getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups,
                          const MCSubtargetInfo &STI) const;

  /// Returns the binary encoding of operand.
  ///
  /// If the machine operand requires relocation, the relocation is recorded
  /// and zero is returned.
  unsigned getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                             SmallVectorImpl<MCFixup> &Fixups,
                             const MCSubtargetInfo &STI) const;

/*
  void emitInstruction(uint64_t Val, unsigned Size, const MCSubtargetInfo &STI,
                       raw_ostream &OS) const;
*/

  void encodeInstruction(const MCInst &MI, raw_ostream &OS,
                         SmallVectorImpl<MCFixup> &Fixups,
                         const MCSubtargetInfo &STI) const override;

  Z80MCCodeEmitter(const Z80MCCodeEmitter &) = delete;
  void operator=(const Z80MCCodeEmitter &) = delete;

  const MCInstrInfo &MCII;
  MCContext &Ctx;
};

} // end namespace of llvm.

#endif // LLVM_Z80_CODE_EMITTER_H

