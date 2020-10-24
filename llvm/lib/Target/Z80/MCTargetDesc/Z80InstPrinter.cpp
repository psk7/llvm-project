//===-- Z80InstPrinter.cpp - Convert Z80 MCInst to assembly syntax --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an Z80 MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "Z80InstPrinter.h"

#include "Z80InstrInfo.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"

#include <cstring>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "Z80GenAsmWriter.inc"

void Z80InstPrinter::printInst(const MCInst *MI, uint64_t Address,
                               StringRef Annot, const MCSubtargetInfo &STI,
                               raw_ostream &O) {
  unsigned Opcode = MI->getOpcode();

  // First handle load and store instructions with postinc or predec
  // of the form "ld reg, X+".
  // TODO: We should be able to rewrite this using TableGen data.
  switch (Opcode) {
  /*case Z80::LDRdPtr:
  case Z80::LDRdPtrPi:
  case Z80::LDRdPtrPd:
    O << "\tld\t";
    printOperand(MI, 0, O);
    O << ", ";

    if (Opcode == Z80::LDRdPtrPd)
      O << '-';

    printOperand(MI, 1, O);

    if (Opcode == Z80::LDRdPtrPi)
      O << '+';
    break;
  case Z80::STPtrRr:
    O << "\tst\t";
    printOperand(MI, 0, O);
    O << ", ";
    printOperand(MI, 1, O);
    break;
  case Z80::STPtrPiRr:
  case Z80::STPtrPdRr:
    O << "\tst\t";

    if (Opcode == Z80::STPtrPdRr)
      O << '-';

    printOperand(MI, 1, O);

    if (Opcode == Z80::STPtrPiRr)
      O << '+';

    O << ", ";
    printOperand(MI, 2, O);
    break;*/
  default:
    if (!printAliasInstr(MI, Address, O))
      printInstruction(MI, Address, O);

    printAnnotation(O, Annot);
    break;
  }
}

const char *Z80InstPrinter::getPrettyRegisterName(unsigned RegNum,
                                                  MCRegisterInfo const &MRI) {
  /*// GCC prints register pairs by just printing the lower register
  // If the register contains a subregister, print it instead
  if (MRI.getNumSubRegIndices() > 0) {
    unsigned RegLoNum = MRI.getSubReg(RegNum, Z80::sub_lo);
    RegNum = (RegLoNum != Z80::NoRegister) ? RegLoNum : RegNum;
  }*/

  return getRegisterName(RegNum);
}

void Z80InstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperandInfo &MOI = this->MII.get(MI->getOpcode()).OpInfo[OpNo];
  /*if (MOI.RegClass == Z80::ZREGRegClassID) {
    // Special case for the Z register, which sometimes doesn't have an operand
    // in the MCInst.
    O << "Z";
    return;
  }*/

  if (OpNo >= MI->size()) {
    // Not all operands are correctly disassembled at the moment. This means
    // that some machine instructions won't have all the necessary operands
    // set.
    // To avoid asserting, print <unknown> instead until the necessary support
    // has been implemented.
    O << "<unknown>";
    return;
  }

  const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isReg()) {
    bool isPtrReg = false; /*(MOI.RegClass == Z80::PTRREGSRegClassID) ||
                    (MOI.RegClass == Z80::PTRDISPREGSRegClassID) ||
                    (MOI.RegClass == Z80::ZREGRegClassID);*/

    if (isPtrReg) {
      O << getRegisterName(Op.getReg(), Z80::ptr);
    } else {
      O << getPrettyRegisterName(Op.getReg(), MRI);
    }
  } else if (Op.isImm()) {
    O << formatImm(Op.getImm());
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    O << *Op.getExpr();
  }
}

/// This is used to print an immediate value that ends up
/// being encoded as a pc-relative value.
void Z80InstPrinter::printPCRelImm(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  if (OpNo >= MI->size()) {
    // Not all operands are correctly disassembled at the moment. This means
    // that some machine instructions won't have all the necessary operands
    // set.
    // To avoid asserting, print <unknown> instead until the necessary support
    // has been implemented.
    O << "<unknown>";
    return;
  }

  const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isImm()) {
    int64_t Imm = Op.getImm();
    O << '.';

    // Print a position sign if needed.
    // Negative values have their sign printed automatically.
    if (Imm >= 0)
      O << '+';

    O << Imm;
  } else {
    assert(Op.isExpr() && "Unknown pcrel immediate operand");
    O << *Op.getExpr();
  }
}

void Z80InstPrinter::printCondCode(const MCInst *MI, unsigned OpNo,
                                   raw_ostream &O) {
  assert(MI->getOpcode() == Z80::JRCC && "Opcode MUST BE JRCC");

  auto cc = static_cast<Z80CC::CondCodes>(MI->getOperand(OpNo).getImm());

  switch (cc) {
  case Z80CC::COND_NZ:
    O << "nz";
    break;
  case Z80CC::COND_Z:
    O << "z";
    break;
  case Z80CC::COND_NC:
    O << "nc";
    break;
  case Z80CC::COND_C:
    O << "c";
    break;
  case Z80CC::COND_PO:
    O << "po";
    break;
  case Z80CC::COND_PE:
    O << "pe";
    break;
  case Z80CC::COND_P:
    O << "p";
    break;
  case Z80CC::COND_M:
    O << "m";
    break;
  default:
    llvm_unreachable("wrong cond code");
  }
}

void Z80InstPrinter::printMemri(const MCInst *MI, unsigned OpNo,
                                raw_ostream &O) {
  assert(MI->getOperand(OpNo).isReg() && "Expected a register for the first operand");

  const MCOperand &OffsetOp = MI->getOperand(OpNo + 1);

  // Print the register.
  printOperand(MI, OpNo, O);

  // Print the {+,-}offset.
  if (OffsetOp.isImm()) {
    int64_t Offset = OffsetOp.getImm();

    if (Offset >= 0)
      O << '+';

    O << Offset;
  } else if (OffsetOp.isExpr()) {
    O << *OffsetOp.getExpr();
  } else {
    llvm_unreachable("unknown type for offset");
  }
}

} // end of namespace llvm

