//===-- PDPInstPrinter.cpp - Convert PDP MCInst to assembly syntax --------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an PDP MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "PDPInstPrinter.h"

#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "PDPInstrInfo.h"

#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#include <cstring>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "PDPGenAsmWriter.inc"

void PDPInstPrinter::printInst(const MCInst *MI, uint64_t Address,
                               StringRef Annot, const MCSubtargetInfo &STI,
                               raw_ostream &O) {
  unsigned Opcode = MI->getOpcode();

  // First handle load and store instructions with postinc or predec
  // of the form "ld reg, X+".
  // TODO: We should be able to rewrite this using TableGen data.
  switch (Opcode) {
  //case PDP::LDRdPtr:
  /*case PDP::LDRdPtrPi:
  case PDP::LDRdPtrPd:
    O << "\tld\t";
    printOperand(MI, 0, O);
    O << ", ";

    if (Opcode == PDP::LDRdPtrPd)
      O << '-';

    printOperand(MI, 1, O);

    if (Opcode == PDP::LDRdPtrPi)
      O << '+';
    break;*/
  /*case PDP::STPtrRr:
    O << "\tst\t";
    printOperand(MI, 0, O);
    O << ", ";
    printOperand(MI, 1, O);
    break;*/
  /*case PDP::STPtrPiRr:
  case PDP::STPtrPdRr:
    O << "\tst\t";

    if (Opcode == PDP::STPtrPdRr)
      O << '-';

    printOperand(MI, 1, O);

    if (Opcode == PDP::STPtrPiRr)
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

const char *PDPInstPrinter::getPrettyRegisterName(MCRegister Reg,
                                                  MCRegisterInfo const &MRI) {
  return getRegisterName(Reg);

  // GCC prints register pairs by just printing the lower register
  // If the register contains a subregister, print it instead
  if (MRI.getNumSubRegIndices() > 0) {
    MCRegister RegLo = MRI.getSubReg(Reg, PDP::sub_lo);
    Reg = (RegLo != PDP::NoRegister) ? RegLo : Reg;
  }

  return getRegisterName(Reg);
}

void PDPInstPrinter::printOperand(const MCInst *MI, unsigned OpNo,
                                  raw_ostream &O) {
  const MCOperandInfo &MOI = this->MII.get(MI->getOpcode()).operands()[OpNo];
  /*if (MOI.RegClass == PDP::ZREGRegClassID) {
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
    bool isPtrReg = /*(MOI.RegClass == PDP::PTRREGSRegClassID) || */
                    /*(MOI.RegClass == PDP::PTRDISPREGSRegClassID) ||*/
                    /*(MOI.RegClass == PDP::ZREGRegClassID) ||*/
                    (MOI.RegClass == PDP::GPR16RegClassID);

    if (isPtrReg) {
      O << getRegisterName(Op.getReg());
      //O << getRegisterName(Op.getReg(), PDP::ptr);
    } else {
      O << getPrettyRegisterName(Op.getReg(), MRI);
    }
  } else if (Op.isImm()) {
    O << formatImm(Op.getImm() & 0xffff);
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    O << *Op.getExpr();
  }
}

/// This is used to print an immediate value that ends up
/// being encoded as a pc-relative value.
void PDPInstPrinter::printPCRelImm(const MCInst *MI, unsigned OpNo,
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
      O << '+' << formatOct(Imm);
    else
      O << '-' << formatOct(-Imm);
  } else {
    assert(Op.isExpr() && "Unknown pcrel immediate operand");
    O << *Op.getExpr();
  }
}

void PDPInstPrinter::printBrcond(const llvm::MCInst *MI, unsigned int OpNo,
                                llvm::raw_ostream &O) {
  auto cc = MI->getOperand(OpNo).getImm();

  switch (cc) {
  case PDPCC::CondCodes::COND_EQ: O << "COND_EQ"; break; //!< Equal
  case PDPCC::CondCodes::COND_NE: O << "COND_NE"; break; //!< Not equal
  case PDPCC::CondCodes::COND_GE: O << "COND_GE"; break; //!< Greater than or equal
  case PDPCC::CondCodes::COND_GT: O << "COND_GT"; break; //!< Greater than
  case PDPCC::CondCodes::COND_LT: O << "COND_LT"; break; //!< Less than
  case PDPCC::CondCodes::COND_LE: O << "COND_LE"; break; //!< Less than or equal
  case PDPCC::CondCodes::COND_SH: O << "COND_SH"; break; //!< Unsigned same or higher
  case PDPCC::CondCodes::COND_LO: O << "COND_LO"; break; //!< Unsigned lower
  case PDPCC::CondCodes::COND_MI: O << "COND_MI"; break; //!< Minus
  case PDPCC::CondCodes::COND_PL: O << "COND_PL"; break; //!< Plus
    default:
      llvm_unreachable("invalid condition code");
  }
}

void PDPInstPrinter::printUniop(const llvm::MCInst *MI, unsigned int OpNo,
                                llvm::raw_ostream &O) {
  auto reg = MI->getOperand(OpNo).getReg();
  auto mode = MI->getOperand(OpNo + 1).getImm();
  auto offset_or_immediate = MI->getOperand(OpNo + 2);
  auto disp = MI->getOperand(OpNo + 3).getImm();

  long imm = 0;

  auto isImm = offset_or_immediate.isImm();

  if (offset_or_immediate.isImm())
    imm = offset_or_immediate.getImm();

  imm += disp;
  imm &= 0xFFFF;

  if (mode == 0) {
    O << getRegisterName(reg);
  } else if (mode == 1) {
    O << "(" << getRegisterName(reg) << ")";
  } else if (mode == 2 && reg != PDP::R7 && !isImm) {
    O << "(" << getRegisterName(reg) << ")+";
  } else if (mode == 3 && reg != PDP::R7 && !isImm) {
    O << "@(" << getRegisterName(reg) << ")+";
  } else if (mode == 2 && reg == PDP::R7 && isImm) {
    O << "#" << formatImm(imm);
  } else if (mode == 2 && reg == PDP::R7 && !isImm) {
    O << "#";
    printOperand(MI, OpNo + 2, O);
  } else if (mode == 3 && reg == PDP::R7) {
    O << "@#";
    printOperand(MI, OpNo + 2, O);
  } else if (mode == 2) {
    O << "(" << getRegisterName(reg) << ")+";
  } else if (mode == 4) {
    O << "-(" << getRegisterName(reg) << ")";
  } else if (mode == 5) {
    O << "@-(" << getRegisterName(reg) << ")";
  } else if (mode == 6 && reg == PDP::R7) {
    printOperand(MI, OpNo + 2, O);
  } else if (mode == 6 && isImm) {
    O << formatImm(imm) << "(" << getRegisterName(reg)  << ")";
  } else if (mode == 7 && reg == PDP::R7) {
    O << "@";
    printOperand(MI, OpNo + 2, O);
  } else if (mode == 7 && isImm) {
    O << "@" << formatImm(imm) << "(" << getRegisterName(reg) << ")";
  } else
    llvm_unreachable("printUniop");
}

} // end of namespace llvm
