//===-- PDPMCInstLower.h - Lower MachineInstr to MCInst ---------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_MCINST_LOWER_H
#define LLVM_PDP_MCINST_LOWER_H

#include "PDPSubtarget.h"
#include "llvm/Support/Compiler.h"

namespace llvm {

class AsmPrinter;
class MachineInstr;
class MachineOperand;
class MCContext;
class MCInst;
class MCOperand;
class MCSymbol;

/// Lowers `MachineInstr` objects into `MCInst` objects.
class PDPMCInstLower {
public:
  PDPMCInstLower(MCContext &Ctx, AsmPrinter &Printer)
      : Ctx(Ctx), Printer(Printer) {}

  /// Lowers a `MachineInstr` into a `MCInst`.
  void lowerInstruction(const MachineInstr &MI, MCInst &OutMI) const;
  MCOperand lowerSymbolOperand(const MachineOperand &MO, MCSymbol *Sym,
                               const PDPSubtarget &Subtarget) const;

private:
  MCContext &Ctx;
  AsmPrinter &Printer;
};

} // end namespace llvm

#endif // LLVM_PDP_MCINST_LOWER_H
