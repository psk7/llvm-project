//===-- PDPAsmPrinter.cpp - PDP LLVM assembly writer ----------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains a printer that converts from our internal representation
// of machine-dependent LLVM code to GAS-format PDP assembly language.
//
//===----------------------------------------------------------------------===//

#include "PDP.h"
#include "PDPMCInstLower.h"
#include "PDPSubtarget.h"
#include "PDPTargetMachine.h"
#include "MCTargetDesc/PDPInstPrinter.h"
#include "MCTargetDesc/PDPMCExpr.h"
#include "TargetInfo/PDPTargetInfo.h"

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Mangler.h"
#include "llvm/IR/Module.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetLoweringObjectFile.h"

#define DEBUG_TYPE "PDP-asm-printer"

namespace llvm {

/// An PDP assembly code printer.
class PDPAsmPrinter : public AsmPrinter {
public:
  PDPAsmPrinter(TargetMachine &TM, std::unique_ptr<MCStreamer> Streamer)
      : AsmPrinter(TM, std::move(Streamer)), MRI(*TM.getMCRegisterInfo()) {}

  StringRef getPassName() const override { return "PDP Assembly Printer"; }

  void printOperand(const MachineInstr *MI, unsigned OpNo, raw_ostream &O);

  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                       const char *ExtraCode, raw_ostream &O) override;

  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNum,
                             const char *ExtraCode, raw_ostream &O) override;

  void emitInstruction(const MachineInstr *MI) override;

  //const MCExpr *lowerConstant(const Constant *CV) override;

  void emitXXStructor(const DataLayout &DL, const Constant *CV) override;

  bool doFinalization(Module &M) override;

  void emitStartOfAsmFile(Module &M) override;

private:
  const MCRegisterInfo &MRI;
  bool EmittedStructorSymbolAttrs = false;
};

void PDPAsmPrinter::printOperand(const MachineInstr *MI, unsigned OpNo,
                                 raw_ostream &O) {
  const MachineOperand &MO = MI->getOperand(OpNo);

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    O << PDPInstPrinter::getPrettyRegisterName(MO.getReg(), MRI);
    break;
  case MachineOperand::MO_Immediate:
    O << MO.getImm();
    break;
  case MachineOperand::MO_GlobalAddress:
    O << getSymbol(MO.getGlobal());
    break;
  case MachineOperand::MO_ExternalSymbol:
    O << *GetExternalSymbolSymbol(MO.getSymbolName());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    O << *MO.getMBB()->getSymbol();
    break;
  default:
    llvm_unreachable("Not implemented yet!");
  }
}

bool PDPAsmPrinter::PrintAsmOperand(const MachineInstr *MI, unsigned OpNum,
                                    const char *ExtraCode, raw_ostream &O) {
  // Default asm printer can only deal with some extra codes,
  // so try it first.
  if (!AsmPrinter::PrintAsmOperand(MI, OpNum, ExtraCode, O))
    return false;

  const MachineOperand &MO = MI->getOperand(OpNum);

  if (ExtraCode && ExtraCode[0]) {
    // Unknown extra code.
    if (ExtraCode[1] != 0 || ExtraCode[0] < 'A' || ExtraCode[0] > 'Z')
      return true;

    // Operand must be a register when using 'A' ~ 'Z' extra code.
    if (!MO.isReg())
      return true;

    Register Reg = MO.getReg();

    unsigned ByteNumber = ExtraCode[0] - 'A';
    const InlineAsm::Flag OpFlags(MI->getOperand(OpNum - 1).getImm());
    const unsigned NumOpRegs = OpFlags.getNumOperandRegisters();

    const PDPSubtarget &STI = MF->getSubtarget<PDPSubtarget>();
    const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

    const TargetRegisterClass *RC = TRI.getMinimalPhysRegClass(Reg);
    unsigned BytesPerReg = TRI.getRegSizeInBits(*RC) / 8;
    assert(BytesPerReg <= 2 && "Only 8 and 16 bit regs are supported.");

    unsigned RegIdx = ByteNumber / BytesPerReg;
    if (RegIdx >= NumOpRegs)
      return true;
    Reg = MI->getOperand(OpNum + RegIdx).getReg();

    if (BytesPerReg == 2) {
      Reg = TRI.getSubReg(Reg, (ByteNumber % BytesPerReg) ? PDP::sub_hi
                                                          : PDP::sub_lo);
    }

    O << PDPInstPrinter::getPrettyRegisterName(Reg, MRI);
    return false;
  }

  if (MO.getType() == MachineOperand::MO_GlobalAddress)
    PrintSymbolOperand(MO, O); // Print global symbols.
  else
    printOperand(MI, OpNum, O); // Fallback to ordinary cases.

  return false;
}

bool PDPAsmPrinter::PrintAsmMemoryOperand(const MachineInstr *MI,
                                          unsigned OpNum, const char *ExtraCode,
                                          raw_ostream &O) {
  llvm_unreachable("PDPAsmPrinter::PrintAsmMemoryOperand");

  /*if (ExtraCode && ExtraCode[0])
    return true; // Unknown modifier

  const MachineOperand &MO = MI->getOperand(OpNum);
  (void)MO;
  assert(MO.isReg() && "Unexpected inline asm memory operand");

  // TODO: We should be able to look up the alternative name for
  // the register if it's given.
  // TableGen doesn't expose a way of getting retrieving names
  // for registers.
  if (MI->getOperand(OpNum).getReg() == PDP::R31R30) {
    O << "Z";
  } else if (MI->getOperand(OpNum).getReg() == PDP::R29R28) {
    O << "Y";
  } else if (MI->getOperand(OpNum).getReg() == PDP::R27R26) {
    O << "X";
  } else {
    assert(false && "Wrong register class for memory operand.");
  }

  // If NumOpRegs == 2, then we assume it is product of a FrameIndex expansion
  // and the second operand is an Imm.
  const InlineAsm::Flag OpFlags(MI->getOperand(OpNum - 1).getImm());
  const unsigned NumOpRegs = OpFlags.getNumOperandRegisters();

  if (NumOpRegs == 2) {
    assert(MI->getOperand(OpNum).getReg() != PDP::R27R26 &&
           "Base register X can not have offset/displacement.");
    O << '+' << MI->getOperand(OpNum + 1).getImm();
  }

  return false;*/
}

void PDPAsmPrinter::emitInstruction(const MachineInstr *MI) {
  PDP_MC::verifyInstructionPredicates(MI->getOpcode(),
                                      getSubtargetInfo().getFeatureBits());

  PDPMCInstLower MCInstLowering(OutContext, *this);

  MCInst I;
  MCInstLowering.lowerInstruction(*MI, I);
  EmitToStreamer(*OutStreamer, I);
}

/*const MCExpr *PDPAsmPrinter::lowerConstant(const Constant *CV) {
  MCContext &Ctx = OutContext;

  if (const GlobalValue *GV = dyn_cast<GlobalValue>(CV)) {
    bool IsProgMem = GV->getAddressSpace() == PDP::ProgramMemory;
    if (IsProgMem) {
      const MCExpr *Expr = MCSymbolRefExpr::create(getSymbol(GV), Ctx);
      return PDPMCExpr::create(PDPMCExpr::VK_PDP_PM, Expr, false, Ctx);
    }
  }

  return AsmPrinter::lowerConstant(CV);
}*/

void PDPAsmPrinter::emitXXStructor(const DataLayout &DL, const Constant *CV) {
  if (!EmittedStructorSymbolAttrs) {
    OutStreamer->emitRawComment(
        " Emitting these undefined symbol references causes us to link the"
        " libgcc code that runs our constructors/destructors");
    OutStreamer->emitRawComment(" This matches GCC's behavior");

    MCSymbol *CtorsSym = OutContext.getOrCreateSymbol("__do_global_ctors");
    OutStreamer->emitSymbolAttribute(CtorsSym, MCSA_Global);

    MCSymbol *DtorsSym = OutContext.getOrCreateSymbol("__do_global_dtors");
    OutStreamer->emitSymbolAttribute(DtorsSym, MCSA_Global);

    EmittedStructorSymbolAttrs = true;
  }

  AsmPrinter::emitXXStructor(DL, CV);
}

bool PDPAsmPrinter::doFinalization(Module &M) {

  /*const TargetLoweringObjectFile &TLOF = getObjFileLowering();
  const PDPTargetMachine &TM = (const PDPTargetMachine &)MMI->getTarget();
  const PDPSubtarget *SubTM = (const PDPSubtarget *)TM.getSubtargetImpl();

  bool NeedsCopyData = false;
  bool NeedsClearBSS = false;
  for (const auto &GO : M.globals()) {
    if (!GO.hasInitializer() || GO.hasAvailableExternallyLinkage())
      // These globals aren't defined in the current object file.
      continue;

    if (GO.hasCommonLinkage()) {
      // COMMON symbols are put in .bss.
      NeedsClearBSS = true;
      continue;
    }

    auto *Section = cast<MCSectionELF>(TLOF.SectionForGlobal(&GO, TM));
    if (Section->getName().starts_with(".data"))
      NeedsCopyData = true;
    else if (Section->getName().starts_with(".rodata") && SubTM->hasLPM())
      // PDPs that have a separate program memory (that's most PDPs) store
      // .rodata sections in RAM.
      NeedsCopyData = true;
    else if (Section->getName().starts_with(".bss"))
      NeedsClearBSS = true;
  }

  MCSymbol *DoCopyData = OutContext.getOrCreateSymbol("__do_copy_data");
  MCSymbol *DoClearBss = OutContext.getOrCreateSymbol("__do_clear_bss");

  if (NeedsCopyData) {
    OutStreamer->emitRawComment(
        " Declaring this symbol tells the CRT that it should");
    OutStreamer->emitRawComment(
        "copy all variables from program memory to RAM on startup");
    OutStreamer->emitSymbolAttribute(DoCopyData, MCSA_Global);
  }

  if (NeedsClearBSS) {
    OutStreamer->emitRawComment(
        " Declaring this symbol tells the CRT that it should");
    OutStreamer->emitRawComment("clear the zeroed data section on startup");
    OutStreamer->emitSymbolAttribute(DoClearBss, MCSA_Global);
  }*/

  return AsmPrinter::doFinalization(M);
}

void PDPAsmPrinter::emitStartOfAsmFile(Module &M) {
  const PDPTargetMachine &TM = (const PDPTargetMachine &)MMI->getTarget();
  const PDPSubtarget *SubTM = (const PDPSubtarget *)TM.getSubtargetImpl();
  if (!SubTM)
    return;

  /*// Emit __EIND__ if available.
  if (SubTM->hasEIJMPCALL())
    OutStreamer->emitAssignment(
        MMI->getContext().getOrCreateSymbol(StringRef("__EIND__")),
        MCConstantExpr::create(SubTM->getIORegEIND(), MMI->getContext()));
  // Emit __RAMPZ__ if available.
  if (SubTM->hasELPM())
    OutStreamer->emitAssignment(
        MMI->getContext().getOrCreateSymbol(StringRef("__RAMPZ__")),
        MCConstantExpr::create(SubTM->getIORegRAMPZ(), MMI->getContext()));*/
}

} // end of namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPAsmPrinter() {
  llvm::RegisterAsmPrinter<llvm::PDPAsmPrinter> X(llvm::getThePDPTarget());
}
