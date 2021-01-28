//===-- Z80InstrInfo.td - Z80 Instruction defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception

//===----------------------------------------------------------------------===//
//
// This file describes the Z80 instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "Z80InstrFormats.td"

//===----------------------------------------------------------------------===//
// Z80 Type Profiles
//===----------------------------------------------------------------------===//

def SDT_Z80CallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_Z80CallSeqEnd : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_Z80Call : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_Z80Wrapper : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>, SDTCisPtrTy<0>]>;
def SDT_Z80Brcond : SDTypeProfile<0, 2, [SDTCisVT<0, OtherVT>, SDTCisVT<1, i8>]>;
def SDT_Z80Cp : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;
def SDT_Z80Cps : SDTypeProfile<1, 2, [SDTCisSameAs<0, 1>]>;
def SDT_Z80Tst : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_Z80SelectCC : SDTypeProfile<1, 4, [SDTCisSameAs<0, 1>,
                                    SDTCisSameAs<1, 2>, SDTCisVT<3, i8>, SDTCisVT<4, i8>]>;
def SDT_Z80Outport : SDTypeProfile<0, 2, [SDTCisVT<0, i8>, SDTCisVT<1, i16>]>;
def SDT_Z80Inport : SDTypeProfile<1, 1, [ SDTCisPtrTy<1> ]>;
def SDT_Z80RotShift : SDTypeProfile<1, 3, [ SDTCisInt<0>, SDTCisInt<1>, SDTCisInt<2> ]>;

//===----------------------------------------------------------------------===//
// Z80 Specific Node Definitions
//===----------------------------------------------------------------------===//

def Z80retflag : SDNode<"Z80ISD::RET_FLAG", SDTNone,
                        [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def Z80retiflag : SDNode<"Z80ISD::RETI_FLAG", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def Z80callseq_start : SDNode<"ISD::CALLSEQ_START", SDT_Z80CallSeqStart,
                              [SDNPHasChain, SDNPOutGlue]>;
def Z80callseq_end : SDNode<"ISD::CALLSEQ_END", SDT_Z80CallSeqEnd,
                            [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def Z80call : SDNode<"Z80ISD::CALL", SDT_Z80Call,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;

def Z80Wrapper : SDNode<"Z80ISD::WRAPPER", SDT_Z80Wrapper>;

def Z80brcond : SDNode<"Z80ISD::BRCOND", SDT_Z80Brcond,
                       [SDNPHasChain, SDNPInGlue]>;
def Z80cp : SDNode<"Z80ISD::CP", SDT_Z80Cp, [SDNPOutGlue]>;
def Z80cps : SDNode<"Z80ISD::CPS", SDT_Z80Cps, [SDNPOutGlue]>;
def Z80tst : SDNode<"Z80ISD::TST", SDT_Z80Tst, [SDNPOutGlue]>;
def Z80selectcc: SDNode<"Z80ISD::SELECT_CC", SDT_Z80SelectCC, [SDNPInGlue]>;

//// Shift nodes.
def Z80rotshift : SDNode<"Z80ISD::ROTSHIFT", SDT_Z80RotShift>;

def outport : SDNode<"Z80ISD::OUTPORT", SDT_Z80Outport, [SDNPHasChain, SDNPMayStore, SDNPMemOperand]>;
def inport : SDNode<"Z80ISD::INPORT", SDT_Z80Inport, [SDNPHasChain, SDNPMayLoad, SDNPMemOperand]>;

def Z80_ROT_RLC : PatLeaf<(i8 0)>;
def Z80_ROT_RRC : PatLeaf<(i8 1)>;
def Z80_ROT_RL  : PatLeaf<(i8 2)>;
def Z80_ROT_RR  : PatLeaf<(i8 3)>;
def Z80_ROT_SLA : PatLeaf<(i8 4)>;
def Z80_ROT_SRA : PatLeaf<(i8 5)>;
def Z80_ROT_SRL : PatLeaf<(i8 7)>;

def Z80_CC_NZ   : PatLeaf<(i8 0)>;
def Z80_CC_Z    : PatLeaf<(i8 1)>;
def Z80_CC_NC   : PatLeaf<(i8 2)>;
def Z80_CC_C    : PatLeaf<(i8 3)>;
def Z80_CC_PO   : PatLeaf<(i8 4)>;
def Z80_CC_PE   : PatLeaf<(i8 5)>;
def Z80_CC_P    : PatLeaf<(i8 6)>;
def Z80_CC_M    : PatLeaf<(i8 7)>;

//===----------------------------------------------------------------------===//
// Z80 Operands, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//
def rot : PatLeaf<(i8 timm), [{
   auto r = (Z80II::Rotation)(N->getSExtValue());
   return r != Z80II::ROT_INVALID;
}]>;

//def imm8_neg_XFORM : SDNodeXForm<imm,
//[{
//  return CurDAG->getTargetConstant(-N->getAPIntValue(), SDLoc(N), MVT::i8);
//}]>;
//
//def imm16_neg_XFORM : SDNodeXForm<imm,
//[{
//  return CurDAG->getTargetConstant(-N->getAPIntValue(), SDLoc(N), MVT::i16);
//}]>;
//
//def imm0_63_neg : PatLeaf<(imm),
//[{
//  int64_t val = -N->getSExtValue();
//  return val >= 0 && val < 64;
//}], imm16_neg_XFORM>;
//
//def uimm6 : PatLeaf<(imm), [{ return isUInt<6>(N->getZExtValue()); }]>;

def uimm8 : PatLeaf<(imm), [{ return isUInt<8>(N->getZExtValue()); }]>;
def immzero : PatLeaf<(imm), [{ return N->getZExtValue() == 0; }]>;

//
//// imm_com8_XFORM - Return the complement of a imm_com8 value
//def imm_com8_XFORM : SDNodeXForm<imm, [{
//  return CurDAG->getTargetConstant(~((uint8_t)N->getZExtValue()), SDLoc(N),
//                                   MVT::i8);
//}]>;
//
//// imm_com8 - Match an immediate that is a complement
//// of a 8-bit immediate.
//// Note: this pattern doesn't require an encoder method and such, as it's
//// only used on aliases (Pat<> and InstAlias<>). The actual encoding
//// is handled by the destination instructions, which use imm_com8.
//def imm_com8_asmoperand : AsmOperandClass { let Name = "ImmCom8"; }
//def imm_com8 : Operand<i8> {
//  let ParserMatchClass = imm_com8_asmoperand;
//}
//
//def ioaddr_XFORM : SDNodeXForm<imm,
//[{
//  uint8_t offset = Subtarget->getIORegisterOffset();
//  return CurDAG->getTargetConstant(uint8_t(N->getZExtValue()) - offset,
//                                   SDLoc(N), MVT::i8);
//}]>;

def iobitpos8_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint8_t(N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;

def iobitposn8_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint8_t(~N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;

def iobitpos16_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint16_t(N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;

def iobitposn16_XFORM : SDNodeXForm<imm,
[{
  return CurDAG->getTargetConstant(Log2_32(uint16_t(~N->getZExtValue())),
                                   SDLoc(N), MVT::i8);
}]>;

//def ioaddr8 : PatLeaf<(imm),
//[{
//  uint8_t offset = Subtarget->getIORegisterOffset();
//  uint64_t val = N->getZExtValue() - offset;
//  return val < 0x40;
//}], ioaddr_XFORM>;
//
//def lowioaddr8 : PatLeaf<(imm),
//[{
//  uint8_t offset = Subtarget->getIORegisterOffset();
//  uint64_t val = N->getZExtValue() - offset;
//  return val < 0x20;
//}], ioaddr_XFORM>;
//
//def ioaddr16 : PatLeaf<(imm),
//[{
//  uint8_t offset = Subtarget->getIORegisterOffset();
//  uint64_t val = N->getZExtValue() - offset;
//  return val < 0x3f;
//}], ioaddr_XFORM>;

def iobitpos8 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint8_t(N->getZExtValue()));
}], iobitpos8_XFORM>;

def iobitposn8 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint8_t(~N->getZExtValue()));
}], iobitposn8_XFORM>;

def iobitpos16 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint16_t(N->getZExtValue()));
}], iobitpos16_XFORM>;

def iobitposn16 : PatLeaf<(imm),
[{
  return isPowerOf2_32(uint16_t(~N->getZExtValue()));
}], iobitposn16_XFORM>;

def MemriAsmOperand : AsmOperandClass {
  let Name = "Memri";
  let ParserMethod = "parseMemriOperand";
}

/// Address operand for `reg+imm` used by STD and LDD.
def memri : Operand<iPTR>
{
  let MIOperandInfo = (ops PTRDISPREGS, i16imm);

  let PrintMethod = "printMemri";
  let EncoderMethod = "encodeMemri";

  let ParserMatchClass = MemriAsmOperand;
}

def rottype : Operand<i8>
{
  let MIOperandInfo = (ops i8imm);
}

// Address operand for `SP+imm` used by STD{W}SPQRr
def memspi : Operand<iPTR>
{
  let MIOperandInfo = (ops SPREG, i16imm);
}

def relbrtarget : Operand<OtherVT>
{
    let PrintMethod   = "printPCRelImm";
    let EncoderMethod = "encodeRelCondBrTarget<Z80::fixup_8_pcrel>";
}

def condcode : Operand<i8>
{
    let PrintMethod   = "printCondCode";
}

def icall_target : Operand<iPTR>
{
}

def call_target : Operand<iPTR>
{
    let EncoderMethod = "encodeCallTarget";
    let DecoderMethod = "decodeCallTarget";
}

def imm16 : Operand<i16>
{
    let EncoderMethod = "encodeImm<Z80::fixup_16, 1>";
}

///// A 6-bit immediate used in the ADIW/SBIW instructions.
//def imm_arith6 : Operand<i16>
//{
//    let EncoderMethod = "encodeImm<Z80::fixup_6_adiw, 0>";
//}

/// An 8-bit immediate inside an instruction with the same format
/// as the `LDI` instruction (the `FRdK` format).
def imm_ldi8 : Operand<i8>
{
    //let EncoderMethod = "encodeImm<Z80::fixup_ldi, 0>";
}

///// A 5-bit port number used in SBIC and friends (the `FIOBIT` format).
//def imm_port5 : Operand<i8>
//{
//    let EncoderMethod = "encodeImm<Z80::fixup_port5, 0>";
//}
//
///// A 6-bit port number used in the `IN` instruction and friends (the
///// `FIORdA` format.
//def imm_port6 : Operand<i8>
//{
//    let EncoderMethod = "encodeImm<Z80::fixup_port6, 0>";
//}

// Addressing mode pattern reg+imm8
def addr : ComplexPattern<iPTR, 2, "SelectAddr", [], [SDNPWantRoot]>;

//// AsmOperand class for a pointer register.
//// Used with the LD/ST family of instructions.
//// See FSTLD in Z80InstrFormats.td
//def PtrRegAsmOperand : AsmOperandClass
//{
//   let Name = "Reg";
//}
//
//// A special operand type for the LD/ST instructions.
//// It converts the pointer register number into a two-bit field used in the
//// instruction.
//def LDSTPtrReg : Operand<i16>
//{
//    let MIOperandInfo = (ops PTRREGS);
//    let EncoderMethod = "encodeLDSTPtrReg";
//
//    let ParserMatchClass = PtrRegAsmOperand;
//}
//
//// A special operand type for the LDD/STD instructions.
//// It behaves identically to the LD/ST version, except restricts
//// the pointer registers to Y and Z.
//def LDDSTDPtrReg : Operand<i16>
//{
//    let MIOperandInfo = (ops PTRDISPREGS);
//    let EncoderMethod = "encodeLDSTPtrReg";
//
//    let ParserMatchClass = PtrRegAsmOperand;
//}

//===----------------------------------------------------------------------===//
// Z80 predicates for subtarget features
//===----------------------------------------------------------------------===//
def Z80_COND_EQ : PatLeaf<(i8 0)>;
def Z80_COND_NE : PatLeaf<(i8 1)>;
def Z80_COND_GE : PatLeaf<(i8 2)>;
def Z80_COND_LT : PatLeaf<(i8 3)>;
def Z80_COND_SH : PatLeaf<(i8 4)>;
def Z80_COND_LO : PatLeaf<(i8 5)>;
def Z80_COND_MI : PatLeaf<(i8 6)>;
def Z80_COND_PL : PatLeaf<(i8 7)>;


//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// Z80 Instruction list
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
let Defs = [SP, SREG],
Uses = [SP] in
{
  def ADJCALLSTACKDOWN : Pseudo<(outs),
                                (ins i16imm:$amt, i16imm:$amt2),
                                "#ADJCALLSTACKDOWN",
                                [(Z80callseq_start timm:$amt, timm:$amt2)]>;

  // R31R30 is used to update SP, since it is a scratch reg and this instruction
  // is placed after the function call then R31R30 should be always free.
  //let Defs = [R31R30],
  //Uses = [R31R30] in
  //:TODO: if we enable this, the pseudo is killed because it looks dead
  def ADJCALLSTACKUP : Pseudo<(outs),
                              (ins i16imm:$amt1, i16imm:$amt2),
                              "#ADJCALLSTACKUP",
                              [(Z80callseq_end timm:$amt1, timm:$amt2)]>;
}

//===----------------------------------------------------------------------===//
// Addition
//===----------------------------------------------------------------------===//
let isCommutable = 0,
Constraints = "$src = $rd",
Defs = [SREG] in
{
    def ADDRdRr8 : FormB<0b10000000,
                         (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "add  $rd, $rr",
                         [(set i8:$rd, (add i8:$src, i8:$rr)), (implicit SREG)]>;

    def ADDRdRr16 : FormC<0b00001001, (outs HLIXIYREGS:$rd), (ins HLIXIYREGS:$src, DREGS:$rr), "add  $rd, $rr", []>;

    let isCommutable = 1 in
    def ADDWRdRr : Pseudo<(outs HLREG:$rd),
                        (ins HLREG:$src, DREGS:$rr),
                        "addw\t$rd, $rr",
                        [(set i16:$rd, (add i16:$src, i16:$rr)),
                         (implicit SREG)]>;


    let Uses = [SREG] in
    def ADCRdRr8 : FormB<0xc6,
                         (outs ACC:$rd), (ins GPR8:$src, GPR8:$rr), "adc  $rd, $rr",
                         [(set i8:$rd, (adde i8:$src, i8:$rr)), (implicit SREG)]>;

    let Uses = [SREG] in
    def ADCRdRr16 : FormC<0b01001010,
                          (outs HLREG:$rd), (ins DREGS:$src, DREGS:$rr), "adc  $rd, $rr",
                          [(set i16:$rd, (adde i16:$src, i16:$rr)), (implicit SREG)]>, ED;
}

//===----------------------------------------------------------------------===//
// Subtraction
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd",
Defs = [SREG] in
{
   def SUBRdRr8 : FormB<0x90,
                         (outs ACC:$rd), (ins GPR8:$src, GPR8:$rr), "sub  $rd, $rr",
                         [(set i8:$rd, (sub i8:$src, i8:$rr)), (implicit SREG)]>;

   def SUBRdK : FormI<0xd6,
                     (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "sub  $rd, $d8",
                     [(set i8:$rd, (sub i8:$src, imm:$d8)), (implicit SREG)]>;

   def SUBRdRr16 : Pseudo<(outs HLREG:$rd), (ins HLREG:$src, DREGS:$rr), "sub  $rd, $rr", []>;

   let Uses = [SREG] in
     def SBCRdRr16 : FormC<0x42, (outs HLREG:$rd), (ins HLREG:$src, DREGS:$rr), "sbc  $rd, $rr", []>, ED;

  // Subtract with carry operations which must read the carry flag in SREG.
  let Uses = [SREG] in
  {
      def SBCRdRr8 : FormB<0x98,
                           (outs ACC:$rd), (ins GPR8:$src, GPR8:$rr), "sbc  $rd, $rr",
                           [(set i8:$rd, (sube i8:$src, i8:$rr)), (implicit SREG)]>;
  }
}

//===----------------------------------------------------------------------===//
// Increment and Decrement
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd" in
{
  let Defs = [SREG] in
  {
    def INCRd : FormH<0x04, (outs GPR8:$rd), (ins GPR8:$src), "inc  $rd",
                      [(set i8:$rd, (add i8:$src, 1)), (implicit SREG)]>;

    def DECRd : FormH<0x05, (outs GPR8:$rd), (ins GPR8:$src), "dec  $rd",
                      [(set i8:$rd, (add i8:$src, -1)), (implicit SREG)]>;
  }

  def INCWRd : FormC<0x03, (outs INCDECDREGS:$rd), (ins INCDECDREGS:$src), "inc  $rd",
                     [(set i16:$rd, (add i16:$src, 1))]>;

  def DECWRd : FormC<0x0b, (outs INCDECDREGS:$rd), (ins INCDECDREGS:$src), "dec  $rd",
                     [(set i16:$rd, (add i16:$src, -1))]>;
}

let Defs = [SREG], rd = 6 in
{
  def INCRdPTR : FormH<0x04, (outs), (ins memri:$addr), "inc  $addr",
                       [(store (i8 (add (i8 (load addr:$addr)), 1)), addr:$addr)]>, IsPtrInstr;

  def DECRdPTR : FormH<0x05, (outs), (ins memri:$addr), "dec  $addr",
                       [(store (i8 (add (i8 (load addr:$addr)), -1)), addr:$addr)]>, IsPtrInstr;
}


//===----------------------------------------------------------------------===//
// Logic
//===----------------------------------------------------------------------===//
def SETBITPTR : FormH<0xC6, (outs), (ins memri:$addr, i8imm:$rd), "set  $rd, $addr",
                      [(store (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)), addr:$addr)]>, IsPtrInstr, CB;

def SETBITPTRE : FormH<0xC6, (outs GPR8:$o), (ins memri:$addr, i8imm:$rd), "ld   $o, set $rd, $addr",
                      [(store (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)), addr:$addr),
                       (set i8:$o, (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)))]>, IsPtrInstr, CB;

def RESBITPTR : FormH<0x86, (outs), (ins memri:$addr, i8imm:$rd), "res  $rd, $addr",
                      [(store (i8 (and (i8 (load addr:$addr)), iobitposn8:$rd)), addr:$addr)]>, IsPtrInstr, CB;

let Constraints = "$dst = $rr" in
def RESBIT : FormG<0x80, (outs GPR8:$dst), (ins GPR8:$rr, i16imm:$rd), "res  $rd, $rr", []>, CB;

let Constraints = "$dst = $rr" in
def SETBIT : FormG<0xC0, (outs GPR8:$dst), (ins GPR8:$rr, i16imm:$rd), "set  $rd, $rr", []>, CB;

let Constraints = "$dst = $rr", Defs = [SREG] in
def TESTBIT : FormG<0x40, (outs GPR8:$dst), (ins GPR8:$rr, i8imm:$rd), "bit  $rd, $rr", []>, CB;

//def : Pat<(i16 (and i16:$rr, iobitposn16h:$rd)),
//            (INSERT_SUBREG
//                (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (RESBIT (EXTRACT_SUBREG $rr, sub_hi), (i16 iobitposn16h:$rd) ), sub_hi),
//                (i8 (EXTRACT_SUBREG $rr, sub_lo)),
//                sub_lo)>;

let Constraints = "$src = $rd",
Defs = [SREG] in
{
  // Register-Register logic instructions (which have the
  // property of commutativity).
  let isCommutable = 0 in
  {
      def ORrr8 : FormB<0xb0, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "or   $rd, $rr",
                       [(set i8:$rd, (or i8:$src, i8:$rr)), (implicit SREG)]>;

      def ORimm8 : FormI<0xf6, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "or   a, $d8",
                         [(set i8:$rd, (or i8:$src, imm:$d8)), (implicit SREG)]>;

      def XORrr8 : FormB<0xA8, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "xor  $rd, $rr",
                         [(set i8:$rd, (xor i8:$src, i8:$rr)), (implicit SREG)]>;

      def XORimm8 : FormI<0xee, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "xor  a, $d8",
                         [(set i8:$rd, (xor i8:$src, imm:$d8)), (implicit SREG)]>;

      def ANDrr8 : FormB<0xa0, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "and  $rd, $rr",
                         [(set i8:$rd, (and i8:$src, i8:$rr)), (implicit SREG)]>;

      def ANDimm8 : FormI<0xe6, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "and  a, $d8",
                         [(set i8:$rd, (and i8:$src, imm:$d8)), (implicit SREG)]>;

      let rr=6 in
      {
        def ORrr8PTR : FormB<0xb0, (outs ACC:$rd), (ins ACC:$src, memri:$addr), "or   $rd, $addr",
                             [(set i8:$rd, (or i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;

        def XORrr8PTR : FormB<0xa8, (outs ACC:$rd), (ins ACC:$src, memri:$addr), "xor  $rd, $addr",
                              [(set i8:$rd, (xor i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;

        def ANDrr8PTR : FormB<0xa0, (outs ACC:$rd), (ins ACC:$src, memri:$addr), "and  $rd, $addr",
                              [(set i8:$rd, (and i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;
      }
  } // let isCommutable = 0 in
} // let Constraints = "$src = $rd", Defs = [SREG] in

let Constraints = "$src = $rd" in
{
  def ANDIWRdK_RESBIT : Pseudo<(outs DREGS:$rd), (ins DREGS:$src, i16imm:$k), "andiw $rd, $k",
                               [(set i16:$rd, (and i16:$src, iobitposn16:$k))]>;

  def ORIWRdK_SETBIT : Pseudo<(outs DREGS:$rd), (ins DREGS:$src, i16imm:$k), "oriw $rd, $k",
                              [(set i16:$rd, (or i16:$src, iobitpos16:$k))]>;
}

////===----------------------------------------------------------------------===//
//// One's/Two's Complement
////===----------------------------------------------------------------------===//
//let Constraints = "$src = $rd",
//Defs = [SREG] in
//{
//  def COMRd : FRd<0b1001,
//                  0b0100000,
//                  (outs GPR8:$rd),
//                  (ins GPR8:$src),
//                  "com\t$rd",
//                  [(set i8:$rd, (not i8:$src)), (implicit SREG)]>;
//
//  // COMW Rd+1:Rd
//  //
//  // Expands to:
//  // com Rd
//  // com Rd+1
//  def COMWRd : Pseudo<(outs DREGS:$rd),
//                      (ins DREGS:$src),
//                      "comw\t$rd",
//                      [(set i16:$rd, (not i16:$src)), (implicit SREG)]>;
//
//  //:TODO: optimize NEG for wider types
//  def NEGRd : FRd<0b1001,
//                  0b0100001,
//                  (outs GPR8:$rd),
//                  (ins GPR8:$src),
//                  "neg\t$rd",
//                  [(set i8:$rd, (ineg i8:$src)), (implicit SREG)]>;
//}
//
//// TST Rd
//// Test for zero of minus.
//// This operation is identical to a `Rd AND Rd`.
//def : InstAlias<"tst\t$rd", (ANDRdRr GPR8:$rd, GPR8:$rd)>;
//
//// SBR Rd, K
////
//// Mnemonic alias to 'ORI Rd, K'. Same bit pattern, same operands,
//// same everything.
//def : InstAlias<"sbr\t$rd, $k",
//                (ORIRdK LD8:$rd, imm_ldi8:$k),
//                /* Disable display, so we don't override ORI */ 0>;

//===----------------------------------------------------------------------===//
// Jump instructions
//===----------------------------------------------------------------------===//
let isBarrier = 1,
isBranch = 1,
isTerminator = 1 in
{
  def JRk : FormI<0x18, (outs), (ins relbrtarget:$d8), "jr   $d8", [(br bb:$d8)]>;
  def JMPk : FormF<0xc3, (outs), (ins call_target:$d16), "jp   $d16", []>;
}

//===----------------------------------------------------------------------===//
// Call instructions
//===----------------------------------------------------------------------===//
let isCall = 1 in
{
  // SP is marked as a use to prevent stack-pointer assignments that appear
  // immediately before calls from potentially appearing dead.
  let Uses = [SP, HL] in
  def ICALL : FormF<0xcd, (outs), (ins call_target:$d16), "call $d16", []>;

  // SP is marked as a use to prevent stack-pointer assignments that appear
  // immediately before calls from potentially appearing dead.
  let Uses = [SP] in
  def CALLk : FormF<0xcd, (outs), (ins call_target:$d16), "call $d16", [(Z80call imm:$d16)]>;
}

////===----------------------------------------------------------------------===//
//// Return instructions.
////===----------------------------------------------------------------------===//
let isTerminator = 1,
isReturn = 1,
isBarrier = 1 in
{
    def RET : FormA<0xc9, (outs), (ins), "ret", [(Z80retflag)]>;
    def RETI : FormA<0x4d, (outs), (ins), "reti", [(Z80retiflag)]>, ED;
}

//===----------------------------------------------------------------------===//
// Compare operations.
//===----------------------------------------------------------------------===//
let Defs = [SREG], Uses = [SREG] in
def CPIMPLICIT : Pseudo<(outs), (ins GPR8:$rr), "cp", []>;

let Defs = [SREG] in
{
    def CPRr8 : FormB<0xb8, (outs), (ins ACC:$rd, GPR8:$rr), "cp   $rd, $rr", []>;

    def CPImm8 : FormI<0xfe, (outs), (ins ACC:$rd, imm_ldi8:$d8), "cp   $rd, $d8", []>;

    let Constraints = "$dst = $src" in
    def CPWRdRr : Pseudo<(outs HLREG:$dst), (ins HLREG:$src, DREGS:$src2), "!cpw $src, $src2", []>;
}

def : Pat<(Z80cp i16:$v1, i16:$v2), (CPWRdRr $v1, $v2)>;

//===----------------------------------------------------------------------===//
// PC-relative conditional branches
//===----------------------------------------------------------------------===//
def jrcondcode : PatLeaf<(timm), [{
    return isUInt<2>(N->getZExtValue());
}]>;

let isBranch = 1,
isTerminator = 1,
Uses = [SREG] in
{
  def JRCC : FormJ<0x20, (outs), (ins relbrtarget:$d8, condcode:$cc), "jr   $cc, $d8", [(Z80brcond bb:$d8, jrcondcode:$cc)]>;
  def JPCC : FormL<0xc2, (outs), (ins call_target:$d16, condcode:$cc), "jp   $cc, $d16", []>;

  let usesCustomInserter = 1, Uses = [SREG] in
  def BRCOND : Pseudo<(outs), (ins variable_ops), "#BRCOND", []>;
}

//===----------------------------------------------------------------------===//
// Data transfer instructions
//===----------------------------------------------------------------------===//
// 8 and 16-bit register move instructions.
let hasSideEffects = 0 in
{
    def LDRdRr8 : FormG<0x40, (outs GPR8ANDXY:$rd), (ins GPR8ANDXY:$rr), "ld   $rd, $rr", []>;

    def LDSP : FormA<0xf9, (outs SPREG:$rd), (ins HLIXIYREGS:$rr), "ld   sp, $rr", []>;
}

// Load immediate values into registers.
let isReMaterializable = 1 in
{
  def LDI16: FormE<0x01, (outs LDITARGETREGS:$rd), (ins i16imm:$d16), "ld   $rd, $d16", []>;

  def LDIRdK : FormD<0x06, (outs GPR8ANDXY:$rd), (ins imm_ldi8:$d8), "ld   $rd, $d8",
       [(set i8:$rd, imm:$d8)]>;

  def LDIWRdK : Pseudo<(outs LDITARGETREGS:$dst), (ins i16imm:$src), "LDIWRdK\t$dst, $src",
                       [(set i16:$dst, imm:$src)]>;
}

//// Load from data space into register.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDSRdK : FormF<0x3a, (outs ACC:$rd), (ins imm16:$d16), "ld   $rd, ( $d16 )",
                     [(set i8:$rd, (load imm:$d16))]>;

  //def LD16MEMPTR : FormE<0x4b, (outs DREGS:$rd), (ins imm16:$ptr), "ld   $rd, ($ptr)", []>, ED;

  def LDSWRdK : Pseudo<(outs DREGS:$dst), (ins i16imm:$src), "ldsw\t$dst, $src",
                       [(set i16:$dst, (load imm:$src))]>;
}

// Indirect loads.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDRdPtr : FormH<0x46, (outs GPR8:$rd), (ins HLIXIYREGS:$ptrreg), "ld   $rd, ($ptrreg )",
                      [(set i8:$rd, (load i16:$ptrreg))]>;

  let Constraints = "@earlyclobber $reg" in
  def LDWRdPtr : Pseudo<(outs DREGS:$reg), (ins PTRDISPREGS:$ptrreg), "ldw  $reg, ($ptrreg)",
                        [(set i16:$reg, (load i16:$ptrreg))]>;
}

// Load indirect with displacement operations.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{

  let Constraints = "@earlyclobber $rd" in
  def LDDRdPtrQ : FormH<0x46, (outs GPR8:$rd), (ins memri:$d8), "ld   $rd, $d8",
                         [(set i8:$rd, (load addr:$d8))]>, IsPtrInstr;

  let Constraints = "@earlyclobber $dst" in
  def LDDWRdPtrQ : Pseudo<(outs DREGS:$dst), (ins memri:$memri), "lddw $dst, $memri",
                           [(set i16:$dst, (load addr:$memri))]>;

  let mayLoad = 1,
  hasSideEffects = 0 in
  def LDDWRdYQ : Pseudo<(outs DREGS:$dst), (ins memri:$memri), "lddw\t$dst, $memri", []>;
}

// Indirect store from register to data space.
def STSKRr : FormF<0x32, (outs), (ins imm16:$d16, ACC:$rd), "ld   ($d16 ), $rd", [(store i8:$rd, imm:$d16)]>;

def STSWKRr : Pseudo<(outs), (ins i16imm:$dst, DREGS:$src), "stsw\t$dst, $src", [(store i16:$src, imm:$dst)]>;

def OUT16K : FormH<0x41, (outs), (ins BCREG:$port, GPR8:$rd), "out  (c), $rd", [(outport i8:$rd, i16:$port)]>, ED;
def OUT8K  : FormI<0xd3, (outs), (ins i16imm:$d8, ACC:$rd), "out  ($d8 ), $rd", [(outport i8:$rd, uimm8:$d8)]>;
def OUT16Z : FormA<0x71, (outs), (ins BCREG:$port, i8imm:$imm), "out  (c), $imm", [(outport immzero:$imm, i16:$port)]>, ED;

let Defs = [SREG] in
def IN16K : FormH<0x40, (outs GPR8:$rd), (ins BCREG:$port), "in   $rd, (c)", [(set i8:$rd, (inport i16:$port))]>, ED;

def IN8K  : FormI<0xDB, (outs ACC:$rd), (ins i16imm:$d8), "in   $rd, ($d8 )", [(set i8:$rd, (inport uimm8:$d8))]>;

// Indirect stores.
def STPtrRr : FormB<0x70, (outs), (ins HLIXIYREGS:$ptrreg, GPR8:$rr), "ld   ($ptrreg ), $rr",
                    [(store GPR8:$rr, i16:$ptrreg)]>;

def STWPtrRr : Pseudo<(outs), (ins PTRDISPREGS:$ptrreg, DREGS:$reg), "stw  $ptrreg, $reg",
                     [(store i16:$reg, i16:$ptrreg)]>;

def STDPtrQRr : FormB<0x70, (outs), (ins memri:$d8, GPR8:$rr), "ld   $d8, $rr",
                      [(store i8:$rr, addr:$d8)]>, IsPtrInstr;

def STDWPtrQRr : Pseudo<(outs), (ins memri:$memri, DREGS:$src), "stdw\t$memri, $src",
                        [(store i16:$src, addr:$memri)]>;

// Stack push/pop operations.
let Defs = [SP],
Uses = [SP],
hasSideEffects = 0 in
{
  // Stack push operations.
  let mayStore = 1 in
    def PUSHRr : FormC<0b11000101, (outs), (ins DREGSTOSTACK:$rr), "push $rr", []>;

  // Stack pop operations.
  let mayLoad = 1 in
    def POPRd : FormC<0b11000001, (outs DREGSTOSTACK:$rr), (ins), "pop  $rr", []>;
}

////===----------------------------------------------------------------------===//
//// Bit and bit-test instructions
////===----------------------------------------------------------------------===//
def singlerotshift: PatFrag<(ops node:$src, node:$cc), (Z80rotshift node:$src, node:$cc, (i8 1))>;

// Bit shift/rotate operations.
let Constraints = "$src = $rr",
Defs = [SREG] in
{
  let usesCustomInserter = 1 in
  def ROT8LOOP : Pseudo<(outs GPR8:$rr), (ins GPR8:$src, Operand<i8>:$rot, ACC:$amt), "#ROT8 $rr, $rot, $amt",
                        [(set i8:$rr, (Z80rotshift i8:$src, rot:$rot, i8:$amt)), (implicit SREG)]>;

  let usesCustomInserter = 1 in
  def ROT16LOOP : Pseudo<(outs DREGS:$rr), (ins DREGS:$src, Operand<i8>:$rot, ACC:$amt), "#ROT16 $rr, $rot, $amt", []>;

  def ROT16 : Pseudo<(outs DREGS:$rr), (ins DREGS:$src, Operand<i8>:$rot), "#ROT16 $rr, $rot",
                     [(set i16:$rr, (Z80rotshift i16:$src, rot:$rot, (i8 1))), (implicit SREG)]>;

  def SRLRd : FormB<0x38, (outs GPR8:$rr), (ins GPR8:$src), "srl  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SRL)), (implicit SREG)]>, CB;

  def SRARd : FormB<0x28, (outs GPR8:$rr), (ins GPR8:$src), "sra  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SRA)), (implicit SREG)]>, CB;

  def RLCRd : FormB<0x00, (outs GPR8:$rr), (ins GPR8:$src), "rlc  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RLC)), (implicit SREG)]>, CB;

  def RRCRd : FormB<0x08, (outs GPR8:$rr), (ins GPR8:$src), "rrc  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RRC)), (implicit SREG)]>, CB;

  def SLARd : FormB<0x20, (outs GPR8:$rr), (ins GPR8:$src), "sla  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SLA)), (implicit SREG)]>, CB;

  let Uses = [SREG] in
  {
    def RRRd : FormB<0x18, (outs GPR8:$rr), (ins GPR8:$src), "rr   $rr",
                     [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RR)), (implicit SREG)]>, CB;

    def RLRd : FormB<0x10, (outs GPR8:$rr), (ins GPR8:$src), "rl   $rr",
                     [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RL)), (implicit SREG)]>, CB;
  }
}

def : Pat<(Z80rotshift i16:$src, rot:$rot, i16:$amt), (ROT16LOOP $src, $rot, (EXTRACT_SUBREG $amt, sub_lo))>;
def : Pat<(Z80rotshift i16:$src, rot:$rot, i8:$amt), (ROT16LOOP $src, $rot, $amt)>;


//def LDRdIXoffs : FormD<0x46, (outs GPR8:$rd), (ins memri:$d8), "ld   $rd, ($d8)",
//                         [(set i8:$rd, (load addr:$d8))]>;

//def SRLRdPTR : FormB<0x38, (outs), (ins memri:$addr), "srl  $addr",
//                     [(store (i8 (srl (i8 (load addr:$addr)), 0)), addr:$addr)]>, CB;
//                     [(store (Z80rotshift (i8 (load addr:$addr)), Z80_ROT_SRL), addr:$addr)]>, CB;

//def : Pat<(store (Z80rotshift (i8 (load addr:$addr)), Z80_ROT_SRL), addr:$addr),
//          (SRLRdPTR addr:$addr)>;

//// CLR Rd
//// Alias for EOR Rd, Rd
//// -------------
//// Clears all bits in a register.
//def CLR : InstAlias<"clr\t$rd", (EORRdRr GPR8:$rd, GPR8:$rd)>;
//
//// LSL Rd
//// Alias for ADD Rd, Rd
//// --------------
//// Logical shift left one bit.
//def LSL : InstAlias<"lsl\t$rd", (ADDRdRr GPR8:$rd, GPR8:$rd)>;
//
//def ROL : InstAlias<"rol\t$rd", (ADCRdRr GPR8:$rd, GPR8:$rd)>;
//
//// SER Rd
//// Alias for LDI Rd, 0xff
//// ---------
//// Sets all bits in a register.
//def : InstAlias<"ser\t$rd", (LDIRdK LD8:$rd, 0xff), 0>;
//
//let Defs = [SREG] in
//def BSETs : FS<0,
//               (outs),
//               (ins i8imm:$s),
//               "bset\t$s",
//               []>;
//
//let Defs = [SREG] in
//def BCLRs : FS<1,
//               (outs),
//               (ins i8imm:$s),
//               "bclr\t$s",
//               []>;
//
//// Set/clear aliases for the carry (C) status flag (bit 0).
//def : InstAlias<"sec", (BSETs 0)>;
//def : InstAlias<"clc", (BCLRs 0)>;
//
//// Set/clear aliases for the zero (Z) status flag (bit 1).
//def : InstAlias<"sez", (BSETs 1)>;
//def : InstAlias<"clz", (BCLRs 1)>;
//
//// Set/clear aliases for the negative (N) status flag (bit 2).
//def : InstAlias<"sen", (BSETs 2)>;
//def : InstAlias<"cln", (BCLRs 2)>;
//
//// Set/clear aliases for the overflow (V) status flag (bit 3).
//def : InstAlias<"sev", (BSETs 3)>;
//def : InstAlias<"clv", (BCLRs 3)>;
//
//// Set/clear aliases for the signed (S) status flag (bit 4).
//def : InstAlias<"ses", (BSETs 4)>;
//def : InstAlias<"cls", (BCLRs 4)>;
//
//// Set/clear aliases for the half-carry (H) status flag (bit 5).
//def : InstAlias<"seh", (BSETs 5)>;
//def : InstAlias<"clh", (BCLRs 5)>;
//
//// Set/clear aliases for the T status flag (bit 6).
//def : InstAlias<"set", (BSETs 6)>;
//def : InstAlias<"clt", (BCLRs 6)>;
//
//// Set/clear aliases for the interrupt (I) status flag (bit 7).
//def : InstAlias<"sei", (BSETs 7)>;
//def : InstAlias<"cli", (BCLRs 7)>;

let Defs = [SREG] in
def CLEARC : FormA<0xb7, (outs), (ins), "or   a, a", []>;

//===----------------------------------------------------------------------===//
// Special/Control instructions
//===----------------------------------------------------------------------===//

def NOP : FormA<0, (outs), (ins), "nop", []>;
def HALT : FormA<0x76, (outs), (ins), "halt", []>;

//===----------------------------------------------------------------------===//
// Pseudo instructions for later expansion
//===----------------------------------------------------------------------===//

//:TODO: Optimize this for wider types AND optimize the following code
//       compile int foo(char a, char b, char c, char d) {return d+b;}
//       looks like a missed sext_inreg opportunity.
def SEXT : ExtensionPseudo<
  (outs DREGS:$dst),
  (ins GPR8:$src),
  "sext\t$dst, $src",
  [(set i16:$dst, (sext i8:$src)), (implicit SREG)]
>;

def ZEXT : ExtensionPseudo<
  (outs DREGS:$dst),
  (ins GPR8:$src),
  "zext\t$dst, $src",
  [(set i16:$dst, (zext i8:$src)), (implicit SREG)]
>;

// This pseudo gets expanded into a movw+adiw thus it clobbers SREG.
let Defs = [SREG],
    hasSideEffects = 0 in
def FRMIDX : Pseudo<(outs HLIXIYREGS:$dst),
                    (ins HLIXIYREGS:$src, i16imm:$src2),
                    "frmidx\t$dst, $src, $src2",
                    []>;

// This pseudo is either converted to a regular store or a push which clobbers
// SP.
def STDSPQRr : StorePseudo<(outs), (ins memspi:$dst, GPR8:$src), "stdstk\t$dst, $src",
                           [(store i8:$src, addr:$dst)]>;

// This pseudo is either converted to a regular store or a push which clobbers
// SP.
def STDWSPQRr : StorePseudo<(outs), (ins memri:$dst, DREGS:$src), "stdwstk\t$dst, $src",
                            [(store i16:$src, addr:$dst)]>;

def Select8 : SelectPseudo<(outs GPR8:$dst), (ins GPR8:$src, GPR8:$src2, variable_ops), "# Select8 PSEUDO", []>;
def Select16 : SelectPseudo<(outs DREGS:$dst), (ins DREGS:$src, DREGS:$src2, variable_ops), "# Select16 PSEUDO", []>;

//===----------------------------------------------------------------------===//
// Non-Instruction Patterns
//===----------------------------------------------------------------------===//

//:TODO: look in x86InstrCompiler.td for odd encoding trick related to
// add x, 128 -> sub x, -128. Clang is emitting an eor for this (ldi+eor)

// the add instruction always writes the carry flag
def : Pat<(addc i8:$src, i8:$src2),
          (ADDRdRr8 i8:$src, i8:$src2)>;
def : Pat<(addc DREGS:$src, DREGS:$src2),
          (ADDRdRr16 DREGS:$src, DREGS:$src2)>;

//// all sub instruction variants always writes the carry flag
//def : Pat<(subc i8:$src, i8:$src2),
//          (SUBRdRr i8:$src, i8:$src2)>;
//def : Pat<(subc i16:$src, i16:$src2),
//          (SUBWRdRr i16:$src, i16:$src2)>;
//def : Pat<(subc i8:$src, imm:$src2),
//          (SUBIRdK i8:$src, imm:$src2)>;
//def : Pat<(subc i16:$src, imm:$src2),
//          (SUBIWRdK i16:$src, imm:$src2)>;
//
//// These patterns convert add (x, -imm) to sub (x, imm) since we dont have
//// any add with imm instructions. Also take care of the adiw/sbiw instructions.
//def : Pat<(add i16:$src1, imm0_63_neg:$src2),
//          (SBIWRdK i16:$src1, (imm0_63_neg:$src2))>;
//def : Pat<(add i16:$src1, imm:$src2),
//          (SUBIWRdK i16:$src1, (imm16_neg_XFORM imm:$src2))>;
//def : Pat<(addc i16:$src1, imm:$src2),
//          (SUBIWRdK i16:$src1, (imm16_neg_XFORM imm:$src2))>;

//def : Pat<(add i8:$src1, imm:$src2),
//          (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
//def : Pat<(addc i8:$src1, imm:$src2),
//          (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
//def : Pat<(adde i8:$src1, imm:$src2),
//          (SBCIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;

// Calls.
def : Pat<(Z80call (i16 tglobaladdr:$dst)), (CALLk tglobaladdr:$dst)>;
def : Pat<(Z80call (i16 texternalsym:$dst)), (CALLk texternalsym:$dst)>;

class Ex16Pat<SDNode Op, Instruction I>
    : Pat<(i16 (Op DREGS:$a, DREGS:$b)),
          (INSERT_SUBREG
                         (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                        (I (EXTRACT_SUBREG $a, sub_hi),
                                           (EXTRACT_SUBREG $b, sub_hi)),
                                        sub_hi),
                         (I (EXTRACT_SUBREG $a, sub_lo),
                            (EXTRACT_SUBREG $b, sub_lo)),
                         sub_lo)>;

def : Ex16Pat<or, ORrr8>;
def : Ex16Pat<and, ANDrr8>;
def : Ex16Pat<xor, XORrr8>;

// `anyext`
def : Pat<(i16 (anyext i8:$src)),
          (INSERT_SUBREG (i16 (IMPLICIT_DEF)), i8:$src, sub_lo)>;

// `trunc`
def : Pat<(i8 (trunc i16:$src)),
          (EXTRACT_SUBREG i16:$src, sub_lo)>;

// sext_inreg
def : Pat<(sext_inreg i16:$src, i8),
          (SEXT (i8 (EXTRACT_SUBREG i16:$src, sub_lo)))>;

// GlobalAddress
def : Pat<(i16 (Z80Wrapper tglobaladdr:$dst)),
          (LDIWRdK tglobaladdr:$dst)>;
//def : Pat<(add i16:$src, (Z80Wrapper tglobaladdr:$src2)),
//          (SUBIWRdK i16:$src, tglobaladdr:$src2)>;
def : Pat<(i8 (load (Z80Wrapper tglobaladdr:$dst))),
          (LDSRdK tglobaladdr:$dst)>;
def : Pat<(i16 (load (Z80Wrapper tglobaladdr:$dst))),
          (LDSWRdK tglobaladdr:$dst)>;
def : Pat<(i16 (load (Z80Wrapper tglobaladdr:$dst))),
          (LDSWRdK tglobaladdr:$dst)>;
def : Pat<(store i8:$src, (i16 (Z80Wrapper tglobaladdr:$dst))),
          (STSKRr tglobaladdr:$dst, i8:$src)>;
def : Pat<(store i16:$src, (i16 (Z80Wrapper tglobaladdr:$dst))),
          (STSWKRr tglobaladdr:$dst, i16:$src)>;

// BlockAddress
def : Pat<(i16 (Z80Wrapper tblockaddress:$dst)),
          (LDIWRdK tblockaddress:$dst)>;

//// hi-reg truncation : trunc(int16 >> 8)
////:FIXME: i think it's better to emit an extract subreg node in the DAG than
//// all this mess once we get optimal shift code
//// lol... I think so, too. [@agnat]
//def : Pat<(i8 (trunc (Z80lsr (Z80lsr (Z80lsr (Z80lsr (Z80lsr (Z80lsr (Z80lsr
//                     (Z80lsr DREGS:$src)))))))))),
//          (EXTRACT_SUBREG DREGS:$src, sub_hi)>;
//
//// :FIXME: DAGCombiner produces an shl node after legalization from these seq:
//// BR_JT -> (mul x, 2) -> (shl x, 1)
//def : Pat<(shl i16:$src1, (i8 1)),
//          (LSLWRd i16:$src1)>;
//
//// Lowering of 'tst' node to 'TST' instruction.
//// TST is an alias of AND Rd, Rd.
//def : Pat<(Z80tst i8:$rd),
//          (ANDRdRr GPR8:$rd, GPR8:$rd)>;
//
//// Lowering of 'lsl' node to 'LSL' instruction.
//// LSL is an alias of 'ADD Rd, Rd'
//def : Pat<(Z80lsl i8:$rd),
//          (ADDRdRr GPR8:$rd, GPR8:$rd)>;
//

def : Pat<(Z80cps i8:$rd, (i8 1)), (DECRd $rd)>;
def : Pat<(Z80cps i8:$rd, (i8 imm:$rr)), (SUBRdK $rd, imm:$rr)>;
def : Pat<(Z80cps i8:$rd, i8:$rr), (SUBRdRr8 $rd, $rr)>;
def : Pat<(Z80cps i16:$rd, i16:$rr), (SUBRdRr16 $rd, $rr)>;

def : Pat<(Z80cp i8:$rd, i8:$rr),(CPRr8 $rd, $rr)>;
def : Pat<(Z80cp i8:$rd, imm:$d8),(CPImm8 $rd, $d8)>;

def : Pat<(Z80cp (add i8:$v, -1), 0), (CPIMPLICIT (DECRd $v))>;
def : Pat<(Z80cp (sub i8:$v, 1),  0), (CPIMPLICIT (DECRd $v))>;

def : Pat<(Z80cp (and i8:$v, iobitposn8:$a),  0), (CPIMPLICIT (TESTBIT $v, iobitposn8:$a))>;

//def : Pat<(Z80cp (and i8:$v, imm:$a),  0), (CPIMPLICIT (ANDimm8 $v, imm:$a))>;
//def : Pat<(Z80cp (or i8:$v, imm:$a),  0), (CPIMPLICIT (ORimm8 $v, imm:$a))>;

def : Pat<(Z80cp i8:$rd, 0), (ORrr8 $rd, $rd)>;
def : Pat<(Z80cp i16:$rd, 0), (ORrr8 (EXTRACT_SUBREG $rd, sub_lo), (EXTRACT_SUBREG $rd, sub_hi))>;

//def : Pat<(add i16:$a, (zext i8:$b)), (ADDWRdRr $a,
// (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (LDIRdK (i8 0)), sub_hi), $b, sub_lo))>;

// (char)(v >> 8)
// (char)(v & 255)
def : Pat<(i8 (trunc (srl i16:$v, (i8 8)))),  (i8 (EXTRACT_SUBREG $v, sub_hi))>;
def : Pat<(i8 (trunc (and i16:$v, (i16 255)))),  (i8 (EXTRACT_SUBREG $v, sub_lo))>;

// v & 255
// v % 256
def : Pat<(i16 (and i16:$v, (i16 255))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (LDIRdK (i8 0)), sub_hi), (i8 (EXTRACT_SUBREG $v, sub_lo)), sub_lo))>;

// (i16)((i32)v >> 8)
def : Pat<(i16 (or (srl i16:$a, (i8 8)), (shl i16:$b, (i8 8)))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (i8 (EXTRACT_SUBREG $a, sub_hi)), sub_lo),
                                    (i8 (EXTRACT_SUBREG $b, sub_lo)), sub_hi))>;

// v << 8
def : Pat<(i16 (shl i16:$v, (i8 8))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (i8 (EXTRACT_SUBREG $v, sub_hi)), sub_hi), (LDIRdK (i8 0)), sub_lo))>;

// (v1 << 8) | v2
def : Pat<(i16 (or (shl (i16 (anyext i8:$v1)), (i8 8)), (i16 (zext i8:$v2)))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), $v1, sub_hi), $v2, sub_lo))>;


def : Pat<(i16 (zext (i8 (load addr:$a)))),
           (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (i8 (LDIRdK (i8 0))), sub_hi),
                                                 (LDDRdPtrQ addr:$a), sub_lo))>;

