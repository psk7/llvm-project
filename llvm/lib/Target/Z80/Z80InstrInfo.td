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
def SDT_Z80Tsts : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_Z80SelectCC : SDTypeProfile<1, 4, [SDTCisSameAs<0, 1>,
                                    SDTCisSameAs<1, 2>, SDTCisVT<3, i8>, SDTCisVT<4, i8>]>;
def SDT_Z80Outport : SDTypeProfile<0, 3, [SDTCisVT<0, i8>, SDTCisVT<1, i16>, SDTCisVT<2, i8>]>;
def SDT_Z80Inport : SDTypeProfile<1, 2, [ SDTCisPtrTy<1> ]>;
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
def Z80tsts : SDNode<"Z80ISD::TSTS", SDT_Z80Tst, [SDNPOutGlue]>;
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
def Z80_ROT_SLI : PatLeaf<(i8 6)>;
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

def uimm8 : PatLeaf<(imm), [{ return isUInt<8>(N->getZExtValue()); }]>;
def immzero : PatLeaf<(imm), [{ return N->getZExtValue() == 0; }]>;

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

def iobitpos8 : PatLeaf<(imm), [{ return isPowerOf2_32(uint8_t(N->getZExtValue())); }], iobitpos8_XFORM>;
def iobitposn8 : PatLeaf<(imm), [{ return isPowerOf2_32(uint8_t(~N->getZExtValue())); }], iobitposn8_XFORM>;
def iobitpos16 : PatLeaf<(imm), [{ return isPowerOf2_32(uint16_t(N->getZExtValue())); }], iobitpos16_XFORM>;
def iobitposn16 : PatLeaf<(imm), [{ return isPowerOf2_32(uint16_t(~N->getZExtValue())); }], iobitposn16_XFORM>;

def as_i16timm : SDNodeXForm<imm, [{ return CurDAG->getTargetConstant(N->getSExtValue(), SDLoc(N), MVT::i16); }]>;
def as_i8timm : SDNodeXForm<imm, [{ return CurDAG->getTargetConstant(N->getSExtValue(), SDLoc(N), MVT::i8); }]>;
def as_minus8_timm : SDNodeXForm<imm, [{ return CurDAG->getTargetConstant(N->getSExtValue() - 8, SDLoc(N), MVT::i8); }]>;

def jrcondcode : PatLeaf<(timm), [{ return isUInt<2>(N->getZExtValue()); }]>;

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

def addr8: Operand<i16>
{
    let MIOperandInfo = (ops i16imm);

    let PrintMethod = "printAddr8";
}

def regref: Operand<i16>
{
    let MIOperandInfo = (ops PTRREGS, i8imm);

    let DecoderMethod = "DecodeRegref";
    let PrintMethod = "printRegref";
}

def rstnum: Operand<i8>
{
    let PrintMethod = "printRST";
}

/// Address operand for `reg+0` used by STD and LDD.
def memriz : Operand<iPTR>
{
  let MIOperandInfo = (ops PTRREGS);

  let PrintMethod = "printMemriz";
  //let EncoderMethod = "encodeMemri";

  let ParserMatchClass = MemriAsmOperand;
}

def rottype : Operand<i8>
{
  let MIOperandInfo = (ops i8imm);
}

def relbrtarget : Operand<OtherVT>
{
    let PrintMethod   = "printPCRelImm";
    let EncoderMethod = "encodeRelCondBrTarget<Z80::fixup_8_pcrel>";
    let DecoderMethod = "decodeRelTarget";
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
    let DecoderMethod = "DecodeImm";
}

/// An 8-bit immediate inside an instruction with the same format
/// as the `LDI` instruction (the `FRdK` format).
def imm_ldi8 : Operand<i8>
{
    //let EncoderMethod = "encodeImm<Z80::fixup_ldi, 0>";
}

// Addressing mode pattern reg+imm8
def addr : ComplexPattern<iPTR, 2, "SelectAddr", [], [SDNPWantRoot]>;

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
    def ADD : FormB<0x80, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "add  $rr",
                       [(set i8:$rd, (add i8:$src, i8:$rr)), (implicit SREG)]>;

    let DecoderMethod="DecodeADDPTR" in
    def ADDPTR : FormA<0x86, (outs ACC:$rd), (ins ACC:$src, regref:$ptr), "add  $ptr", []>;

    let DecoderMethod = "DecodeAAk" in
    def ADDk : FormI<0xC6,
                       (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "add  $d8",
                       [(set i8:$rd, (add i8:$src, imm:$d8)), (implicit SREG)]>;

    def ADDW : FormHLRP<0x9, "add  $rd, $rr",
                       [(set i16:$rd, (add i16:$src, i16:$rr)), (implicit SREG)]>;

    def XADDW : Pseudo<(outs PTRREGS:$rd), (ins PTRREGS:$src, BDREGS:$rr), "add  $rd, $rr", []>;

    let Uses = [SREG] in {
       def ADC : FormB<0x88, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "adc  $rr",
                       [(set i8:$rd, (adde i8:$src, i8:$rr)), (implicit SREG)]>;

       let DecoderMethod="DecodeADDPTR" in
       def ADCPTR : FormA<0x8E, (outs ACC:$rd), (ins ACC:$src, regref:$ptr), "adc  $ptr", []>;

       let DecoderMethod = "DecodeAAk" in
       def ADCk : FormI<0xCE, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "adc  $d8",
                       [(set i8:$rd, (adde i8:$src, imm:$d8)), (implicit SREG)]>;

       def ADCW : FormC_ED<0b01001010, (outs HLREG:$rd), (ins HLREG:$src, BDREGS:$rr), "adc  $rd, $rr",
                       [(set i16:$rd, (adde i16:$src, i16:$rr)), (implicit SREG)]>;
    }
}

//===----------------------------------------------------------------------===//
// Subtraction
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd",
Defs = [SREG] in
{
   def SUB : FormB<0x90, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "sub  $rr",
                     [(set i8:$rd, (sub i8:$src, i8:$rr)), (implicit SREG)]>;

   let DecoderMethod="DecodeADDPTR" in
   def SUBPTR : FormA<0x96, (outs ACC:$rd), (ins ACC:$src, regref:$ptr), "sub  $ptr", []>;

   let DecoderMethod = "DecodeAAk" in
   def SUBk : FormI<0xd6,
                     (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "sub  $d8",
                     [(set i8:$rd, (sub i8:$src, imm:$d8)), (implicit SREG)]>;

   def SUBW : Pseudo<(outs HLREG:$rd), (ins HLREG:$src, BDREGS:$rr), "!sub  $rd, $rr",
                     [(set i16:$rd, (sub i16:$src, i16:$rr)), (implicit SREG)]>;

   let Uses = [SREG] in {
      def SBC : FormB<0x98, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "sbc  $rr",
                     [(set i8:$rd, (sube i8:$src, i8:$rr)), (implicit SREG)]>;

      let DecoderMethod="DecodeADDPTR" in
      def SBCPTR : FormA<0x9E, (outs ACC:$rd), (ins ACC:$src, regref:$ptr), "sbc  $ptr", []>;

      let DecoderMethod = "DecodeAAk" in
      def SBCk : FormI<0xde, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "sbc  $d8",
                     [(set i8:$rd, (sube i8:$src, imm:$d8)), (implicit SREG)]>;

      def SBCW : Pseudo<(outs HLREG:$rd), (ins HLREG:$src, BDREGS:$rr), "sbc  $rd, $rr",
                     [(set i16:$rd, (sube i16:$src, i16:$rr)), (implicit SREG)]>;

      def XSBCW : Pseudo<(outs PTRREGS:$rd), (ins PTRREGS:$src, BDREGS:$rr), "sbc  $rd, $rr", []>, ED;
  }
}

def SBCWD : FormC_ED<0x42, (outs HLREG:$rd), (ins HLREG:$src, BDREGS:$rr), "sbc  $rd, $rr", []>;

//===----------------------------------------------------------------------===//
// Increment and Decrement
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd" in
{
  let Defs = [SREG] in
  {
    def INC : FormH<0x04, (outs GPR8:$rd), (ins GPR8:$src), "inc  $rd",
                      [(set i8:$rd, (add i8:$src, 1)), (implicit SREG)]>;

    def DEC : FormH<0x05, (outs GPR8:$rd), (ins GPR8:$src), "dec  $rd",
                      [(set i8:$rd, (add i8:$src, -1)), (implicit SREG)]>;
  }
}

let Constraints = "$src = $rr" in
{
  def INCW : FormC<0x03, (outs DREGS:$rr), (ins DREGS:$src), "inc  $rr", [(set i16:$rr, (add i16:$src, 1))]>;
  def DECW : FormC<0x0b, (outs DREGS:$rr), (ins DREGS:$src), "dec  $rr", [(set i16:$rr, (add i16:$src, -1))]>;
}

def : Pat<(add i16:$src, 2), (INCW (INCW $src))>;
def : Pat<(add i16:$src, 3), (INCW (INCW (INCW $src)))>;
def : Pat<(add i16:$src, 4), (INCW (INCW (INCW (INCW $src))))>;

def : Pat<(add i8:$src, 2), (INC (INC $src))>;
def : Pat<(add i8:$src, 3), (INC (INC (INC $src)))>;
def : Pat<(add i8:$src, 4), (INC (INC (INC (INC $src))))>;

def : Pat<(add i16:$src, -2), (DECW (DECW $src))>;
def : Pat<(add i16:$src, -3), (DECW (DECW (DECW $src)))>;
def : Pat<(add i16:$src, -4), (DECW (DECW (DECW (DECW $src))))>;

def : Pat<(add i8:$src, -2), (DEC (DEC $src))>;
def : Pat<(add i8:$src, -3), (DEC (DEC (DEC $src)))>;
def : Pat<(add i8:$src, -4), (DEC (DEC (DEC (DEC $src))))>;

let Defs = [SREG], DecoderMethod="DecodeSingleRegref" in
{
  def INCPTR : FormA<0x34, (outs), (ins regref:$addr), "inc  $addr",
                      [(store (i8 (add (i8 (load addr:$addr)), 1)), addr:$addr)]>, IsPtrInstr;

  def DECPTR : FormA<0x35, (outs), (ins regref:$addr), "dec  $addr",
                      [(store (i8 (add (i8 (load addr:$addr)), -1)), addr:$addr)]>, IsPtrInstr;
}

//===----------------------------------------------------------------------===//
// Logic
//===----------------------------------------------------------------------===//
let DecoderMethod="DecodeExtBitOpsPtr" in {
    def SETBITPTR : FormH_CB<0xC6, (outs), (ins regref:$addr, i8imm:$rd), "set  $rd, $addr",
                          [(store (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)), addr:$addr)]>, IsPtrInstr;

    def RESBITPTR : FormH_CB<0x86, (outs), (ins regref:$addr, i8imm:$rd), "res  $rd, $addr",
                          [(store (i8 (and (i8 (load addr:$addr)), iobitposn8:$rd)), addr:$addr)]>, IsPtrInstr;

    let Defs = [SREG], mayLoad=1 in
        def TESTBITPTR : FormH_CB<0x46, (outs), (ins regref:$addr, i8imm:$rd), "bit  $rd, $addr", []>, IsPtrInstr;
}

def SETBITWPTR : Pseudo<(outs), (ins memri:$addr, i8imm:$rd), "set  $rd, $addr",
                      [(store (i16 (or (i16 (load addr:$addr)), iobitpos16:$rd)), addr:$addr)]>;

def SETBITPTRE : Pseudo<(outs GPR8:$o), (ins memri:$addr, i8imm:$rd), "ld   $o, set $rd, $addr",
                      [(store (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)), addr:$addr),
                       (set i8:$o, (i8 (or (i8 (load addr:$addr)), iobitpos8:$rd)))]>, IsPtrInstr, CB;

def RESBITWPTR : Pseudo<(outs), (ins memri:$addr, i8imm:$rd), "res  $rd, $addr",
                      [(store (i16 (and (i16 (load addr:$addr)), iobitposn16:$rd)), addr:$addr)]>;

let Defs = [SREG] in
{
  def SCF : FormA<0x37, (outs), (ins), "scf", []>;
  def CCF : FormA<0x3F, (outs), (ins), "ccf", []>;
  def NEG : FormA_ED<0x44, (outs), (ins), "neg", []>;
}

let Constraints = "$src = $rd",
Defs = [SREG] in
{
  let isCommutable = 0 in
  {
      def OR : FormB<0xb0, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "or   $rr",
                       [(set i8:$rd, (or i8:$src, i8:$rr)), (implicit SREG)]>;

      def XOR : FormB<0xA8, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "xor  $rr",
                       [(set i8:$rd, (xor i8:$src, i8:$rr)), (implicit SREG)]>;

      def AND : FormB<0xa0, (outs ACC:$rd), (ins ACC:$src, GPR8:$rr), "and  $rr",
                       [(set i8:$rd, (and i8:$src, i8:$rr)), (implicit SREG)]>;

      let DecoderMethod = "DecodeAAk" in
      {
          def ORk : FormI<0xf6, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "or   $d8",
                           [(set i8:$rd, (or i8:$src, imm:$d8)), (implicit SREG)]>;

          def XORk : FormI<0xee, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "xor  $d8",
                           [(set i8:$rd, (xor i8:$src, imm:$d8)), (implicit SREG)]>;

          def ANDk : FormI<0xe6, (outs ACC:$rd), (ins ACC:$src, i8imm:$d8), "and  $d8",
                           [(set i8:$rd, (and i8:$src, imm:$d8)), (implicit SREG)]>;
      }

        let DecoderMethod = "DecodeADDPTR" in {
            def ORPTR : FormA<0xB6, (outs ACC:$rd), (ins ACC:$src, regref:$addr), "or   $addr",
                           [(set i8:$rd, (or i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;

            def XORPTR : FormA<0xAE, (outs ACC:$rd), (ins ACC:$src, regref:$addr), "xor  $addr",
                           [(set i8:$rd, (xor i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;

            def ANDPTR : FormA<0xA6, (outs ACC:$rd), (ins ACC:$src, regref:$addr), "and  $addr",
                           [(set i8:$rd, (and i8:$src, (i8 (load addr:$addr)))), (implicit SREG)]>, IsPtrInstr;
        }
  } // let isCommutable = 0 in
} // let Constraints = "$src = $rd", Defs = [SREG] in

let Constraints = "$dst = $rr" in {
    def RESBIT : FormG_CB<0x80, (outs BGPR8:$dst), (ins BGPR8:$rr, i16imm:$rd), "res  $rd, $rr", []>;
    def SETBIT : FormG_CB<0xC0, (outs BGPR8:$dst), (ins BGPR8:$rr, i16imm:$rd), "set  $rd, $rr", []>;
}

let Constraints = "$src = $rd" in
{
  def RESBITW : Pseudo<(outs BDREGS:$rd), (ins BDREGS:$src, i16imm:$k), "!res $rd, $k",
                       [(set i16:$rd, (and i16:$src, iobitposn16:$k)), (implicit SREG)]>;

  def SETBITW : Pseudo<(outs BDREGS:$rd), (ins BDREGS:$src, i16imm:$k), "!set $rd, $k",
                       [(set i16:$rd, (or i16:$src, iobitpos16:$k)), (implicit SREG)]>;
}

def SETRESBITWPTR : Pseudo<(outs), (ins memri:$addr, i16imm:$resbit, i16imm:$setbit),
                           "!setres $addr, $resbit, $setbit", []>;

let Defs = [SREG] in {
    //let DecoderMethod="DecodeBITOps" in
    def TESTBIT : FormG_CB<0x40, (outs), (ins BGPR8:$rr, i8imm:$rd), "bit  $rd, $rr", []>;
}

def : Pat<(store (i16 (or (i16 (and (i16 (load addr:$src)), imm:$resbit)), imm:$setbit)), addr:$src),
          (SETRESBITWPTR addr:$src, (as_i16timm $resbit), (as_i16timm $setbit))>;

//===----------------------------------------------------------------------===//
// Jump instructions
//===----------------------------------------------------------------------===//
let isBarrier = 1,
isBranch = 1,
isTerminator = 1 in
{
  def JRk : FormI<0x18, (outs), (ins relbrtarget:$d8), "jr   $d8", [(br bb:$d8)]>;
  def JMPk : FormF<0xc3, (outs), (ins call_target:$d16), "jp   $d16", []>;

  let DecoderMethod = "DecodeJMPHL" in
  def JMPHL : FormA<0xE9, (outs), (ins addr8:$ptr), "jp   $ptr", []>;
}

let isBarrier = 1,
isBranch = 1,
isTerminator = 1,
Uses = [B], Defs = [B] in
{
  def DJNZ : FormI<0x10, (outs), (ins relbrtarget:$d8), "djnz $d8", []>;
}


//===----------------------------------------------------------------------===//
// Call instructions
//===----------------------------------------------------------------------===//
let isCall = 1 in
{
  // SP is marked as a use to prevent stack-pointer assignments that appear
  // immediately before calls from potentially appearing dead.
  let Uses = [SP, HL] in
  def ICALL : Pseudo<(outs), (ins call_target:$d16), "call $d16", []>;

  // SP is marked as a use to prevent stack-pointer assignments that appear
  // immediately before calls from potentially appearing dead.
  let Uses = [SP] in {
    def CALLk : FormF<0xcd, (outs), (ins call_target:$d16), "call $d16", [(Z80call imm:$d16)]>;

    def CALLCCk : FormL<0xc4, (outs), (ins condcode:$cc, call_target:$d16), "call $cc, $d16", []>;

    def RST : FormH<0xc7, (outs), (ins rstnum:$rd), "rst  $rd", []>;
  }
}

////===----------------------------------------------------------------------===//
//// Return instructions
////===----------------------------------------------------------------------===//
let isTerminator = 1,
isReturn = 1,
isBarrier = 1 in
{
    def RET : FormA<0xc9, (outs), (ins), "ret", [(Z80retflag)]>;
    def RETI : FormA_ED<0x4d, (outs), (ins), "reti", [(Z80retiflag)]>;

    def RETCC : FormH<0xC0, (outs), (ins condcode:$rd), "ret  $rd", []>;
}

//===----------------------------------------------------------------------===//
// Compare operations
//===----------------------------------------------------------------------===//
let Defs = [SREG] in
{
    let DecoderMethod = "DecodeCP" in
    def CP : FormB<0xb8, (outs), (ins ACC:$rd, GPR8:$rr), "cp   $rr", []>;

    let DecoderMethod = "DecodeCPPTR" in
    def CPPTR : FormA<0xBE, (outs), (ins ACC:$rd, regref:$addr), "cp   $addr", []>, IsPtrInstr;

    let DecoderMethod = "DecodeAk" in
    def CPImm8 : FormI<0xfe, (outs), (ins ACC:$rd, i8imm:$d8), "cp   $d8", []>;

    let Constraints = "$dst = $src" in
    def CPW : Pseudo<(outs HLREG:$dst), (ins HLREG:$src, BDREGS:$src2), "!cpw $src, $src2", []>;
}

def : Pat<(Z80cp i16:$v1, i16:$v2), (CPW $v1, $v2)>;

//===----------------------------------------------------------------------===//
// PC-relative conditional branches
//===----------------------------------------------------------------------===//
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
let hasSideEffects = 0 in
{
    def LD : Pseudo<(outs GPR8:$rd), (ins GPR8:$rr), "ld   $rd, $rr", []>;

    let DecoderMethod = "DecodeLDSPHL" in
    def LDSP : FormA<0xf9, (outs SPREG:$rd), (ins PTRREGS:$rr), "ld   sp, $rr", []>;
    def COPYREG : Pseudo<(outs GPR8:$d), (ins GPR8:$s), "!copyreg $d, $s", []>;
    def COPYREGW : Pseudo<(outs DREGS:$d), (ins DREGS:$s), "!copyregw $d, $s", []>;
}

// Load immediate values into registers.
let isReMaterializable = 1 in
{
  def LDWk: FormE<0x01, (outs DREGS:$rd), (ins i16imm:$d16), "ld   $rd, $d16", []>;

  def LDk : FormD<0x06, (outs GPR8:$rd), (ins imm_ldi8:$d8), "ld   $rd, $d8",
       [(set i8:$rd, imm:$d8)]>;

  def LDIWRdK : Pseudo<(outs DREGS:$dst), (ins i16imm:$src), "!LDIWRdK\t$dst, $src",
                       [(set i16:$dst, imm:$src)]>;
}

let DecoderMethod = "DecodeLDPTRk" in
def LDPTRk : FormI<0x36, (outs), (ins regref:$rd, imm_ldi8:$d8), "ld   $rd, $d8", []>;

let DecoderMethod = "DecodeLDPTR" in
def LDPTR : FormH<0x46, (outs GPR8:$rd), (ins regref:$ptr), "ld   $rd, $ptr", []>;


def LDWMEM_: FormE<0x2A, (outs HLREG:$rd), (ins addr8:$d16), "ld   $rd, $d16", []>;
def LDWMEM: FormE_ED<0x4B, (outs DREGS:$rd), (ins addr8:$d16), "ld   $rd, $d16", []>;

//// Load from data space into register.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  let DecoderMethod = "DecodeLDAMEM" in
  def LDSRdK : FormF<0x3a, (outs ACC:$rd), (ins addr8:$d16), "ld   $rd, $d16",
                     [(set i8:$rd, (load imm:$d16))]>;

  //def LD16MEMPTR : FormE<0x4b, (outs DREGS:$rd), (ins imm16:$ptr), "ld   $rd, ($ptr)", []>, ED;

  def LDSWRdK : Pseudo<(outs DREGS:$dst), (ins i16imm:$src), "ldsw\t$dst, $src", [(set i16:$dst, (load imm:$src))]>;

  def LDSWRdKOp : Pseudo<(outs HLREG:$rd), (ins i16imm:$d16), "ld   $rd, ( $d16 )", []>;
  def LDSWRdKOpExt : Pseudo<(outs DREGS:$rd), (ins i16imm:$d16), "ld   $rd, ( $d16 )", []>;

  def LDADE : FormA<0x1a, (outs ACC:$rd), (ins DEREG:$rs), "ld   a, (de)", []>;
  def LDABC : FormA<0x0a, (outs ACC:$rd), (ins BCREG:$rs), "ld   a, (bc)", []>;
}

let canFoldAsLoad = 1,
isReMaterializable = 1 in
{
  def LDRdPtr : Pseudo<(outs BGPR8:$rd), (ins memriz:$ptrreg), "ld   $rd, $ptrreg",
                      [(set i8:$rd, (load i16:$ptrreg))]>;

  let Constraints = "@earlyclobber $reg" in
  def LDWRdPtr : Pseudo<(outs DREGS:$reg), (ins PTRDISPREGS:$ptrreg), "ldw  $reg, ($ptrreg)",
                        [(set i16:$reg, (load i16:$ptrreg))]>;
}

let Uses=[A] in {
    def LDIA : FormA_ED<0x47, (outs), (ins), "ld   i, a", []>;
    def LDRA : FormA_ED<0x4F, (outs), (ins), "ld   r, a", []>;
}

let Defs=[A] in {
    def LDAI : FormA_ED<0x57, (outs), (ins), "ld   a, i", []>;
    def LDAR : FormA_ED<0x5F, (outs), (ins), "ld   a, r", []>;
}

// Load indirect with displacement operations.
let canFoldAsLoad = 1,
isReMaterializable = 1 in
{

  let Constraints = "@earlyclobber $rd" in
  def LDDRdPtrQ : Pseudo<(outs BGPR8:$rd), (ins memri:$d8), "ld   $rd, $d8",
                         [(set i8:$rd, (load addr:$d8))]>, IsPtrInstr;

  let Constraints = "@earlyclobber $dst" in
  def LDDWRdPtrQ : Pseudo<(outs DREGS:$dst), (ins memri:$memri), "lddw $dst, $memri",
                           [(set i16:$dst, (load addr:$memri))]>;

  let mayLoad = 1,
  hasSideEffects = 0 in
  def LDDWRdYQ : Pseudo<(outs DREGS:$dst), (ins memri:$memri), "lddw\t$dst, $memri", []>;
}

// Indirect store from register to data space.
def STSKRr : Pseudo<(outs), (ins imm16:$d16, ACC:$rd), "ld   ($d16 ), $rd", [(store i8:$rd, imm:$d16)]>;

let DecoderMethod="DecodeSTSWMEM" in
def STSWMEM : FormF<0x22, (outs), (ins addr8:$d16, HLREG:$rd), "ld   $d16, $rd", [(store i16:$rd, imm:$d16)]>;

//def STSWMEMEXT : FormE<0x43, (outs), (ins imm16:$d16, DREGS:$rd), "ld   ($d16 ), $rd", [(store i16:$rd, imm:$d16)]>, ED;

def STMEMRP : FormSTMEMRP<0x43, (outs), (ins addr8:$d16, BDREGS:$rs), "ld   $d16, $rs", []>;
def STMEMA : FormF<0x32, (outs), (ins addr8:$d16), "ld   $d16, a", []>;
def STDEA : FormA<0x12, (outs), (ins), "ld   (de), a", []>;
def STBCA : FormA<0x2, (outs), (ins), "ld   (bc), a", []>;

def STSWKRr : Pseudo<(outs), (ins i16imm:$dst, DREGS:$src), "stsw\t$dst, $src", [(store i16:$src, imm:$dst)]>;

let DecoderMethod = "DecodeOUT16" in
def OUT16K : FormH_ED<0x41, (outs), (ins BCREG:$port, BGPR8:$rd), "out  (c), $rd", []>;

def OUT8K  : FormIA<0xd3, (outs), (ins ACC:$rd, addr8:$d8), "out  $d8, $rd", []>;

def PORTS_LONG : PatLeaf<(i8 2)>;
def PORTS_SHORT : PatLeaf<(i8 3)>;

def : Pat<(outport i8:$rd, i16:$port, PORTS_LONG), (OUT16K $port, $rd)>;
def : Pat<(outport i8:$rd, uimm8:$port, PORTS_SHORT), (OUT8K $rd, uimm8:$port)>;

// Unstable command on CMOS chip
//def OUT16Z : FormA<0x71, (outs), (ins BCREG:$port, i8imm:$imm), "out  (c), $imm", [(outport immzero:$imm, i16:$port)]>, ED;

let Defs = [SREG] in
def IN16K : FormH_ED<0x40, (outs BGPR8:$rd), (ins BCREG:$port), "in   $rd, (c)",
                        [(set i8:$rd, (inport i16:$port, PORTS_LONG))]>;

def IN8K  : FormIA<0xDB, (outs ACC:$rd), (ins addr8:$d8), "in   $rd, $d8",
                        [(set i8:$rd, (inport uimm8:$d8, PORTS_SHORT))]>;

// Indirect stores.
let DecoderMethod = "DecodeSTPTR" in
def STPTR : FormB<0x70, (outs), (ins regref:$ptr, BGPR8:$rr), "ld   $ptr, $rr", [(store i8:$rr, addr:$ptr)]>;

def MOVE : FormG<0x40, (outs BGPR8:$rd), (ins BGPR8:$rr), "ld   $rd, $rr", []>;

def STWPtrRr : Pseudo<(outs), (ins PTRDISPREGS:$ptrreg, DREGS:$reg), "stw  $ptrreg, $reg",
                     [(store i16:$reg, i16:$ptrreg)]>;

def STDPTR : Pseudo<(outs), (ins memri:$d8, BGPR8:$rr), "ld   $d8, $rr", []>, IsPtrInstr;

def STDPTR_P : Pseudo<(outs), (ins memri:$d8, GPR8:$rr), "!ld   $d8, $rr", []>;

def STDPtrQRr : Pseudo<(outs), (ins memri:$d8, BGPR8:$rr), "ld   $d8, $rr",
                      [(store i8:$rr, addr:$d8)]>, IsPtrInstr;

def STDWPtrQRr : Pseudo<(outs), (ins memri:$memri, DREGS:$src), "stdw\t$memri, $src",
                        [(store i16:$src, addr:$memri)]>;

// Stack push/pop operations.
let Defs = [SP],
Uses = [SP],
hasSideEffects = 0 in
{
  // Stack push operations.
  let mayStore = 1, DecoderMethod="DecodePUSHPOP" in
    def PUSH : FormC<0b11000101, (outs), (ins REGSTOSTACK:$rr), "push $rr", []>;

  // Stack pop operations.
  let mayLoad = 1, DecoderMethod="DecodePUSHPOP" in
    def POP : FormC<0b11000001, (outs REGSTOSTACK:$rr), (ins), "pop  $rr", []>;
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
  def ROT8LOOP : Pseudo<(outs BGPR8:$rr), (ins BGPR8:$src, Operand<i8>:$rot, ACC:$amt), "#ROT8 $rr, $rot, $amt",
                        [(set i8:$rr, (Z80rotshift i8:$src, rot:$rot, i8:$amt)), (implicit SREG)]>;

  let usesCustomInserter = 1 in
  def ROT16LOOP : Pseudo<(outs BDREGS:$rr), (ins BDREGS:$src, Operand<i8>:$rot, ACC:$amt), "#ROT16 $rr, $rot, $amt", []>;

  def ROT16 : Pseudo<(outs BDREGS:$rr), (ins BDREGS:$src, Operand<i8>:$rot), "#ROT16 $rr, $rot",
                     [(set i16:$rr, (Z80rotshift i16:$src, rot:$rot, (i8 1))), (implicit SREG)]>;

  def SRL : FormB_CB<0x38, (outs BGPR8:$rr), (ins BGPR8:$src), "srl  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SRL)), (implicit SREG)]>;

  def SRA : FormB_CB<0x28, (outs BGPR8:$rr), (ins BGPR8:$src), "sra  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SRA)), (implicit SREG)]>;

  def RLC : FormB_CB<0x00, (outs BGPR8:$rr), (ins BGPR8:$src), "rlc  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RLC)), (implicit SREG)]>;

  def RLCA : FormA<0x07, (outs ACC:$rr), (ins ACC:$src), "rlca", []>;

  def RRC : FormB_CB<0x08, (outs BGPR8:$rr), (ins BGPR8:$src), "rrc  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RRC)), (implicit SREG)]>;

  def RRCA : FormA<0x0f, (outs ACC:$rr), (ins ACC:$src), "rrca", []>;

  def SLA : FormB_CB<0x20, (outs BGPR8:$rr), (ins BGPR8:$src), "sla  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SLA)), (implicit SREG)]>;

  def SLI : FormB_CB<0x30, (outs BGPR8:$rr), (ins BGPR8:$src), "sli  $rr",
                    [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_SLI)), (implicit SREG)]>;

  let Uses = [SREG] in
  {
    def RR : FormB_CB<0x18, (outs BGPR8:$rr), (ins BGPR8:$src), "rr   $rr",
                     [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RR)), (implicit SREG)]>;

    def RRA  : FormA<0x1f, (outs ACC:$rr), (ins ACC:$src), "rra", []>;

    def RL : FormB_CB<0x10, (outs BGPR8:$rr), (ins BGPR8:$src), "rl   $rr",
                     [(set i8:$rr, (singlerotshift i8:$src, Z80_ROT_RL)), (implicit SREG)]>;

    def RLA  : FormA<0x17, (outs ACC:$rr), (ins ACC:$src), "rla", []>;
  }
}

let Defs = [SREG], DecoderMethod = "DecodeSingleRegref" in {
  def RLCPTR : FormA_CB<0x06, (outs), (ins regref:$ptr), "rlc  $ptr", []>;
  def RRCPTR : FormA_CB<0x0E, (outs), (ins regref:$ptr), "rrc  $ptr", []>;
  def SLAPTR : FormA_CB<0x26, (outs), (ins regref:$ptr), "sla  $ptr", []>;
  def SRAPTR : FormA_CB<0x2E, (outs), (ins regref:$ptr), "sra  $ptr", []>;
  def SLIPTR : FormA_CB<0x36, (outs), (ins regref:$ptr), "sli  $ptr", []>;
  def SRLPTR : FormA_CB<0x3E, (outs), (ins regref:$ptr), "srl  $ptr", []>;

   let Uses = [SREG] in {
      def RLPTR : FormA_CB<0x16, (outs), (ins regref:$ptr), "rl   $ptr", []>;
      def RRPTR : FormA_CB<0x1E, (outs), (ins regref:$ptr), "rr   $ptr", []>;
   }
}

class RotRdPtr<PatFrag Rot, string n, bits<3> rc> :
    Pseudo<(outs), (ins memri:$src), !strconcat(n, "  $src"),
              [(store (i8 (singlerotshift (i8 (load addr:$src)), Rot)), addr:$src), (implicit SREG)]>, CB, IsPtrInstr;

class RotWPtr<PatFrag Rot, string n> :
    Pseudo<(outs), (ins memri:$src), !strconcat(n, "!  $src"),
                    [(store (i16 (singlerotshift (i16 (load addr:$src)), Rot)), addr:$src), (implicit SREG)]>;

let Defs = [SREG] in
{
    def : RotRdPtr<Z80_ROT_SLA, "sla", 4>;
    def : RotRdPtr<Z80_ROT_SRA, "sra", 5>;
    def : RotRdPtr<Z80_ROT_SLI, "sli", 6>;
    def : RotRdPtr<Z80_ROT_SRL, "srl", 7>;
    def : RotRdPtr<Z80_ROT_RLC, "rlc", 0>;
    def : RotRdPtr<Z80_ROT_RRC, "rrc", 1>;
    def : RotRdPtr<Z80_ROT_RR,  "rr ", 3>;
    def : RotRdPtr<Z80_ROT_RL,  "rl ", 2>;

    def : RotWPtr<Z80_ROT_SLA, "slaw">;
    def : RotWPtr<Z80_ROT_SRA, "sraw">;
    def : RotWPtr<Z80_ROT_SRL, "srlw">;
    def : RotWPtr<Z80_ROT_RLC, "rlcw">;
    def : RotWPtr<Z80_ROT_RRC, "rrcw">;
    def : RotWPtr<Z80_ROT_RR, "rrw ">;
    def : RotWPtr<Z80_ROT_RL, "rlw ">;
}

def : Pat<(Z80rotshift i16:$src, rot:$rot, i16:$amt), (ROT16LOOP $src, $rot, (EXTRACT_SUBREG $amt, sub_lo))>;
def : Pat<(Z80rotshift i16:$src, rot:$rot, i8:$amt), (ROT16LOOP $src, $rot, $amt)>;

let Defs=[SREG], Constraints = "$rs = $rd" in
def CPL : FormA<0x2f, (outs ACC:$rd), (ins ACC:$rs), "cpl", [(set i8:$rd, (xor i8:$rs, -1)), (implicit SREG)]>;

let Defs = [SREG] in
def CLEARC : FormA<0xb7, (outs), (ins), "or   a", []>;

let Defs = [SREG, A], Uses = [SREG, A] in
def DAA : FormA<0x27, (outs), (ins), "daa", []>;

let Defs = [DE], DecoderMethod = "DecodeSingleRegref" in
def EXD : FormA<0xEB, (outs BDREGS:$r), (ins), "ex   de, $r", []>;

let DecoderMethod = "DecodeSingleRegref" in
def EXSPRP : FormA<0xE3, (outs BDREGS:$r), (ins), "ex   (sp), $r", []>;

let Defs = [BC, DE, HL] in
def EXX : FormA<0xd9, (outs), (ins), "exx", []>;

let Defs = [AF] in
def EXAF : FormA<0x08, (outs), (ins), "ex   af, af'", []>;

//===----------------------------------------------------------------------===//
// Special/Control instructions
//===----------------------------------------------------------------------===//

def NOP : FormA<0, (outs), (ins), "nop", []>;
def HALT : FormA<0x76, (outs), (ins), "halt", []>;

def IM : FormIM<0x46, "im   $n">;

let Defs = [SREG, HL, DE, BC] in {
    def LDIR : FormA_ED<0xb0, (outs), (ins HLREG:$src, DEREG:$dst, BCREG:$cnt), "ldir", []>;
    def LDI  : FormA_ED<0xa0, (outs), (ins HLREG:$src, DEREG:$dst, BCREG:$cnt), "ldi",  []>;

    def LDDR : FormA_ED<0xb8, (outs), (ins HLREG:$src, DEREG:$dst, BCREG:$cnt), "lddr", []>;
    def LDD  : FormA_ED<0xa8, (outs), (ins HLREG:$src, DEREG:$dst, BCREG:$cnt), "ldd",  []>;
}

//===----------------------------------------------------------------------===//
// Pseudo instructions for later expansion
//===----------------------------------------------------------------------===//

let Defs=[SREG] in {
    def SEXT : Pseudo<(outs DREGS:$dst), (ins GPR8:$src), "!sext\t$dst, $src",
                       [(set i16:$dst, (sext i8:$src)), (implicit SREG)]>;

    def ZEXT : Pseudo<(outs DREGS:$dst), (ins GPR8:$src), "!zext\t$dst, $src",
                       [(set i16:$dst, (zext i8:$src)), (implicit SREG)]>;
}

// This pseudo gets expanded into a movw+adiw thus it clobbers SREG.
let Defs = [SREG], hasSideEffects = 0 in
def FRMIDX : Pseudo<(outs PTRREGS:$dst),
                    (ins PTRREGS:$src, i16imm:$src2),
                    "!frmidx\t$dst, $src, $src2",
                    []>;

// This pseudo is either converted to a regular store or a push which clobbers SP.
def STDSPQRr : StorePseudo<(outs), (ins memri:$dst, BGPR8:$src), "!stdstk\t$dst, $src",
                           [(store i8:$src, addr:$dst)]>;

// This pseudo is either converted to a regular store or a push which clobbers SP.
def STDWSPQRr : StorePseudo<(outs), (ins memri:$dst, DREGS:$src), "!stdwstk\t$dst, $src",
                            [(store i16:$src, addr:$dst)]>;

def Select8 : SelectPseudo<(outs GPR8:$dst), (ins GPR8:$src, GPR8:$src2, variable_ops), "# Select8 PSEUDO", []>;
def Select16 : SelectPseudo<(outs DREGS:$dst), (ins DREGS:$src, DREGS:$src2, variable_ops), "# Select16 PSEUDO", []>;

//==============================================================================
def DI : FormA<0xf3, (outs), (ins), "di", []>;
def EI : FormA<0xfb, (outs), (ins), "ei", []>;

//==============================================================================

//===----------------------------------------------------------------------===//
// Non-Instruction Patterns
//===----------------------------------------------------------------------===//

//:TODO: look in x86InstrCompiler.td for odd encoding trick related to
// add x, 128 -> sub x, -128. Clang is emitting an eor for this (ldi+eor)

// the add instruction always writes the carry flag
def : Pat<(addc i8:$src, i8:$src2), (ADD $src, $src2)>;
def : Pat<(addc i16:$src, i16:$src2), (ADDW $src, $src2)>;

//// all sub instruction variants always writes the carry flag
//def : Pat<(subc i8:$src, i8:$src2),
//          (SUBRdRr i8:$src, i8:$src2)>;
def : Pat<(subc i16:$src, i16:$src2), (SUBW i16:$src, i16:$src2)>;
//def : Pat<(subc i8:$src, imm:$src2),
//          (SUBIRdK i8:$src, imm:$src2)>;
//def : Pat<(subc i16:$src, imm:$src2),
//          (SUBIWRdK i16:$src, imm:$src2)>;
//


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

def : Ex16Pat<or, OR>;
def : Ex16Pat<and, AND>;
def : Ex16Pat<xor, XOR>;

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

def : Pat<(Z80tst i8:$rd), (AND ACC:$rd, GPR8:$rd)>;
def : Pat<(Z80tsts i8:$rd), (TESTBIT $rd, 7)>;
def : Pat<(Z80tsts i16:$rd), (TESTBIT (EXTRACT_SUBREG $rd, sub_hi), 7)>;

//// Lowering of 'lsl' node to 'LSL' instruction.
//// LSL is an alias of 'ADD Rd, Rd'
//def : Pat<(Z80lsl i8:$rd),
//          (ADDRdRr GPR8:$rd, GPR8:$rd)>;
//

def : Pat<(Z80cps i8:$rd, (i8 1)), (DEC $rd)>;
def : Pat<(Z80cps i8:$rd, (i8 imm:$rr)), (SUBk $rd, imm:$rr)>;
def : Pat<(Z80cps i8:$rd, i8:$rr), (SUB $rd, $rr)>;
def : Pat<(Z80cps i16:$rd, i16:$rr), (SUBW $rd, $rr)>;

def : Pat<(Z80cp i8:$rd, i8:$rr),(CP $rd, $rr)>;
def : Pat<(Z80cp i8:$rd, (load addr:$addr)), (CPPTR $rd, regref:$addr)>;
def : Pat<(Z80cp i8:$rd, imm:$d8), (CPImm8 $rd, (as_i8timm $d8))>;
def : Pat<(Z80cp (and i8:$v, iobitpos8:$a),  0), (TESTBIT $v, iobitpos8:$a)>;
//def : Pat<(Z80cp (and (i8 (load addr:$v)), iobitpos8:$a),  0), (TESTBITPTR addr:$v, iobitpos8:$a)>;

def : Pat<(Z80cp i8:$rd, 0), (OR $rd, $rd)>;
def : Pat<(Z80cp i16:$rd, 0), (OR (EXTRACT_SUBREG $rd, sub_lo), (EXTRACT_SUBREG $rd, sub_hi))>;

//def : Pat<(add i16:$a, (zext i8:$b)), (ADDWRdRr $a,
// (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (LDIRdK (i8 0)), sub_hi), $b, sub_lo))>;

// (char)(v >> 8)
// (char)(v & 255)
def : Pat<(i8 (trunc (srl i16:$v, (i8 8)))),  (i8 (EXTRACT_SUBREG $v, sub_hi))>;
def : Pat<(i8 (trunc (and i16:$v, (i16 255)))),  (i8 (EXTRACT_SUBREG $v, sub_lo))>;

// i16 ((i16 v) >> 8)
def : Pat<(i16 (srl i16:$v, (i8 8))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                     (i8 (EXTRACT_SUBREG $v, sub_hi)), sub_lo),
                                     (LDk (i8 0)), sub_hi))>;

// v & 255
// v % 256
def : Pat<(i16 (and i16:$v, (i16 255))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (LDk (i8 0)), sub_hi), (i8 (EXTRACT_SUBREG $v, sub_lo)), sub_lo))>;

// (i16)((i32)v >> 8)
def : Pat<(i16 (or (srl i16:$a, (i8 8)), (shl i16:$b, (i8 8)))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (i8 (EXTRACT_SUBREG $a, sub_hi)), sub_lo),
                                    (i8 (EXTRACT_SUBREG $b, sub_lo)), sub_hi))>;

// v << 8
def : Pat<(i16 (shl i16:$v, (i8 8))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (i8 (EXTRACT_SUBREG $v, sub_lo)), sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 9
def : Pat<(i16 (shl i16:$v, (i8 9))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (i8 (EXTRACT_SUBREG $v, sub_lo)))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 10
def : Pat<(i16 (shl i16:$v, (i8 10))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (SLA (i8 (EXTRACT_SUBREG $v, sub_lo))))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 11
def : Pat<(i16 (shl i16:$v, (i8 11))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (SLA (SLA (i8 (EXTRACT_SUBREG $v, sub_lo)))))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 12
def : Pat<(i16 (shl i16:$v, (i8 12))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (SLA (SLA (SLA (i8 (EXTRACT_SUBREG $v, sub_lo))))))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 13
def : Pat<(i16 (shl i16:$v, (i8 13))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (SLA (SLA (SLA (SLA (i8 (EXTRACT_SUBREG $v, sub_lo)))))))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

// v << 14
def : Pat<(i16 (shl i16:$v, (i8 14))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                                    (SLA (SLA (SLA (SLA (SLA (SLA (i8 (EXTRACT_SUBREG $v, sub_lo))))))))
                                    , sub_hi), (LDk (i8 0)), sub_lo))>;

def : Pat<(i16 (shl (i16 (anyext i8:$v)), (i8 8))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), $v, sub_hi), (LDk (i8 0)), sub_lo))>;

// (v1 << 8) | v2
def : Pat<(i16 (or (shl (i16 (anyext i8:$v1)), (i8 8)), (i16 (zext i8:$v2)))),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), $v1, sub_hi), $v2, sub_lo))>;


def : Pat<(i16 (zext (i8 (load addr:$a)))),
           (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (i8 (LDk (i8 0))), sub_hi),
                                                 (LDDRdPtrQ addr:$a), sub_lo))>;

def : Pat<(i16 (bswap i16:$v)),
    (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)),
                            (i8 (EXTRACT_SUBREG $v, sub_lo)), sub_hi),
                            (i8 (EXTRACT_SUBREG $v, sub_hi)), sub_lo))>;

def : Pat<(i8 (trunc (srl i16:$v, (i8 15)))), (ANDk (RLC (EXTRACT_SUBREG $v, sub_hi)), 1)>;

def : Pat<(i16 (srl i16:$v, (i8 15))),
        (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (i8 (LDk (i8 0))), sub_hi),
                            (ANDk (RLC (EXTRACT_SUBREG $v, sub_hi)), 1), sub_lo))>;

def : Pat<(i16 (shl i16:$v, (i8 15))),
        (i16 (INSERT_SUBREG (INSERT_SUBREG (i16 (IMPLICIT_DEF)), (ANDk (RRC (EXTRACT_SUBREG $v, sub_lo)), 0x80), sub_hi),
                            (i8 (LDk (i8 0))), sub_lo))>;