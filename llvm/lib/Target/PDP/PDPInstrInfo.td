//===-- PDPInstrInfo.td - PDP Instruction defs -------------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes the PDP instructions in TableGen format.
//
//===----------------------------------------------------------------------===//

include "PDPInstrFormats.td"

//===----------------------------------------------------------------------===//
// PDP Type Profiles
//===----------------------------------------------------------------------===//

def SDT_PDPCallSeqStart : SDCallSeqStart<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_PDPCallSeqEnd : SDCallSeqEnd<[SDTCisVT<0, i16>, SDTCisVT<1, i16>]>;
def SDT_PDPCall : SDTypeProfile<0, -1, [SDTCisVT<0, iPTR>]>;
def SDT_PDPWrapper : SDTypeProfile<1, 1, [SDTCisSameAs<0, 1>, SDTCisPtrTy<0>]>;

def SDT_PDPBrcond : SDTypeProfile<0, 2, [SDTCisVT<0, OtherVT>, SDTCisVT<1, i8>]>;
def SDT_PDPBrcond2 : SDTypeProfile<0, 4, [SDTCisSameAs<0, 1>, SDTCisVT<2, OtherVT>, SDTCisVT<3, i8>]>;
def SDT_PDPBrbit : SDTypeProfile<0, 4, [SDTCisSameAs<0, 1>, SDTCisVT<2, OtherVT>, SDTCisVT<3, i8>]>;

def SDT_PDPCmp : SDTypeProfile<0, 2, [SDTCisSameAs<0, 1>]>;

def SDT_PDPTst : SDTypeProfile<0, 1, [SDTCisInt<0>]>;
def SDT_PDPSxt : SDTypeProfile<1, 1, [SDTCisInt<0>]>;

def SDT_PDPSelectCC
    : SDTypeProfile<1, 3,
                    [SDTCisSameAs<0, 1>, SDTCisSameAs<1, 2>, SDTCisVT<3, i8>]>;

def SDT_PDPSelectCC2
    : SDTypeProfile<1, 5,
                    [SDTCisSameAs<0, 1>, SDTCisSameAs<1, 2>, SDTCisSameAs<2, 3>, SDTCisSameAs<3, 4>, SDTCisVT<5, i8>]>;

//===----------------------------------------------------------------------===//
// PDP Specific Node Definitions
//===----------------------------------------------------------------------===//

def PDPretglue : SDNode<"PDPISD::RET_GLUE", SDTNone,
                        [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;
def PDPretiglue : SDNode<"PDPISD::RETI_GLUE", SDTNone,
                         [SDNPHasChain, SDNPOptInGlue, SDNPVariadic]>;

def PDPcallseq_start : SDNode<"ISD::CALLSEQ_START", SDT_PDPCallSeqStart,
                              [SDNPHasChain, SDNPOutGlue]>;
def PDPcallseq_end : SDNode<"ISD::CALLSEQ_END", SDT_PDPCallSeqEnd,
                            [SDNPHasChain, SDNPOptInGlue, SDNPOutGlue]>;

def PDPcall : SDNode<"PDPISD::CALL", SDT_PDPCall,
                     [SDNPHasChain, SDNPOutGlue, SDNPOptInGlue, SDNPVariadic]>;

def PDPWrapper : SDNode<"PDPISD::WRAPPER", SDT_PDPWrapper>;

def PDPbrcond : SDNode<"PDPISD::BRCOND", SDT_PDPBrcond, [SDNPHasChain, SDNPInGlue]>;
def PDPbrcond2 : SDNode<"PDPISD::BRCOND2", SDT_PDPBrcond2, [SDNPHasChain, SDNPInGlue]>;
def PDPbrcond2b : SDNode<"PDPISD::BRCOND2B", SDT_PDPBrcond2, [SDNPHasChain, SDNPInGlue]>;

def brbit : SDNode<"PDPISD::BRBIT", SDT_PDPBrbit, [SDNPHasChain, SDNPInGlue]>;
def brbitb : SDNode<"PDPISD::BRBITB", SDT_PDPBrbit, [SDNPHasChain, SDNPInGlue]>;

def PDPcmp : SDNode<"PDPISD::CMP", SDT_PDPCmp, [SDNPOutGlue]>;
def PDPcmpc : SDNode<"PDPISD::CMPC", SDT_PDPCmp, [SDNPInGlue, SDNPOutGlue]>;
def PDPtst : SDNode<"PDPISD::TST", SDT_PDPTst, [SDNPOutGlue]>;
def PDPsxt : SDNode<"PDPISD::SXT", SDT_PDPSxt, [SDNPInGlue]>;
def PDPselectcc : SDNode<"PDPISD::SELECT_CC", SDT_PDPSelectCC, [SDNPInGlue]>;
def PDPselectcc2 : SDNode<"PDPISD::SELECT_CC2", SDT_PDPSelectCC2, []>;

// Shift nodes.
def PDPasl : SDNode<"PDPISD::ASL", SDTIntUnaryOp>;
def PDPlsr : SDNode<"PDPISD::LSR", SDTIntUnaryOp>;
def PDProl : SDNode<"PDPISD::ROL", SDTIntUnaryOp>;
def PDPror : SDNode<"PDPISD::ROR", SDTIntUnaryOp>;
def PDPasr : SDNode<"PDPISD::ASR", SDTIntUnaryOp>;

// Pseudo shift nodes for non-constant shift amounts.
def PDPlslLoop : SDNode<"PDPISD::LSLLOOP", SDTIntShiftOp>;
def PDPlsrLoop : SDNode<"PDPISD::LSRLOOP", SDTIntShiftOp>;
def PDProlLoop : SDNode<"PDPISD::ROLLOOP", SDTIntShiftOp>;
def PDProrLoop : SDNode<"PDPISD::RORLOOP", SDTIntShiftOp>;
def PDPasrLoop : SDNode<"PDPISD::ASRLOOP", SDTIntShiftOp>;

// SWAP node.
def PDPSwap : SDNode<"PDPISD::SWAP", SDTIntUnaryOp>;

//===----------------------------------------------------------------------===//
// PDP Operands, Complex Patterns and Transformations Definitions.
//===----------------------------------------------------------------------===//

def as_i16imm : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getAPIntValue(), SDLoc(N), MVT::i16);
}]>;

def as_i8imm : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(N->getAPIntValue(), SDLoc(N), MVT::i8);
}]>;

def imm16_neg_XFORM : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(-N->getAPIntValue(), SDLoc(N), MVT::i16);
}]>;

def imm8_not_XFORM : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(~N->getAPIntValue(), SDLoc(N), MVT::i8);
}]>;

def imm16_not_XFORM : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(~N->getAPIntValue(), SDLoc(N), MVT::i16);
}]>;

def uimm6 : PatLeaf<(imm), [{ return isUInt<6>(N->getZExtValue()); }]>;
def uimm8 : PatLeaf<(imm), [{ return isUInt<8>(N->getZExtValue()); }]>;

// imm_com8_XFORM - Return the complement of a imm_com8 value
def imm_com8_XFORM : SDNodeXForm<imm, [{
  return CurDAG->getTargetConstant(
      ~((uint8_t) N->getZExtValue()), SDLoc(N), MVT::i8);
}]>;

// imm_com8 - Match an immediate that is a complement
// of a 8-bit immediate.
// Note: this pattern doesn't require an encoder method and such, as it's
// only used on aliases (Pat<> and InstAlias<>). The actual encoding
// is handled by the destination instructions, which use imm_com8.
def imm_com8_asmoperand : AsmOperandClass { let Name = "ImmCom8"; }
def imm_com8 : Operand<i8> { let ParserMatchClass = imm_com8_asmoperand; }

def UniopAsmOperand : AsmOperandClass {
  let Name = "Uniop";
  let ParserMethod = "parseUniopOperand";
}

def br_cond : Operand<iPTR> {
  let MIOperandInfo = (ops i8imm:$cc);
  let PrintMethod = "printBrcond";
}

def uniop : Operand<iPTR> {
  let MIOperandInfo = (ops unknown:$reg, i8imm:$mode, i16imm:$immediate, i16imm:$offset);
  let PrintMethod = "printUniop";

  let ParserMatchClass = UniopAsmOperand;
}

def uniop16 : Operand<iPTR> {
  let MIOperandInfo = (ops GPR16:$reg, i8imm:$mode, i16imm:$immediate, i16imm:$offset);
  let PrintMethod = "printUniop";
  let EncoderMethod = "encodeUniop";
  let DecoderMethod = "decodeUniop";

  let ParserMatchClass = UniopAsmOperand;
}

def uniop8 : Operand<iPTR> {
  let MIOperandInfo = (ops GPR16l:$reg, i8imm:$mode, i16imm:$immediate, i16imm:$offset);
  let PrintMethod = "printUniop";

  let ParserMatchClass = UniopAsmOperand;
}

def relbrtarget_8 : Operand<OtherVT> {
  let PrintMethod = "printPCRelImm";
  let EncoderMethod = "encodeRelCondBrTarget<PDP::fixup_8_pcrel>";
  let DecoderMethod = "decodeRelCondBrTarget";
}

def relsobtarget : Operand<OtherVT> {
  let PrintMethod = "printPCRelImm";
  let EncoderMethod = "encodeRelCondBrTarget<PDP::fixup_sob_pcrel>";
  let DecoderMethod = "decodeSobTarget";
}

// A 16-bit address (which can lead to an R_PDP_16 relocation).
def imm16 : Operand<i16> { let EncoderMethod = "encodeImm<PDP::fixup_16, 2>"; }

let WantsParent = true in def uniSrc : ComplexPattern<iAny, 4, "SelectUniSrc">;

def notimm : ComplexPattern<iAny, 1, "SelectNotImm">;
def uniTgt : ComplexPattern<iAny, 4, "SelectUniTgt">;
def uniRmw : ComplexPattern<iAny, 4, "SelectUniRMW">;

let WantsRoot = true in {
    def uniLoad : ComplexPattern<iPTR, 4, "SelectUniLoad">;
    def uniStore : ComplexPattern<iPTR, 4, "SelectUniStore">;
}

def inegc : PatFrag<(ops node:$in), (subc 0, node:$in)>;
def load_mem : PatFrag<(ops node:$x), (i16(load(PDPWrapper node:$x)))>;

def R16 : OutPatFrag<(ops node:$exp), (INSERT_SUBREG (i16 (IMPLICIT_DEF)), $exp, sub_lo)>;

//===----------------------------------------------------------------------===//
// PDP predicates for subtarget features
//===----------------------------------------------------------------------===//

def HasIMUL : Predicate<"Subtarget->hasIMUL()">,
              AssemblerPredicate<(all_of FeatureIMUL)>;

//def HasNonTinyEncoding : Predicate<"!Subtarget->hasTinyEncoding()">,
//                         AssemblerPredicate<(any_of (not FeatureTinyEncoding))>;

// PDP specific condition code. These correspond to PDP_*_COND in
// PDPInstrInfo.td. They must be kept in synch.
def PDP_COND_EQ : PatLeaf<(i8 0)>;
def PDP_COND_NE : PatLeaf<(i8 1)>;
def PDP_COND_GE : PatLeaf<(i8 2)>;
def PDP_COND_GT : PatLeaf<(i8 3)>;
def PDP_COND_LT : PatLeaf<(i8 4)>;
def PDP_COND_LE : PatLeaf<(i8 5)>;
def PDP_COND_SH : PatLeaf<(i8 6)>;
def PDP_COND_LO : PatLeaf<(i8 7)>;
def PDP_COND_SL : PatLeaf<(i8 8)>;
def PDP_COND_HI : PatLeaf<(i8 9)>;
def PDP_COND_MI : PatLeaf<(i8 10)>;
def PDP_COND_PL : PatLeaf<(i8 11)>;

//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//
// PDP Instruction list
//===----------------------------------------------------------------------===//
//===----------------------------------------------------------------------===//

// ADJCALLSTACKDOWN/UP implicitly use/def SP because they may be expanded into
// a stack adjustment and the codegen must know that they may modify the stack
// pointer before prolog-epilog rewriting occurs.
// Pessimistically assume ADJCALLSTACKDOWN / ADJCALLSTACKUP will become
// sub / add which can clobber SREG.
let Defs = [SREG]/*, Uses = [R6]*/, hasSideEffects = 1 in {
  def ADJCALLSTACKDOWN : Pseudo<(outs), (ins i16imm:$amt, i16imm:$amt2),
                                "#ADJCALLSTACKDOWN",
                                [(PDPcallseq_start timm:$amt, timm:$amt2)]>;

  // R31R30 is used to update SP. It is normally free because it is a
  // call-clobbered register but it is necessary to set it as a def as the
  // register allocator might use it in rare cases (for rematerialization, it
  // seems). hasSideEffects needs to be set to true so this instruction isn't
  // considered dead.
  let /*Defs = [R31R30],*/ hasSideEffects = 1 in def ADJCALLSTACKUP
      : Pseudo<(outs), (ins i16imm:$amt1, i16imm:$amt2),
               "#ADJCALLSTACKUP", [(PDPcallseq_end timm:$amt1, timm:$amt2)]>;
}

//===----------------------------------------------------------------------===//
// Addition
//===----------------------------------------------------------------------===//
let isCommutable = 1, Constraints = "$rr = $rd",  Defs = [SREG] in {
  def ADDREG : Pseudo<(outs GPR16:$rr), (ins uniop:$rs, GPR16:$rd), "add\t$rs, $rd", []>;
  // ADDB не существует

  let Uses = [SREG] in {
    def ADCREG : Pseudo<(outs GPR16:$rr), (ins GPR16:$rd), "adc\t$rd", []>;
    def ADCBREG : Pseudo<(outs GPR16l:$rr), (ins GPR16l:$rd), "adcb\t$rd", []>;
  }
}

let isCommutable = 1, Defs = [SREG] in
  def ADD : TwoArgInst<6, (outs), (ins uniop16:$rs, uniop16:$rd), "add\t$rs, $rd", []>;

let Uses = [SREG], Defs = [SREG] in {
  def ADC : OneArgInst<45, (outs), (ins uniop16:$rd), "adc\t$rd", []>;
  def ADCB : OneArgInst<557, (outs), (ins uniop16:$rd), "adcb\t$rd", []>;
}

let Constraints = "$rr = $rd", Defs = [SREG] in {
  def SUBREG : Pseudo<(outs GPR16:$rr), (ins uniop:$rs, GPR16:$rd), "sub\t$rs, $rd", []>;
  // SUBB не существует

  let Uses = [SREG] in {
    def SBCREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rr), "_sbcreg\t$rd", []>;
    def SBCBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rr), "_sbcregb\t$rd", []>;
  }
}

let Constraints = "$src = $rd", Defs = [SREG] in {
  def NEGREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$src), "neg\t$rd", []>;
  def NEGBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$src), "negb\t$rd", []>;
}

let Defs = [SREG] in {
  def NEG : OneArgInst<44, (outs), (ins uniop16:$rd), "neg\t$rd", []>;
  def NEGB : OneArgInst<556, (outs), (ins uniop16:$rd), "negb\t$rd", []>;
}

let Defs = [SREG] in
  def SUB : TwoArgInst<14, (outs), (ins uniop16:$rs, uniop16:$rd), "sub\t$rs, $rd", []>;

let Defs = [SREG], Uses = [SREG] in {
  def SBC : OneArgInst<46, (outs), (ins uniop16:$rd), "sbc\t$rd", []>;
  def SBCB : OneArgInst<558, (outs), (ins uniop16:$rd), "sbcb\t$rd", []>;
}

def : Pat<(add uniSrc:$rs, notimm:$rd), (ADDREG uniop:$rs, $rd)>;
def : Pat<(add GPR16:$rs, GPR16:$rd), (ADDREG $rs, 0, 0, 0, $rd)>;
def : Pat<(addc uniSrc:$rs, notimm:$rd), (ADDREG uniop:$rs, $rd)>;
def : Pat<(addc GPR16:$rs, GPR16:$rd), (ADDREG $rs, 0, 0, 0, $rd)>;

def : Pat<(add GPR16l:$rd, GPR16l:$rs),  (EXTRACT_SUBREG (ADDREG (R16 i8:$rs), 0, 0, 0, (R16 i8:$rd)), sub_lo)>;
def : Pat<(sub GPR16l:$rd, GPR16l:$rs),  (EXTRACT_SUBREG (SUBREG (R16 i8:$rs), 0, 0, 0, (R16 i8:$rd)), sub_lo)>;

def : Pat<(i16 (ineg GPR16:$v)), (NEGREG $v)>;
def : Pat<(i8 (ineg GPR16l:$v)), (NEGBREG $v)>;

def : Pat<(sub notimm:$rd, uniSrc:$rs), (SUBREG uniop:$rs, $rd)>;
def : Pat<(sub GPR16:$rd, GPR16:$rs), (SUBREG $rs, 0, 0, 0, $rd)>;

def : Pat<(adde GPR16:$rs, notimm:$rd), (ADCREG (ADDREG $rs, 0, 0, 0, $rd))>;
def : Pat<(adde GPR16:$rs, GPR16:$rd), (ADCREG (ADDREG $rs, 0, 0, 0, $rd))>;


//===----------------------------------------------------------------------===//
// Increment and Decrement
//===----------------------------------------------------------------------===//
let Constraints = "$rs = $rd", Defs = [SREG] in {
  def INCREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "inc\t$rd", [(set i16:$rd, (add i16:$rs, 1))]>;
  def INCBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rs), "incb\t$rd", [(set i8:$rd, (add i8:$rs, 1))]>;

  def DECREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "dec\t$rd", [(set i16:$rd, (add i16:$rs, -1))]>;
  def DECBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rs), "decb\t$rd", [(set i8:$rd, (add i8:$rs, -1))]>;
}

let Defs = [SREG] in {
  def INC : OneArgInst<42, (outs), (ins uniop16:$rd), "inc\t$rd", []>;
  def DEC : OneArgInst<43, (outs), (ins uniop16:$rd), "dec\t$rd", []>;

  def INCB : OneArgInst<554, (outs), (ins uniop16:$rd), "incb\t$rd", []>;
  def DECB : OneArgInst<555, (outs), (ins uniop16:$rd), "decb\t$rd", []>;
}

//===----------------------------------------------------------------------===//
// Multiplication
//===----------------------------------------------------------------------===//

let isCommutable = 1, Defs = [/*R1, R0, */SREG] in {
  // MUL Rd, Rr
  // Multiplies Rd by Rr and places the result into R1:R0.
  let usesCustomInserter = 1 in {
    //def MULRdRr : FRdRr<0b1001, 0b11, (outs), (ins GPR8:$rd, GPR8:$rr),
    //                    "mul\t$rd, $rr", []>,
    //              Requires<[/*SupportsMultiplication*/]>;

    //def MULSRdRr : FMUL2RdRr<0, (outs), (ins LD8:$rd, LD8:$rr),
    //                         "muls\t$rd, $rr", []>,
    //               Requires<[/*SupportsMultiplication*/]>;
  }

  //def MULSURdRr : FMUL2RdRr<1, (outs), (ins LD8lo:$rd, LD8lo:$rr),
  //                          "mulsu\t$rd, $rr", []>,
  //                Requires<[/*SupportsMultiplication*/]>;

  //def FMUL : FFMULRdRr<0b01, (outs), (ins LD8lo:$rd, LD8lo:$rr),
  //                     "fmul\t$rd, $rr", []>,
  //           Requires<[/*SupportsMultiplication*/]>;

  //def FMULS : FFMULRdRr<0b10, (outs), (ins LD8lo:$rd, LD8lo:$rr),
  //                      "fmuls\t$rd, $rr", []>,
  //            Requires<[/*SupportsMultiplication*/]>;

  //def FMULSU : FFMULRdRr<0b11, (outs), (ins LD8lo:$rd, LD8lo:$rr),
  //                       "fmulsu\t$rd, $rr", []>,
  //             Requires<[/*SupportsMultiplication*/]>;
}

//===----------------------------------------------------------------------===//
// One's/Two's Complement
//===----------------------------------------------------------------------===//
let Constraints = "$src = $rd", Defs = [SREG] in {
  def COMREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$src), "com\t$rd", []>;
  def COMBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$src), "comb\t$rd", []>;
}

let Defs = [SREG] in {
  def COM : OneArgInst<41, (outs), (ins uniop16:$rd), "com\t$rd", []>;
  def COMB : OneArgInst<553, (outs), (ins uniop16:$rd), "comb\t$rd", []>;
}

def : Pat<(i16 (not GPR16:$v)), (COMREG $v)>;
def : Pat<(i8 (not GPR16l:$v)), (COMBREG $v)>;

//===----------------------------------------------------------------------===//
// Logic
//===----------------------------------------------------------------------===//
let Constraints = "$rr = $rd", Defs = [SREG] in {
  def BICREG : Pseudo<(outs GPR16:$rr), (ins uniop:$rs, GPR16:$rd), "bic\t$rs, $rd", []>;
  def BICBREG : Pseudo<(outs GPR16l:$rr), (ins uniop:$rs, GPR16l:$rd), "bicb\t$rs, $rd", []>;

  let isCommutable = 1 in {
    def BISREG : Pseudo<(outs GPR16:$rr), (ins uniop:$rs, GPR16:$rd), "bis\t$rs, $rd", []>;
    def BISBREG : Pseudo<(outs GPR16l:$rr), (ins uniop:$rs, GPR16l:$rd), "bisb\t$rs, $rd", []>;

    def XORREG : Pseudo<(outs GPR16:$rr), (ins GPR16:$rs, GPR16:$rd), "xor\t$rs, $rd", []>;
  }
}

let Defs = [SREG] in {
  def BITREG : Pseudo<(outs), (ins uniop:$rs, GPR16:$rd), "bit\t$rs, $rd", []>;
  def BITBREG : Pseudo<(outs), (ins uniop:$rs, GPR16l:$rd), "bitb\t$rs, $rd", []>;
}

let Defs = [SREG] in {
   def BIC : TwoArgInst<4, (outs), (ins uniop16:$rs, uniop16:$rd), "bic\t$rs, $rd", []>;
   def BICB : TwoArgInst<12, (outs), (ins uniop16:$rs, uniop16:$rd), "bicb\t$rs, $rd", []>;
}

let isCommutable = 1, Defs = [SREG] in {
  def BIS : TwoArgInst<5, (outs), (ins uniop16:$rs, uniop16:$rd), "bis\t$rs, $rd", []>;
  def BISB : TwoArgInst<13, (outs), (ins uniop16:$rs, uniop16:$rd), "bisb\t$rs, $rd", []>;

  def BIT : TwoArgInst<3, (outs), (ins uniop16:$rs, uniop16:$rd), "bit\t$rs, $rd", []>;
  def BITB : TwoArgInst<11, (outs), (ins uniop16:$rs, uniop16:$rd), "bitb\t$rs, $rd", []>;

  def XOR : XorInst<(outs), (ins GPR16:$rs, uniop16:$rd), "xor\t$rs, $rd", []>;
}

def : Pat<(and GPR16:$rd, imm:$rs), (BICREG R7, 2, (imm16_not_XFORM imm:$rs), 0, $rd)>;
def : Pat<(and GPR16l:$rd, imm:$rs), (BICBREG R7, 2, (imm8_not_XFORM imm:$rs), 0, $rd)>;

def : Pat<(and GPR16:$rs, GPR16:$rd), (BICREG (COMREG $rs), 0, 0, 0, $rd)>;
def : Pat<(and GPR16l:$rs, GPR16l:$rd), (BICBREG (COMBREG $rs), 0, 0, 0, $rd)>;

def : Pat<(and (xor GPR16:$rs, -1), GPR16:$rd), (BICREG $rs, 0, 0, 0, $rd)>;
def : Pat<(and (xor GPR16l:$rs, -1), GPR16l:$rd), (BICBREG $rs, 0, 0, 0, $rd)>;
def : Pat<(and GPR16:$rd, (xor GPR16:$rs, -1)), (BICREG $rs, 0, 0, 0, $rd)>;
def : Pat<(and GPR16l:$rd, (xor GPR16l:$rs, -1)), (BICBREG $rs, 0, 0, 0, $rd)>;

def : Pat<(or GPR16:$rd, uniSrc:$rs),(BISREG uniop:$rs, $rd)>;
def : Pat<(or GPR16l:$rd, uniSrc:$rs),(BISBREG uniop:$rs, $rd)>;

def : Pat<(or GPR16:$rd, GPR16:$rs),(BISREG $rs, 0, 0, 0, $rd)>;
def : Pat<(or GPR16l:$rd, GPR16l:$rs),(BISBREG $rs, 0, 0, 0, $rd)>;

def : Pat<(xor GPR16:$rd, GPR16:$rs), (XORREG $rs, $rd)>;
def : Pat<(xor GPR16l:$rs, GPR16l:$rd), (EXTRACT_SUBREG (XORREG (R16 i8:$rs), (R16 i8:$rd)), sub_lo)>;

//===----------------------------------------------------------------------===//
// Jump instructions
//===----------------------------------------------------------------------===//
let isBarrier = 1, isBranch = 1, isTerminator = 1 in {
  def BR : BranchInst<1, (outs), (ins relbrtarget_8:$k), "br\t$k", [(br bb:$k)]>;

  def JMP : OneArgInst<1, (outs), (ins uniop16:$rd), "jmp\t$rd", []>;
}

//===----------------------------------------------------------------------===//
// Call instructions
//===----------------------------------------------------------------------===//
let isCall = 1 in {
  def JSR : JsrInst<(outs), (ins GPR16:$r, uniop16:$k), "jsr\t$r, $k", []>;
}

def : Pat<(PDPcall imm:$k), (JSR R7, R7, 1, $k, 0)>;
def : Pat<(PDPcall i16:$k), (JSR R7, $k, 1, 0, 0)>;
def : Pat<(PDPcall(i16 tglobaladdr:$dst)), (JSR R7, R7, 3, tglobaladdr:$dst, 0)>;
def : Pat<(PDPcall(i16 texternalsym:$dst)), (JSR R7, R7, 3, texternalsym:$dst, 0)>;

//===----------------------------------------------------------------------===//
// Return instructions.
//===----------------------------------------------------------------------===//
let isTerminator = 1, isReturn = 1, isBarrier = 1 in {
  def RET : Pseudo<(outs), (ins), "ret", [(PDPretglue)]>;
  def RETI : Pseudo<(outs), (ins), "reti", [(PDPretiglue)]>;

  def RTS : RetInst<(outs), (ins GPR16:$r), "rts\t$r">;
}

def : InstAlias<"return", (RTS R7)>;

//===----------------------------------------------------------------------===//
// Compare operations.
//===----------------------------------------------------------------------===//
let Defs = [SREG], isCompare = 1/*, isTerminator = 1*/ in {
  def CMP : TwoArgInst<2, (outs), (ins uniop16:$rs, uniop16:$rd), "cmp\t$rs, $rd", []>;
  def CMPB : TwoArgInst<10, (outs), (ins uniop16:$rs, uniop16:$rd), "cmpb\t$rs, $rd", []>;

//  def CMPREG : Pseudo<(outs), (ins uniop:$rs, GPR16:$rd), "cmp\t$rs, $rd", []>;
//  def CMPBREG : Pseudo<(outs), (ins uniop:$rs, GPR16l:$rd), "cmpb\t$rs, $rd", []>;
}

def : Pat<(PDPcmp (i16 uniSrc:$a1), (i16 uniSrc:$a2)), (CMP uniop:$a1, uniop:$a2)>;
def : Pat<(PDPcmp (i8 uniSrc:$a1), (i8 uniSrc:$a2)), (CMPB uniop:$a1, uniop:$a2)>;

def : Pat<(PDPcmp i16:$a1, i16:$a2), (CMP $a1, 0, 0, 0, $a2, 0, 0, 0)>;
def : Pat<(PDPcmp i16:$a1, imm:$a2), (CMP $a1, 0, 0, 0, R7, 2, (as_i16imm $a2), 0)>;

def : Pat<(PDPcmp i8:$a1, i8:$a2), (CMPB (R16 $a1), 0, 0, 0, (R16 $a2), 0, 0, 0)>;
def : Pat<(PDPcmp i8:$a1, imm:$a2), (CMPB (R16 $a1), 0, 0, 0, R7, 2, (as_i8imm $a2), 0)>;

//def : Pat<(PDPcmp (load_mem tglobaladdr:$a1), i16:$a2), (CPWRdRr_ _R7, 3, $a1, 0, $a2, 0, 0, 0)>;
//def : Pat<(PDPcmp i16:$a1, (load_mem tglobaladdr:$a2)), (CPWRdRr_ $a1, 0, 0, 0, _R7, 3, $a2, 0)>;
//def : Pat<(PDPcmp (load_mem tglobaladdr:$a1), (load_mem tglobaladdr:$a2)),(CPWRdRr_ _R7, 3, $a1, 0, _R7, 3, $a2, 0)>;

//===----------------------------------------------------------------------===//
// Conditional branches
//===----------------------------------------------------------------------===//
// Based on status register. We cannot simplify these into instruction aliases
// because we also need to be able to specify a pattern to match for ISel.
let isBranch = 1, isTerminator = 1 in {
  def CBR : Pseudo<(outs), (ins uniop:$lhs, uniop:$rhs, i8imm:$cc, relbrtarget_8:$k, i8imm:$mode),
            "cond_br $lhs, $rhs, $cc, $k\t $$k", []>;
  def CBRB : Pseudo<(outs), (ins uniop:$lhs, uniop:$rhs, i8imm:$cc, relbrtarget_8:$k, i8imm:$mode),
            "cond_br $lhs, $rhs, $cc, $k\t $$k", []>;
}

def : Pat<(PDPbrcond2 (i16 uniSrc:$lhs), (i16 uniSrc:$rhs), bb:$k, imm:$cc), (CBR uniop16:$lhs, uniop16:$rhs, (as_i8imm $cc), bb:$k, 0)>;
//def : Pat<(PDPbrcond2 GPR16:$lhs, GPR16:$rhs, bb:$k, imm:$cc), (CBR $lhs, 0, 0, 0, $rhs, 0, 0, 0, (as_i8imm $cc), bb:$k)>;

def : Pat<(PDPbrcond2b (i8 uniSrc:$lhs), (i8 uniSrc:$rhs), bb:$k, imm:$cc), (CBRB uniop8:$lhs, uniop8:$rhs, (as_i8imm $cc), bb:$k, 0)>;

def : Pat<(brbit (i16 uniSrc:$lhs), (i16 uniSrc:$rhs), bb:$k, imm:$cc), (CBR uniop16:$lhs, uniop16:$rhs, (as_i8imm $cc), bb:$k, 1)>;
def : Pat<(brbitb (i8 uniSrc:$lhs), (i8 uniSrc:$rhs), bb:$k, imm:$cc), (CBRB uniop8:$lhs, uniop8:$rhs, (as_i8imm $cc), bb:$k, 1)>;

def SOB : SobInst<(outs), (ins GPR16:$r, relsobtarget:$k), "sob\t$r, $k", []>;

let isBranch = 1, isTerminator = 1, /*isBarrier = 1,*/ Uses = [SREG] in {

  def BNE : BranchInst<2, (outs), (ins relbrtarget_8:$k), "bne\t$k", [(PDPbrcond bb:$k, PDP_COND_NE)]>;
  def BEQ : BranchInst<3, (outs), (ins relbrtarget_8:$k), "beq\t$k", [(PDPbrcond bb:$k, PDP_COND_EQ)]>;
  def BVC : BranchInst<132, (outs), (ins relbrtarget_8:$k), "bvc\t$k", []>;
  def BVS : BranchInst<133, (outs), (ins relbrtarget_8:$k), "bvs\t$k", []>;

  // Знаковые переходы
  def BPL : BranchInst<128, (outs), (ins relbrtarget_8:$k), "bpl\t$k", [(PDPbrcond bb:$k, PDP_COND_PL)]>;
  def BMI : BranchInst<129, (outs), (ins relbrtarget_8:$k), "bmi\t$k", [(PDPbrcond bb:$k, PDP_COND_MI)]>;
  def BGE : BranchInst<4, (outs), (ins relbrtarget_8:$k), "bge\t$k", [(PDPbrcond bb:$k, PDP_COND_GE)]>;
  def BLT : BranchInst<5, (outs), (ins relbrtarget_8:$k), "blt\t$k", [(PDPbrcond bb:$k, PDP_COND_LT)]>;
  def BGT : BranchInst<6, (outs), (ins relbrtarget_8:$k), "bgt\t$k", [(PDPbrcond bb:$k, PDP_COND_GT)]>;
  def BLE : BranchInst<7, (outs), (ins relbrtarget_8:$k), "ble\t$k", [(PDPbrcond bb:$k, PDP_COND_LE)]>;

  // Беззнаковые переходы
  def BHI : BranchInst<130, (outs), (ins relbrtarget_8:$k), "bhi\t$k", [(PDPbrcond bb:$k, PDP_COND_HI)]>;
  def BLOS: BranchInst<131, (outs), (ins relbrtarget_8:$k), "blos\t$k", [(PDPbrcond bb:$k, PDP_COND_SL)]>;
  def BHIS: BranchInst<134, (outs), (ins relbrtarget_8:$k), "bhis\t$k", [(PDPbrcond bb:$k, PDP_COND_SH)]>;
  def BLO : BranchInst<135, (outs), (ins relbrtarget_8:$k), "blo\t$k", [(PDPbrcond bb:$k, PDP_COND_LO)]>;
}

def : InstAlias<"bcc\t$k", (BHIS relbrtarget_8:$k)>;
def : InstAlias<"bcs\t$k", (BLO relbrtarget_8:$k)>;

//===----------------------------------------------------------------------===//
// Data transfer instructions
//===----------------------------------------------------------------------===//

let isReMaterializable = 1 in {
  let Defs = [SREG] in {
    def MOV : TwoArgInst<1, (outs), (ins uniop16:$rs, uniop16:$rd), "mov\t$rs, $rd", []>;
    def MOVB : TwoArgInst<9, (outs), (ins uniop16:$rs, uniop16:$rd), "movb\t$rs, $rd", []>;
  }
}

let canFoldAsLoad = 1, isReMaterializable = 1, Defs = [SREG] in {
    def LDST_ : Pseudo<(outs), (ins i16imm:$dst, i16imm:$src), "#LDST\t$dst, $src", []>;
}

let canFoldAsLoad = 1, isReMaterializable = 1, mayLoad = 1, Defs = [SREG] in {
  def L__ : Pseudo<(outs GPR16:$reg),  (ins uniop:$ptr), "mov\t$ptr, $reg", []>;
  def LB_ : Pseudo<(outs GPR16l:$reg), (ins uniop:$ptr), "movb\t$ptr, $reg", []>;
}

let Defs = [SREG], Constraints = "$reg = $hi" in {
  def LU_ : Pseudo<(outs GPR16:$reg),  (ins GPR16:$hi, uniop:$lo), "movu\t$lo, $reg", []>;
}

let Defs = [SREG] in {
  def S__ : Pseudo<(outs), (ins GPR16:$reg,  uniop:$ptr), "mov\t$reg, $ptr", []>;
  def S__I : Pseudo<(outs GPR16:$wb), (ins GPR16:$reg,  uniop:$ptr), "mov\t$reg, $ptr", []>;
  def SB_ : Pseudo<(outs), (ins GPR16l:$reg, uniop:$ptr), "movb\t$reg, $ptr", []>;

  let Constraints = "$o = $ptr" in
    def SB_PD : Pseudo<(outs GPR16:$o), (ins uniop:$reg, GPR16:$ptr, i8imm:$m), "movb_sbpd\t$reg, $ptr, $m", []>;
}

let Constraints = "$rd = $rs", Defs = [SREG] in {
  def ASLREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "asl\t$rd", []>;
  def ASLBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rs), "aslb\t$rd", []>;

  def ASRREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "asr\t$rd", []>;
  def ASRBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rs), "asrb\t$rd", []>;

  def LSRREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "lsr\t$rd", []>;
  def LSRBREG : Pseudo<(outs GPR16l:$rd), (ins GPR16l:$rs), "lsrb\t$rd", []>;
}

let Defs = [SREG] in {
  def ASL : OneArgInst<51, (outs), (ins uniop16:$rd), "asl\t$rd", []>;
  def ASLB : OneArgInst<563, (outs), (ins uniop16:$rd), "aslb\t$rd", []>;

  def ASR : OneArgInst<50, (outs), (ins uniop16:$rd), "asr\t$rd", []>;
  def ASRB : OneArgInst<562, (outs), (ins uniop16:$rd), "asrb\t$rd", []>;
}

let Uses = [SREG], Defs = [SREG] in {
  def ROR : OneArgInst<48, (outs), (ins uniop16:$rd), "ror\t$rd", []>;
  def RORB : OneArgInst<560, (outs), (ins uniop16:$rd), "rorb\t$rd", []>;

  def ROL : OneArgInst<49, (outs), (ins uniop16:$rd), "rol\t$rd", []>;
  def ROLB : OneArgInst<561, (outs), (ins uniop16:$rd), "rolb\t$rd", []>;
}

def : Pat<(i16 (imm:$v)), (L__ R7, 2, (as_i16imm $v), 0)>;
def : Pat<(i8 (imm:$v)), (LB_ R7, 2, (as_i8imm $v), 0)>;

def : Pat<(i16 (PDPasl GPR16:$t)), (ASLREG $t)>;
def : Pat<(i8 (PDPasl GPR16l:$t)), (ASLBREG $t)>;

def : Pat<(i16 (PDPasr GPR16:$t)), (ASRREG $t)>;
def : Pat<(i16 (PDPlsr GPR16:$t)), (LSRREG $t)>;
def : Pat<(i8 (PDPlsr GPR16l:$t)), (LSRBREG $t)>;

//def : Pat<(i8(load(PDPWrapper tglobaladdr:$dst))), (LB__ R7, 3, tglobaladdr:$dst,0)>;

// Stack push/pop operations.
let Defs = [R6]/*, Uses = [R6]*/, hasSideEffects = 0 in {
  // Stack push operations.
  let mayStore = 1 in {
    def PUSH : Pseudo<(outs), (ins GPR16:$reg), "#push\t$reg", []>;
  }

  // Stack pop operations.
  let mayLoad = 1 in {
    def POP : Pseudo<(outs GPR16:$reg), (ins), "#popw\t$reg", []>;
  }
}

//===----------------------------------------------------------------------===//
// Bit and bit-test instructions
//===----------------------------------------------------------------------===//

let Constraints = "$src = $rd", Defs = [SREG] in
def SWABREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$src), "swab\t$rd", []>;

let Defs = [SREG] in
def SWAB : OneArgInst<3, (outs), (ins uniop16:$rd), "swab\t$rd", []>;

def : Pat<(i8(trunc (i16(srl i16:$v, (i8 8))))), (EXTRACT_SUBREG (SWABREG $v), sub_lo)>;
def : Pat<(i8(trunc (i16(PDPlsr (PDPlsr (PDPlsr (PDPlsr (PDPlsr (PDPlsr (PDPlsr (PDPlsr i16:$v))))))))))), (EXTRACT_SUBREG (SWABREG $v), sub_lo)>;

let Defs = [SREG] in {
  def CLC : SimpleInst<161, (outs), (ins), "clc", []>;
  def CLV : SimpleInst<162, (outs), (ins), "clv", []>;
  def CLZ : SimpleInst<164, (outs), (ins), "clz", []>;
  def CLN : SimpleInst<168, (outs), (ins), "cln", []>;
  def CCC : SimpleInst<175, (outs), (ins), "ccc", []>;

  def SEC : SimpleInst<177, (outs), (ins), "sec", []>;
  def SEV : SimpleInst<178, (outs), (ins), "sev", []>;
  def SEZ : SimpleInst<180, (outs), (ins), "sez", []>;
  def SEN : SimpleInst<184, (outs), (ins), "sen", []>;
  def SCC : SimpleInst<191, (outs), (ins), "scc", []>;
}

//===----------------------------------------------------------------------===//
// Pseudo instructions for later expansion
//===----------------------------------------------------------------------===//

let Defs = [SREG], hasSideEffects = 0 in
  def FRMIDX : Pseudo<(outs GPR16:$dst), (ins GPR16:$src, i16imm:$src2), "frmidx\t$dst, $src, $src2", []>;

def to_tframeindex : SDNodeXForm<frameindex, [{
  return CurDAG->getTargetFrameIndex(N->getIndex(), N->getValueType(0));
}]>;

def : Pat<(i16 (frameindex:$ptr)), (ADDREG R7, 2, (to_tframeindex $ptr), 0, (COPY R6))>;

def Select8 : SelectPseudo<(outs GPR16l:$dst),
                           (ins GPR16l:$src, GPR16l:$src2, i8imm:$cc),
                           "# Select8 PSEUDO", [(set i8:$dst, (PDPselectcc i8:$src, i8:$src2, imm:$cc))]>;

def Select16 : SelectPseudo<(outs GPR16:$dst),
                            (ins GPR16:$src, GPR16:$src2, i8imm:$cc),
                            "# Select16 PSEUDO", [(set i16:$dst, (PDPselectcc i16:$src, i16:$src2, imm:$cc))]>;

def Select2_8 : Select2Pseudo<(outs GPR16l:$dst),
                            (ins uniop:$lhs, uniop:$rhs, GPR16l:$tv, GPR16l:$fv, br_cond:$cc),
                            "# Select2_16 PSEUDO", [(set i8:$dst, (PDPselectcc2 uniSrc:$lhs, uniSrc:$rhs, i8:$tv, i8:$fv, imm:$cc))]>;

def Select2_16 : Select2Pseudo<(outs GPR16:$dst),
                            (ins uniop:$lhs, uniop:$rhs, GPR16:$tv, GPR16:$fv, br_cond:$cc),
                            "# Select2_16 PSEUDO", [(set i16:$dst, (PDPselectcc2 uniSrc:$lhs, uniSrc:$rhs, i16:$tv, i16:$fv, imm:$cc))]>;

let Defs = [SREG] in {
    def CLRREG : Pseudo<(outs GPR16:$rd), (ins), "clr\t$rd", [(set i16:$rd, 0)]>;
    def CLRBREG : Pseudo<(outs GPR16l:$rd), (ins), "clrb\t$rd", [(set i8:$rd, 0)]>;

    def CLR : OneArgInst<40, (outs), (ins uniop16:$rd), "clr\t$rd", []>;
    def CLRB : OneArgInst<552, (outs), (ins uniop16:$rd), "clrb\t$rd", []>;
}

// Перенос младшего байта в старший
def : Pat<(i16(PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl i16:$v))))))))), (INSERT_SUBREG (SWABREG $v), (CLRBREG), sub_lo)>;

def unaligned_op : PatFrag<(ops node:$ophi, node:$oplo), (i16(or (i16(PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl (PDPasl $ophi))))))))), $oplo))>;

def : Pat<(unaligned_op (i16(zext i8:$hi)), (i16 (zext uniSrc:$lo))), (LU_ (SWABREG (R16 $hi)), uniop:$lo)>;
def : Pat<(unaligned_op (i16(anyext i8:$hi)), (i16 (zext uniSrc:$lo))), (LU_ (SWABREG (R16 $hi)), uniop:$lo)>;
//def : Pat<(unaligned_op (i16(zext i8:$hi)), (i16 (zext i8:$lo))), (LU_ (SWAPREG (R16 $hi)), (R16 $lo), 0, 0, 0)>;
//def : Pat<(unaligned_op (i16(anyext i8:$hi)), (i16 (zext i8:$lo))), (LU_ (SWAPREG (R16 $hi)), (R16 $lo), 0, 0, 0)>;

def : Pat<(i16(load i16:$ptr)), (L__ $ptr,1,0,0)>;
def : Pat<(i16(load uniLoad:$ptr)), (L__ uniop:$ptr)>;

def : Pat<(i16(sextload uniLoad:$ptr)), (R16 (LB_ uniop:$ptr))>;  // При загрузке байта автоматически происходит знаковое расширение 16бит регистра
def : Pat<(i16(extload uniLoad:$ptr)), (R16 (LB_ uniop:$ptr))>; 

def : Pat<(i16(zext GPR16l:$v)), (BICREG R7, 2, 65280, 0, (R16 $v))>;

def : Pat<(i8(load i16:$ptr)), (LB_ $ptr,1,0,0)>;
def : Pat<(i8(load uniLoad:$ptr)), (LB_ uniop:$ptr)>;

def : Pat<(store (i16 0), uniStore:$ptr), (CLR uniop16:$ptr)>;
def : Pat<(store (i8 0), uniStore:$ptr), (CLRB uniop16:$ptr)>;

def : Pat<(post_store GPR16:$v, uniStore:$ptr, 2), (S__I $v, uniop:$ptr)>;
def : Pat<(store GPR16:$v, uniStore:$ptr), (S__ $v, uniop:$ptr)>;
def : Pat<(store GPR16:$v, GPR16:$ptr), (S__ $v, $ptr, 1, 0, 0)>;

def : Pat<(store GPR16l:$v, GPR16:$ptr), (SB_ $v, $ptr, 1, 0, 0)>;
def : Pat<(store GPR16l:$v, uniStore:$ptr),(SB_ $v, uniop:$ptr)>;

def : Pat<(store (i16(load (PDPWrapper tglobaladdr:$sp))), uniStore:$ptr), (MOV R7, 3, $sp, 0, uniop:$ptr)>;
def : Pat<(store (i16(PDPWrapper tglobaladdr:$sp)), uniStore:$ptr), (MOV R7, 2, $sp, 0, uniop:$ptr)>;
def : Pat<(store (i16(imm:$v)), uniStore:$ptr), (MOV R7, 2, (as_i16imm $v), 0, uniop:$ptr)>;
def : Pat<(store (i8(imm:$v)), uniStore:$ptr), (MOVB R7, 2, (as_i8imm $v), 0, uniop:$ptr)>;

//def : Pat<(i16 (pre_store (i8(uniSrc:$v)), GPR16:$ptr, -1)), (SB_PD uniop:$v, $ptr, 4)>;
//def : Pat<(i16 (pre_store (i8(imm:$v)), GPR16:$ptr, -1)), (SB_PD R7, 2, (as_i8imm $v), 0, $ptr, 4)>;
//def : Pat<(i16 (pre_store GPR16l:$v, GPR16:$ptr, -1)), (SB_PD $v, 0, 0, 0, $ptr, 4)>;

//===----------------------------------------------------------------------===//
// Non-Instruction Patterns
//===----------------------------------------------------------------------===//

//: TODO: look in x86InstrCompiler.td for odd encoding trick related to
// add x, 128 -> sub x, -128. Clang is emitting an eor for this (ldi+eor)

// all sub instruction variants always writes the carry flag
//def : Pat<(subc i8 : $src, i8 : $src2), (SUBRdRr i8 : $src, i8 : $src2)>;
//def : Pat<(subc i16 : $src, i16 : $src2), (SUBWRdRr i16 : $src, i16 : $src2)>;
//def : Pat<(subc i8 : $src, imm : $src2), (SUBIRdK i8 : $src, imm : $src2)>;
//def : Pat<(subc i16 : $src, imm : $src2), (SUBIWRdK i16 : $src, imm : $src2)>;

//def : Pat<(add i8:$src1, imm:$src2), (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
//def : Pat<(addc i8:$src1, imm:$src2), (SUBIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;
//def : Pat<(adde i8:$src1, imm:$src2), (SBCIRdK i8:$src1, (imm8_neg_XFORM imm:$src2))>;


// Emit NEGWRd with an extra zero register operand.
//def : Pat<(ineg i16:$src), (NEGWRd i16:$src, (CopyZero))>;

// Calls.
//let Predicates = [HasJMPCALL] in {
//  def : Pat<(PDPcall(i16 tglobaladdr:$dst)), (CALLk tglobaladdr:$dst)>;
//  def : Pat<(PDPcall(i16 texternalsym:$dst)), (CALLk texternalsym:$dst)>;
//}

// `anyext`
//def ANYEXT : Pseudo<(outs GPR16:$rd), (ins GPR16l:$rs), "any\t$rs -> $rd", [(set i16:$rd, (i16(anyext i8:$rs)))]>;

def : Pat<(i16(anyext i8:$src)), (R16 i8:$src)>;

// `trunc`
def : Pat<(i8(trunc i16 : $src)), (EXTRACT_SUBREG i16 : $src, sub_lo)>;

// sext_inreg
//def : Pat<(sext_inreg i16:$src, i8), (SEXT(i8(EXTRACT_SUBREG i16 : $src, sub_lo)))>;

// GlobalAddress
//def : Pat<(i16(PDPWrapper tglobaladdr:$dst)), (LDIWRdK tglobaladdr:$dst)>;

//def : Pat<(add i16 : $src, (PDPWrapper tglobaladdr : $src2)), (SUBIWRdK i16 : $src, tglobaladdr : $src2)>;

//def : Pat<(i8(load(PDPWrapper tglobaladdr:$dst))), (LDSRdK tglobaladdr:$dst)>;
//def : Pat<(i16(load(PDPWrapper tglobaladdr:$dst))), (LDSWRdK_ tglobaladdr:$dst)>;
//def : Pat<(store i8:$src, (i16(PDPWrapper tglobaladdr:$dst))), (STSKRr tglobaladdr:$dst, i8:$src)>;
//def : Pat<(store i16:$src, (i16(PDPWrapper tglobaladdr:$dst))), (STSWKRr_ tglobaladdr:$dst, i16:$src)>;

//def : Pat<(store (i16(load(i16(PDPWrapper(tglobaladdr:$src))))), (i16(PDPWrapper tglobaladdr:$dst))), (LDST_ tglobaladdr:$dst, tglobaladdr:$src)>;
//def : Pat<(store (i16(imm:$src)), (i16(PDPWrapper tglobaladdr:$dst))), (LDST_ tglobaladdr:$dst, imm:$src)>;

// BlockAddress
//def : Pat<(i16(PDPWrapper tblockaddress : $dst)), (LDIWRdK tblockaddress : $dst)>;

//def : Pat<(i8(trunc(PDPlsrwn DLDREGS : $src, (i16 8)))), (EXTRACT_SUBREG DREGS : $src, sub_hi)>;

// :FIXME: DAGCombiner produces an shl node after legalization from these seq:
// BR_JT -> (mul x, 2) -> (shl x, 1)
//def : Pat<(shl i16 : $src1, (i8 1)), (LSLWRd i16 : $src1)>;

// Lowering of 'tst' node to 'TST' instruction.
// TST is an alias of AND Rd, Rd.
//def : Pat<(PDPtst i8 : $rd), (ANDRdRr GPR8 : $rd, GPR8 : $rd)>;

// Lowering of 'lsl' node to 'LSL' instruction.
// LSL is an alias of 'ADD Rd, Rd'
//def : Pat<(PDPlsl i8 : $rd), (ADDRdRr GPR8 : $rd, GPR8 : $rd)>;

//def : Pat<(ineg (load_mem tglobaladdr:$t)), (NEGWRd (L__ R7, 3, $t, 0))>;
//def : Pat<(inegc i16:$t), (NEGWRd $t)>;

def : Pat<(PDPWrapper tglobaladdr:$addr), (L__ R7, 2, $addr, 0)>;

let Uses = [SREG], Defs = [SREG] in
  def SXT : Pseudo<(outs GPR16:$hi), (ins unknown:$lo), "#sxt $lo, $hi", []>;

let Constraints = "$rs = $rd", Defs = [SREG] in {
    def SXTREG : Pseudo<(outs GPR16:$rd), (ins GPR16:$rs), "sxt\t$rd", []>;
}

def : Pat<(i16(sext GPR16l:$v)), (SXTREG (R16 $v))>;

let Defs = [SREG], isCompare = 1 /*, isTerminator = 1*/ in {
  def TSTREG : Pseudo<(outs), (ins GPR16:$s), "tst\t$s", []>;
  def TSTBREG : Pseudo<(outs), (ins GPR16l:$s), "tstb\t$s", []>;

  //def NETST : Pseudo<(outs), (ins uniop16:$s), "need_eliminate", []>;

  def TST : OneArgInst<47, (outs), (ins uniop16:$rd), "tst\t$rd", []>;
  def TSTB : OneArgInst<559, (outs), (ins uniop16:$rd), "tstb\t$rd", []>;
}

def : Pat<(PDPsxt i16:$s), (SXT GPR16:$s)>;
def : Pat<(PDPsxt i8:$s), (SXT GPR16l:$s)>;

def : Pat<(PDPtst i16:$rd), (TSTREG GPR16:$rd)>;
def : Pat<(PDPtst i8:$rd), (TSTBREG GPR16l:$rd)>;

def : Pat<(PDPtst (i16 uniSrc:$rd)), (TST uniop:$rd)>;
def : Pat<(PDPtst (i8 uniSrc:$rd)), (TSTB uniop:$rd)>;

/// RMW
class RMWPat<VTInt T, SDNode Op, PDPInst I> : Pat<(store (T (Op uniSrc:$rs, (load uniRmw:$rd))), uniRmw:$rd), (I uniop:$rs, uniop:$rd)>;

def : Pat<(store (i16 (load uniRmw:$rs)), uniRmw:$rd), (MOV uniop:$rs, uniop:$rd)>;
def : Pat<(store (i8 (load uniRmw:$rs)), uniRmw:$rd), (MOVB uniop:$rs, uniop:$rd)>;

//def : Pat<(store (i16 (add uniSrc:$rs, (load uniRmw:$rd))), uniRmw:$rd), (ADD uniop:$rs, uniop:$rd)>;
def : RMWPat<i16, add, ADD>;

def : Pat<(store (i16 (add (load uniRmw:$rd), 1)), uniRmw:$rd), (INC uniop:$rd)>;
def : Pat<(store (i8 (add (load uniRmw:$rd), 1)), uniRmw:$rd), (INCB uniop:$rd)>;

def : Pat<(store (i16 (add (load uniRmw:$rd), -1)), uniRmw:$rd), (DEC uniop:$rd)>;
def : Pat<(store (i8 (add (load uniRmw:$rd), -1)), uniRmw:$rd), (DECB uniop:$rd)>;

def : Pat<(store (i16 (ineg (load uniRmw:$rd))), uniRmw:$rd), (NEG uniop:$rd)>;
def : Pat<(store (i8 (ineg (load uniRmw:$rd))), uniRmw:$rd), (NEGB uniop:$rd)>;

//def : Pat<(store (i16 (or uniSrc:$rs, (load uniRmw:$rd))), uniRmw:$rd), (BIS uniop:$rs, uniop:$rd)>;
//def : Pat<(store (i8 (or uniSrc:$rs, (load uniRmw:$rd))), uniRmw:$rd), (BISB uniop:$rs, uniop:$rd)>;
def : RMWPat<i16, or, BIS>;
def : RMWPat<i8, or, BISB>;

def : Pat<(store (i16 (and (load uniRmw:$rd), imm:$rs)), uniRmw:$rd), (BIC R7, 2, (imm16_not_XFORM imm:$rs), 0, uniop:$rd)>;
def : Pat<(store (i8 (and (load uniRmw:$rd), imm:$rs)), uniRmw:$rd), (BICB R7, 2, (imm8_not_XFORM imm:$rs), 0, uniop:$rd)>;

def : Pat<(store (i16 (xor GPR16:$rs, (load uniRmw:$rd))), uniRmw:$rd), (XOR $rs, uniop:$rd)>;

// EMT
let Defs = [SREG] in {
    def EMT : ImmArgInst<136, (outs), (ins i16imm:$n, variable_ops), "emt\t$n", []>;
    def TRAP : ImmArgInst<137, (outs), (ins i16imm:$n, variable_ops), "trap\t$n", []>;
}

def HALT : SimpleInst<0, (outs), (ins), "halt", []>;
def WAIT : SimpleInst<1, (outs), (ins), "wait", []>;
def RTI : SimpleInst<2, (outs), (ins), "rti", []>;
def BPT : SimpleInst<3, (outs), (ins), "bpt", []>;
def IOT : SimpleInst<4, (outs), (ins), "iot", []>;
def RESET : SimpleInst<5, (outs), (ins), "reset", []>;
def RTT : SimpleInst<6, (outs), (ins), "rtt", []>;

def START : SimpleInst<10, (outs), (ins), "start", []>;
def STEP : SimpleInst<14, (outs), (ins), "step", []>;

def NOP : SimpleInst<160, (outs), (ins), "nop", []>;

def MTPS : OneArgInst<564, (outs), (ins uniop16:$rd), "mtps\t$rd", []>;
def MFPS : OneArgInst<567, (outs), (ins uniop16:$rd), "mfps\t$rd", []>;
def MFPSREG : Pseudo<(outs GPR16:$rd), (ins), "mfps\t$rd", [(set i16:$rd, (int_pdp11_read_psw))]>;


