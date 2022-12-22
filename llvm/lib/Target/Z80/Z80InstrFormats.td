//===-- Z80InstrInfo.td - Z80 Instruction Formats ----------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Z80 Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

// Class specifying the prefix used an opcode extension.
class Prefix<bits<3> val> {
  bits<3> Value = val;
}

def NoPrfx : Prefix<0>;
def CB     : Prefix<1>;
def ED     : Prefix<2>;
def DD     : Prefix<3>;
def FD     : Prefix<4>;
def DDCB   : Prefix<5>;
def FDCB   : Prefix<6>;

class CB   { Prefix OpPrefix = CB; }
class ED   { Prefix OpPrefix = ED; }
class DD   { Prefix OpPrefix = DD; }
class FD   { Prefix OpPrefix = FD; }
class DDCB { Prefix OpPrefix = DDCB; }
class FDCB { Prefix OpPrefix = FDCB; }

// A generic Z80 instruction.
class Z80Inst<dag outs, dag ins, string asmstr, list<dag> pattern> : Instruction
{
  let Namespace = "Z80";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  bits<1> IsPtrInstruction = 0;
  Prefix OpPrefix = NoPrfx;
  bits<3> OpPrefixBits = OpPrefix.Value;

  bits<2> payloadSize = 0;
  bits<1> HasDisplacement = 0;

  let TSFlags{6} = HasDisplacement;
  let TSFlags{5-4} = payloadSize;
  let TSFlags{3} = IsPtrInstruction;
  let TSFlags{2-0} = OpPrefixBits;

  field bits<32> SoftFail = 0;
}

class IsPtrInstr {
  bits<1> IsPtrInstruction = 1;
}

class HasDisplacement {
  bits<1> HasDisplacement = 1;
}

class Has8BitPayload { bits<2> payloadSize = 1; }
class Has16BitPayload { bits<2> payloadSize = 2; }

class Form8<bits<8> opcode> {
  field bits<8> Inst;
  int Size = 1;
  let Inst{7-0} = opcode;
}

class Form16<bits<8> opcode> {
  field bits<16> Inst;
  int Size = 2;
  let Inst{7-0} = opcode;
}

class Form24<bits<8> opcode> {
  field bits<24> Inst;
  int Size = 3;
  let Inst{7-0} = opcode;
}

class FormA<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Form8<opcode>
{
}

class FormA_ED<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xED;
  let Inst{15-8} = opcode;
  let Size = 2;
}

class FormB<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Form8<opcode>
{
  bits<3> rr;

  let Inst{2-0} = rr;

  let DecoderMethod = "DecodeFormB";
}

class FormB_CB<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xCB;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<3> rr;

  let Inst{10-8} = rr;
}

class FormIM<bits<8> opcode, string asmstr>
  : Z80Inst<(outs), (ins i8imm:$n), asmstr, []>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xED;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<2> n;

  let Inst{12-11} = n;

  let DecoderMethod="DecodeIM";
}

class FormC<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<2> rr;

  let Inst{5-4} = rr;
}

class FormHLRP<bits<8> opcode, string asmstr, list<dag> pattern>
  : FormA<opcode, (outs HLREG:$rd), (ins HLREG:$src, BDREGS:$rr), asmstr, pattern>
{
  let Constraints = "$src = $rd";
  bits<2> rr;
  let Inst{5-4} = rr;

  let DecoderMethod = "DecodeHLRP";
}

class FormC_ED<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xED;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<2> rr;

  let Inst{13-12} = rr;

  let DecoderMethod = "DecodeFormC_ED";
}

class FormD<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has8BitPayload, Form16<opcode>
{
  bits<3> rd;
  bits<8> d8;

  let Inst{15-8} = d8;
  let Inst{5-3} = rd;
}

class FormE<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has16BitPayload, Form24<opcode>
{
  bits<2> rd;
  bits<16> d16;

  let Inst{23-16} = d16{15-8};
  let Inst{15-8} = d16{7-0};
  let Inst{5-4} = rd;
}

class FormE_ED<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has16BitPayload
{
  field bits<32> Inst;
  int Size = 4;

  bits<2> rd;
  bits<16> d16;

  let Inst{31-16} = d16;
  let Inst{15-8} = opcode;
  let Inst{7-0} = 0xED;
  let Inst{13-12} = rd;
}

class FormF<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has16BitPayload, Form24<opcode>
{
  bits<16> d16;

  let Inst{23-8} = d16;
}

class FormG<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rd;
  bits<3> rr;

  let Inst{5-3} = rd;
  let Inst{2-0} = rr;
}

class FormG_CB<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xCB;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<3> rd;
  bits<3> rr;

  let Inst{13-11} = rd;
  let Inst{10-8} = rr;
}

class FormH<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rd;

  let Inst{5-3} = rd;
}

class FormH_ED<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xED;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<3> rd;

  let Inst{13-11} = rd;
}

class FormH_CB<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  field bits<16> Inst;

  let Inst{7-0} = 0xCB;
  let Inst{15-8} = opcode;
  let Size = 2;

  bits<3> rd;

  let Inst{13-11} = rd;
}

class FormI<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has8BitPayload, Form16<opcode>
{
  bits<8> d8;

  let Inst{15-8} = d8;
}

class FormIA<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormI<opcode, outs, ins, asmstr, pattern>
{
    let DecoderMethod = "DecodeFormIA";
}

class FormJ<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has8BitPayload, Form16<opcode>
{
  bits<2> cc;
  bits<8> d8;

  let Inst{15-8} = d8;
  let Inst{4-3} = cc;
}

class FormK<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has8BitPayload, Form16<opcode>
{
  bits<8> d8;
  bits<3> rr;

  let Inst{15-8} = d8;
  let Inst{2-0} = rr;
}

class FormL<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>, Has16BitPayload, Form24<opcode>
{
  bits<3> cc;
  bits<16> d16;

  let Inst{23-8} = d16;
  let Inst{5-3} = cc;
}

// A class for pseudo instructions.
// Pseudo instructions are not real Z80 instructions. The DAG stores
// pseudo instructions which are replaced by real Z80 instructions by
// Z80ExpandPseudoInsts.cpp.
class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  let Pattern = pattern;

  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

class FRdK : Instruction { let isAsCheapAsAMove = 1; }
class FSTLD : Instruction { let PostEncoderMethod = "loadStorePostEncoder"; }

class StorePseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let Defs = [SP];
}

class SelectPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let usesCustomInserter = 1;

  let Uses = [SREG];
}

class ShiftPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let usesCustomInserter = 1;

  let Defs = [SREG];
}
