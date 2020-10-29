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

  let TSFlags{3} = IsPtrInstruction;
  let TSFlags{2-0} = OpPrefixBits;

  field bits<64> Inst;

  field bits<32> SoftFail = 0;
}

class IsPtrInstr {
  bits<1> IsPtrInstruction = 1;
}

class FormA<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : Z80Inst<outs, ins, asmstr, pattern>
{
  let Inst{7-0} = opcode;
  let Size = 1;
}

class FormB<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rr;

  let Inst{2-0} = rr;
}

class FormC<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<2> rr;

  let Inst{5-4} = rr;
}

class FormD<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rd;
  bits<8> d8;

  let Inst{15-8} = d8;
  let Inst{5-3} = rd;
  let Size = 2;
}

class FormE<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<2> rd;
  bits<16> d16;

  let Inst{23-16} = d16{15-8};
  let Inst{15-8} = d16{7-0};
  let Inst{5-4} = rd;
  let Size = 3;
}

class FormF<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<16> d16;

  let Inst{23-16} = d16{15-8};
  let Inst{15-8} = d16{7-0};
  let Size = 3;
}

class FormG<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rd;
  bits<3> rr;

  let Inst{5-3} = rd;
  let Inst{2-0} = rr;
  let Size = 1;
}

class FormH<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<3> rd;

  let Inst{5-3} = rd;
  let Size = 1;
}

class FormI<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<8> d8;

  let Inst{15-8} = d8;
  let Size = 2;
}

class FormJ<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<2> cc;
  bits<8> d8;

  let Inst{15-8} = d8;
  let Inst{4-3} = cc;
  let Size = 2;
}

class FormK<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
  : FormA<opcode, outs, ins, asmstr, pattern>
{
  bits<8> d8;
  bits<3> rr;

  let Inst{15-8} = d8;
  let Inst{2-0} = rr;
  let Size = 2;
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

class ExtensionPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
  : Pseudo<outs, ins, asmstr, pattern>
{
  let Defs = [SREG];
}

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
