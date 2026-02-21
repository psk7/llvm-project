//===-- PDPInstrInfo.td - PDP Instruction Formats ----------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// PDP Instruction Format Definitions.
//
//===----------------------------------------------------------------------===//

// A generic PDP instruction.
class PDPInst<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Instruction {
  let Namespace = "PDP";

  dag OutOperandList = outs;
  dag InOperandList = ins;
  let AsmString = asmstr;
  let Pattern = pattern;

  let Size = 2;

  field bits<32> SoftFail = 0;
}

class XorInst<dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<3> rs;
  bits<6> rd;

  let Inst{15 - 9} = 60;
  let Inst{8 - 6} = rs;
  let Inst{5 - 0} = rd;
}

class SobInst<dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<3> r;
  bits<6> k;

  let Inst{15 - 9} = 63;
  let Inst{8 - 6} = r;
  let Inst{5 - 0} = k;
}

class JsrInst<dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<3> r;
  bits<6> k;

  let Inst{15 - 9} = 4;
  let Inst{8 - 6} = r;
  let Inst{5 - 0} = k;

  let TSFlags{2-0} = 3;
}

class TwoArgInst<bits<4> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<6> rs;
  bits<6> rd;

  let Inst{15 - 12} = opcode;
  let Inst{11 - 6} = rs;
  let Inst{5 - 0} = rd;

  let TSFlags{2-0} = 1; // Инструкции с двумя аргументами
}

class OneArgInst<bits<10> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<6> rd;

  let Inst{15 - 6} = opcode;
  let Inst{5 - 0} = rd;

  let TSFlags{2-0} = 2; // Инструкции с одним аргументом
}

class ImmArgInst<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<8> n;

  let Inst{15 - 8} = opcode;
  let Inst{7 - 0} = n;
}

class BranchInst<bits<8> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  bits<8> k;

  let Inst{15 - 8} = opcode;
  let Inst{7 - 0} = k;
}

class SimpleInst<bits<16> opcode, dag outs, dag ins, string asmstr, list<dag> pattern> : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;

  let Inst = opcode;
}

class RetInst<dag outs, dag ins, string asmstr> : PDPInst<outs, ins, asmstr, []> {
  field bits<16> Inst;

  bits<3> r;

  let Inst{15-3} = 16;
  let Inst{2-0} = r;
}

class PDPInst16<dag outs, dag ins, string asmstr, list<dag> pattern>
    : PDPInst<outs, ins, asmstr, pattern> {
  field bits<16> Inst;
}

class Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : PDPInst16<outs, ins, asmstr, pattern> {
  let Pattern = pattern;

  let isPseudo = 1;
  let isCodeGenOnly = 1;
}

class F16<bits<16> opcode, dag outs, dag ins, string asmstr, list<dag> pattern>
    : PDPInst16<outs, ins, asmstr, pattern> {
  let Inst = opcode;
}

class ExtensionPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Pseudo<outs, ins, asmstr, pattern> {
  let Defs = [SREG];
}

class StorePseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Pseudo<outs, ins, asmstr, pattern> {
  let Defs = [R6];
}

class SelectPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Pseudo<outs, ins, asmstr, pattern> {
  let usesCustomInserter = 1;

  let Uses = [SREG];
}

class Select2Pseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Pseudo<outs, ins, asmstr, pattern> {
  let usesCustomInserter = 1;
}

class ShiftPseudo<dag outs, dag ins, string asmstr, list<dag> pattern>
    : Pseudo<outs, ins, asmstr, pattern> {
  let usesCustomInserter = 1;

  let Defs = [SREG];
}
