//===-- PDPISelLowering.cpp - PDP DAG Lowering Implementation -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that PDP uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "PDPISelLowering.h"

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "PDP.h"
#include "PDPMachineFunctionInfo.h"
#include "PDPSubtarget.h"
#include "PDPTargetMachine.h"

namespace llvm {

PDPTargetLowering::PDPTargetLowering(const PDPTargetMachine &TM,
                                     const PDPSubtarget &STI)
    : TargetLowering(TM), Subtarget(STI) {

  addRegisterClass(MVT::i8, &PDP::GPR16lRegClass);
  addRegisterClass(MVT::i16, &PDP::GPR16RegClass);

  // Compute derived properties from the register classes.

  setBooleanContents(UndefinedBooleanContent);
  setBooleanVectorContents(UndefinedBooleanContent);
  setSchedulingPreference(Sched::RegPressure);
  setStackPointerRegisterToSaveRestore(PDP::R6);
  setSupportsUnalignedAtomics(false);

  setOperationAction(ISD::GlobalAddress, MVT::i16, Custom);
  setOperationAction(ISD::BlockAddress, MVT::i16, Custom);

  setOperationAction(ISD::STACKSAVE, MVT::Other, Expand);
  setOperationAction(ISD::STACKRESTORE, MVT::Other, Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i8, Expand);
  setOperationAction(ISD::DYNAMIC_STACKALLOC, MVT::i16, Expand);

  setOperationAction(ISD::INLINEASM, MVT::Other, Custom);

  // setOperationAction(ISD::SIGN_EXTEND, MVT::i8, Custom);
  // setOperationAction(ISD::SIGN_EXTEND, MVT::i16, Custom);
  //setOperationAction(ISD::SIGN_EXTEND, MVT::i32, Custom);
  //setOperationAction(ISD::LOAD, MVT::i32, Custom);

  for (MVT VT : MVT::integer_valuetypes()) {
    for (auto N : {ISD::EXTLOAD, ISD::SEXTLOAD, ISD::ZEXTLOAD}) {
      setLoadExtAction(N, VT, MVT::i1, Promote);
      setLoadExtAction(N, VT, MVT::i8, Promote);
    }
  }

  setLoadExtAction(ISD::SEXTLOAD, MVT::i16, MVT::i8, Legal);
  setLoadExtAction(ISD::ZEXTLOAD, MVT::i16, MVT::i8, Expand);
  setLoadExtAction(ISD::EXTLOAD, MVT::i16, MVT::i8, Legal);

  setTruncStoreAction(MVT::i16, MVT::i8, Expand);

  for (MVT VT : MVT::integer_valuetypes()) {
    setOperationAction(ISD::ADDC, VT, Legal);
    setOperationAction(ISD::SUBC, VT, Legal);
    setOperationAction(ISD::ADDE, VT, Legal);
    setOperationAction(ISD::SUBE, VT, Legal);
  }

  // sub (x, imm) gets canonicalized to add (x, -imm), so for illegal types
  // revert into a sub since we don't have an add with immediate instruction.
  // setOperationAction(ISD::ADD, MVT::i32, Custom);
  // setOperationAction(ISD::ADD, MVT::i64, Custom);
  setOperationAction(ISD::ADD, MVT::i8, Promote);
  setOperationAction(ISD::ADDC, MVT::i8, Promote);
  setOperationAction(ISD::ADDE, MVT::i8, Promote);
  //setOperationAction(ISD::ADD, MVT::i64, Expand);

  // our shift instructions are only able to shift 1 bit at a time, so handle
  // this in a custom way.
  setOperationAction(ISD::SRA, MVT::i8, Custom);
  setOperationAction(ISD::SHL, MVT::i8, Custom);
  setOperationAction(ISD::SRL, MVT::i8, Custom);
  setOperationAction(ISD::SRA, MVT::i16, Custom);
  setOperationAction(ISD::SHL, MVT::i16, Custom);
  setOperationAction(ISD::SRL, MVT::i16, Custom);
  //setOperationAction(ISD::SRA, MVT::i32, Custom);
  //setOperationAction(ISD::SHL, MVT::i32, Custom);
  //setOperationAction(ISD::SRL, MVT::i32, Custom);
  setOperationAction(ISD::SHL_PARTS, MVT::i16, Expand);
  setOperationAction(ISD::SRA_PARTS, MVT::i16, Expand);
  setOperationAction(ISD::SRL_PARTS, MVT::i16, Expand);

  setOperationAction(ISD::ROTL, MVT::i8, Custom);
  setOperationAction(ISD::ROTL, MVT::i16, Expand);
  setOperationAction(ISD::ROTR, MVT::i8, Custom);
  setOperationAction(ISD::ROTR, MVT::i16, Expand);

  setOperationAction(ISD::BR_CC, MVT::i8, Custom);
  setOperationAction(ISD::BR_CC, MVT::i16, Custom);
  //setOperationAction(ISD::BR_CC, MVT::i32, Custom);
  //setOperationAction(ISD::BR_CC, MVT::i64, Custom);
  setOperationAction(ISD::BRCOND, MVT::Other, Expand);

  setOperationAction(ISD::SELECT_CC, MVT::i8, Custom);
  setOperationAction(ISD::SELECT_CC, MVT::i16, Custom);
  //setOperationAction(ISD::SELECT_CC, MVT::i32, Expand);
  //setOperationAction(ISD::SELECT_CC, MVT::i64, Expand);
  setOperationAction(ISD::SETCC, MVT::i8, Custom);
  setOperationAction(ISD::SETCC, MVT::i16, Custom);
  //setOperationAction(ISD::SETCC, MVT::i32, Custom);
  //setOperationAction(ISD::SETCC, MVT::i64, Custom);
  setOperationAction(ISD::SELECT, MVT::i8, Expand);
  setOperationAction(ISD::SELECT, MVT::i16, Expand);

  setOperationAction(ISD::BSWAP, MVT::i16, Expand);

  // Add support for postincrement and predecrement load/stores.
  setIndexedLoadAction(ISD::POST_INC, MVT::i8, Legal);
  setIndexedLoadAction(ISD::POST_INC, MVT::i16, Legal);
  setIndexedLoadAction(ISD::PRE_DEC, MVT::i8, Legal);
  setIndexedLoadAction(ISD::PRE_DEC, MVT::i16, Legal);

  setIndexedStoreAction(ISD::POST_INC, MVT::i8, Legal);
  setIndexedStoreAction(ISD::POST_INC, MVT::i16, Legal);
  setIndexedStoreAction(ISD::PRE_DEC, MVT::i8, Legal);
  setIndexedStoreAction(ISD::PRE_DEC, MVT::i16, Legal);

  setOperationAction(ISD::BR_JT, MVT::Other, Expand);

  setOperationAction(ISD::VASTART, MVT::Other, Custom);
  setOperationAction(ISD::VAEND, MVT::Other, Expand);
  setOperationAction(ISD::VAARG, MVT::Other, Expand);
  setOperationAction(ISD::VACOPY, MVT::Other, Expand);

  // Atomic operations which must be lowered to rtlib calls
/*  for (MVT VT : MVT::integer_valuetypes()) {
    setOperationAction(ISD::ATOMIC_SWAP, VT, Expand);
    setOperationAction(ISD::ATOMIC_CMP_SWAP, VT, Expand);
    setOperationAction(ISD::ATOMIC_LOAD_NAND, VT, Expand);
    setOperationAction(ISD::ATOMIC_LOAD_MAX, VT, Expand);
    setOperationAction(ISD::ATOMIC_LOAD_MIN, VT, Expand);
    setOperationAction(ISD::ATOMIC_LOAD_UMAX, VT, Expand);
    setOperationAction(ISD::ATOMIC_LOAD_UMIN, VT, Expand);
  }*/

  // Division/remainder
  //setOperationAction(ISD::UDIV, MVT::i8, Expand);
  //setOperationAction(ISD::UDIV, MVT::i16, Expand);
  //setOperationAction(ISD::UREM, MVT::i8, Expand);
  //setOperationAction(ISD::UREM, MVT::i16, Expand);
  //setOperationAction(ISD::SDIV, MVT::i8, Expand);
  //setOperationAction(ISD::SDIV, MVT::i16, Expand);
  //setOperationAction(ISD::SREM, MVT::i8, Expand);
  //setOperationAction(ISD::SREM, MVT::i16, Expand);

  // Make division and modulus custom
  //setOperationAction(ISD::UDIVREM, MVT::i8, Custom);
  //setOperationAction(ISD::UDIVREM, MVT::i16, Custom);
  //setOperationAction(ISD::UDIVREM, MVT::i32, Custom);
  //setOperationAction(ISD::SDIVREM, MVT::i8, Custom);
  //setOperationAction(ISD::SDIVREM, MVT::i16, Custom);
  //setOperationAction(ISD::SDIVREM, MVT::i32, Custom);

  setOperationAction(ISD::MUL, MVT::i8, /*Expand*/Promote);
  setOperationAction(ISD::MUL, MVT::i16, /*Expand*/LibCall);

  setOperationAction(ISD::UDIV, MVT::i8, /*Expand*/Promote);
  setOperationAction(ISD::UDIV, MVT::i16, /*Expand*/LibCall);

  // Expand 16 bit multiplications.
  setOperationAction(ISD::SMUL_LOHI, MVT::i16, /*Expand*/LibCall);
  setOperationAction(ISD::UMUL_LOHI, MVT::i16, /*Expand*/LibCall);

  // Expand multiplications to libcalls when there is
  // no hardware MUL.
  /*if (!Subtarget.supportsMultiplication()) {
    setOperationAction(ISD::SMUL_LOHI, MVT::i8, Expand);
    setOperationAction(ISD::UMUL_LOHI, MVT::i8, Expand);
  }*/

  for (MVT VT : MVT::integer_valuetypes()) {
    setOperationAction(ISD::MULHS, VT, Expand);
    setOperationAction(ISD::MULHU, VT, Expand);
  }

  for (MVT VT : MVT::integer_valuetypes()) {
    setOperationAction(ISD::CTPOP, VT, Expand);
    setOperationAction(ISD::CTLZ, VT, Expand);
    setOperationAction(ISD::CTTZ, VT, Expand);
  }

  for (MVT VT : MVT::integer_valuetypes()) {
    setOperationAction(ISD::SIGN_EXTEND_INREG, VT, Expand);
    // TODO: The generated code is pretty poor. Investigate using the
    // same "shift and subtract with carry" trick that we do for
    // extending 8-bit to 16-bit. This may require infrastructure
    // improvements in how we treat 16-bit "registers" to be feasible.
  }

  // Division and modulus rtlib functions
  //setLibcallName(RTLIB::SDIVREM_I8, "__divmodqi4");
  //setLibcallName(RTLIB::SDIVREM_I16, "__divmodhi4");
  //setLibcallName(RTLIB::SDIVREM_I32, "__divmodsi4");
  //setLibcallName(RTLIB::UDIVREM_I8, "__udivmodqi4");
  //setLibcallName(RTLIB::UDIVREM_I16, "__udivmodhi4");
  //setLibcallName(RTLIB::UDIVREM_I32, "__udivmodsi4");

  // Several of the runtime library functions use a special calling conv
  //setLibcallCallingConv(RTLIB::SDIVREM_I8, CallingConv::PDP_BUILTIN);
  //setLibcallCallingConv(RTLIB::SDIVREM_I16, CallingConv::PDP_BUILTIN);
  //setLibcallCallingConv(RTLIB::UDIVREM_I8, CallingConv::PDP_BUILTIN);
  //setLibcallCallingConv(RTLIB::UDIVREM_I16, CallingConv::PDP_BUILTIN);

  // Trigonometric rtlib functions
  setLibcallName(RTLIB::SIN_F32, "sin");
  setLibcallName(RTLIB::COS_F32, "cos");

  setMinFunctionAlignment(Align(2));
  setMinimumJumpTableEntries(UINT_MAX);

  computeRegisterProperties(Subtarget.getRegisterInfo());
}

const char *PDPTargetLowering::getTargetNodeName(unsigned Opcode) const {
#define NODE(name)                                                             \
  case PDPISD::name:                                                           \
    return #name

  switch (Opcode) {
  default:
    return nullptr;
    NODE(RET_GLUE);
    NODE(RETI_GLUE);
    NODE(CALL);
    NODE(WRAPPER);
    NODE(ASL);
    NODE(ASR);
//    NODE(LSL);
//    NODE(LSLW);
    NODE(LSR);
//    NODE(LSRW);
    NODE(ROL);
    NODE(ROR);
//    NODE(ASRW);
    NODE(LSLLOOP);
    NODE(LSRLOOP);
    NODE(ROLLOOP);
    NODE(RORLOOP);
    NODE(ASRLOOP);
    NODE(BRCOND);
    NODE(BRCOND2);
    NODE(BRCOND2B);
    NODE(BRBIT);
    NODE(BRBITB);
    NODE(CMP);
    NODE(CMPC);
    NODE(TST);
    NODE(SELECT_CC);
    NODE(SELECT_CC2);
    NODE(SXT);
#undef NODE
  }
}

EVT PDPTargetLowering::getSetCCResultType(const DataLayout &DL, LLVMContext &,
                                          EVT VT) const {
  assert(!VT.isVector() && "No PDP SetCC type for vectors!");
  return MVT::i16;
}

SDValue PDPTargetLowering::LowerShifts(SDValue Op, SelectionDAG &DAG) const {
  unsigned Opc8;
  const SDNode *N = Op.getNode();
  EVT VT = Op.getValueType();
  SDLoc dl(N);
  assert(llvm::has_single_bit<uint32_t>(VT.getSizeInBits()) &&
         "Expected power-of-2 shift amount");

  /*if (VT.getSizeInBits() == 32) {
    if (!isa<ConstantSDNode>(N->getOperand(1))) {
      // 32-bit shifts are converted to a loop in IR.
      // This should be unreachable.
      report_fatal_error("Expected a constant shift amount!");
    }
    SDVTList ResTys = DAG.getVTList(MVT::i16, MVT::i16);
    SDValue SrcLo =
        DAG.getNode(ISD::EXTRACT_ELEMENT, dl, MVT::i16, Op.getOperand(0),
                    DAG.getConstant(0, dl, MVT::i16));
    SDValue SrcHi =
        DAG.getNode(ISD::EXTRACT_ELEMENT, dl, MVT::i16, Op.getOperand(0),
                    DAG.getConstant(1, dl, MVT::i16));
    uint64_t ShiftAmount = N->getConstantOperandVal(1);
    if (ShiftAmount == 16) {
      // Special case these two operations because they appear to be used by the
      // generic codegen parts to lower 32-bit numbers.
      // TODO: perhaps we can lower shift amounts bigger than 16 to a 16-bit
      // shift of a part of the 32-bit value?
      switch (Op.getOpcode()) {
      case ISD::SHL: {
        SDValue Zero = DAG.getConstant(0, dl, MVT::i16);
        return DAG.getNode(ISD::BUILD_PAIR, dl, MVT::i32, Zero, SrcLo);
      }
      case ISD::SRL: {
        SDValue Zero = DAG.getConstant(0, dl, MVT::i16);
        return DAG.getNode(ISD::BUILD_PAIR, dl, MVT::i32, SrcHi, Zero);
      }
      }
    }
    SDValue Cnt = DAG.getTargetConstant(ShiftAmount, dl, MVT::i8);
    unsigned Opc;
    switch (Op.getOpcode()) {
    default:
      llvm_unreachable("Invalid 32-bit shift opcode!");
    case ISD::SHL:
      Opc = PDPISD::LSLW;
      break;
    case ISD::SRL:
      Opc = PDPISD::LSRW;
      break;
    case ISD::SRA:
      Opc = PDPISD::ASRW;
      break;
    }
    SDValue Result = DAG.getNode(Opc, dl, ResTys, SrcLo, SrcHi, Cnt);
    return DAG.getNode(ISD::BUILD_PAIR, dl, MVT::i32, Result.getValue(0),
                       Result.getValue(1));
  }*/

  // Expand non-constant shifts to loops.
  if (!isa<ConstantSDNode>(N->getOperand(1))) {
    switch (Op.getOpcode()) {
    default:
      llvm_unreachable("Invalid shift opcode!");
    case ISD::SHL:
      return DAG.getNode(PDPISD::LSLLOOP, dl, VT, N->getOperand(0),
                         N->getOperand(1));
    case ISD::SRL:
      return DAG.getNode(PDPISD::LSRLOOP, dl, VT, N->getOperand(0),
                         N->getOperand(1));
    case ISD::ROTL: {
      SDValue Amt = N->getOperand(1);
      EVT AmtVT = Amt.getValueType();
      Amt = DAG.getNode(ISD::AND, dl, AmtVT, Amt,
                        DAG.getConstant(VT.getSizeInBits() - 1, dl, AmtVT));
      return DAG.getNode(PDPISD::ROLLOOP, dl, VT, N->getOperand(0), Amt);
    }
    case ISD::ROTR: {
      SDValue Amt = N->getOperand(1);
      EVT AmtVT = Amt.getValueType();
      Amt = DAG.getNode(ISD::AND, dl, AmtVT, Amt,
                        DAG.getConstant(VT.getSizeInBits() - 1, dl, AmtVT));
      return DAG.getNode(PDPISD::RORLOOP, dl, VT, N->getOperand(0), Amt);
    }
    case ISD::SRA:
      return DAG.getNode(PDPISD::ASRLOOP, dl, VT, N->getOperand(0),
                         N->getOperand(1));
    }
  }

  uint64_t ShiftAmount = N->getConstantOperandVal(1);
  SDValue Victim = N->getOperand(0);

  switch (Op.getOpcode()) {
    case ISD::SHL:
      Opc8 = PDPISD::ASL;
      break;
    case ISD::SRL:
      Opc8 = PDPISD::LSR;
      break;
  case ISD::SRA:
    Opc8 = PDPISD::ASR;
    break;
  /*case ISD::ROTL:
    Opc8 = PDPISD::ROL;
    ShiftAmount = ShiftAmount % VT.getSizeInBits();
    break;
  case ISD::ROTR:
    Opc8 = PDPISD::ROR;
    ShiftAmount = ShiftAmount % VT.getSizeInBits();
    break;
  case ISD::SRL:
    Opc8 = PDPISD::LSR;
    break;
  case ISD::SHL:
    Opc8 = PDPISD::LSL;
    break;*/
  default:
    llvm_unreachable("Invalid shift opcode");
  }

  // Optimize int8/int16 shifts.
  /*if (VT.getSizeInBits() == 8) {
    if (Op.getOpcode() == ISD::SHL && 4 <= ShiftAmount && ShiftAmount < 7) {
      // Optimize LSL when 4 <= ShiftAmount <= 6.
      Victim = DAG.getNode(PDPISD::SWAP, dl, VT, Victim);
      Victim =
          DAG.getNode(ISD::AND, dl, VT, Victim, DAG.getConstant(0xf0, dl, VT));
      ShiftAmount -= 4;
    } else if (Op.getOpcode() == ISD::SRL && 4 <= ShiftAmount &&
               ShiftAmount < 7) {
      // Optimize LSR when 4 <= ShiftAmount <= 6.
      Victim = DAG.getNode(PDPISD::SWAP, dl, VT, Victim);
      Victim =
          DAG.getNode(ISD::AND, dl, VT, Victim, DAG.getConstant(0x0f, dl, VT));
      ShiftAmount -= 4;
    } else if (Op.getOpcode() == ISD::SHL && ShiftAmount == 7) {
      // Optimize LSL when ShiftAmount == 7.
      Victim = DAG.getNode(PDPISD::LSLBN, dl, VT, Victim,
                           DAG.getConstant(7, dl, VT));
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::SRL && ShiftAmount == 7) {
      // Optimize LSR when ShiftAmount == 7.
      Victim = DAG.getNode(PDPISD::LSRBN, dl, VT, Victim,
                           DAG.getConstant(7, dl, VT));
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::SRA && ShiftAmount == 6) {
      // Optimize ASR when ShiftAmount == 6.
      Victim = DAG.getNode(PDPISD::ASRBN, dl, VT, Victim,
                           DAG.getConstant(6, dl, VT));
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::SRA && ShiftAmount == 7) {
      // Optimize ASR when ShiftAmount == 7.
      Victim = DAG.getNode(PDPISD::ASRBN, dl, VT, Victim,
                           DAG.getConstant(7, dl, VT));
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::ROTL && ShiftAmount == 3) {
      // Optimize left rotation 3 bits to swap then right rotation 1 bit.
      Victim = DAG.getNode(PDPISD::SWAP, dl, VT, Victim);
      Victim = DAG.getNode(PDPISD::ROR, dl, VT, Victim);
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::ROTR && ShiftAmount == 3) {
      // Optimize right rotation 3 bits to swap then left rotation 1 bit.
      Victim = DAG.getNode(PDPISD::SWAP, dl, VT, Victim);
      Victim = DAG.getNode(PDPISD::ROL, dl, VT, Victim);
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::ROTL && ShiftAmount == 7) {
      // Optimize left rotation 7 bits to right rotation 1 bit.
      Victim = DAG.getNode(PDPISD::ROR, dl, VT, Victim);
      ShiftAmount = 0;
    } else if (Op.getOpcode() == ISD::ROTR && ShiftAmount == 7) {
      // Optimize right rotation 7 bits to left rotation 1 bit.
      Victim = DAG.getNode(PDPISD::ROL, dl, VT, Victim);
      ShiftAmount = 0;
    } else if ((Op.getOpcode() == ISD::ROTR || Op.getOpcode() == ISD::ROTL) &&
               ShiftAmount >= 4) {
      // Optimize left/right rotation with the SWAP instruction.
      Victim = DAG.getNode(PDPISD::SWAP, dl, VT, Victim);
      ShiftAmount -= 4;
    }
  } else if (VT.getSizeInBits() == 16) {
    if (Op.getOpcode() == ISD::SRA)
      // Special optimization for int16 arithmetic right shift.
      switch (ShiftAmount) {
      case 15:
        Victim = DAG.getNode(PDPISD::ASRWN, dl, VT, Victim,
                             DAG.getConstant(15, dl, VT));
        ShiftAmount = 0;
        break;
      case 14:
        Victim = DAG.getNode(PDPISD::ASRWN, dl, VT, Victim,
                             DAG.getConstant(14, dl, VT));
        ShiftAmount = 0;
        break;
      case 7:
        Victim = DAG.getNode(PDPISD::ASRWN, dl, VT, Victim,
                             DAG.getConstant(7, dl, VT));
        ShiftAmount = 0;
        break;
      default:
        break;
      }
    if (4 <= ShiftAmount && ShiftAmount < 8)
      switch (Op.getOpcode()) {
      case ISD::SHL:
        Victim = DAG.getNode(PDPISD::LSLWN, dl, VT, Victim,
                             DAG.getConstant(4, dl, VT));
        ShiftAmount -= 4;
        break;
      case ISD::SRL:
        Victim = DAG.getNode(PDPISD::LSRWN, dl, VT, Victim,
                             DAG.getConstant(4, dl, VT));
        ShiftAmount -= 4;
        break;
      default:
        break;
      }
    else if (8 <= ShiftAmount && ShiftAmount < 12)
      switch (Op.getOpcode()) {
      case ISD::SHL:
        Victim = DAG.getNode(PDPISD::LSLWN, dl, VT, Victim,
                             DAG.getConstant(8, dl, VT));
        ShiftAmount -= 8;
        // Only operate on the higher byte for remaining shift bits.
        Opc8 = PDPISD::LSLHI;
        break;
      case ISD::SRL:
        Victim = DAG.getNode(PDPISD::LSRWN, dl, VT, Victim,
                             DAG.getConstant(8, dl, VT));
        ShiftAmount -= 8;
        // Only operate on the lower byte for remaining shift bits.
        Opc8 = PDPISD::LSRLO;
        break;
      case ISD::SRA:
        Victim = DAG.getNode(PDPISD::ASRWN, dl, VT, Victim,
                             DAG.getConstant(8, dl, VT));
        ShiftAmount -= 8;
        // Only operate on the lower byte for remaining shift bits.
        Opc8 = PDPISD::ASRLO;
        break;
      default:
        break;
      }
    else if (12 <= ShiftAmount)
      switch (Op.getOpcode()) {
      case ISD::SHL:
        Victim = DAG.getNode(PDPISD::LSLWN, dl, VT, Victim,
                             DAG.getConstant(12, dl, VT));
        ShiftAmount -= 12;
        // Only operate on the higher byte for remaining shift bits.
        Opc8 = PDPISD::LSLHI;
        break;
      case ISD::SRL:
        Victim = DAG.getNode(PDPISD::LSRWN, dl, VT, Victim,
                             DAG.getConstant(12, dl, VT));
        ShiftAmount -= 12;
        // Only operate on the lower byte for remaining shift bits.
        Opc8 = PDPISD::LSRLO;
        break;
      case ISD::SRA:
        Victim = DAG.getNode(PDPISD::ASRWN, dl, VT, Victim,
                             DAG.getConstant(8, dl, VT));
        ShiftAmount -= 8;
        // Only operate on the lower byte for remaining shift bits.
        Opc8 = PDPISD::ASRLO;
        break;
      default:
        break;
      }
  }*/

  while (ShiftAmount--) {
    Victim = DAG.getNode(Opc8, dl, VT, Victim);
  }

  return Victim;
}

SDValue PDPTargetLowering::LowerDivRem(SDValue Op, SelectionDAG &DAG) const {
  unsigned Opcode = Op->getOpcode();
  assert((Opcode == ISD::SDIVREM || Opcode == ISD::UDIVREM) &&
         "Invalid opcode for Div/Rem lowering");
  bool IsSigned = (Opcode == ISD::SDIVREM);
  EVT VT = Op->getValueType(0);
  Type *Ty = VT.getTypeForEVT(*DAG.getContext());

  RTLIB::Libcall LC;
  switch (VT.getSimpleVT().SimpleTy) {
  default:
    llvm_unreachable("Unexpected request for libcall!");
  case MVT::i8:
    LC = IsSigned ? RTLIB::SDIVREM_I8 : RTLIB::UDIVREM_I8;
    break;
  case MVT::i16:
    LC = IsSigned ? RTLIB::SDIVREM_I16 : RTLIB::UDIVREM_I16;
    break;
  case MVT::i32:
    LC = IsSigned ? RTLIB::SDIVREM_I32 : RTLIB::UDIVREM_I32;
    break;
  }

  SDValue InChain = DAG.getEntryNode();

  TargetLowering::ArgListTy Args;
  TargetLowering::ArgListEntry Entry;
  for (SDValue const &Value : Op->op_values()) {
    Entry.Node = Value;
    Entry.Ty = Value.getValueType().getTypeForEVT(*DAG.getContext());
    Entry.IsSExt = IsSigned;
    Entry.IsZExt = !IsSigned;
    Args.push_back(Entry);
  }

  SDValue Callee = DAG.getExternalSymbol(getLibcallName(LC),
                                         getPointerTy(DAG.getDataLayout()));

  Type *RetTy = (Type *)StructType::get(Ty, Ty);

  SDLoc dl(Op);
  TargetLowering::CallLoweringInfo CLI(DAG);
  CLI.setDebugLoc(dl)
      .setChain(InChain)
      .setLibCallee(getLibcallCallingConv(LC), RetTy, Callee, std::move(Args))
      .setInRegister()
      .setSExtResult(IsSigned)
      .setZExtResult(!IsSigned);

  std::pair<SDValue, SDValue> CallInfo = LowerCallTo(CLI);
  return CallInfo.first;
}

SDValue PDPTargetLowering::LowerGlobalAddress(SDValue Op,
                                              SelectionDAG &DAG) const {
  auto DL = DAG.getDataLayout();

  const GlobalValue *GV = cast<GlobalAddressSDNode>(Op)->getGlobal();
  int64_t Offset = cast<GlobalAddressSDNode>(Op)->getOffset();

  // Create the TargetGlobalAddress node, folding in the constant offset.
  SDValue Result =
      DAG.getTargetGlobalAddress(GV, SDLoc(Op), getPointerTy(DL), Offset);
  return DAG.getNode(PDPISD::WRAPPER, SDLoc(Op), getPointerTy(DL), Result);
}

SDValue PDPTargetLowering::LowerBlockAddress(SDValue Op,
                                             SelectionDAG &DAG) const {
  auto DL = DAG.getDataLayout();
  const BlockAddress *BA = cast<BlockAddressSDNode>(Op)->getBlockAddress();

  SDValue Result = DAG.getTargetBlockAddress(BA, getPointerTy(DL));

  return DAG.getNode(PDPISD::WRAPPER, SDLoc(Op), getPointerTy(DL), Result);
}

/// IntCCToPDPCC - Convert a DAG integer condition code to an PDP CC.
static PDPCC::CondCodes intCCToPDPCC(const ISD::CondCode CC) {
  switch (CC) {
  default:
    llvm_unreachable("Unknown condition code!");
  case ISD::SETEQ:
    return PDPCC::COND_EQ;  // ==
  case ISD::SETNE:
    return PDPCC::COND_NE;  // !=

  case ISD::SETGT:
    return PDPCC::COND_GT;  // Signed >
  case ISD::SETGE:
    return PDPCC::COND_GE;  // Signed >=
  case ISD::SETLT:
    return PDPCC::COND_LT;  // Signed <
  case ISD::SETLE:
    return PDPCC::COND_LE;  // Signed <=

  case ISD::SETUGT:
    return PDPCC::COND_HI;  // Unsigned >
  case ISD::SETUGE:
    return PDPCC::COND_SH;  // Unsigned >=
  case ISD::SETULT:
    return PDPCC::COND_LO;  // Unsigned <
  case ISD::SETULE:
    return PDPCC::COND_SL;  // Unsigned >=
  }
}

/// Returns appropriate CP/CPI/CPC nodes code for the given 8/16-bit operands.
SDValue PDPTargetLowering::getPDPCmp(SDValue LHS, SDValue RHS,
                                     SelectionDAG &DAG, SDLoc DL) const {
  assert((LHS.getSimpleValueType() == RHS.getSimpleValueType()) &&
         "LHS and RHS have different types");
  assert(((LHS.getSimpleValueType() == MVT::i16) ||
          (LHS.getSimpleValueType() == MVT::i8)) &&
         "invalid comparison type");

  SDValue Cmp;

  /*if (LHS.getSimpleValueType() == MVT::i16 && isa<ConstantSDNode>(RHS)) {
    uint64_t Imm = RHS->getAsZExtVal();
    // Generate a CPI/CPC pair if RHS is a 16-bit constant. Use the zero
    // register for the constant RHS if its lower or higher byte is zero.
    SDValue LHSlo = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue LHShi = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHS,
                                DAG.getIntPtrConstant(1, DL));
    SDValue RHSlo = (Imm & 0xff) == 0
                        ? DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8)
                        : DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, RHS,
                                      DAG.getIntPtrConstant(0, DL));
    SDValue RHShi = (Imm & 0xff00) == 0
                        ? DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8)
                        : DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, RHS,
                                      DAG.getIntPtrConstant(1, DL));
    Cmp = DAG.getNode(PDPISD::CMP, DL, MVT::Glue, LHSlo, RHSlo);
    Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHShi, RHShi, Cmp);
  } else if (RHS.getSimpleValueType() == MVT::i16 && isa<ConstantSDNode>(LHS)) {
    // Generate a CPI/CPC pair if LHS is a 16-bit constant. Use the zero
    // register for the constant LHS if its lower or higher byte is zero.
    uint64_t Imm = LHS->getAsZExtVal();
    SDValue LHSlo = (Imm & 0xff) == 0
                        ? DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8)
                        : DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHS,
                                      DAG.getIntPtrConstant(0, DL));
    SDValue LHShi = (Imm & 0xff00) == 0
                        ? DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8)
                        : DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHS,
                                      DAG.getIntPtrConstant(1, DL));
    SDValue RHSlo = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, RHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue RHShi = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, RHS,
                                DAG.getIntPtrConstant(1, DL));
    Cmp = DAG.getNode(PDPISD::CMP, DL, MVT::Glue, LHSlo, RHSlo);
    Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHShi, RHShi, Cmp);
  } else*/ {
    // Generate ordinary 16-bit comparison.
    Cmp = DAG.getNode(PDPISD::CMP, DL, MVT::Glue, LHS, RHS);
  }

  return Cmp;
}

/// Returns appropriate PDP CMP/CMPC nodes and corresponding condition code for
/// the given operands.
SDValue PDPTargetLowering::getPDPCmp(SDValue LHS, SDValue RHS, ISD::CondCode CC,
                                     SDValue &PDPcc, SelectionDAG &DAG,
                                     SDLoc DL) const {
  SDValue Cmp;
  EVT VT = LHS.getValueType();
  bool UseTest = false;

  switch (CC) {
  default:
    break;
  case ISD::SETLE: {
    // Swap operands and reverse the branching condition.
    std::swap(LHS, RHS);
    CC = ISD::SETGE;
    break;
  }
  case ISD::SETEQ: {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(RHS)) {
      if (C->getSExtValue() == 0) {
        UseTest = true;
        PDPcc = DAG.getConstant(PDPCC::COND_EQ, DL, MVT::i8);
        break;
      }
    }
    break;
  }
  case ISD::SETNE: {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(RHS)) {
      if (C->getSExtValue() == 0) {
        UseTest = true;
        PDPcc = DAG.getConstant(PDPCC::COND_NE, DL, MVT::i8);
        break;
      }
    }
    break;
  }
  case ISD::SETGT: {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(RHS)) {
      switch (C->getSExtValue()) {
      case -1: {
        // When doing lhs > -1 use a tst instruction on the top part of lhs
        // and use brpl instead of using a chain of cp/cpc.
        UseTest = true;
        PDPcc = DAG.getConstant(PDPCC::COND_PL, DL, MVT::i8);
        break;
      }
      case 0: {
        // Turn lhs > 0 into 0 < lhs since 0 can be materialized with
        // __zero_reg__ in lhs.
        RHS = LHS;
        LHS = DAG.getConstant(0, DL, VT);
        CC = ISD::SETLT;
        break;
      }
      default: {
        // Turn lhs < rhs with lhs constant into rhs >= lhs+1, this allows
        // us to  fold the constant into the cmp instruction.
        RHS = DAG.getConstant(C->getSExtValue() + 1, DL, VT);
        CC = ISD::SETGE;
        break;
      }
      }
      break;
    }
    // Swap operands and reverse the branching condition.
    std::swap(LHS, RHS);
    CC = ISD::SETLT;
    break;
  }
  case ISD::SETLT: {
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(RHS)) {
      switch (C->getSExtValue()) {
      case 1: {
        // Turn lhs < 1 into 0 >= lhs since 0 can be materialized with
        // __zero_reg__ in lhs.
        RHS = LHS;
        LHS = DAG.getConstant(0, DL, VT);
        CC = ISD::SETGE;
        break;
      }
      case 0: {
        // When doing lhs < 0 use a tst instruction on the top part of lhs
        // and use brmi instead of using a chain of cp/cpc.
        UseTest = true;
        PDPcc = DAG.getConstant(PDPCC::COND_MI, DL, MVT::i8);
        break;
      }
      }
    }
    break;
  }
  case ISD::SETULE: {
    // Swap operands and reverse the branching condition.
    std::swap(LHS, RHS);
    CC = ISD::SETUGE;
    break;
  }
  /*case ISD::SETUGT: {
    // Turn lhs < rhs with lhs constant into rhs >= lhs+1, this allows us to
    // fold the constant into the cmp instruction.
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(RHS)) {
      RHS = DAG.getConstant(C->getSExtValue() + 1, DL, VT);
      CC = ISD::SETUGE;
      break;
    }
    // Swap operands and reverse the branching condition.
    std::swap(LHS, RHS);
    CC = ISD::SETULT;
    break;
  }*/
  }

  // Expand 32 and 64 bit comparisons with custom CMP and CMPC nodes instead of
  // using the default and/or/xor expansion code which is much longer.
  /*if (VT == MVT::i32) {
    SDValue LHSlo = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue LHShi = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS,
                                DAG.getIntPtrConstant(1, DL));
    SDValue RHSlo = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue RHShi = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS,
                                DAG.getIntPtrConstant(1, DL));

    if (UseTest) {
      // When using tst we only care about the highest part.
      SDValue Top = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHShi,
                                DAG.getIntPtrConstant(1, DL));
      Cmp = DAG.getNode(PDPISD::TST, DL, MVT::Glue, Top);
    } else {
      Cmp = getPDPCmp(LHSlo, RHSlo, DAG, DL);
      Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHShi, RHShi, Cmp);
    }
  }*/ /*else if (VT == MVT::i64) {
    SDValue LHS_0 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, LHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue LHS_1 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, LHS,
                                DAG.getIntPtrConstant(1, DL));

    SDValue LHS0 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS_0,
                               DAG.getIntPtrConstant(0, DL));
    SDValue LHS1 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS_0,
                               DAG.getIntPtrConstant(1, DL));
    SDValue LHS2 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS_1,
                               DAG.getIntPtrConstant(0, DL));
    SDValue LHS3 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, LHS_1,
                               DAG.getIntPtrConstant(1, DL));

    SDValue RHS_0 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, RHS,
                                DAG.getIntPtrConstant(0, DL));
    SDValue RHS_1 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i32, RHS,
                                DAG.getIntPtrConstant(1, DL));

    SDValue RHS0 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS_0,
                               DAG.getIntPtrConstant(0, DL));
    SDValue RHS1 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS_0,
                               DAG.getIntPtrConstant(1, DL));
    SDValue RHS2 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS_1,
                               DAG.getIntPtrConstant(0, DL));
    SDValue RHS3 = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i16, RHS_1,
                               DAG.getIntPtrConstant(1, DL));

    if (UseTest) {
      // When using tst we only care about the highest part.
      SDValue Top = DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8, LHS3,
                                DAG.getIntPtrConstant(1, DL));
      Cmp = DAG.getNode(PDPISD::TST, DL, MVT::Glue, Top);
    } else {
      Cmp = getPDPCmp(LHS0, RHS0, DAG, DL);
      Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHS1, RHS1, Cmp);
      Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHS2, RHS2, Cmp);
      Cmp = DAG.getNode(PDPISD::CMPC, DL, MVT::Glue, LHS3, RHS3, Cmp);
    }
  }else*/ if (VT == MVT::i8 || VT == MVT::i16) {
    if (UseTest) {
      Cmp = DAG.getNode(PDPISD::TST, DL, MVT::Glue, LHS);

      // When using tst we only care about the highest part.
      /*Cmp = DAG.getNode(PDPISD::TST, DL, MVT::Glue,
                        (VT == MVT::i8)
                            ? LHS
                            : DAG.getNode(ISD::EXTRACT_ELEMENT, DL, MVT::i8,
                                          LHS, DAG.getIntPtrConstant(1, DL)))*/;
    } else {
      Cmp = getPDPCmp(LHS, RHS, DAG, DL);
    }
  } else {
    llvm_unreachable("Invalid comparison size");
  }

  // When using a test instruction PDPcc is already set.
  if (!UseTest) {
    PDPcc = DAG.getConstant(intCCToPDPCC(CC), DL, MVT::i8);
  }

  return Cmp;
}

SDValue PDPTargetLowering::LowerBR_CC(SDValue Op, SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue LHS = Op.getOperand(2);
  SDValue RHS = Op.getOperand(3);
  SDValue Dest = Op.getOperand(4);
  SDLoc dl(Op);

  SDValue TargetCC;
  SDValue Cmp = getPDPCmp(LHS, RHS, CC, TargetCC, DAG, dl);

  return DAG.getNode(PDPISD::BRCOND, dl, MVT::Other, Chain, Dest, TargetCC,
                     Cmp);
}

SDValue PDPTargetLowering::LowerBR_CC2(SDValue Op, SelectionDAG &DAG) const {
  SDValue Chain = Op.getOperand(0);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(1))->get();
  SDValue LHS = Op.getOperand(2);
  SDValue RHS = Op.getOperand(3);
  SDValue Dest = Op.getOperand(4);
  SDLoc dl(Op);

  SDValue TargetCC = DAG.getConstant(intCCToPDPCC(CC), dl, MVT::i8);
  //SDValue Cmp = getPDPCmp(LHS, RHS, CC, TargetCC, DAG, dl);
  auto svt = LHS.getSimpleValueType();

  auto rhsConst = dyn_cast<ConstantSDNode>(RHS);

  if (LHS.getOpcode() == ISD::AND && (CC == ISD::SETEQ || CC == ISD::SETNE) &&
      rhsConst && rhsConst->getZExtValue() == 0) {

    return DAG.getNode(svt == MVT::i16 ? PDPISD::BRBIT : PDPISD::BRBITB, dl,
                       MVT::Other, Chain, LHS.getOperand(1), LHS.getOperand(0), Dest, TargetCC);
  }

  return DAG.getNode(svt == MVT::i16 ? PDPISD::BRCOND2 : PDPISD::BRCOND2B,
    dl, MVT::Other, Chain, LHS, RHS, Dest, TargetCC);
}

SDValue PDPTargetLowering::LowerSELECT_CC(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0);
  SDValue RHS = Op.getOperand(1);
  SDValue TrueV = Op.getOperand(2);
  SDValue FalseV = Op.getOperand(3);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(4))->get();
  SDLoc dl(Op);

  SDValue TargetCC;
  SDValue Cmp = getPDPCmp(LHS, RHS, CC, TargetCC, DAG, dl);

  SDValue Ops[] = {TrueV, FalseV, TargetCC, Cmp};

  return DAG.getNode(PDPISD::SELECT_CC, dl, Op.getValueType(), Ops);
}

SDValue PDPTargetLowering::LowerSELECT_CC2(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0);
  SDValue RHS = Op.getOperand(1);
  SDValue TrueV = Op.getOperand(2);
  SDValue FalseV = Op.getOperand(3);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(4))->get();
  SDLoc dl(Op);

  SDValue TargetCC = DAG.getConstant(intCCToPDPCC(CC), dl, MVT::i8);

  SDValue Ops[] = {LHS, RHS, TrueV, FalseV, TargetCC};

  return DAG.getNode(PDPISD::SELECT_CC2, dl, Op.getValueType(), Ops);
}

SDValue PDPTargetLowering::LowerSETCC(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0);
  SDValue RHS = Op.getOperand(1);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();
  SDLoc DL(Op);

  SDValue TargetCC;
  SDValue Cmp = getPDPCmp(LHS, RHS, CC, TargetCC, DAG, DL);

  SDValue TrueV = DAG.getConstant(1, DL, Op.getValueType());
  SDValue FalseV = DAG.getConstant(0, DL, Op.getValueType());
  SDValue Ops[] = {TrueV, FalseV, TargetCC, Cmp};

  return DAG.getNode(PDPISD::SELECT_CC, DL, Op.getValueType(), Ops);
}

SDValue PDPTargetLowering::LowerSETCC2(SDValue Op, SelectionDAG &DAG) const {
  SDValue LHS = Op.getOperand(0);
  SDValue RHS = Op.getOperand(1);
  ISD::CondCode CC = cast<CondCodeSDNode>(Op.getOperand(2))->get();
  SDLoc DL(Op);

  SDValue TrueV = DAG.getConstant(1, DL, Op.getValueType());
  SDValue FalseV = DAG.getConstant(0, DL, Op.getValueType());

  SDValue TargetCC = DAG.getConstant(intCCToPDPCC(CC), DL, MVT::i8);

  SDValue Ops[] = {LHS, RHS, TrueV, FalseV, TargetCC};

  return DAG.getNode(PDPISD::SELECT_CC2, DL, Op.getValueType(), Ops);
}

SDValue PDPTargetLowering::LowerVASTART(SDValue Op, SelectionDAG &DAG) const {
  const MachineFunction &MF = DAG.getMachineFunction();
  const PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();
  const Value *SV = cast<SrcValueSDNode>(Op.getOperand(2))->getValue();
  auto DL = DAG.getDataLayout();
  SDLoc dl(Op);

  // Vastart just stores the address of the VarArgsFrameIndex slot into the
  // memory location argument.
  SDValue FI = DAG.getFrameIndex(AFI->getVarArgsFrameIndex(), getPointerTy(DL));

  return DAG.getStore(Op.getOperand(0), dl, FI, Op.getOperand(1),
                      MachinePointerInfo(SV));
}

// Modify the existing ISD::INLINEASM node to add the implicit zero register.
SDValue PDPTargetLowering::LowerINLINEASM(SDValue Op, SelectionDAG &DAG) const {
  return Op;

  /*SDValue ZeroReg = DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8);
  if (Op.getOperand(Op.getNumOperands() - 1) == ZeroReg ||
      Op.getOperand(Op.getNumOperands() - 2) == ZeroReg) {
    // Zero register has already been added. Don't add it again.
    // If this isn't handled, we get called over and over again.
    return Op;
  }*/

  // Get a list of operands to the new INLINEASM node. This is mostly a copy,
  // with some edits.
  // Add the following operands at the end (but before the glue node, if it's
  // there):
  //  - The flags of the implicit zero register operand.
  //  - The implicit zero register operand itself.
  SDLoc dl(Op);
  SmallVector<SDValue, 8> Ops;
  SDNode *N = Op.getNode();
  SDValue Glue;
  for (unsigned I = 0; I < N->getNumOperands(); I++) {
    SDValue Operand = N->getOperand(I);
    if (Operand.getValueType() == MVT::Glue) {
      // The glue operand always needs to be at the end, so we need to treat it
      // specially.
      Glue = Operand;
    } else {
      Ops.push_back(Operand);
    }
  }
  InlineAsm::Flag Flags(InlineAsm::Kind::RegUse, 1);
  Ops.push_back(DAG.getTargetConstant(Flags, dl, MVT::i32));
  //Ops.push_back(ZeroReg);
  if (Glue) {
    Ops.push_back(Glue);
  }

  // Replace the current INLINEASM node with a new one that has the zero
  // register as implicit parameter.
  SDValue New = DAG.getNode(N->getOpcode(), dl, N->getVTList(), Ops);
  DAG.ReplaceAllUsesOfValueWith(Op, New);
  DAG.ReplaceAllUsesOfValueWith(Op.getValue(1), New.getValue(1));

  return New;
}

SDValue PDPTargetLowering::LowerLOAD(SDNode *N,
                                     SmallVectorImpl<SDValue> &Results,
                                     llvm::SelectionDAG &DAG) const {
  SDLoc Dl(N);

  if (const auto *LD = dyn_cast<LoadSDNode>(N)) {
    if (LD->getExtensionType() == ISD::SEXTLOAD &&
        LD->getValueType(0) == MVT::i32) {
      auto T = LD->getMemoryVT();
      auto Chain = LD->getChain();
      auto Src = LD->getOperand(1);

      SDValue NewPtr = DAG.getLoad(
          LD->getAddressingMode(), ISD::SEXTLOAD, MVT::i16, Dl, Chain, Src,
          DAG.getNode(ISD::UNDEF, SDLoc(), MVT::i16), T, LD->getMemOperand());

      // ARMISelLowering.cpp : 10204
      SDValue Lo = NewPtr;
      SDValue Hi = DAG.getNode(PDPISD::SXT, Dl, MVT::i16, NewPtr);
      SDValue Pair = DAG.getNode(ISD::BUILD_PAIR, Dl, MVT::i32, Lo, Hi);

      Results.append({Pair, NewPtr.getValue(1)});
    }
  }

  return SDValue();
}

SDValue PDPTargetLowering::LowerOperation(SDValue Op, SelectionDAG &DAG) const {
  switch (Op.getOpcode()) {
  default:
    Op.dumpr(&DAG);
    llvm_unreachable("Don't know how to custom lower this!");
//  case ISD::LOAD:
//    return LowerLOAD(Op, DAG);
  case ISD::SIGN_EXTEND:
    // return DAG.getNode(PDPISD::SXT, SDLoc(Op), MVT::i32, Op.getOperand(0));
    if (Op.getValueType() == MVT::i32) {
      auto &Src = Op.getOperand(0);
      auto Ext = DAG.getNode(PDPISD::SXT, SDLoc(Op), MVT::i16, Src);

      return DAG.getNode(ISD::BUILD_PAIR, SDLoc(Op), MVT::i32, Src, Ext);
    } else
      llvm_unreachable("Don't know how to custom lower SIGN_EXTEND!");
    // DAG.getConstant(456, SDLoc(Op), MVT::i16));
  case ISD::SHL:
  case ISD::SRA:
  case ISD::SRL:
  case ISD::ROTL:
  case ISD::ROTR:
    return LowerShifts(Op, DAG);
  case ISD::GlobalAddress:
    return LowerGlobalAddress(Op, DAG);
  case ISD::BlockAddress:
    return LowerBlockAddress(Op, DAG);
  case ISD::BR_CC:
    return LowerBR_CC2(Op, DAG);
  case ISD::SELECT_CC:
    return LowerSELECT_CC2(Op, DAG);
  case ISD::SETCC:
    return LowerSETCC2(Op, DAG);
  case ISD::VASTART:
    return LowerVASTART(Op, DAG);
  case ISD::SDIVREM:
  case ISD::UDIVREM:
    return LowerDivRem(Op, DAG);
  case ISD::INLINEASM:
    return LowerINLINEASM(Op, DAG);
  }

  return SDValue();
}

/// Replace a node with an illegal result type
/// with a new node built out of custom code.
void PDPTargetLowering::ReplaceNodeResults(SDNode *N,
                                           SmallVectorImpl<SDValue> &Results,
                                           SelectionDAG &DAG) const {
  SDLoc DL(N);

  switch (N->getOpcode()) {
  case ISD::ADD: {
    // Convert add (x, imm) into sub (x, -imm).
    if (const ConstantSDNode *C = dyn_cast<ConstantSDNode>(N->getOperand(1))) {
      SDValue Sub = DAG.getNode(
          ISD::SUB, DL, N->getValueType(0), N->getOperand(0),
          DAG.getConstant(-C->getAPIntValue(), DL, C->getValueType(0)));
      Results.push_back(Sub);
    }
    break;
  }
  case ISD::LOAD:
    LowerLOAD(N, Results, DAG);
    break;
  default: {
    SDValue Res = LowerOperation(SDValue(N, 0), DAG);

    if (!Res)
      break;

    for (unsigned I = 0, E = Res->getNumValues(); I != E; ++I)
      Results.push_back(Res.getValue(I));

    break;
  }
  }
}

/// Return true if the addressing mode represented
/// by AM is legal for this target, for a load/store of the specified type.
bool PDPTargetLowering::isLegalAddressingMode(const DataLayout &DL,
                                              const AddrMode &AM, Type *Ty,
                                              unsigned AS,
                                              Instruction *I) const {
  int64_t Offs = AM.BaseOffs;

  // Allow absolute addresses.
  if (AM.BaseGV && !AM.HasBaseReg && AM.Scale == 0 && Offs == 0) {
    return true;
  }

  // Flash memory instructions only allow zero offsets.
  /*if (isa<PointerType>(Ty) && AS == PDP::ProgramMemory) {
    return false;
  }*/

  if (AM.BaseGV == nullptr && AM.HasBaseReg && AM.Scale == 0)
    return true;

  if (AM.BaseGV == nullptr && !AM.HasBaseReg && AM.Scale == 1)
    return true;

  return false;
}

/// Returns true by value, base pointer and
/// offset pointer and addressing mode by reference if the node's address
/// can be legally represented as pre-indexed load / store address.
bool PDPTargetLowering::getPreIndexedAddressParts(SDNode *N, SDValue &Base,
                                                  SDValue &Offset,
                                                  ISD::MemIndexedMode &AM,
                                                  SelectionDAG &DAG) const {

  llvm_unreachable("PDPTargetLowering::getPreIndexedAddressParts");

  EVT VT;
  const SDNode *Op;
  SDLoc DL(N);

  if (const LoadSDNode *LD = dyn_cast<LoadSDNode>(N)) {
    VT = LD->getMemoryVT();
    Op = LD->getBasePtr().getNode();
    if (LD->getExtensionType() != ISD::NON_EXTLOAD)
      return false;
    /*if (PDP::isProgramMemoryAccess(LD)) {
      return false;
    }*/
  } else if (const StoreSDNode *ST = dyn_cast<StoreSDNode>(N)) {
    VT = ST->getMemoryVT();
    Op = ST->getBasePtr().getNode();
    /*if (PDP::isProgramMemoryAccess(ST)) {
      return false;
    }*/
  } else {
    return false;
  }

  if (VT != MVT::i8 && VT != MVT::i16) {
    return false;
  }

  if (Op->getOpcode() != ISD::ADD && Op->getOpcode() != ISD::SUB) {
    return false;
  }

  if (const ConstantSDNode *RHS = dyn_cast<ConstantSDNode>(Op->getOperand(1))) {
    int RHSC = RHS->getSExtValue();
    if (Op->getOpcode() == ISD::SUB)
      RHSC = -RHSC;

    if ((VT == MVT::i16 && RHSC != -2) || (VT == MVT::i8 && RHSC != -1)) {
      return false;
    }

    Base = Op->getOperand(0);
    Offset = DAG.getSignedConstant(RHSC, DL, MVT::i8);
    AM = ISD::PRE_DEC;

    return true;
  }

  return false;
}

/// Returns true by value, base pointer and
/// offset pointer and addressing mode by reference if this node can be
/// combined with a load / store to form a post-indexed load / store.
bool PDPTargetLowering::getPostIndexedAddressParts(SDNode *N, SDNode *Op,
                                                   SDValue &Base,
                                                   SDValue &Offset,
                                                   ISD::MemIndexedMode &AM,
                                                   SelectionDAG &DAG) const {

  llvm_unreachable("PDPTargetLowering::getPostIndexedAddressParts");

  EVT VT;
  SDLoc DL(N);

  if (const LoadSDNode *LD = dyn_cast<LoadSDNode>(N)) {
    VT = LD->getMemoryVT();
    if (LD->getExtensionType() != ISD::NON_EXTLOAD)
      return false;
  } else if (const StoreSDNode *ST = dyn_cast<StoreSDNode>(N)) {
    VT = ST->getMemoryVT();
    // We can not store to program memory.
    /*if (PDP::isProgramMemoryAccess(ST))
      return false;*/
    // Since the high byte need to be stored first, we can not emit
    // i16 post increment store like:
    // st X+, r24
    // st X+, r25
    /*if (VT == MVT::i16 && !Subtarget.hasLowByteFirst())
      return false;*/
  } else {
    return false;
  }

  if (VT != MVT::i8 && VT != MVT::i16) {
    return false;
  }

  if (Op->getOpcode() != ISD::ADD && Op->getOpcode() != ISD::SUB) {
    return false;
  }

  if (const ConstantSDNode *RHS = dyn_cast<ConstantSDNode>(Op->getOperand(1))) {
    int RHSC = RHS->getSExtValue();
    if (Op->getOpcode() == ISD::SUB)
      RHSC = -RHSC;
    if ((VT == MVT::i16 && RHSC != 2) || (VT == MVT::i8 && RHSC != 1)) {
      return false;
    }

    // FIXME: We temporarily disable post increment load from program memory,
    //        due to bug https://github.com/llvm/llvm-project/issues/59914.
    ///if (const LoadSDNode *LD = dyn_cast<LoadSDNode>(N))
      /*if (PDP::isProgramMemoryAccess(LD))
        return false;*/

    Base = Op->getOperand(0);
    Offset = DAG.getConstant(RHSC, DL, MVT::i16);
    AM = ISD::POST_INC;

    return true;
  }

  return false;
}

bool PDPTargetLowering::isOffsetFoldingLegal(
    const GlobalAddressSDNode *GA) const {
  return true;
}

//===----------------------------------------------------------------------===//
//             Formal Arguments Calling Convention Implementation
//===----------------------------------------------------------------------===//

#include "PDPGenCallingConv.inc"

/// Registers for calling conventions, ordered in reverse as required by ABI.
/// Both arrays must be of the same length.
static const MCPhysReg RegList8PDP[] = {PDP::R0l, PDP::R1l, PDP::R2l, PDP::R3l};

static const MCPhysReg RegList16PDP[] = {PDP::R0, PDP::R1, PDP::R2, PDP::R3};

static_assert(std::size(RegList8PDP) == std::size(RegList16PDP),
              "8-bit and 16-bit register arrays must be of equal length");

/// Analyze incoming and outgoing function arguments. We need custom C++ code
/// to handle special constraints in the ABI.
/// In addition, all pieces of a certain argument have to be passed either
/// using registers or the stack but never mixing both.
template <typename ArgT>
static void
analyzeArguments(TargetLowering::CallLoweringInfo *CLI, const Function *F,
                 const DataLayout *TD, const SmallVectorImpl<ArgT> &Args,
                 SmallVectorImpl<CCValAssign> &ArgLocs, CCState &CCInfo) {

  // Choose the proper register list for argument passing according to the ABI.
  ArrayRef<MCPhysReg> RegList8;
  ArrayRef<MCPhysReg> RegList16;

  RegList8 = ArrayRef(RegList8PDP);
  RegList16 = ArrayRef(RegList16PDP);

  unsigned NumArgs = Args.size();
  // This is the index of the last used register, in RegList*.
  // -1 means R26 (R26 is never actually used in CC).
  int RegLastIdx = -1;
  // Once a value is passed to the stack it will always be used
  bool UseStack = false;
  for (unsigned i = 0; i != NumArgs;) {
    MVT VT = Args[i].VT;
    // We have to count the number of bytes for each function argument, that is
    // those Args with the same OrigArgIndex. This is important in case the
    // function takes an aggregate type.
    // Current argument will be between [i..j).
    unsigned ArgIndex = Args[i].OrigArgIndex;
    unsigned TotalBytes = VT.getStoreSize();
    unsigned j = i + 1;
    for (; j != NumArgs; ++j) {
      if (Args[j].OrigArgIndex != ArgIndex)
        break;
      TotalBytes += Args[j].VT.getStoreSize();
    }
    // Round up to even number of bytes.
    TotalBytes = alignTo(TotalBytes, 2);
    // Skip zero sized arguments
    if (TotalBytes == 0)
      continue;
    // The index of the first register to be used
    unsigned RegIdx = RegLastIdx + TotalBytes;
    RegLastIdx = RegIdx;
    // If there are not enough registers, use the stack
    if (RegIdx >= RegList8.size()) {
      UseStack = true;
    }
    for (; i != j; ++i) {
      MVT VT = Args[i].VT;

      if (UseStack) {
        auto evt = EVT(VT).getTypeForEVT(CCInfo.getContext());
        unsigned Offset = CCInfo.AllocateStack(TD->getTypeAllocSize(evt),
                                               TD->getABITypeAlign(evt));
        CCInfo.addLoc(
            CCValAssign::getMem(i, VT, Offset, VT, CCValAssign::Full));
      } else {
        unsigned Reg;
        if (VT == MVT::i8) {
          Reg = CCInfo.AllocateReg(RegList8[RegIdx]);
        } else if (VT == MVT::i16) {
          Reg = CCInfo.AllocateReg(RegList16[RegIdx]);
        } else {
          llvm_unreachable(
              "calling convention can only manage i8 and i16 types");
        }
        assert(Reg && "register not available in calling convention");
        CCInfo.addLoc(CCValAssign::getReg(i, VT, Reg, VT, CCValAssign::Full));
        // Registers inside a particular argument are sorted in increasing order
        // (remember the array is reversed).
        RegIdx -= VT.getStoreSize();
      }
    }
  }
}

/// Count the total number of bytes needed to pass or return these arguments.
template <typename ArgT>
static unsigned
getTotalArgumentsSizeInBytes(const SmallVectorImpl<ArgT> &Args) {
  unsigned TotalBytes = 0;

  for (const ArgT &Arg : Args) {
    TotalBytes += Arg.VT.getStoreSize();
  }
  return TotalBytes;
}

/// Analyze incoming and outgoing value of returning from a function.
/// The algorithm is similar to analyzeArguments, but there can only be
/// one value, possibly an aggregate, and it is limited to 8 bytes.
template <typename ArgT>
static void analyzeReturnValues(const SmallVectorImpl<ArgT> &Args, CCState &CCInfo) {

  llvm_unreachable("analyzeReturnValues");

  /*unsigned NumArgs = Args.size();
  unsigned TotalBytes = getTotalArgumentsSizeInBytes(Args);
  // CanLowerReturn() guarantees this assertion.
  /*if (Tiny)
    assert(TotalBytes <= 4 &&
           "return values greater than 4 bytes cannot be lowered on PDPTiny");
  else#1#
  assert(TotalBytes <= 8 &&
         "return values greater than 8 bytes cannot be lowered on PDP");

  // Choose the proper register list for argument passing according to the ABI.
  ArrayRef<MCPhysReg> RegList8;
  ArrayRef<MCPhysReg> RegList16;
  if (Tiny) {
    RegList8 = ArrayRef(RegList8Tiny);
    RegList16 = ArrayRef(RegList16Tiny);
  } else {
    RegList8 = ArrayRef(RegList8PDP);
    RegList16 = ArrayRef(RegList16PDP);
  }

  // GCC-ABI says that the size is rounded up to the next even number,
  // but actually once it is more than 4 it will always round up to 8.
  if (TotalBytes > 4) {
    TotalBytes = 8;
  } else {
    TotalBytes = alignTo(TotalBytes, 2);
  }

  // The index of the first register to use.
  int RegIdx = TotalBytes - 1;
  for (unsigned i = 0; i != NumArgs; ++i) {
    MVT VT = Args[i].VT;
    unsigned Reg;
    if (VT == MVT::i8) {
      Reg = CCInfo.AllocateReg(RegList8[RegIdx]);
    } else if (VT == MVT::i16) {
      Reg = CCInfo.AllocateReg(RegList16[RegIdx]);
    } else {
      llvm_unreachable("calling convention can only manage i8 and i16 types");
    }
    assert(Reg && "register not available in calling convention");
    CCInfo.addLoc(CCValAssign::getReg(i, VT, Reg, VT, CCValAssign::Full));
    // Registers sort in increasing order
    RegIdx -= VT.getStoreSize();
  }*/
}

SDValue PDPTargetLowering::LowerFormalArguments(
    SDValue Chain, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto DL = DAG.getDataLayout();

  // Assign locations to all of the incoming arguments.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());

  // Variadic functions do not need all the analysis below.
  if (isVarArg) {
    CCInfo.AnalyzeFormalArguments(Ins, ArgCC_PDP_Vararg);
  } else {
    CCInfo.AnalyzeFormalArguments(Ins, ArgCC_PDP_C);
    // analyzeArguments(nullptr, &MF.getFunction(), &DL, Ins, ArgLocs, CCInfo,
    // Subtarget.hasTinyEncoding());
  }

  SDValue ArgValue;
  for (CCValAssign &VA : ArgLocs) {

    // Arguments stored on registers.
    if (VA.isRegLoc()) {
      EVT RegVT = VA.getLocVT();
      const TargetRegisterClass *RC;
      if (RegVT == MVT::i8) {
        RC = &PDP::GPR16lRegClass;
      } else if (RegVT == MVT::i16) {
        RC = &PDP::GPR16RegClass;
      } else {
        llvm_unreachable("Unknown argument type!");
      }

      Register Reg = MF.addLiveIn(VA.getLocReg(), RC);
      ArgValue = DAG.getCopyFromReg(Chain, dl, Reg, RegVT);

      // :NOTE: Clang should not promote any i8 into i16 but for safety the
      // following code will handle zexts or sexts generated by other
      // front ends. Otherwise:
      // If this is an 8 bit value, it is really passed promoted
      // to 16 bits. Insert an assert[sz]ext to capture this, then
      // truncate to the right size.
      switch (VA.getLocInfo()) {
      default:
        llvm_unreachable("Unknown loc info!");
      case CCValAssign::Full:
        break;
      case CCValAssign::BCvt:
        ArgValue = DAG.getNode(ISD::BITCAST, dl, VA.getValVT(), ArgValue);
        break;
      case CCValAssign::SExt:
        ArgValue = DAG.getNode(ISD::AssertSext, dl, RegVT, ArgValue,
                               DAG.getValueType(VA.getValVT()));
        ArgValue = DAG.getNode(ISD::TRUNCATE, dl, VA.getValVT(), ArgValue);
        break;
      case CCValAssign::ZExt:
        ArgValue = DAG.getNode(ISD::AssertZext, dl, RegVT, ArgValue,
                               DAG.getValueType(VA.getValVT()));
        ArgValue = DAG.getNode(ISD::TRUNCATE, dl, VA.getValVT(), ArgValue);
        break;
      }

      InVals.push_back(ArgValue);
    } else {
      // Only arguments passed on the stack should make it here.
      assert(VA.isMemLoc());

      EVT LocVT = VA.getLocVT();

      // Create the frame index object for this incoming parameter.
      int FI = MFI.CreateFixedObject(LocVT.getSizeInBits() / 8,
                                     VA.getLocMemOffset(), true);

      // Create the SelectionDAG nodes corresponding to a load
      // from this parameter.
      SDValue FIN = DAG.getFrameIndex(FI, getPointerTy(DL));
      InVals.push_back(DAG.getLoad(LocVT, dl, Chain, FIN,
                                   MachinePointerInfo::getFixedStack(MF, FI)));
    }
  }

  // If the function takes variable number of arguments, make a frame index for
  // the start of the first vararg value... for expansion of llvm.va_start.
  if (isVarArg) {
    unsigned StackSize = CCInfo.getStackSize();
    PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();

    AFI->setVarArgsFrameIndex(MFI.CreateFixedObject(2, StackSize, true));
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//                  Call Calling Convention Implementation
//===----------------------------------------------------------------------===//

SDValue PDPTargetLowering::LowerCall(TargetLowering::CallLoweringInfo &CLI,
                                     SmallVectorImpl<SDValue> &InVals) const {
  SelectionDAG &DAG = CLI.DAG;
  SDLoc &DL = CLI.DL;
  SmallVectorImpl<ISD::OutputArg> &Outs = CLI.Outs;
  SmallVectorImpl<SDValue> &OutVals = CLI.OutVals;
  SmallVectorImpl<ISD::InputArg> &Ins = CLI.Ins;
  SDValue Chain = CLI.Chain;
  SDValue Callee = CLI.Callee;
  bool &isTailCall = CLI.IsTailCall;
  CallingConv::ID CallConv = CLI.CallConv;
  bool isVarArg = CLI.IsVarArg;

  MachineFunction &MF = DAG.getMachineFunction();

  // PDP does not yet support tail call optimization.
  isTailCall = false;

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), ArgLocs,
                 *DAG.getContext());

  // If the callee is a GlobalAddress/ExternalSymbol node (quite common, every
  // direct call is) turn it into a TargetGlobalAddress/TargetExternalSymbol
  // node so that legalize doesn't hack it.
  const Function *F = nullptr;
  if (const GlobalAddressSDNode *G = dyn_cast<GlobalAddressSDNode>(Callee)) {
    const GlobalValue *GV = G->getGlobal();
    if (isa<Function>(GV))
      F = cast<Function>(GV);
    Callee =
        DAG.getTargetGlobalAddress(GV, DL, getPointerTy(DAG.getDataLayout()));
  } else if (const ExternalSymbolSDNode *ES =
                 dyn_cast<ExternalSymbolSDNode>(Callee)) {
    Callee = DAG.getTargetExternalSymbol(ES->getSymbol(),
                                         getPointerTy(DAG.getDataLayout()));
  }

  // Variadic functions do not need all the analysis below.
  if (isVarArg) {
    CCInfo.AnalyzeCallOperands(Outs, ArgCC_PDP_Vararg);
  } else {
    CCInfo.AnalyzeCallOperands(Outs, ArgCC_PDP_C);
    //analyzeArguments(&CLI, F, &DAG.getDataLayout(), Outs, ArgLocs, CCInfo);
  }

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = CCInfo.getStackSize();

  Chain = DAG.getCALLSEQ_START(Chain, NumBytes, 0, DL);

  SmallVector<std::pair<unsigned, SDValue>, 8> RegsToPass;

  // First, walk the register assignments, inserting copies.
  unsigned AI, AE;
  bool HasStackArgs = false;
  for (AI = 0, AE = ArgLocs.size(); AI != AE; ++AI) {
    CCValAssign &VA = ArgLocs[AI];
    EVT RegVT = VA.getLocVT();
    SDValue Arg = OutVals[AI];

    // Promote the value if needed. With Clang this should not happen.
    switch (VA.getLocInfo()) {
    default:
      llvm_unreachable("Unknown loc info!");
    case CCValAssign::Full:
      break;
    case CCValAssign::SExt:
      Arg = DAG.getNode(ISD::SIGN_EXTEND, DL, RegVT, Arg);
      break;
    case CCValAssign::ZExt:
      Arg = DAG.getNode(ISD::ZERO_EXTEND, DL, RegVT, Arg);
      break;
    case CCValAssign::AExt:
      Arg = DAG.getNode(ISD::ANY_EXTEND, DL, RegVT, Arg);
      break;
    case CCValAssign::BCvt:
      Arg = DAG.getNode(ISD::BITCAST, DL, RegVT, Arg);
      break;
    }

    // Stop when we encounter a stack argument, we need to process them
    // in reverse order in the loop below.
    if (VA.isMemLoc()) {
      HasStackArgs = true;
      break;
    }

    // Arguments that can be passed on registers must be kept in the RegsToPass
    // vector.
    RegsToPass.push_back(std::make_pair(VA.getLocReg(), Arg));
  }

  // Second, stack arguments have to walked.
  // Previously this code created chained stores but those chained stores appear
  // to be unchained in the legalization phase. Therefore, do not attempt to
  // chain them here. In fact, chaining them here somehow causes the first and
  // second store to be reversed which is the exact opposite of the intended
  // effect.
  if (HasStackArgs) {
    auto &mfi = *MF.getInfo<PDPMachineFunctionInfo>();
    mfi.setHasStacksCall(true);

    SmallVector<SDValue, 8> MemOpChains;
    for (; AI != AE; AI++) {
      CCValAssign &VA = ArgLocs[AI];
      SDValue Arg = OutVals[AI];

      assert(VA.isMemLoc());

      // SP points to one stack slot further so add one to adjust it.
      SDValue PtrOff = DAG.getNode(
          ISD::ADD, DL, getPointerTy(DAG.getDataLayout()),
          DAG.getRegister(PDP::R6, getPointerTy(DAG.getDataLayout())),
          DAG.getIntPtrConstant(VA.getLocMemOffset() /*+ 1*/, DL));

      auto st = DAG.getStore(Chain, DL, Arg, PtrOff, MachinePointerInfo::getStack(MF, VA.getLocMemOffset()));

      MemOpChains.push_back(st);

      /*auto pushNode = SDValue(DAG.getMachineNode(PDP::PUSH, DL, MVT::Other, Arg, Chain), 0);

      MemOpChains.push_back(pushNode);*/
    }

    if (!MemOpChains.empty())
      Chain = DAG.getNode(ISD::TokenFactor, DL, MVT::Other, MemOpChains);
  }

  // Build a sequence of copy-to-reg nodes chained together with token chain and
  // flag operands which copy the outgoing args into registers.  The InGlue in
  // necessary since all emited instructions must be stuck together.
  SDValue InGlue;
  for (auto Reg : RegsToPass) {
    Chain = DAG.getCopyToReg(Chain, DL, Reg.first, Reg.second, InGlue);
    InGlue = Chain.getValue(1);
  }

  // Returns a chain & a flag for retval copy to use.
  SDVTList NodeTys = DAG.getVTList(MVT::Other, MVT::Glue);
  SmallVector<SDValue, 8> Ops;
  Ops.push_back(Chain);
  Ops.push_back(Callee);

  // Add argument registers to the end of the list so that they are known live
  // into the call.
  for (auto Reg : RegsToPass) {
    Ops.push_back(DAG.getRegister(Reg.first, Reg.second.getValueType()));
  }

  // The zero register (usually R1) must be passed as an implicit register so
  // that this register is correctly zeroed in interrupts.
  // Ops.push_back(DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8));

  // Add a register mask operand representing the call-preserved registers.
  const TargetRegisterInfo *TRI = Subtarget.getRegisterInfo();
  const uint32_t *Mask =
      TRI->getCallPreservedMask(DAG.getMachineFunction(), CallConv);
  assert(Mask && "Missing call preserved mask for calling convention");
  Ops.push_back(DAG.getRegisterMask(Mask));

  if (InGlue.getNode()) {
    Ops.push_back(InGlue);
  }

  Chain = DAG.getNode(PDPISD::CALL, DL, NodeTys, Ops);
  InGlue = Chain.getValue(1);

  // Create the CALLSEQ_END node.
  Chain = DAG.getCALLSEQ_END(Chain, NumBytes, 0, InGlue, DL);

  if (!Ins.empty()) {
    InGlue = Chain.getValue(1);
  }

  // Handle result values, copying them out of physregs into vregs that we
  // return.
  return LowerCallResult(Chain, InGlue, CallConv, isVarArg, Ins, DL, DAG,
                         InVals);
}

/// Lower the result values of a call into the
/// appropriate copies out of appropriate physical registers.
///
SDValue PDPTargetLowering::LowerCallResult(
    SDValue Chain, SDValue InGlue, CallingConv::ID CallConv, bool isVarArg,
    const SmallVectorImpl<ISD::InputArg> &Ins, const SDLoc &dl,
    SelectionDAG &DAG, SmallVectorImpl<SDValue> &InVals) const {

  // Assign locations to each value returned by this call.
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  // Handle runtime calling convs.
  if (CallConv == CallingConv::PDP_BUILTIN) {
    CCInfo.AnalyzeCallResult(Ins, RetCC_PDP_BUILTIN);
  } else {
    CCInfo.AnalyzeCallResult(Ins, RetCC_PDP_C);
    // analyzeReturnValues(Ins, CCInfo, Subtarget.hasTinyEncoding());
  }

  // Copy all of the result registers out of their specified physreg.
  for (CCValAssign const &RVLoc : RVLocs) {
    Chain = DAG.getCopyFromReg(Chain, dl, RVLoc.getLocReg(), RVLoc.getValVT(),
                               InGlue)
                .getValue(1);
    InGlue = Chain.getValue(2);
    InVals.push_back(Chain.getValue(0));
  }

  return Chain;
}

//===----------------------------------------------------------------------===//
//               Return Value Calling Convention Implementation
//===----------------------------------------------------------------------===//

bool PDPTargetLowering::CanLowerReturn(
    CallingConv::ID CallConv, MachineFunction &MF, bool isVarArg,
    const SmallVectorImpl<ISD::OutputArg> &Outs, LLVMContext &Context,
    const Type *RetTy) const {
  SmallVector<CCValAssign, 16> RVLocs;
  CCState CCInfo(CallConv, isVarArg, MF, RVLocs, Context);

  if (CallConv == CallingConv::PDP_BUILTIN)
    return CCInfo.CheckReturn(Outs, RetCC_PDP_BUILTIN);

  return CCInfo.CheckReturn(Outs, RetCC_PDP_C);

  /*if (CallConv == CallingConv::PDP_BUILTIN) {
    SmallVector<CCValAssign, 16> RVLocs;
    CCState CCInfo(CallConv, isVarArg, MF, RVLocs, Context);
    return CCInfo.CheckReturn(Outs, RetCC_PDP_BUILTIN);
  }

  unsigned TotalBytes = getTotalArgumentsSizeInBytes(Outs);
  return TotalBytes <= (unsigned)(Subtarget.hasTinyEncoding() ? 4 : 8);*/
}

SDValue
PDPTargetLowering::LowerReturn(SDValue Chain, CallingConv::ID CallConv,
                               bool isVarArg,
                               const SmallVectorImpl<ISD::OutputArg> &Outs,
                               const SmallVectorImpl<SDValue> &OutVals,
                               const SDLoc &dl, SelectionDAG &DAG) const {

  // CCValAssign - represent the assignment of the return value to locations.
  SmallVector<CCValAssign, 16> RVLocs;

  // CCState - Info about the registers and stack slot.
  CCState CCInfo(CallConv, isVarArg, DAG.getMachineFunction(), RVLocs,
                 *DAG.getContext());

  MachineFunction &MF = DAG.getMachineFunction();

  // Analyze return values.
  if (CallConv == CallingConv::PDP_BUILTIN) {
    CCInfo.AnalyzeReturn(Outs, RetCC_PDP_BUILTIN);
  } else {
    CCInfo.AnalyzeReturn(Outs, RetCC_PDP_C);
    // analyzeReturnValues(Outs, CCInfo);
  }

  SDValue Glue;
  SmallVector<SDValue, 4> RetOps(1, Chain);

  // Copy the result values into the output registers.
  for (unsigned i = 0, e = RVLocs.size(); i != e; ++i) {
    CCValAssign &VA = RVLocs[i];
    assert(VA.isRegLoc() && "Can only return in registers!");

    Chain = DAG.getCopyToReg(Chain, dl, VA.getLocReg(), OutVals[i], Glue);

    // Guarantee that all emitted copies are stuck together with flags.
    Glue = Chain.getValue(1);
    RetOps.push_back(DAG.getRegister(VA.getLocReg(), VA.getLocVT()));
  }

  // Don't emit the ret/reti instruction when the naked attribute is present in
  // the function being compiled.
  if (MF.getFunction().getAttributes().hasFnAttr(Attribute::Naked)) {
    return Chain;
  }

  const PDPMachineFunctionInfo *AFI = MF.getInfo<PDPMachineFunctionInfo>();

  /*if (!AFI->isInterruptOrSignalHandler()) {
    // The return instruction has an implicit zero register operand: it must
    // contain zero on return.
    // This is not needed in interrupts however, where the zero register is
    // handled specially (only pushed/popped when needed).
    RetOps.push_back(DAG.getRegister(Subtarget.getZeroRegister(), MVT::i8));
  }*/

  unsigned RetOpc =
      AFI->isInterruptOrSignalHandler() ? PDPISD::RETI_GLUE : PDPISD::RET_GLUE;

  RetOps[0] = Chain; // Update chain.

  if (Glue.getNode()) {
    RetOps.push_back(Glue);
  }

  return DAG.getNode(RetOpc, dl, MVT::Other, RetOps);
}

//===----------------------------------------------------------------------===//
//  Custom Inserters
//===----------------------------------------------------------------------===//

MachineBasicBlock *PDPTargetLowering::insertShift(MachineInstr &MI,
                                                  MachineBasicBlock *BB,
                                                  bool Tiny) const {
  llvm_unreachable("*PDPTargetLowering::insertShift");

  unsigned Opc;
  const TargetRegisterClass *RC;
  bool HasRepeatedOperand = false;
  MachineFunction *F = BB->getParent();
  MachineRegisterInfo &RI = F->getRegInfo();
  const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  DebugLoc dl = MI.getDebugLoc();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid shift opcode!");
  /*case PDP::Lsl8:
    Opc = PDP::ADDRdRr; // LSL is an alias of ADD Rd, Rd
    RC = &PDP::GPR8RegClass;
    HasRepeatedOperand = true;
    break;
  case PDP::Lsl16:
    Opc = PDP::LSLWRd;
    RC = &PDP::DREGSRegClass;
    break;
  case PDP::Asr8:
    Opc = PDP::ASRRd;
    RC = &PDP::GPR8RegClass;
    break;
  case PDP::Asr16:
    Opc = PDP::ASRWRd;
    RC = &PDP::DREGSRegClass;
    break;
  case PDP::Lsr8:
    Opc = PDP::LSRRd;
    RC = &PDP::GPR8RegClass;
    break;
  case PDP::Lsr16:
    Opc = PDP::LSRWRd;
    RC = &PDP::DREGSRegClass;
    break;*/
  /*case PDP::Rol8:
    Opc = Tiny ? PDP::ROLBRdR17 : PDP::ROLBRdR1;
    RC = &PDP::GPR8RegClass;
    break;
  case PDP::Rol16:
    Opc = PDP::ROLWRd;
    RC = &PDP::DREGSRegClass;
    break;
  case PDP::Ror8:
    Opc = PDP::RORBRd;
    RC = &PDP::GPR8RegClass;
    break;
  case PDP::Ror16:
    Opc = PDP::RORWRd;
    RC = &PDP::DREGSRegClass;
    break;*/
  }

  const BasicBlock *LLVM_BB = BB->getBasicBlock();

  MachineFunction::iterator I;
  for (I = BB->getIterator(); I != F->end() && &(*I) != BB; ++I)
    ;
  if (I != F->end())
    ++I;

  // Create loop block.
  MachineBasicBlock *LoopBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *CheckBB = F->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *RemBB = F->CreateMachineBasicBlock(LLVM_BB);

  F->insert(I, LoopBB);
  F->insert(I, CheckBB);
  F->insert(I, RemBB);

  // Update machine-CFG edges by transferring all successors of the current
  // block to the block containing instructions after shift.
  RemBB->splice(RemBB->begin(), BB, std::next(MachineBasicBlock::iterator(MI)),
                BB->end());
  RemBB->transferSuccessorsAndUpdatePHIs(BB);

  // Add edges BB => LoopBB => CheckBB => RemBB, CheckBB => LoopBB.
  BB->addSuccessor(CheckBB);
  LoopBB->addSuccessor(CheckBB);
  CheckBB->addSuccessor(LoopBB);
  CheckBB->addSuccessor(RemBB);

  Register ShiftAmtReg = RI.createVirtualRegister(&PDP::GPR16RegClass);
  Register ShiftAmtReg2 = RI.createVirtualRegister(&PDP::GPR16RegClass);
  Register ShiftReg = RI.createVirtualRegister(RC);
  Register ShiftReg2 = RI.createVirtualRegister(RC);
  Register ShiftAmtSrcReg = MI.getOperand(2).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  Register DstReg = MI.getOperand(0).getReg();

  // BB:
  // rjmp CheckBB
  BuildMI(BB, dl, TII.get(PDP::BR)).addMBB(CheckBB);

  // LoopBB:
  // ShiftReg2 = shift ShiftReg
  auto ShiftMI = BuildMI(LoopBB, dl, TII.get(Opc), ShiftReg2).addReg(ShiftReg);
  if (HasRepeatedOperand)
    ShiftMI.addReg(ShiftReg);

  // CheckBB:
  // ShiftReg = phi [%SrcReg, BB], [%ShiftReg2, LoopBB]
  // ShiftAmt = phi [%N,      BB], [%ShiftAmt2, LoopBB]
  // DestReg  = phi [%SrcReg, BB], [%ShiftReg,  LoopBB]
  // ShiftAmt2 = ShiftAmt - 1;
  // if (ShiftAmt2 >= 0) goto LoopBB;
  BuildMI(CheckBB, dl, TII.get(PDP::PHI), ShiftReg)
      .addReg(SrcReg)
      .addMBB(BB)
      .addReg(ShiftReg2)
      .addMBB(LoopBB);
  BuildMI(CheckBB, dl, TII.get(PDP::PHI), ShiftAmtReg)
      .addReg(ShiftAmtSrcReg)
      .addMBB(BB)
      .addReg(ShiftAmtReg2)
      .addMBB(LoopBB);
  BuildMI(CheckBB, dl, TII.get(PDP::PHI), DstReg)
      .addReg(SrcReg)
      .addMBB(BB)
      .addReg(ShiftReg2)
      .addMBB(LoopBB);

  //!!BuildMI(CheckBB, dl, TII.get(PDP::DECRd), ShiftAmtReg2).addReg(ShiftAmtReg);
  BuildMI(CheckBB, dl, TII.get(PDP::BPL)).addMBB(LoopBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return RemBB;
}

// Do a multibyte PDP shift. Insert shift instructions and put the output
// registers in the Regs array.
// Because PDP does not have a normal shift instruction (only a single bit shift
// instruction), we have to emulate this behavior with other instructions.
// It first tries large steps (moving registers around) and then smaller steps
// like single bit shifts.
// Large shifts actually reduce the number of shifted registers, so the below
// algorithms have to work independently of the number of registers that are
// shifted.
// For more information and background, see this blogpost:
// https://aykevl.nl/2021/02/PDP-bitshift
/*static void insertMultibyteShift(MachineInstr &MI, MachineBasicBlock *BB,
                                 MutableArrayRef<std::pair<Register, int>> Regs,
                                 ISD::NodeType Opc, int64_t ShiftAmt) {
  const TargetInstrInfo &TII = *BB->getParent()->getSubtarget().getInstrInfo();
  const PDPSubtarget &STI = BB->getParent()->getSubtarget<PDPSubtarget>();
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();
  const DebugLoc &dl = MI.getDebugLoc();

  const bool ShiftLeft = Opc == ISD::SHL;
  const bool ArithmeticShift = Opc == ISD::SRA;

  // Zero a register, for use in later operations.
  Register ZeroReg = MRI.createVirtualRegister(&PDP::GPR8RegClass);
  BuildMI(*BB, MI, dl, TII.get(PDP::COPY), ZeroReg)
      .addReg(STI.getZeroRegister());

  // Do a shift modulo 6 or 7. This is a bit more complicated than most shifts
  // and is hard to compose with the rest, so these are special cased.
  // The basic idea is to shift one or two bits in the opposite direction and
  // then move registers around to get the correct end result.
  if (ShiftLeft && (ShiftAmt % 8) >= 6) {
    // Left shift modulo 6 or 7.

    // Create a slice of the registers we're going to modify, to ease working
    // with them.
    size_t ShiftRegsOffset = ShiftAmt / 8;
    size_t ShiftRegsSize = Regs.size() - ShiftRegsOffset;
    MutableArrayRef<std::pair<Register, int>> ShiftRegs =
        Regs.slice(ShiftRegsOffset, ShiftRegsSize);

    // Shift one to the right, keeping the least significant bit as the carry
    // bit.
    insertMultibyteShift(MI, BB, ShiftRegs, ISD::SRL, 1);

    // Rotate the least significant bit from the carry bit into a new register
    // (that starts out zero).
    Register LowByte = MRI.createVirtualRegister(&PDP::GPR8RegClass);
    BuildMI(*BB, MI, dl, TII.get(PDP::RORRd), LowByte).addReg(ZeroReg);

    // Shift one more to the right if this is a modulo-6 shift.
    if (ShiftAmt % 8 == 6) {
      insertMultibyteShift(MI, BB, ShiftRegs, ISD::SRL, 1);
      Register NewLowByte = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      BuildMI(*BB, MI, dl, TII.get(PDP::RORRd), NewLowByte).addReg(LowByte);
      LowByte = NewLowByte;
    }

    // Move all registers to the left, zeroing the bottom registers as needed.
    for (size_t I = 0; I < Regs.size(); I++) {
      int ShiftRegsIdx = I + 1;
      if (ShiftRegsIdx < (int)ShiftRegs.size()) {
        Regs[I] = ShiftRegs[ShiftRegsIdx];
      } else if (ShiftRegsIdx == (int)ShiftRegs.size()) {
        Regs[I] = std::pair(LowByte, 0);
      } else {
        Regs[I] = std::pair(ZeroReg, 0);
      }
    }

    return;
  }

  // Right shift modulo 6 or 7.
  if (!ShiftLeft && (ShiftAmt % 8) >= 6) {
    // Create a view on the registers we're going to modify, to ease working
    // with them.
    size_t ShiftRegsSize = Regs.size() - (ShiftAmt / 8);
    MutableArrayRef<std::pair<Register, int>> ShiftRegs =
        Regs.slice(0, ShiftRegsSize);

    // Shift one to the left.
    insertMultibyteShift(MI, BB, ShiftRegs, ISD::SHL, 1);

    // Sign or zero extend the most significant register into a new register.
    // The HighByte is the byte that still has one (or two) bits from the
    // original value. The ExtByte is purely a zero/sign extend byte (all bits
    // are either 0 or 1).
    Register HighByte = MRI.createVirtualRegister(&PDP::GPR8RegClass);
    Register ExtByte = 0;
    if (ArithmeticShift) {
      // Sign-extend bit that was shifted out last.
      BuildMI(*BB, MI, dl, TII.get(PDP::SBCRdRr), HighByte)
          .addReg(HighByte, RegState::Undef)
          .addReg(HighByte, RegState::Undef);
      ExtByte = HighByte;
      // The highest bit of the original value is the same as the zero-extend
      // byte, so HighByte and ExtByte are the same.
    } else {
      // Use the zero register for zero extending.
      ExtByte = ZeroReg;
      // Rotate most significant bit into a new register (that starts out zero).
      BuildMI(*BB, MI, dl, TII.get(PDP::ADCRdRr), HighByte)
          .addReg(ExtByte)
          .addReg(ExtByte);
    }

    // Shift one more to the left for modulo 6 shifts.
    if (ShiftAmt % 8 == 6) {
      insertMultibyteShift(MI, BB, ShiftRegs, ISD::SHL, 1);
      // Shift the topmost bit into the HighByte.
      Register NewExt = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      BuildMI(*BB, MI, dl, TII.get(PDP::ADCRdRr), NewExt)
          .addReg(HighByte)
          .addReg(HighByte);
      HighByte = NewExt;
    }

    // Move all to the right, while sign or zero extending.
    for (int I = Regs.size() - 1; I >= 0; I--) {
      int ShiftRegsIdx = I - (Regs.size() - ShiftRegs.size()) - 1;
      if (ShiftRegsIdx >= 0) {
        Regs[I] = ShiftRegs[ShiftRegsIdx];
      } else if (ShiftRegsIdx == -1) {
        Regs[I] = std::pair(HighByte, 0);
      } else {
        Regs[I] = std::pair(ExtByte, 0);
      }
    }

    return;
  }

  // For shift amounts of at least one register, simply rename the registers and
  // zero the bottom registers.
  while (ShiftLeft && ShiftAmt >= 8) {
    // Move all registers one to the left.
    for (size_t I = 0; I < Regs.size() - 1; I++) {
      Regs[I] = Regs[I + 1];
    }

    // Zero the least significant register.
    Regs[Regs.size() - 1] = std::pair(ZeroReg, 0);

    // Continue shifts with the leftover registers.
    Regs = Regs.drop_back(1);

    ShiftAmt -= 8;
  }

  // And again, the same for right shifts.
  Register ShrExtendReg = 0;
  if (!ShiftLeft && ShiftAmt >= 8) {
    if (ArithmeticShift) {
      // Sign extend the most significant register into ShrExtendReg.
      ShrExtendReg = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      Register Tmp = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      BuildMI(*BB, MI, dl, TII.get(PDP::ADDRdRr), Tmp)
          .addReg(Regs[0].first, 0, Regs[0].second)
          .addReg(Regs[0].first, 0, Regs[0].second);
      BuildMI(*BB, MI, dl, TII.get(PDP::SBCRdRr), ShrExtendReg)
          .addReg(Tmp)
          .addReg(Tmp);
    } else {
      ShrExtendReg = ZeroReg;
    }
    for (; ShiftAmt >= 8; ShiftAmt -= 8) {
      // Move all registers one to the right.
      for (size_t I = Regs.size() - 1; I != 0; I--) {
        Regs[I] = Regs[I - 1];
      }

      // Zero or sign extend the most significant register.
      Regs[0] = std::pair(ShrExtendReg, 0);

      // Continue shifts with the leftover registers.
      Regs = Regs.drop_front(1);
    }
  }

  // The bigger shifts are already handled above.
  assert((ShiftAmt < 8) && "Unexpect shift amount");

  // Shift by four bits, using a complicated swap/eor/andi/eor sequence.
  // It only works for logical shifts because the bits shifted in are all
  // zeroes.
  // To shift a single byte right, it produces code like this:
  //   swap r0
  //   andi r0, 0x0f
  // For a two-byte (16-bit) shift, it adds the following instructions to shift
  // the upper byte into the lower byte:
  //   swap r1
  //   eor r0, r1
  //   andi r1, 0x0f
  //   eor r0, r1
  // For bigger shifts, it repeats the above sequence. For example, for a 3-byte
  // (24-bit) shift it adds:
  //   swap r2
  //   eor r1, r2
  //   andi r2, 0x0f
  //   eor r1, r2
  if (!ArithmeticShift && ShiftAmt >= 4) {
    Register Prev = 0;
    for (size_t I = 0; I < Regs.size(); I++) {
      size_t Idx = ShiftLeft ? I : Regs.size() - I - 1;
      Register SwapReg = MRI.createVirtualRegister(&PDP::LD8RegClass);
      BuildMI(*BB, MI, dl, TII.get(PDP::SWAPRd), SwapReg)
          .addReg(Regs[Idx].first, 0, Regs[Idx].second);
      if (I != 0) {
        Register R = MRI.createVirtualRegister(&PDP::GPR8RegClass);
        BuildMI(*BB, MI, dl, TII.get(PDP::EORRdRr), R)
            .addReg(Prev)
            .addReg(SwapReg);
        Prev = R;
      }
      Register AndReg = MRI.createVirtualRegister(&PDP::LD8RegClass);
      BuildMI(*BB, MI, dl, TII.get(PDP::ANDIRdK), AndReg)
          .addReg(SwapReg)
          .addImm(ShiftLeft ? 0xf0 : 0x0f);
      if (I != 0) {
        Register R = MRI.createVirtualRegister(&PDP::GPR8RegClass);
        BuildMI(*BB, MI, dl, TII.get(PDP::EORRdRr), R)
            .addReg(Prev)
            .addReg(AndReg);
        size_t PrevIdx = ShiftLeft ? Idx - 1 : Idx + 1;
        Regs[PrevIdx] = std::pair(R, 0);
      }
      Prev = AndReg;
      Regs[Idx] = std::pair(AndReg, 0);
    }
    ShiftAmt -= 4;
  }

  // Shift by one. This is the fallback that always works, and the shift
  // operation that is used for 1, 2, and 3 bit shifts.
  while (ShiftLeft && ShiftAmt) {
    // Shift one to the left.
    for (ssize_t I = Regs.size() - 1; I >= 0; I--) {
      Register Out = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      Register In = Regs[I].first;
      Register InSubreg = Regs[I].second;
      if (I == (ssize_t)Regs.size() - 1) { // first iteration
        BuildMI(*BB, MI, dl, TII.get(PDP::ADDRdRr), Out)
            .addReg(In, 0, InSubreg)
            .addReg(In, 0, InSubreg);
      } else {
        BuildMI(*BB, MI, dl, TII.get(PDP::ADCRdRr), Out)
            .addReg(In, 0, InSubreg)
            .addReg(In, 0, InSubreg);
      }
      Regs[I] = std::pair(Out, 0);
    }
    ShiftAmt--;
  }
  while (!ShiftLeft && ShiftAmt) {
    // Shift one to the right.
    for (size_t I = 0; I < Regs.size(); I++) {
      Register Out = MRI.createVirtualRegister(&PDP::GPR8RegClass);
      Register In = Regs[I].first;
      Register InSubreg = Regs[I].second;
      if (I == 0) {
        unsigned Opc = ArithmeticShift ? PDP::ASRRd : PDP::LSRRd;
        BuildMI(*BB, MI, dl, TII.get(Opc), Out).addReg(In, 0, InSubreg);
      } else {
        BuildMI(*BB, MI, dl, TII.get(PDP::RORRd), Out).addReg(In, 0, InSubreg);
      }
      Regs[I] = std::pair(Out, 0);
    }
    ShiftAmt--;
  }

  if (ShiftAmt != 0) {
    llvm_unreachable("don't know how to shift!"); // sanity check
  }
}*/

/*// Do a wide (32-bit) shift.
MachineBasicBlock *
PDPTargetLowering::insertWideShift(MachineInstr &MI,
                                   MachineBasicBlock *BB) const {
  const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  const DebugLoc &dl = MI.getDebugLoc();

  // How much to shift to the right (meaning: a negative number indicates a left
  // shift).
  int64_t ShiftAmt = MI.getOperand(4).getImm();
  ISD::NodeType Opc;
  switch (MI.getOpcode()) {
  case PDP::Lsl32:
    Opc = ISD::SHL;
    break;
  case PDP::Lsr32:
    Opc = ISD::SRL;
    break;
  case PDP::Asr32:
    Opc = ISD::SRA;
    break;
  }

  // Read the input registers, with the most significant register at index 0.
  std::array<std::pair<Register, int>, 4> Registers = {
      std::pair(MI.getOperand(3).getReg(), PDP::sub_hi),
      std::pair(MI.getOperand(3).getReg(), PDP::sub_lo),
      std::pair(MI.getOperand(2).getReg(), PDP::sub_hi),
      std::pair(MI.getOperand(2).getReg(), PDP::sub_lo),
  };

  // Do the shift. The registers are modified in-place.
  insertMultibyteShift(MI, BB, Registers, Opc, ShiftAmt);

  // Combine the 8-bit registers into 16-bit register pairs.
  // This done either from LSB to MSB or from MSB to LSB, depending on the
  // shift. It's an optimization so that the register allocator will use the
  // fewest movs possible (which order we use isn't a correctness issue, just an
  // optimization issue).
  //   - lsl prefers starting from the most significant byte (2nd case).
  //   - lshr prefers starting from the least significant byte (1st case).
  //   - for ashr it depends on the number of shifted bytes.
  // Some shift operations still don't get the most optimal mov sequences even
  // with this distinction. TODO: figure out why and try to fix it (but we're
  // already equal to or faster than PDP-gcc in all cases except ashr 8).
  if (Opc != ISD::SHL &&
      (Opc != ISD::SRA || (ShiftAmt < 16 || ShiftAmt >= 22))) {
    // Use the resulting registers starting with the least significant byte.
    BuildMI(*BB, MI, dl, TII.get(PDP::REG_SEQUENCE), MI.getOperand(0).getReg())
        .addReg(Registers[3].first, 0, Registers[3].second)
        .addImm(PDP::sub_lo)
        .addReg(Registers[2].first, 0, Registers[2].second)
        .addImm(PDP::sub_hi);
    BuildMI(*BB, MI, dl, TII.get(PDP::REG_SEQUENCE), MI.getOperand(1).getReg())
        .addReg(Registers[1].first, 0, Registers[1].second)
        .addImm(PDP::sub_lo)
        .addReg(Registers[0].first, 0, Registers[0].second)
        .addImm(PDP::sub_hi);
  } else {
    // Use the resulting registers starting with the most significant byte.
    BuildMI(*BB, MI, dl, TII.get(PDP::REG_SEQUENCE), MI.getOperand(1).getReg())
        .addReg(Registers[0].first, 0, Registers[0].second)
        .addImm(PDP::sub_hi)
        .addReg(Registers[1].first, 0, Registers[1].second)
        .addImm(PDP::sub_lo);
    BuildMI(*BB, MI, dl, TII.get(PDP::REG_SEQUENCE), MI.getOperand(0).getReg())
        .addReg(Registers[2].first, 0, Registers[2].second)
        .addImm(PDP::sub_hi)
        .addReg(Registers[3].first, 0, Registers[3].second)
        .addImm(PDP::sub_lo);
  }

  // Remove the pseudo instruction.
  MI.eraseFromParent();
  return BB;
}*/

static bool isCopyMulResult(MachineBasicBlock::iterator const &I) {
  if (I->getOpcode() == PDP::COPY) {
    Register SrcReg = I->getOperand(1).getReg();
    return (SrcReg == PDP::R0 || SrcReg == PDP::R1);
  }

  return false;
}

// The mul instructions wreak havock on our zero_reg R1. We need to clear it
// after the result has been evacuated. This is probably not the best way to do
// it, but it works for now.
MachineBasicBlock *PDPTargetLowering::insertMul(MachineInstr &MI,
                                                MachineBasicBlock *BB) const {
  llvm_unreachable("PDPTargetLowering::insertMul");

  /*const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  MachineBasicBlock::iterator I(MI);
  ++I; // in any case insert *after* the mul instruction
  if (isCopyMulResult(I))
    ++I;
  if (isCopyMulResult(I))
    ++I;
  BuildMI(*BB, I, MI.getDebugLoc(), TII.get(PDP::EORRdRr), PDP::R1)
      .addReg(PDP::R1)
      .addReg(PDP::R1);
  return BB;*/
}

// Insert a read from the zero register.
/*MachineBasicBlock *
PDPTargetLowering::insertCopyZero(MachineInstr &MI,
                                  MachineBasicBlock *BB) const {
  const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  MachineBasicBlock::iterator I(MI);
  BuildMI(*BB, I, MI.getDebugLoc(), TII.get(PDP::COPY))
      .add(MI.getOperand(0))
      .addReg(Subtarget.getZeroRegister());
  MI.eraseFromParent();
  return BB;
}*/

// Lower atomicrmw operation to disable interrupts, do operation, and restore
// interrupts. This works because all PDP microcontrollers are single core.
/*MachineBasicBlock *PDPTargetLowering::insertAtomicArithmeticOp(
    MachineInstr &MI, MachineBasicBlock *BB, unsigned Opcode, int Width) const {
  MachineRegisterInfo &MRI = BB->getParent()->getRegInfo();
  const TargetInstrInfo &TII = *Subtarget.getInstrInfo();
  MachineBasicBlock::iterator I(MI);
  DebugLoc dl = MI.getDebugLoc();

  // Example instruction sequence, for an atomic 8-bit add:
  //   ldi r25, 5
  //   in r0, SREG
  //   cli
  //   ld r24, X
  //   add r25, r24
  //   st X, r25
  //   out SREG, r0

  const TargetRegisterClass *RC =
      (Width == 8) ? &PDP::GPR8RegClass : &PDP::DREGSRegClass;
  unsigned LoadOpcode = (Width == 8) ? PDP::LDRdPtr : PDP::LDWRdPtr;
  unsigned StoreOpcode = (Width == 8) ? PDP::STPtrRr : PDP::STWPtrRr;

  // Disable interrupts.
  BuildMI(*BB, I, dl, TII.get(PDP::INRdA), Subtarget.getTmpRegister())
      .addImm(Subtarget.getIORegSREG());
  BuildMI(*BB, I, dl, TII.get(PDP::BCLRs)).addImm(7);

  // Load the original value.
  BuildMI(*BB, I, dl, TII.get(LoadOpcode), MI.getOperand(0).getReg())
      .add(MI.getOperand(1));

  // Do the arithmetic operation.
  Register Result = MRI.createVirtualRegister(RC);
  BuildMI(*BB, I, dl, TII.get(Opcode), Result)
      .addReg(MI.getOperand(0).getReg())
      .add(MI.getOperand(2));

  // Store the result.
  BuildMI(*BB, I, dl, TII.get(StoreOpcode))
      .add(MI.getOperand(1))
      .addReg(Result);

  // Restore interrupts.
  BuildMI(*BB, I, dl, TII.get(PDP::OUTARr))
      .addImm(Subtarget.getIORegSREG())
      .addReg(Subtarget.getTmpRegister());

  // Remove the pseudo instruction.
  MI.eraseFromParent();
  return BB;
}*/

MachineBasicBlock *
PDPTargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI,
                                               MachineBasicBlock *MBB) const {
  int Opc = MI.getOpcode();
  const PDPSubtarget &STI = MBB->getParent()->getSubtarget<PDPSubtarget>();

  // Pseudo shift instructions with a non constant shift amount are expanded
  // into a loop.
  switch (Opc) {
  /*case PDP::Lsl8:
  case PDP::Lsl16:
  case PDP::Lsr8:
  case PDP::Lsr16:
  case PDP::Rol8:
  case PDP::Rol16:
  case PDP::Ror8:
  case PDP::Ror16:
  case PDP::Asr8:
  case PDP::Asr16:
    return insertShift(MI, MBB, STI.hasTinyEncoding());*/
  /*case PDP::Lsl32:
  case PDP::Lsr32:
  case PDP::Asr32:
    return insertWideShift(MI, MBB);*/
  /*case PDP::MULRdRr:
  case PDP::MULSRdRr:
    return insertMul(MI, MBB);*/
    //  case PDP::CopyZero:
    //    return insertCopyZero(MI, MBB);
/*
  case PDP::AtomicLoadAdd8:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ADDRdRr, 8);
  case PDP::AtomicLoadAdd16:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ADDWRdRr, 16);
  case PDP::AtomicLoadSub8:
    return insertAtomicArithmeticOp(MI, MBB, PDP::SUBRdRr, 8);
  case PDP::AtomicLoadSub16:
    return insertAtomicArithmeticOp(MI, MBB, PDP::SUBWRdRr, 16);
  case PDP::AtomicLoadAnd8:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ANDRdRr, 8);
  case PDP::AtomicLoadAnd16:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ANDWRdRr, 16);
  case PDP::AtomicLoadOr8:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ORRdRr, 8);
  case PDP::AtomicLoadOr16:
    return insertAtomicArithmeticOp(MI, MBB, PDP::ORWRdRr, 16);
  case PDP::AtomicLoadXor8:
    return insertAtomicArithmeticOp(MI, MBB, PDP::EORRdRr, 8);
  case PDP::AtomicLoadXor16:
    return insertAtomicArithmeticOp(MI, MBB, PDP::EORWRdRr, 16);
*/
  }

  switch (Opc) {
  case PDP::Select8:
  case PDP::Select16:
    return EmitSelect(MI, MBB);

  case PDP::Select2_8:
  case PDP::Select2_16:
    return EmitSelect2(MI, MBB);

  default:
    llvm_unreachable("Unexpected instr type to insert");
  }
}

MachineBasicBlock *
PDPTargetLowering::EmitSelect(MachineInstr &MI, MachineBasicBlock *MBB) const {
  int Opc = MI.getOpcode();
  const PDPSubtarget &STI = MBB->getParent()->getSubtarget<PDPSubtarget>();

  assert((Opc == PDP::Select16 || Opc == PDP::Select8) &&
         "Unexpected instr type to insert");

  const PDPInstrInfo &TII = (const PDPInstrInfo &)*MI.getParent()
                                ->getParent()
                                ->getSubtarget()
                                .getInstrInfo();
  DebugLoc dl = MI.getDebugLoc();

  // To "insert" a SELECT instruction, we insert the diamond
  // control-flow pattern. The incoming instruction knows the
  // destination vreg to set, the condition code register to branch
  // on, the true/false values to select between, and a branch opcode
  // to use.

  MachineFunction *MF = MBB->getParent();
  const BasicBlock *LLVM_BB = MBB->getBasicBlock();
  MachineBasicBlock *FallThrough = MBB->getFallThrough();

  // If the current basic block falls through to another basic block,
  // we must insert an unconditional branch to the fallthrough destination
  // if we are to insert basic blocks at the prior fallthrough point.
  if (FallThrough != nullptr) {
    BuildMI(MBB, dl, TII.get(PDP::BR)).addMBB(FallThrough);
  }

  MachineBasicBlock *trueMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *falseMBB = MF->CreateMachineBasicBlock(LLVM_BB);

  MachineFunction::iterator I;
  for (I = MF->begin(); I != MF->end() && &(*I) != MBB; ++I)
    ;
  if (I != MF->end())
    ++I;
  MF->insert(I, trueMBB);
  MF->insert(I, falseMBB);

  // Set the call frame size on entry to the new basic blocks.
  unsigned CallFrameSize = TII.getCallFrameSizeAt(MI);
  trueMBB->setCallFrameSize(CallFrameSize);
  falseMBB->setCallFrameSize(CallFrameSize);

  // Transfer remaining instructions and all successors of the current
  // block to the block which will contain the Phi node for the
  // select.
  trueMBB->splice(trueMBB->begin(), MBB,
                  std::next(MachineBasicBlock::iterator(MI)), MBB->end());
  trueMBB->transferSuccessorsAndUpdatePHIs(MBB);

  PDPCC::CondCodes CC = (PDPCC::CondCodes)MI.getOperand(3).getImm();
  BuildMI(MBB, dl, TII.getBrCond(CC)).addMBB(trueMBB);
  BuildMI(MBB, dl, TII.get(PDP::BR)).addMBB(falseMBB);
  MBB->addSuccessor(falseMBB);
  MBB->addSuccessor(trueMBB);

  // Unconditionally flow back to the true block
  BuildMI(falseMBB, dl, TII.get(PDP::BR)).addMBB(trueMBB);
  falseMBB->addSuccessor(trueMBB);

  // Set up the Phi node to determine where we came from
  BuildMI(*trueMBB, trueMBB->begin(), dl, TII.get(PDP::PHI),
          MI.getOperand(0).getReg())
      .addReg(MI.getOperand(1).getReg())
      .addMBB(MBB)
      .addReg(MI.getOperand(2).getReg())
      .addMBB(falseMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return trueMBB;
}

MachineBasicBlock *
PDPTargetLowering::EmitSelect2(MachineInstr &MI, MachineBasicBlock *MBB) const {
  int Opc = MI.getOpcode();
  const PDPSubtarget &STI = MBB->getParent()->getSubtarget<PDPSubtarget>();

  assert((Opc == PDP::Select2_16 || Opc == PDP::Select2_8) &&
         "Unexpected instr type to insert");

  const PDPInstrInfo &TII = (const PDPInstrInfo &)*MI.getParent()
                                ->getParent()
                                ->getSubtarget()
                                .getInstrInfo();
  DebugLoc dl = MI.getDebugLoc();

  // To "insert" a SELECT instruction, we insert the diamond
  // control-flow pattern. The incoming instruction knows the
  // destination vreg to set, the condition code register to branch
  // on, the true/false values to select between, and a branch opcode
  // to use.

  MachineFunction *MF = MBB->getParent();
  const BasicBlock *LLVM_BB = MBB->getBasicBlock();
  MachineBasicBlock *FallThrough = MBB->getFallThrough();

  // If the current basic block falls through to another basic block,
  // we must insert an unconditional branch to the fallthrough destination
  // if we are to insert basic blocks at the prior fallthrough point.
  if (FallThrough != nullptr) {
    BuildMI(MBB, dl, TII.get(PDP::BR)).addMBB(FallThrough);
  }

  MachineBasicBlock *trueMBB = MF->CreateMachineBasicBlock(LLVM_BB);
  MachineBasicBlock *falseMBB = MF->CreateMachineBasicBlock(LLVM_BB);

  MachineFunction::iterator I;
  for (I = MF->begin(); I != MF->end() && &(*I) != MBB; ++I)
    ;
  if (I != MF->end())
    ++I;
  MF->insert(I, trueMBB);
  MF->insert(I, falseMBB);

  // Set the call frame size on entry to the new basic blocks.
  unsigned CallFrameSize = TII.getCallFrameSizeAt(MI);
  trueMBB->setCallFrameSize(CallFrameSize);
  falseMBB->setCallFrameSize(CallFrameSize);

  // Transfer remaining instructions and all successors of the current
  // block to the block which will contain the Phi node for the
  // select.
  trueMBB->splice(trueMBB->begin(), MBB,
                  std::next(MachineBasicBlock::iterator(MI)), MBB->end());
  trueMBB->transferSuccessorsAndUpdatePHIs(MBB);

  PDPCC::CondCodes CC = (PDPCC::CondCodes)MI.getOperand(11).getImm();

  auto lr = MI.getOperand(1).getReg();
  auto rr = MI.getOperand(5).getReg();


  unsigned OpCode = 0;

  if (rr != PDP::R7) {
    auto lc = MI.getParent()->getParent()->getRegInfo().getRegClassOrNull(lr);
    auto rc = MI.getParent()->getParent()->getRegInfo().getRegClassOrNull(rr);

    auto lhsIsWord = lc->getID() == PDP::GPR16RegClassID;
    auto rhsIsWord = rc->getID() == PDP::GPR16RegClassID;

    assert(lhsIsWord == rhsIsWord);

    OpCode = lhsIsWord ? PDP::CBR : PDP::CBRB;
  } else {
    auto lc = MI.getParent()->getParent()->getRegInfo().getRegClassOrNull(lr);

    auto lhsIsWord = lc->getID() == PDP::GPR16RegClassID;
    OpCode = lhsIsWord ? PDP::CBR : PDP::CBRB;
  }

  assert(OpCode != 0);

  auto cmp = BuildMI(MBB, dl, TII.get(OpCode));
  cmp->addOperand(MI.getOperand(1));
  cmp->addOperand(MI.getOperand(2));
  cmp->addOperand(MI.getOperand(3));
  cmp->addOperand(MI.getOperand(4));
  cmp->addOperand(MI.getOperand(5));
  cmp->addOperand(MI.getOperand(6));
  cmp->addOperand(MI.getOperand(7));
  cmp->addOperand(MI.getOperand(8));
  cmp->addOperand(MI.getOperand(11));
  cmp.addMBB(trueMBB).addImm(0);

//  BuildMI(MBB, dl, TII.getBrCond(CC)).addMBB(trueMBB);
  BuildMI(MBB, dl, TII.get(PDP::BR)).addMBB(falseMBB);
  MBB->addSuccessor(falseMBB);
  MBB->addSuccessor(trueMBB);

  // Unconditionally flow back to the true block
  BuildMI(falseMBB, dl, TII.get(PDP::BR)).addMBB(trueMBB);
  falseMBB->addSuccessor(trueMBB);

  // Set up the Phi node to determine where we came from
  BuildMI(*trueMBB, trueMBB->begin(), dl, TII.get(PDP::PHI),
          MI.getOperand(0).getReg())
      .addReg(MI.getOperand(9).getReg())
      .addMBB(MBB)
      .addReg(MI.getOperand(10).getReg())
      .addMBB(falseMBB);

  MI.eraseFromParent(); // The pseudo instruction is gone now.
  return trueMBB;
}

//===----------------------------------------------------------------------===//
//  Inline Asm Support
//===----------------------------------------------------------------------===//

PDPTargetLowering::ConstraintType
PDPTargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    // See http://www.nongnu.org/PDP-libc/user-manual/inline_asm.html
    switch (Constraint[0]) {
    default:
      break;
    case 'r':
      return C_RegisterClass;
    case 'a':
    case 'b':
    case 'c':
    case 'd':
    case 'e':
    case 'f':
    case 't': // Temporary register
    case 'x':
    case 'X': // Pointer register pair X
    case 'y':
    case 'Y': // Pointer register pair Y
    case 'z':
    case 'Z': // Pointer register pair Z
      return C_Register;
    case 'Q': // A memory address based on Y or Z pointer with displacement.
      return C_Memory;
    case 'G': // Floating point constant
    case 'I': // 6-bit positive integer constant
    case 'J': // 6-bit negative integer constant
    case 'K': // Integer constant (Range: 2)
    case 'L': // Integer constant (Range: 0)
    case 'M': // 8-bit integer constant
    case 'N': // Integer constant (Range: -1)
    case 'O': // Integer constant (Range: 8, 16, 24)
    case 'P': // Integer constant (Range: 1)
    case 'R': // Integer constant (Range: -6 to 5)x
      return C_Immediate;
    }
  }

  return TargetLowering::getConstraintType(Constraint);
}

InlineAsm::ConstraintCode
PDPTargetLowering::getInlineAsmMemConstraint(StringRef ConstraintCode) const {
  // Not sure if this is actually the right thing to do, but we got to do
  // *something* [agnat]
  switch (ConstraintCode[0]) {
  case 'Q':
    return InlineAsm::ConstraintCode::Q;
  }
  return TargetLowering::getInlineAsmMemConstraint(ConstraintCode);
}

PDPTargetLowering::ConstraintWeight
PDPTargetLowering::getSingleConstraintMatchWeight(
    AsmOperandInfo &info, const char *constraint) const {
  ConstraintWeight weight = CW_Invalid;
  Value *CallOperandVal = info.CallOperandVal;

  // If we don't have a value, we can't do a match,
  // but allow it at the lowest weight.
  // (this behaviour has been copied from the ARM backend)
  if (!CallOperandVal) {
    return CW_Default;
  }

  // Look at the constraint type.
  switch (*constraint) {
  default:
    weight = TargetLowering::getSingleConstraintMatchWeight(info, constraint);
    break;
  case 'd':
  case 'r':
  case 'l':
    weight = CW_Register;
    break;
  case 'a':
  case 'b':
  case 'e':
  case 'q':
  case 't':
  case 'w':
  case 'x':
  case 'X':
  case 'y':
  case 'Y':
  case 'z':
  case 'Z':
    weight = CW_SpecificReg;
    break;
  case 'G':
    if (const ConstantFP *C = dyn_cast<ConstantFP>(CallOperandVal)) {
      if (C->isZero()) {
        weight = CW_Constant;
      }
    }
    break;
  case 'I':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (isUInt<6>(C->getZExtValue())) {
        weight = CW_Constant;
      }
    }
    break;
  case 'J':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if ((C->getSExtValue() >= -63) && (C->getSExtValue() <= 0)) {
        weight = CW_Constant;
      }
    }
    break;
  case 'K':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (C->getZExtValue() == 2) {
        weight = CW_Constant;
      }
    }
    break;
  case 'L':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (C->getZExtValue() == 0) {
        weight = CW_Constant;
      }
    }
    break;
  case 'M':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (isUInt<8>(C->getZExtValue())) {
        weight = CW_Constant;
      }
    }
    break;
  case 'N':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (C->getSExtValue() == -1) {
        weight = CW_Constant;
      }
    }
    break;
  case 'O':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if ((C->getZExtValue() == 8) || (C->getZExtValue() == 16) ||
          (C->getZExtValue() == 24)) {
        weight = CW_Constant;
      }
    }
    break;
  case 'P':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if (C->getZExtValue() == 1) {
        weight = CW_Constant;
      }
    }
    break;
  case 'R':
    if (const ConstantInt *C = dyn_cast<ConstantInt>(CallOperandVal)) {
      if ((C->getSExtValue() >= -6) && (C->getSExtValue() <= 5)) {
        weight = CW_Constant;
      }
    }
    break;
  case 'Q':
    weight = CW_Memory;
    break;
  }

  return weight;
}

std::pair<unsigned, const TargetRegisterClass *>
PDPTargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                                                StringRef Constraint,
                                                MVT VT) const {

  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    /*case 'a': // Simple upper registers r16..r23.
      if (VT == MVT::i8)
        return std::make_pair(0U, &PDP::LD8loRegClass);
      else if (VT == MVT::i16)
        return std::make_pair(0U, &PDP::DREGSLD8loRegClass);
      break;
    case 'b': // Base pointer registers: y, z.
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(0U, &PDP::PTRDISPREGSRegClass);
      break;
    case 'd': // Upper registers r16..r31.
      if (VT == MVT::i8)
        return std::make_pair(0U, &PDP::LD8RegClass);
      else if (VT == MVT::i16)
        return std::make_pair(0U, &PDP::DLDREGSRegClass);
      break;
    case 'l': // Lower registers r0..r15.
      if (VT == MVT::i8)
        return std::make_pair(0U, &PDP::GPR8loRegClass);
      else if (VT == MVT::i16)
        return std::make_pair(0U, &PDP::DREGSloRegClass);
      break;
    case 'e': // Pointer register pairs: x, y, z.
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(0U, &PDP::PTRREGSRegClass);
      break;
    case 'q': // Stack pointer register: SPH:SPL.
      return std::make_pair(0U, &PDP::GPRSPRegClass);*/
    case 'a': return std::make_pair(unsigned(PDP::R0), &PDP::GPR16RegClass);
    case 'b': return std::make_pair(unsigned(PDP::R1), &PDP::GPR16RegClass);
    case 'c': return std::make_pair(unsigned(PDP::R2), &PDP::GPR16RegClass);
    case 'd': return std::make_pair(unsigned(PDP::R3), &PDP::GPR16RegClass);
    case 'e': return std::make_pair(unsigned(PDP::R4), &PDP::GPR16RegClass);
    case 'f': return std::make_pair(unsigned(PDP::R5), &PDP::GPR16RegClass);
    case 'r': // Any register: r0..r31.
      /*if (VT == MVT::i8)
        return std::make_pair(0U, &PDP::GPR8RegClass);
      else*/ if (VT == MVT::i16)
        return std::make_pair(0U, &PDP::GPR16RegClass);
      break;
    /*case 't': // Temporary register: r0.
      if (VT == MVT::i8)
        return std::make_pair(unsigned(Subtarget.getTmpRegister()),
                              &PDP::GPR8RegClass);
      break;
    case 'w': // Special upper register pairs: r24, r26, r28, r30.
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(0U, &PDP::IWREGSRegClass);
      break;
    case 'x': // Pointer register pair X: r27:r26.
    case 'X':
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(unsigned(PDP::R27R26), &PDP::PTRREGSRegClass);
      break;
    case 'y': // Pointer register pair Y: r29:r28.
    case 'Y':
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(unsigned(PDP::R29R28), &PDP::PTRREGSRegClass);
      break;
    case 'z': // Pointer register pair Z: r31:r30.
    case 'Z':
      if (VT == MVT::i8 || VT == MVT::i16)
        return std::make_pair(unsigned(PDP::R31R30), &PDP::PTRREGSRegClass);
      break;*/
    default:
      break;
    }
  }

  return TargetLowering::getRegForInlineAsmConstraint(
      Subtarget.getRegisterInfo(), Constraint, VT);
}

void PDPTargetLowering::LowerAsmOperandForConstraint(SDValue Op,
                                                     StringRef Constraint,
                                                     std::vector<SDValue> &Ops,
                                                     SelectionDAG &DAG) const {
  SDValue Result;
  SDLoc DL(Op);
  EVT Ty = Op.getValueType();

  // Currently only support length 1 constraints.
  if (Constraint.size() != 1) {
    return;
  }

  char ConstraintLetter = Constraint[0];
  switch (ConstraintLetter) {
  default:
    break;
  // Deal with integers first:
  case 'I':
  case 'J':
  case 'K':
  case 'L':
  case 'M':
  case 'N':
  case 'O':
  case 'P':
  case 'R': {
    const ConstantSDNode *C = dyn_cast<ConstantSDNode>(Op);
    if (!C) {
      return;
    }

    int64_t CVal64 = C->getSExtValue();
    uint64_t CUVal64 = C->getZExtValue();
    switch (ConstraintLetter) {
    case 'I': // 0..63
      if (!isUInt<6>(CUVal64))
        return;
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'J': // -63..0
      if (CVal64 < -63 || CVal64 > 0)
        return;
      Result = DAG.getTargetConstant(CVal64, DL, Ty);
      break;
    case 'K': // 2
      if (CUVal64 != 2)
        return;
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'L': // 0
      if (CUVal64 != 0)
        return;
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'M': // 0..255
      if (!isUInt<8>(CUVal64))
        return;
      // i8 type may be printed as a negative number,
      // e.g. 254 would be printed as -2,
      // so we force it to i16 at least.
      if (Ty.getSimpleVT() == MVT::i8) {
        Ty = MVT::i16;
      }
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'N': // -1
      if (CVal64 != -1)
        return;
      Result = DAG.getTargetConstant(CVal64, DL, Ty);
      break;
    case 'O': // 8, 16, 24
      if (CUVal64 != 8 && CUVal64 != 16 && CUVal64 != 24)
        return;
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'P': // 1
      if (CUVal64 != 1)
        return;
      Result = DAG.getTargetConstant(CUVal64, DL, Ty);
      break;
    case 'R': // -6..5
      if (CVal64 < -6 || CVal64 > 5)
        return;
      Result = DAG.getTargetConstant(CVal64, DL, Ty);
      break;
    }

    break;
  }
  case 'G':
    const ConstantFPSDNode *FC = dyn_cast<ConstantFPSDNode>(Op);
    if (!FC || !FC->isZero())
      return;
    // Soften float to i8 0
    Result = DAG.getTargetConstant(0, DL, MVT::i8);
    break;
  }

  if (Result.getNode()) {
    Ops.push_back(Result);
    return;
  }

  return TargetLowering::LowerAsmOperandForConstraint(Op, Constraint, Ops, DAG);
}

Register PDPTargetLowering::getRegisterByName(const char *RegName, LLT VT,
                                              const MachineFunction &MF) const {
  llvm_unreachable("PDPTargetLowering::getRegisterByName");

  /*Register Reg;

  if (VT == LLT::scalar(8)) {
    Reg = StringSwitch<unsigned>(RegName)
              .Case("r0", PDP::R0)
              .Case("r1", PDP::R1)
              .Default(0);
  } else {
    Reg = StringSwitch<unsigned>(RegName)
              .Case("r0", PDP::R1R0)
              .Case("sp", PDP::SP)
              .Default(0);
  }

  if (Reg)
    return Reg;

  report_fatal_error(
      Twine("Invalid register name \"" + StringRef(RegName) + "\"."));*/
}
bool PDPTargetLowering::splitValueIntoRegisterParts(
    SelectionDAG &DAG, const SDLoc &DL, SDValue Val, SDValue *Parts,
    unsigned int NumParts, MVT PartVT,
    std::optional<CallingConv::ID> CC) const {

  return false;

  Val.dumpr(&DAG);

  auto OpCode = Val.getOpcode();

  if (OpCode == ISD::SIGN_EXTEND && (PartVT == MVT::i16 || PartVT == MVT::i8) &&
      Val.getValueType() == MVT::i32 && NumParts == 2) {
    auto &Src = Val.getOperand(0);

    auto *Sxt =
        DAG.getNode(PDPISD::SXT, DL, {MVT::i16, MVT::i16}, Src).getNode();

    Parts[0] = Src;
    Parts[1] = SDValue(Sxt, 1);

    return true;
  }

  return TargetLowering::splitValueIntoRegisterParts(DAG, DL, Val, Parts,
                                                     NumParts, PartVT, CC);
}

SDValue PDPTargetLowering::joinRegisterPartsIntoValue(
    SelectionDAG &DAG, const SDLoc &DL, const SDValue *Parts,
    unsigned int NumParts, MVT PartVT, EVT ValueVT,
    std::optional<CallingConv::ID> CC) const {

  return TargetLowering::joinRegisterPartsIntoValue(DAG, DL, Parts, NumParts,
                                                    PartVT, ValueVT, CC);
}

} // end of namespace llvm
