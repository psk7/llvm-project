//===-- Z80SelectionDAGInfo.cpp - Z80 SelectionDAG Info -------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the Z80SelectionDAGInfo class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "Z80TargetMachine.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/IR/DerivedTypes.h"
using namespace llvm;

#define DEBUG_TYPE "z80-selectiondag-info"

SDValue Z80SelectionDAGInfo::EmitTargetCodeForMemset(
    SelectionDAG &DAG, const SDLoc &dl, SDValue Chain, SDValue Dst, SDValue Src,
    SDValue Size, Align Alignment, bool isVolatile,
    MachinePointerInfo DstPtrInfo) const {

  Dst = DAG.getNode(ISD::ADD, dl, MVT::i16, Dst,
                    DAG.getConstant(0, dl, MVT::i16));

  Chain = DAG.getStore(Chain, dl, Src, Dst, DstPtrInfo, Alignment);

  auto Dest = DAG.getNode(ISD::ADD, dl, MVT::i16, Dst,
                          DAG.getConstant(1, dl, MVT::i16));

  if (Size.getValueType() != MVT::i16)
    Size = DAG.getZExtOrTrunc(Size, dl, MVT::i16);

  Size = DAG.getNode(ISD::SUB, dl, MVT::i16, Size,
                     DAG.getConstant(1, dl, MVT::i16));

  SDNode *n =
      DAG.getMachineNode(Z80::LDIR, dl, MVT::Other, {Dst, Dest, Size, Chain});

  return SDValue(n, 0);
}

SDValue Z80SelectionDAGInfo::EmitTargetCodeForMemcpy(
    SelectionDAG &DAG, const SDLoc &dl, SDValue Chain, SDValue Op1, SDValue Op2,
    SDValue Op3, Align Alignment, bool isVolatile, bool AlwaysInline,
    MachinePointerInfo DstPtrInfo, MachinePointerInfo SrcPtrInfo) const {

  SDNode *n =
      DAG.getMachineNode(Z80::LDIR, dl, MVT::Other, {Op2, Op1, Op3, Chain});

  return SDValue(n, 0);
}
