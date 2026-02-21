//===- IntrinsicsPDP11.td - Defines PDP11 intrinsics ---------*- tablegen -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines all of the PDP11-specific intrinsics.
//
//===----------------------------------------------------------------------===//

let TargetPrefix = "pdp11" in {  // All intrinsics start with "llvm.pdp11.".

//def int_pdp11_emt : ClangBuiltin<"__builtin_pdp11_emt">, Intrinsic<[], [llvm_i16_ty], [ImmArg<ArgIndex<0>>]>;
//def int_pdp11_trap : ClangBuiltin<"__builtin_pdp11_trap">, Intrinsic<[], [llvm_i16_ty], [ImmArg<ArgIndex<0>>]>;

def int_pdp11_read_psw : ClangBuiltin<"__builtin_pdp11_read_psw">, Intrinsic<[llvm_i16_ty], [], []>;
def int_pdp11_write_psw : ClangBuiltin<"__builtin_pdp11_write_psw">, Intrinsic<[], [llvm_i16_ty], []>;
}
