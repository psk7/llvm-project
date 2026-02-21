//===-- PDPFixupKinds.h - PDP Specific Fixup Entries ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_PDP_FIXUP_KINDS_H
#define LLVM_PDP_FIXUP_KINDS_H

#include "llvm/MC/MCFixup.h"

namespace llvm {
namespace PDP {

/// The set of supported fixups.
///
/// Although most of the current fixup types reflect a unique relocation
/// one can have multiple fixup types for a given relocation and thus need
/// to be uniquely named.
///
/// \note This table *must* be in the same order of
///       MCFixupKindInfo Infos[PDP::NumTargetFixupKinds]
///       in `PDPAsmBackend.cpp`.
enum Fixups {
  fixup_16 = FirstTargetFixupKind,

  fixup_sob_pcrel,
  fixup_6_pcrel,
  fixup_8_pcrel,
  fixup_16_pcrel,

  // Marker
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};

namespace fixups {

/// Adjusts the value of a branch target.
/// All branch targets in PDP are rightshifted by 1 to take advantage
/// of the fact that all instructions are aligned to addresses of size
/// 2, so bit 0 of an address is always 0. This gives us another bit
/// of precision.
/// \param [in,out] val The target to adjust.
template <typename T> inline void adjustBranchTarget(T &val) { val >>= 1; }

} // end of namespace fixups
} // namespace PDP
} // namespace llvm

#endif // LLVM_PDP_FIXUP_KINDS_H
