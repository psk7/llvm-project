#ifndef LLVM_PDP11_PDP_H
#define LLVM_PDP11_PDP_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/Support/Compiler.h"
#include "llvm/TargetParser/Triple.h"

namespace clang {
namespace targets {

class LLVM_LIBRARY_VISIBILITY PDPTargetInfo : public TargetInfo {
public:
  PDPTargetInfo(const llvm::Triple &Triple, const TargetOptions &)
      : TargetInfo(Triple) {
    TLSSupported = false;
    PointerWidth = 16;
    PointerAlign = 16;
    ShortWidth = 16;
    ShortAlign = 16;
    IntWidth = 16;
    IntAlign = 16;
    LongWidth = 32;
    LongAlign = 16;
    LongLongWidth = 64;
    LongLongAlign = 16;
    SuitableAlign = 16;
    DefaultAlignForAttributeAligned = 16;
    HalfWidth = 16;
    HalfAlign = 16;
    FloatWidth = 32;
    FloatAlign = 16;
    DoubleWidth = 32;
    DoubleAlign = 16;
    DoubleFormat = &llvm::APFloat::IEEEsingle();
    LongDoubleWidth = 32;
    LongDoubleAlign = 16;
    LongDoubleFormat = &llvm::APFloat::IEEEsingle();
    SizeType = UnsignedInt;
    PtrDiffType = SignedInt;
    IntPtrType = SignedInt;
    Char16Type = UnsignedInt;
    WIntType = SignedInt;
    Int16Type = SignedInt;
    Char32Type = UnsignedLong;
    SigAtomicType = SignedChar;
    resetDataLayout("e-p:16:16-i8:8-i16:16-i32:16-i64:16-n8:16");
  }

  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override;

  ArrayRef<Builtin::Info> getTargetBuiltins() const override;

  bool allowsLargerPreferedTypeAlignment() const override { return false; }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  std::string_view getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override {
    static const char *const GCCRegNames[] = {
        "R0",  "R1",  "R2",  "R3",  "R4",  "R5",  "R6",  "R7" };
    return llvm::ArrayRef(GCCRegNames);
  }

  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override {
    return {};
  }

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override {
    // There aren't any multi-character PDP specific constraints.
    if (StringRef(Name).size() > 1)
      return false;

    switch (*Name) {
    default:
      return false;
    case 'a':   // R0
    case 'b':   // R1
    case 'c':   // R2
    case 'd':   // R3
    case 'e':   // R4
    case 'f':   // R5
    case 'r':   // GPR16
      Info.setAllowsRegister();
      return true;
    }

    return false;
  }

  IntType getIntTypeByWidth(unsigned BitWidth, bool IsSigned) const final {
    // PDP prefers int for 16-bit integers.
    return BitWidth == 16 ? (IsSigned ? SignedInt : UnsignedInt)
                          : TargetInfo::getIntTypeByWidth(BitWidth, IsSigned);
  }

  IntType getLeastIntTypeByWidth(unsigned BitWidth, bool IsSigned) const final {
    // PDP uses int for int_least16_t and int_fast16_t.
    return BitWidth == 16
               ? (IsSigned ? SignedInt : UnsignedInt)
               : TargetInfo::getLeastIntTypeByWidth(BitWidth, IsSigned);
  }

  bool isValidCPUName(StringRef Name) const override;
  void fillValidCPUList(SmallVectorImpl<StringRef> &Values) const override;
  bool setCPU(const std::string &Name) override;
  StringRef getABI() const override { return ABI; }

  std::pair<unsigned, unsigned> hardwareInterferenceSizes() const override {
    return std::make_pair(2, 2);
  }

protected:
  std::string CPU;
  StringRef ABI;
  StringRef DefineName;
};

} // namespace targets
} // namespace clang

#endif //LLVM_PDP11_PDP_H