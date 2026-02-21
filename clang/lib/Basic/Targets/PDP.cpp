#include "PDP.h"
#include "clang/Basic/Builtins.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetBuiltins.h"
#include "llvm/ADT/StringSwitch.h"

using namespace clang;
using namespace clang::targets;

namespace clang {
namespace targets {

static constexpr Builtin::Info BuiltinInfo[] = {
#define BUILTIN(ID, TYPE, ATTRS)                                               \
  {#ID, TYPE, ATTRS, nullptr, HeaderDesc::NO_HEADER, ALL_LANGUAGES},
#define LIBBUILTIN(ID, TYPE, ATTRS, HEADER)                                    \
  {#ID, TYPE, ATTRS, nullptr, HeaderDesc::HEADER, ALL_LANGUAGES},
#include "clang/Basic/BuiltinsPDP11.def"

};

/// Information about a specific microcontroller.
struct LLVM_LIBRARY_VISIBILITY CPUInfo {
  const char *Name;
  const char *DefineName;
};

static CPUInfo PDPCpus[] = {
    {"vm1a", "__1801_VM1A__"},
    {"vm1g", "__1801_VM1G__"},
};

} // namespace targets
} // namespace clang

bool PDPTargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::any_of(PDPCpus,
                      [&](const CPUInfo &Info) { return Info.Name == Name; });
}

void PDPTargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  for (const CPUInfo &Info : PDPCpus)
    Values.push_back(Info.Name);
}

bool PDPTargetInfo::setCPU(const std::string &Name) {
  // Set the ABI field based on the device or family name.
  auto It = llvm::find_if(
      PDPCpus, [&](const CPUInfo &Info) { return Info.Name == Name; });

  if (It != std::end(PDPCpus)) {
    CPU = Name;
    ABI = "none";
    DefineName = It->DefineName;
    return true;
  }

  // Parameter Name is neither valid family name nor valid device name.
  return false;
}

void PDPTargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("PDP11");
  Builder.defineMacro("__PDP11");
  Builder.defineMacro("__PDP11__");

  if (DefineName.size() != 0)
       Builder.defineMacro(DefineName);

  switch (getTriple().getOS()) {
  case llvm::Triple::Mon10:
    Builder.defineMacro("__BK_MON_10__");
    break;
  case llvm::Triple::Mon11M:
    Builder.defineMacro("__BK_MON_11M__");
    break;
  case llvm::Triple::Rt11:
    Builder.defineMacro("__OS_RT_11__");
    break;
  default:
    break;
  }
}

ArrayRef<Builtin::Info> PDPTargetInfo::getTargetBuiltins() const {
  return llvm::ArrayRef(BuiltinInfo,
                        clang::PDP::LastTSBuiltin - Builtin::FirstTSBuiltin);
}
