#ifndef LLVM_PDP11_PDP_H
#define LLVM_PDP11_PDP_H

#include "Gnu.h"
#include "clang/Driver/InputInfo.h"
#include "clang/Driver/Tool.h"
#include "clang/Driver/ToolChain.h"

namespace clang {
namespace driver {
namespace toolchains {

class LLVM_LIBRARY_VISIBILITY PDPToolChain : public Generic_ELF {
public:
  PDPToolChain(const Driver &D, const llvm::Triple &Triple,
               const llvm::opt::ArgList &Args);
  void
  AddClangSystemIncludeArgs(const llvm::opt::ArgList &DriverArgs,
                            llvm::opt::ArgStringList &CC1Args) const override;

  void
  addClangTargetOptions(const llvm::opt::ArgList &DriverArgs,
                        llvm::opt::ArgStringList &CC1Args,
                        Action::OffloadKind DeviceOffloadKind) const override;

  std::string getCompilerRT(const llvm::opt::ArgList &Args, StringRef Component,
                            FileType Type) const override;

  bool HasNativeLLVMSupport() const override { return true; }

protected:
  Tool *buildLinker() const override;
};

} // end namespace toolchains

namespace tools {
namespace PDP {
class LLVM_LIBRARY_VISIBILITY Linker final : public Tool {
public:
  Linker(const llvm::Triple &Triple, const ToolChain &TC)
      : Tool("PDP::Linker", "pdp11-ld", TC), Triple(Triple) {}

  bool hasIntegratedCPP() const override { return false; }
  bool isLinkJob() const override { return true; }
  void ConstructJob(Compilation &C, const JobAction &JA,
                    const InputInfo &Output, const InputInfoList &Inputs,
                    const llvm::opt::ArgList &TCArgs,
                    const char *LinkingOutput) const override;

protected:
  const llvm::Triple &Triple;
};
} // end namespace PDP
} // end namespace tools
} // end namespace driver
} // end namespace clang
#endif //LLVM_PDP11_PDP_H