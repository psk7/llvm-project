#include "PDP.h"
#include "CommonArgs.h"
#include "clang/Driver/Compilation.h"
#include "clang/Driver/DriverDiagnostic.h"
#include "clang/Driver/InputInfo.h"
#include "clang/Driver/Options.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/Path.h"
#include "llvm/TargetParser/SubtargetFeature.h"

using namespace clang::driver;
using namespace clang::driver::toolchains;
using namespace clang::driver::tools;
using namespace clang;
using namespace llvm::opt;

PDPToolChain::PDPToolChain(const Driver &D, const llvm::Triple &Triple,
                           const ArgList &Args)
    : Generic_ELF(D, Triple, Args) {}

void PDPToolChain::AddClangSystemIncludeArgs(const ArgList &DriverArgs,
                                             ArgStringList &CC1Args) const {
  if (DriverArgs.hasArg(options::OPT_nostdinc) ||
      DriverArgs.hasArg(options::OPT_nostdlibinc))
    return;
}

void PDPToolChain::addClangTargetOptions(
    const llvm::opt::ArgList &DriverArgs, llvm::opt::ArgStringList &CC1Args,
    Action::OffloadKind DeviceOffloadKind) const {

  const Driver &D = getDriver();
  std::string CPU = getCPUName(D, DriverArgs, getTriple());

  // By default, use `.ctors` (not `.init_array`), as required by libgcc, which
  // runs constructors/destructors on PDP.
  if (!DriverArgs.hasFlag(options::OPT_fuse_init_array,
                          options::OPT_fno_use_init_array, false))
    CC1Args.push_back("-fno-use-init-array");

  // Use `-fno-use-cxa-atexit` as default
  if (!DriverArgs.hasFlag(options::OPT_fuse_cxa_atexit,
                          options::OPT_fno_use_cxa_atexit, false))
    CC1Args.push_back("-fno-use-cxa-atexit");
}

Tool *PDPToolChain::buildLinker() const {
  return new tools::PDP::Linker(getTriple(), *this);
}

std::string
PDPToolChain::getCompilerRT(const llvm::opt::ArgList &Args, StringRef Component,
                            FileType Type = ToolChain::FT_Static) const {
  assert(Type == ToolChain::FT_Static && "PDP only supports static libraries");

  return "";
}

static std::string MakePath(const std::string &Root,
                            const ArrayRef<std::string> Comps) {
  SmallString<256> Path(Root);

  for (auto &comp : Comps)
    llvm::sys::path::append(Path, comp);

  return std::string(Path);
}

void PDP::Linker::ConstructJob(Compilation &C, const JobAction &JA,
                               const InputInfo &Output,
                               const InputInfoList &Inputs, const ArgList &Args,
                               const char *LinkingOutput) const {
  const auto &TC = static_cast<const PDPToolChain &>(getToolChain());
  const Driver &D = getToolChain().getDriver();
  const auto &TT = TC.getTriple();
  const auto TOS = TT.getOS();

  std::string CPU = getCPUName(D, Args, getToolChain().getTriple());

  const Arg *A = Args.getLastArg(options::OPT_fuse_ld_EQ);
  std::string Linker = A ? getToolChain().GetLinkerPath(nullptr)
                         : getToolChain().GetProgramPath(getShortName());

  ArgStringList CmdArgs;

  CmdArgs.push_back("-flavor");
  CmdArgs.push_back("gnu");

  CmdArgs.push_back("-L");
  CmdArgs.push_back(Args.MakeArgString(MakePath(D.Dir, {"..", "lib"})));

  if (TOS == llvm::Triple::Mon10) {
    CmdArgs.push_back("-T");
    CmdArgs.push_back(
        Args.MakeArgString(MakePath(D.Dir, {"..", "lib", "mon10.ld"})));
  }

  if (TOS == llvm::Triple::Mon10) {
    CmdArgs.push_back("-lc");
    CmdArgs.push_back(
        Args.MakeArgString(MakePath(D.Dir, {"..", "lib", "start_mon10.o"})));
  }

  CmdArgs.push_back("-lrt");

  CmdArgs.push_back("-o");
  CmdArgs.push_back(Output.getFilename());

  if (!Args.hasArg(options::OPT_r))
    CmdArgs.push_back("--gc-sections");

  // Add library search paths before we specify libraries.
  Args.AddAllArgs(CmdArgs, options::OPT_L);
  TC.AddFilePathLibArgs(Args, CmdArgs);

  // Currently we only support libgcc and compiler-rt.
  auto RtLib = TC.GetRuntimeLibType(Args);
  assert(
      (RtLib == ToolChain::RLT_Libgcc || RtLib == ToolChain::RLT_CompilerRT) &&
      "unknown runtime library");

  if (D.isUsingLTO()) {
    assert(!Inputs.empty() && "Must have at least one input.");
    // Find the first filename InputInfo object.
    auto Input = llvm::find_if(
        Inputs, [](const InputInfo &II) -> bool { return II.isFilename(); });

    if (Input == Inputs.end())
      // For a very rare case, all of the inputs to the linker are
      // InputArg. If that happens, just use the first InputInfo.
      Input = Inputs.begin();

    addLTOOptions(TC, Args, CmdArgs, Output, *Input,
                  D.getLTOMode() == LTOK_Thin);
  }

  AddLinkerInputs(getToolChain(), Inputs, Args, CmdArgs, JA);

  C.addCommand(std::make_unique<Command>(
      JA, *this, ResponseFileSupport::AtFileCurCP(), Args.MakeArgString(Linker),
      CmdArgs, Inputs, Output));
}
