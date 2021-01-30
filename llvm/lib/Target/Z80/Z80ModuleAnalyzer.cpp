//===- Z80ModuleAnalyzer.cpp ----------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Z80.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80RegisterInfo.h"
#include "Z80TargetMachine.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "MCTargetDesc/Z80MCCodeEmitter.h"

using namespace llvm;

#define DEBUG_TYPE "z80-module-analyzer"

#define MODULE_ANALYZER_NAME "Z80 Module analyzer pass"

namespace {

class Z80ModuleAnalyzer : public ModulePass {

public:
  static char ID;

  Z80ModuleAnalyzer() : ModulePass(ID) {
    initializeZ80ModuleAnalyzerPass(*PassRegistry::getPassRegistry());
  }

  virtual bool runOnModule(Module &M) override {
    auto mn = M.getName().str();
    std::replace(mn.begin(), mn.end(), '.', '_');

    auto r = false;

    for (GlobalObject &GO : M.global_objects()) {
      if (isa<GlobalVariable>(GO)) {
        auto n = GO.getName();

        if (n.contains('.')) {
          auto nn = ("_" + mn + "_" + n).str();
          std::replace(nn.begin(), nn.end(), '.', '_');
          std::replace(nn.begin(), nn.end(), '/', '_');
          GO.setName(nn);
          r = true;
        }
      }
    }

    return r;
  }

  StringRef getPassName() const override { return MODULE_ANALYZER_NAME; }
};

} // end anonymous namespace

char Z80ModuleAnalyzer::ID = 0;

INITIALIZE_PASS(Z80ModuleAnalyzer, DEBUG_TYPE, MODULE_ANALYZER_NAME, false, true)

namespace llvm {
ModulePass *createZ80ModuleAnalyzerPass() {
  return new Z80ModuleAnalyzer();
}
} // namespace llvm

