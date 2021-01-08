add_llvm_component_library(LLVMZ80Desc
        Z80AsmBackend.cpp
        Z80ELFObjectWriter.cpp
        Z80ELFStreamer.cpp
        Z80InstPrinter.cpp
        Z80MCAsmInfo.cpp
        Z80MCCodeEmitter.cpp
        Z80MCELFStreamer.cpp
        Z80MCExpr.cpp
        Z80MCTargetDesc.cpp
        Z80TargetStreamer.cpp
        )

add_dependencies(LLVMZ80Desc Z80CommonTableGen)

