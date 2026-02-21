add_llvm_component_library(LLVMPDPDesc
  PDPAsmBackend.cpp
  PDPELFObjectWriter.cpp
  PDPELFStreamer.cpp
  PDPInstPrinter.cpp
  PDPMCAsmInfo.cpp
  PDPMCCodeEmitter.cpp
  PDPMCELFStreamer.cpp
  PDPMCExpr.cpp
  PDPMCTargetDesc.cpp
  PDPTargetStreamer.cpp

  LINK_COMPONENTS
  MC
  PDPInfo
  Support
  TargetParser

  ADD_TO_COMPONENT
  PDP
)

add_dependencies(LLVMPDPDesc PDPCommonTableGen)

