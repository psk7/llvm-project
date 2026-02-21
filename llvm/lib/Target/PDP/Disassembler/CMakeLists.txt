add_llvm_component_library(LLVMPDPDisassembler
  PDPDisassembler.cpp

  LINK_COMPONENTS
  PDPDesc
  PDPInfo
  CodeGenTypes
  MC
  MCDisassembler
  Support

  ADD_TO_COMPONENT
  PDP
  )

