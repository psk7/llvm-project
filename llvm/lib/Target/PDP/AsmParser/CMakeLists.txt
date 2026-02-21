add_llvm_component_library(LLVMPDPAsmParser
  PDPAsmParser.cpp

  LINK_COMPONENTS
  PDPDesc
  PDPInfo
  CodeGenTypes
  MC
  MCParser
  Support

  ADD_TO_COMPONENT
  PDP
  )
