add_llvm_component_library(LLVMZ80Disassembler
  Z80Disassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  Z80Info
  Support

  ADD_TO_COMPONENT
  Z80
)

