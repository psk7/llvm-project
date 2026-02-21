add_llvm_component_group(PDP)

set(LLVM_TARGET_DEFINITIONS PDP.td)

tablegen(LLVM PDPGenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM PDPGenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM PDPGenCallingConv.inc -gen-callingconv)
tablegen(LLVM PDPGenDAGISel.inc -gen-dag-isel)
tablegen(LLVM PDPGenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM PDPGenInstrInfo.inc -gen-instr-info)
tablegen(LLVM PDPGenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM PDPGenRegisterInfo.inc -gen-register-info)
tablegen(LLVM PDPGenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(PDPCommonTableGen)

add_llvm_target(PDPCodeGen
  PDPAsmPrinter.cpp
  PDPExpandPseudoInsts.cpp
  PDPOptimizeInstructions.cpp
  PDPFrameLowering.cpp
  PDPInstrInfo.cpp
  PDPISelDAGToDAG.cpp
  PDPISelLowering.cpp
  PDPMCInstLower.cpp
  PDPRegisterInfo.cpp
  PDPShiftExpand.cpp
  PDPSubtarget.cpp
  PDPTargetMachine.cpp
  PDPTargetObjectFile.cpp

  DEPENDS
  intrinsics_gen

  LINK_COMPONENTS
  PDPDesc
  PDPInfo
  AsmPrinter
  CodeGen
  CodeGenTypes
  Core
  MC
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  PDP
  )

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
