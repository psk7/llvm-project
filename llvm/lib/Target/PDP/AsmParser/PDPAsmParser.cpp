//===---- PDPAsmParser.cpp - Parse PDP assembly to MCInst instructions ----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "PDPRegisterInfo.h"
#include "MCTargetDesc/PDPMCELFStreamer.h"
#include "MCTargetDesc/PDPMCExpr.h"
#include "MCTargetDesc/PDPMCTargetDesc.h"
#include "TargetInfo/PDPTargetInfo.h"

#include "llvm/ADT/APInt.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"

#include <array>
#include <sstream>

#define DEBUG_TYPE "PDP-asm-parser"

using namespace llvm;

namespace {
/// Parses PDP assembly from a stream.
class PDPAsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;
  const std::string GENERATE_STUBS = "gs";

  enum PDPMatchResultTy {
    Match_InvalidRegisterOnTiny = FIRST_TARGET_MATCH_RESULT_TY + 1,
  };

#define GET_ASSEMBLER_HEADER
#include "PDPGenAsmMatcher.inc"

  bool matchAndEmitInstruction(SMLoc IDLoc, unsigned &Opcode,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override;

  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override;
  ParseStatus tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                               SMLoc &EndLoc) override;

  bool parseInstruction(ParseInstructionInfo &Info, StringRef Name,
                        SMLoc NameLoc, OperandVector &Operands) override;

  ParseStatus parseDirective(AsmToken DirectiveID) override;

  ParseStatus parseUniopOperand(OperandVector &Operands);

  bool parseOperand(OperandVector &Operands, bool maybeReg);
  MCRegister parseRegisterName(MCRegister (*matchFn)(StringRef));
  MCRegister parseRegisterName();
  MCRegister parseRegister(bool RestoreOnFailure = false);
  bool tryParseRegisterOperand(OperandVector &Operands);
  bool tryParseExpression(OperandVector &Operands, int64_t offset);
  bool tryParseDotOffset(OperandVector &Operands);
  bool tryParseRelocExpression(OperandVector &Operands);
  void eatComma();

  unsigned validateTargetOperandClass(MCParsedAsmOperand &Op,
                                      unsigned Kind) override;

  /*
  MCRegister toDREG(MCRegister Reg, unsigned From = PDP::sub_lo) {
    MCRegisterClass const *Class = &PDPMCRegisterClasses[PDP::DREGSRegClassID];
    return MRI->getMatchingSuperReg(Reg, From, Class);
  }
  */

  bool emit(MCInst &Instruction, SMLoc const &Loc, MCStreamer &Out) const;
  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo);
  bool missingFeature(SMLoc const &Loc, uint64_t const &ErrorInfo);

  ParseStatus parseLiteralValues(unsigned SizeInBytes, SMLoc L);

public:
  PDPAsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
               const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }

  MCAsmParser &getParser() const { return Parser; }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
};

/// An parsed PDP assembly operand.
class PDPOperand : public MCParsedAsmOperand {
  typedef MCParsedAsmOperand Base;
  enum KindTy { k_Immediate, k_Register, k_Token, k_Uniop } Kind;

public:
  PDPOperand(StringRef Tok, SMLoc const &S)
      : Kind(k_Token), Tok(Tok), Start(S), End(S) {}
  PDPOperand(MCRegister Reg, SMLoc const &S, SMLoc const &E)
      : Kind(k_Register), RegImm({Reg, nullptr}), Start(S), End(E) {}
  PDPOperand(MCExpr const *Imm, SMLoc const &S, SMLoc const &E)
      : Kind(k_Immediate), RegImm({0, Imm}), Start(S), End(E) {}
  PDPOperand(MCRegister Reg, MCExpr const *Imm, SMLoc const &S, int Mode)
      : Kind(k_Uniop), RegImm({Reg, Imm, Mode}), Start(S), End(S) {}

  struct RegisterImmediate {
    MCRegister Reg;
    MCExpr const *Imm;
    int Mode;
  };

  union {
    StringRef Tok;
    RegisterImmediate RegImm;
  };

  SMLoc Start, End;

public:
  void addRegOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Register && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addExpr(MCInst &Inst, const MCExpr *Expr) const {
    // Add as immediate when possible
    if (!Expr)
      Inst.addOperand(MCOperand::createImm(0));
    else if (const MCConstantExpr *CE = dyn_cast<MCConstantExpr>(Expr))
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    else
      Inst.addOperand(MCOperand::createExpr(Expr));
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Immediate && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addUniopOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Uniop && "Unexpected operand kind");
    assert(N == 4 && "Invalid number of operands");

    int64_t Res = 0;

    Inst.addOperand(MCOperand::createReg(RegImm.Reg));
    Inst.addOperand(MCOperand::createImm(RegImm.Mode));

    if (RegImm.Imm == nullptr || RegImm.Imm->evaluateAsAbsolute(Res))
      Inst.addOperand(MCOperand::createImm(Res));
    else
      Inst.addOperand(MCOperand::createExpr(RegImm.Imm));

    Inst.addOperand(MCOperand::createImm(0));
  }

  void addImmCom8Operands(MCInst &Inst, unsigned N) const {
    assert(N == 1 && "Invalid number of operands!");
    // The operand is actually a imm8, but we have its bitwise
    // negation in the assembly source, so twiddle it here.
    const auto *CE = cast<MCConstantExpr>(getImm());
    Inst.addOperand(MCOperand::createImm(~(uint8_t)CE->getValue()));
  }

  bool isImmCom8() const {
    if (!isImm())
      return false;
    const auto *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE)
      return false;
    int64_t Value = CE->getValue();
    return isUInt<8>(Value);
  }

  bool isReg() const override { return Kind == k_Register; }
  bool isImm() const override { return Kind == k_Immediate; }
  bool isToken() const override { return Kind == k_Token; }
  bool isUniop() const { return Kind == k_Uniop; }
  bool isMem() const override { return false; }

  StringRef getToken() const {
    assert(Kind == k_Token && "Invalid access!");
    return Tok;
  }

  MCRegister getReg() const override {
    assert((Kind == k_Register || Kind == k_Uniop) && "Invalid access!");

    return RegImm.Reg;
  }

  const MCExpr *getImm() const {
    assert((Kind == k_Immediate) && "Invalid access!");
    return RegImm.Imm;
  }

  static std::unique_ptr<PDPOperand> CreateToken(StringRef Str, SMLoc S) {
    return std::make_unique<PDPOperand>(Str, S);
  }

  static std::unique_ptr<PDPOperand> CreateReg(MCRegister Reg, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<PDPOperand>(Reg, S, E);
  }

  static std::unique_ptr<PDPOperand> CreateImm(const MCExpr *Val, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<PDPOperand>(Val, S, E);
  }

  static std::unique_ptr<PDPOperand>
  CreateUniop(MCRegister Reg, SMLoc S, int Mode, const MCExpr *Val) {
    return std::make_unique<PDPOperand>(Reg, Val, S, Mode);
  }

  void makeToken(StringRef Token) {
    Kind = k_Token;
    Tok = Token;
  }

  void makeReg(MCRegister Reg) {
    Kind = k_Register;
    RegImm = {Reg, nullptr};
  }

  void makeImm(MCExpr const *Ex) {
    Kind = k_Immediate;
    RegImm = {0, Ex};
  }

  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }

  void print(raw_ostream &O) const override {
    switch (Kind) {
    case k_Token:
      O << "Token: \"" << getToken() << "\"";
      break;
    case k_Register:
      O << "Register: " << getReg();
      break;
    case k_Immediate:
      O << "Immediate: \"" << *getImm() << "\"";
      break;
    case k_Uniop: {
      // only manually print the size for non-negative values,
      // as the sign is inserted automatically.
      O << "Uniop: \"" << getReg() << "\"";
      break;
    }
    }
    O << "\n";
  }
};

} // end anonymous namespace.

// Auto-generated Match Functions

/// Maps from the set of all register names to a register number.
/// \note Generated by TableGen.
static MCRegister MatchRegisterName(StringRef Name);

/// Maps from the set of all alternative registernames to a register number.
/// \note Generated by TableGen.
static MCRegister MatchRegisterAltName(StringRef Name);

bool PDPAsmParser::invalidOperand(SMLoc const &Loc,
                                  OperandVector const &Operands,
                                  uint64_t const &ErrorInfo) {
  SMLoc ErrorLoc = Loc;
  char const *Diag = nullptr;

  if (ErrorInfo != ~0U) {
    if (ErrorInfo >= Operands.size()) {
      Diag = "too few operands for instruction.";
    } else {
      PDPOperand const &Op = (PDPOperand const &)*Operands[ErrorInfo];

      // TODO: See if we can do a better error than just "invalid ...".
      if (Op.getStartLoc() != SMLoc()) {
        ErrorLoc = Op.getStartLoc();
      }
    }
  }

  if (!Diag) {
    Diag = "invalid operand for instruction";
  }

  return Error(ErrorLoc, Diag);
}

bool PDPAsmParser::missingFeature(llvm::SMLoc const &Loc,
                                  uint64_t const &ErrorInfo) {
  return Error(Loc, "instruction requires a CPU feature not currently enabled");
}

bool PDPAsmParser::emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
  Inst.setLoc(Loc);
  Out.emitInstruction(Inst, STI);

  return false;
}

bool PDPAsmParser::matchAndEmitInstruction(SMLoc Loc, unsigned &Opcode,
                                           OperandVector &Operands,
                                           MCStreamer &Out, uint64_t &ErrorInfo,
                                           bool MatchingInlineAsm) {
  MCInst Inst;
  unsigned MatchResult =
      MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);

  switch (MatchResult) {
  case Match_Success:
    return emit(Inst, Loc, Out);
  case Match_MissingFeature:
    return missingFeature(Loc, ErrorInfo);
  case Match_InvalidOperand:
    return invalidOperand(Loc, Operands, ErrorInfo);
  case Match_MnemonicFail:
    return Error(Loc, "invalid instruction");
  case Match_InvalidRegisterOnTiny:
    return Error(Loc, "invalid register on PDPtiny");
  default:
    return true;
  }
}

/// Parses a register name using a given matching function.
/// Checks for lowercase or uppercase if necessary.
MCRegister PDPAsmParser::parseRegisterName(MCRegister (*matchFn)(StringRef)) {
  StringRef Name = Parser.getTok().getString();

  MCRegister Reg = matchFn(Name);

  // GCC supports case insensitive register names. Some of the PDP registers
  // are all lower case, some are all upper case but non are mixed. We prefer
  // to use the original names in the register definitions. That is why we
  // have to test both upper and lower case here.
  if (!Reg) {
    Reg = matchFn(Name.lower());
  }
  if (!Reg) {
    Reg = matchFn(Name.upper());
  }

  return Reg;
}

MCRegister PDPAsmParser::parseRegisterName() {
  MCRegister Reg = parseRegisterName(&MatchRegisterName);

  if (!Reg)
    Reg = parseRegisterName(&MatchRegisterAltName);

  return Reg;
}

MCRegister PDPAsmParser::parseRegister(bool RestoreOnFailure) {
  MCRegister Reg;

  if (Parser.getTok().is(AsmToken::Identifier)) {
    // Check for register pair syntax
    if (Parser.getLexer().peekTok().is(AsmToken::Colon)) {
      AsmToken HighTok = Parser.getTok();
      Parser.Lex();
      AsmToken ColonTok = Parser.getTok();
      Parser.Lex(); // Eat high (odd) register and colon

      if (Parser.getTok().is(AsmToken::Identifier)) {
        // Convert lower (even) register to DREG
        //Reg = toDREG(parseRegisterName());
        Reg = parseRegisterName();
      }
      if (!Reg && RestoreOnFailure) {
        getLexer().UnLex(std::move(ColonTok));
        getLexer().UnLex(std::move(HighTok));
      }
    } else {
      Reg = parseRegisterName();
    }
  }
  return Reg;
}

bool PDPAsmParser::tryParseRegisterOperand(OperandVector &Operands) {

  MCRegister Reg = parseRegister();

  if (!Reg)
    return true;

  // Reject R0~R15 on PDPtiny.
  /*if (PDP::R0 <= Reg && Reg <= PDP::R15 &&
      STI.hasFeature(PDP::FeatureTinyEncoding))
    return Error(Parser.getTok().getLoc(), "invalid register on PDPtiny");*/

  AsmToken const &T = Parser.getTok();
  Operands.push_back(PDPOperand::CreateReg(Reg, T.getLoc(), T.getEndLoc()));
  Parser.Lex(); // Eat register token.

  return false;
}

bool PDPAsmParser::tryParseExpression(OperandVector &Operands, int64_t offset) {
  SMLoc S = Parser.getTok().getLoc();

  if (!tryParseRelocExpression(Operands))
    return false;

  if ((Parser.getTok().getKind() == AsmToken::Plus ||
       Parser.getTok().getKind() == AsmToken::Minus) &&
      Parser.getLexer().peekTok().getKind() == AsmToken::Identifier) {
    // Don't handle this case - it should be split into two
    // separate tokens.
    return true;
  }

  // Parse (potentially inner) expression
  MCExpr const *Expression;
  if (getParser().parseExpression(Expression))
    return true;

  if (offset) {
    Expression = MCBinaryExpr::createAdd(
        Expression, MCConstantExpr::create(offset, getContext()), getContext());
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(PDPOperand::CreateImm(Expression, S, E));
  return false;
}

bool PDPAsmParser::tryParseDotOffset(OperandVector &Operands) {
  SMLoc S = Parser.getTok().getLoc();

  Parser.Lex();

  MCExpr const *Expression;

  if (getParser().parseExpression(Expression))
    return true;

  int64_t Res = 0;

  if (!Expression->evaluateAsAbsolute(Res))
    return true;

  Expression = MCConstantExpr::create(Res, getContext()), getContext();

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(PDPOperand::CreateImm(Expression, S, E));

  return false;

  /*if (!tryParseRelocExpression(Operands))
    return false;

  if ((Parser.getTok().getKind() == AsmToken::Plus ||
       Parser.getTok().getKind() == AsmToken::Minus) &&
      Parser.getLexer().peekTok().getKind() == AsmToken::Identifier) {
    // Don't handle this case - it should be split into two
    // separate tokens.
    return true;
  }

  // Parse (potentially inner) expression
  MCExpr const *Expression;
  if (getParser().parseExpression(Expression))
    return true;

  if (offset) {
    Expression = MCBinaryExpr::createAdd(
        Expression, MCConstantExpr::create(offset, getContext()), getContext());
  }

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(PDPOperand::CreateImm(Expression, S, E));*/
  return false;
}

bool PDPAsmParser::tryParseRelocExpression(OperandVector &Operands) {
  bool isNegated = false;
  PDPMCExpr::VariantKind ModifierKind = PDPMCExpr::VK_PDP_None;

  SMLoc S = Parser.getTok().getLoc();

  // Reject the form in which sign comes first. This behaviour is
  // in accordance with PDP-gcc.
  AsmToken::TokenKind CurTok = Parser.getLexer().getKind();
  if (CurTok == AsmToken::Minus || CurTok == AsmToken::Plus)
    return true;

  // Check for sign.
  AsmToken tokens[2];
  if (Parser.getLexer().peekTokens(tokens) == 2)
    if (tokens[0].getKind() == AsmToken::LParen &&
        tokens[1].getKind() == AsmToken::Minus)
      isNegated = true;

  // Check if we have a target specific modifier (lo8, hi8, &c)
  if (CurTok != AsmToken::Identifier ||
      Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
    // Not a reloc expr
    return true;
  }
  StringRef ModifierName = Parser.getTok().getString();
  ModifierKind = PDPMCExpr::getKindByName(ModifierName);

  if (ModifierKind != PDPMCExpr::VK_PDP_None) {
    Parser.Lex();
    Parser.Lex(); // Eat modifier name and parenthesis
    if (Parser.getTok().getString() == GENERATE_STUBS &&
        Parser.getTok().getKind() == AsmToken::Identifier) {
      std::string GSModName = ModifierName.str() + "_" + GENERATE_STUBS;
      ModifierKind = PDPMCExpr::getKindByName(GSModName);
      if (ModifierKind != PDPMCExpr::VK_PDP_None)
        Parser.Lex(); // Eat gs modifier name
    }
  } else {
    return Error(Parser.getTok().getLoc(), "unknown modifier");
  }

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    Parser.Lex();
    assert(Parser.getTok().getKind() == AsmToken::LParen);
    Parser.Lex(); // Eat the sign and parenthesis
  }

  MCExpr const *InnerExpression;
  if (getParser().parseExpression(InnerExpression))
    return true;

  if (tokens[1].getKind() == AsmToken::Minus ||
      tokens[1].getKind() == AsmToken::Plus) {
    assert(Parser.getTok().getKind() == AsmToken::RParen);
    Parser.Lex(); // Eat closing parenthesis
  }

  // If we have a modifier wrap the inner expression
  assert(Parser.getTok().getKind() == AsmToken::RParen);
  Parser.Lex(); // Eat closing parenthesis

  MCExpr const *Expression =
      PDPMCExpr::create(ModifierKind, InnerExpression, isNegated, getContext());

  SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
  Operands.push_back(PDPOperand::CreateImm(Expression, S, E));

  return false;
}

bool PDPAsmParser::parseOperand(OperandVector &Operands, bool maybeReg) {
  LLVM_DEBUG(dbgs() << "parseOperand\n");

  switch (getLexer().getKind()) {
  default:
    return Error(Parser.getTok().getLoc(), "unexpected token in operand");

  case AsmToken::Identifier:
    // Try to parse a register, fall through to the next case if it fails.
    if (maybeReg && !tryParseRegisterOperand(Operands)) {
      return false;
    }
    [[fallthrough]];
  case AsmToken::LParen:
  case AsmToken::Integer:
    return tryParseExpression(Operands, 0);
  case AsmToken::Dot:
    return tryParseDotOffset(Operands);
  case AsmToken::Plus:
  case AsmToken::Minus: {
    // If the sign preceeds a number, parse the number,
    // otherwise treat the sign a an independent token.
    switch (getLexer().peekTok().getKind()) {
    case AsmToken::Integer:
    case AsmToken::BigNum:
    case AsmToken::Identifier:
    case AsmToken::Real:
      if (!tryParseExpression(Operands, 0))
        return false;
      break;
    default:
      break;
    }
    // Treat the token as an independent token.
    Operands.push_back(PDPOperand::CreateToken(Parser.getTok().getString(),
                                               Parser.getTok().getLoc()));
    Parser.Lex(); // Eat the token.
    return false;
  }
  }

  // Could not parse operand
  return true;
}

ParseStatus PDPAsmParser::parseUniopOperand(OperandVector &Operands) {
  LLVM_DEBUG(dbgs() << "parseUniopOperand()\n");

  auto &l = getLexer();
  auto &p = getParser();
  SMLoc S;
  MCExpr const *Expression;
  MCRegister Reg;

  if (l.is(AsmToken::Hash)) {
    p.Lex();

    if (p.parseExpression(Expression))
      return ParseStatus::Failure;

    S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);

    Operands.push_back(PDPOperand::CreateUniop(PDP::R7, S, 2, Expression));
  } else if (l.is(AsmToken::At)) {
    // @
    p.Lex();

    if (l.is(AsmToken::Hash)) {
      // @#
      p.Lex();

      if (p.parseExpression(Expression))
        return ParseStatus::Failure;

      S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);
      Operands.push_back(PDPOperand::CreateUniop(PDP::R7, S, 3, Expression));
    }
    else {
      Reg = parseRegister();

      if (!Reg)
        return ParseStatus::Failure;
      p.Lex();

      S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);

      Operands.push_back(PDPOperand::CreateUniop(Reg, S, 1, nullptr));
    }
  } else if (l.is(AsmToken::Minus)) {
    // -(RN)
    p.Lex();

    if (!l.is(AsmToken::LParen))
      return ParseStatus::Failure;
    p.Lex();

    Reg = parseRegister();

    if (!Reg)
      return ParseStatus::Failure;
    p.Lex();

    if (!l.is(AsmToken::RParen))
      return ParseStatus::Failure;

    p.Lex();

    S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);

    Operands.push_back(PDPOperand::CreateUniop(Reg, S, 4, nullptr));
  } else if (l.is(AsmToken::LParen)) {
    // (RN)+
    p.Lex();

    Reg = parseRegister();

    if (!Reg)
      return ParseStatus::Failure;
    p.Lex();

    if (!l.is(AsmToken::RParen))
      return ParseStatus::Failure;

    p.Lex();

    const bool isIncrement = l.is(AsmToken::Plus);

    if (isIncrement)
      p.Lex();

    S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);

    Operands.push_back(PDPOperand::CreateUniop(Reg, S, isIncrement ? 2 : 1, nullptr));
  } else if (l.is(AsmToken::Integer)) {
    if (p.parseExpression(Expression))
      return ParseStatus::Failure;

    // N
    if (!l.is(AsmToken::LParen))
      return ParseStatus::Failure;

    p.Lex();

    Reg = parseRegister();

    if (!Reg)
      return ParseStatus::Failure;
    p.Lex();

    if (!l.is(AsmToken::RParen))
      return ParseStatus::Failure;

    p.Lex();

    S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);

    Operands.push_back(PDPOperand::CreateUniop(Reg, S, 6, Expression));
  } else {
    // Parse register.
    {
      Reg = parseRegister();

      if (!Reg)
        return ParseStatus::Failure;

      S = SMLoc::getFromPointer(p.getTok().getLoc().getPointer() - 1);
      p.Lex(); // Eat register token.

      Operands.push_back(PDPOperand::CreateUniop(Reg, S, 0, nullptr));
    }
  }

  return ParseStatus::Success;
}

bool PDPAsmParser::parseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                 SMLoc &EndLoc) {
  StartLoc = Parser.getTok().getLoc();
  Reg = parseRegister(/*RestoreOnFailure=*/false);
  EndLoc = Parser.getTok().getLoc();

  return Reg == PDP::NoRegister;
}

ParseStatus PDPAsmParser::tryParseRegister(MCRegister &Reg, SMLoc &StartLoc,
                                           SMLoc &EndLoc) {
  StartLoc = Parser.getTok().getLoc();
  Reg = parseRegister(/*RestoreOnFailure=*/true);
  EndLoc = Parser.getTok().getLoc();

  if (Reg == PDP::NoRegister)
    return ParseStatus::NoMatch;
  return ParseStatus::Success;
}

void PDPAsmParser::eatComma() {
  if (getLexer().is(AsmToken::Comma)) {
    Parser.Lex();
  } else {
    // GCC allows commas to be omitted.
  }
}

bool PDPAsmParser::parseInstruction(ParseInstructionInfo &Info,
                                    StringRef Mnemonic, SMLoc NameLoc,
                                    OperandVector &Operands) {
  Operands.push_back(PDPOperand::CreateToken(Mnemonic, NameLoc));

  int OperandNum = -1;
  while (getLexer().isNot(AsmToken::EndOfStatement)) {
    OperandNum++;
    if (OperandNum > 0)
      eatComma();

    ParseStatus ParseRes = MatchOperandParserImpl(Operands, Mnemonic);

    if (ParseRes.isSuccess())
      continue;

    if (ParseRes.isFailure()) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();

      return Error(Loc, "failed to parse register and immediate pair");
    }

    // These specific operands should be treated as addresses/symbols/labels,
    // other than registers.
    bool maybeReg = true;

    if (OperandNum == 1) {
      std::array<StringRef, 8> Insts = {"lds", "adiw", "sbiw", "ldi"};
      for (auto Inst : Insts) {
        if (Inst == Mnemonic) {
          maybeReg = false;
          break;
        }
      }
    } else if (OperandNum == 0) {
      std::array<StringRef, 8> Insts = {"sts", "call", "rcall", "rjmp", "jmp"};
      for (auto Inst : Insts) {
        if (Inst == Mnemonic) {
          maybeReg = false;
          break;
        }
      }
    }

    if (parseOperand(Operands, maybeReg)) {
      SMLoc Loc = getLexer().getLoc();
      Parser.eatToEndOfStatement();
      return Error(Loc, "unexpected token in argument list");
    }
  }
  Parser.Lex(); // Consume the EndOfStatement
  return false;
}

ParseStatus PDPAsmParser::parseDirective(llvm::AsmToken DirectiveID) {
  StringRef IDVal = DirectiveID.getIdentifier();
  if (IDVal.lower() == ".long")
    return parseLiteralValues(SIZE_LONG, DirectiveID.getLoc());
  if (IDVal.lower() == ".word" || IDVal.lower() == ".short")
    return parseLiteralValues(SIZE_WORD, DirectiveID.getLoc());
  if (IDVal.lower() == ".byte")
    return parseLiteralValues(1, DirectiveID.getLoc());
  return ParseStatus::NoMatch;
}

ParseStatus PDPAsmParser::parseLiteralValues(unsigned SizeInBytes, SMLoc L) {
  MCAsmParser &Parser = getParser();
  PDPMCELFStreamer &PDPStreamer =
      static_cast<PDPMCELFStreamer &>(Parser.getStreamer());
  AsmToken Tokens[2];
  size_t ReadCount = Parser.getLexer().peekTokens(Tokens);
  if (ReadCount == 2 && Parser.getTok().getKind() == AsmToken::Identifier &&
      Tokens[0].getKind() == AsmToken::Minus &&
      Tokens[1].getKind() == AsmToken::Identifier) {
    MCSymbol *Symbol = getContext().getOrCreateSymbol(".text");
    PDPStreamer.emitValueForModiferKind(Symbol, SizeInBytes, L,
                                        PDPMCExpr::VK_PDP_None);
    return ParseStatus::NoMatch;
  }

  if (Parser.getTok().getKind() == AsmToken::Identifier &&
      Parser.getLexer().peekTok().getKind() == AsmToken::LParen) {
    StringRef ModifierName = Parser.getTok().getString();
    PDPMCExpr::VariantKind ModifierKind =
        PDPMCExpr::getKindByName(ModifierName);
    if (ModifierKind != PDPMCExpr::VK_PDP_None) {
      Parser.Lex();
      Parser.Lex(); // Eat the modifier and parenthesis
    } else {
      return Error(Parser.getTok().getLoc(), "unknown modifier");
    }
    MCSymbol *Symbol =
        getContext().getOrCreateSymbol(Parser.getTok().getString());
    PDPStreamer.emitValueForModiferKind(Symbol, SizeInBytes, L, ModifierKind);
    Lex(); // Eat the symbol name.
    if (parseToken(AsmToken::RParen))
      return ParseStatus::Failure;
    return parseEOL();
  }

  auto parseOne = [&]() -> bool {
    const MCExpr *Value;
    if (Parser.parseExpression(Value))
      return true;
    Parser.getStreamer().emitValue(Value, SizeInBytes, L);
    return false;
  };
  return (parseMany(parseOne));
}

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializePDPAsmParser() {
  RegisterMCAsmParser<PDPAsmParser> X(getThePDPTarget());
}

#define GET_REGISTER_MATCHER
#define GET_MATCHER_IMPLEMENTATION
#include "PDPGenAsmMatcher.inc"

// Uses enums defined in PDPGenAsmMatcher.inc
unsigned PDPAsmParser::validateTargetOperandClass(MCParsedAsmOperand &AsmOp,
                                                  unsigned ExpectedKind) {


  PDPOperand &Op = static_cast<PDPOperand &>(AsmOp);
  MatchClassKind Expected = static_cast<MatchClassKind>(ExpectedKind);

  // If need be, GCC converts bare numbers to register names
  // It's ugly, but GCC supports it.
  if (Op.isImm()) {
    if (MCConstantExpr const *Const = dyn_cast<MCConstantExpr>(Op.getImm())) {
      int64_t RegNum = Const->getValue();

      // Reject R0~R15 on PDPtiny.
      /*if (0 <= RegNum && RegNum <= 15 &&
          STI.hasFeature(PDP::FeatureTinyEncoding))
        return Match_InvalidRegisterOnTiny;*/

      std::ostringstream RegName;
      RegName << "r" << RegNum;
      if (MCRegister Reg = MatchRegisterName(RegName.str())) {
        Op.makeReg(Reg);
        if (validateOperandClass(Op, Expected) == Match_Success) {
          return Match_Success;
        }
      }
      // Let the other quirks try their magic.
    }
  }

  if (Op.isReg()) {
    // If the instruction uses a register pair but we got a single, lower
    // register we perform a "class cast".
    if (isSubclass(Expected, MCK_GPR16)) {
      //MCRegister correspondingDREG = toDREG(Op.getReg());
      MCRegister correspondingDREG = Op.getReg();

      if (correspondingDREG) {
        Op.makeReg(correspondingDREG);
        return validateOperandClass(Op, Expected);
      }
    }
  }
  return Match_InvalidOperand;
}
