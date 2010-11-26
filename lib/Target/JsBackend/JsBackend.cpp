//===-- JsBackend.cpp - Library for converting LLVM code to Javascript ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This library converts LLVM IR to a JSON format, suitable for consumption by
// Emscripten.
//
//===----------------------------------------------------------------------===//

#include "JsTargetMachine.h"
#include "llvm/CallingConv.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Instructions.h"
#include "llvm/Pass.h"
#include "llvm/PassManager.h"
#include "llvm/TypeSymbolTable.h"
#include "llvm/Intrinsics.h"
#include "llvm/IntrinsicInst.h"
#include "llvm/InlineAsm.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/ConstantsScanner.h"
#include "llvm/Analysis/FindUsedTypes.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/IntrinsicLowering.h"
#include "llvm/Target/Mangler.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetRegistry.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/CFG.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/System/Host.h"
#include "llvm/Config/config.h"
#include <algorithm>
using namespace llvm;

extern "C" void LLVMInitializeJsBackendTarget() { 
  // Register the target.
  RegisterTargetMachine<JsTargetMachine> X(TheJsBackendTarget);
}

namespace {
  class JsBEMCAsmInfo : public MCAsmInfo {
  public:
    JsBEMCAsmInfo() {
      GlobalPrefix = "";
      PrivateGlobalPrefix = "";
    }
  };
  /// JsBackendNameAllUsedStructsAndMergeFunctions - This pass inserts names for
  /// any unnamed structure types that are used by the program, and merges
  /// external functions with the same name.
  ///
  class JsBackendNameAllUsedStructsAndMergeFunctions : public ModulePass {
  public:
    static char ID;
    JsBackendNameAllUsedStructsAndMergeFunctions() 
      : ModulePass(ID) {
      initializeFindUsedTypesPass(*PassRegistry::getPassRegistry());
    }
    void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequired<FindUsedTypes>();
    }

    virtual const char *getPassName() const {
      return "Javsacript backend type canonicalizer";
    }

    virtual bool runOnModule(Module &M);
  };

  char JsBackendNameAllUsedStructsAndMergeFunctions::ID = 0;

  /// JsWriter - This class is the main chunk of code that converts an LLVM
  /// module to a javascript translation unit.
  class JsWriter : public FunctionPass {
    formatted_raw_ostream &Out;
    IntrinsicLowering *IL;
    Mangler *Mang;
    LoopInfo *LI;
    const Module *TheModule;
    const MCAsmInfo* TAsm;
    MCContext *TCtx;
    const TargetData* TD;
    std::map<const Type *, std::string> TypeNames;
    std::set<Function*> intrinsicPrototypesAlreadyGenerated;
    std::set<const Argument*> ByValParams;
    unsigned FPCounter;
    unsigned LineNumber;
    DenseMap<const Value*, unsigned> AnonValueNumbers;
    unsigned NextAnonValueNumber;
    bool initialized;

  public:
    static char ID;
    explicit JsWriter(formatted_raw_ostream &o)
      : FunctionPass(ID), Out(o), IL(0), Mang(0), LI(0), 
        TheModule(0), TAsm(0), TCtx(0), TD(0), LineNumber(0),
        NextAnonValueNumber(0), initialized(false) {
      initializeLoopInfoPass(*PassRegistry::getPassRegistry());
      FPCounter = 0;
    }

    virtual const char *getPassName() const { return "javascript backend"; }

    void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addRequired<LoopInfo>();
      AU.setPreservesAll();
    }

    virtual bool doInitialization(Module &M);

    bool runOnFunction(Function &F) {
     // Do not codegen any 'available_externally' functions at all, they have
     // definitions outside the translation unit.
     if (F.hasAvailableExternallyLinkage())
       return false;

      LI = &getAnalysis<LoopInfo>();

      printFunction(F);
      return false;
    }

    virtual bool doFinalization(Module &M) {
      Out << "]";
      // Free memory...
      delete IL;
      delete TD;
      delete Mang;
      delete TCtx;
      delete TAsm;
      TypeNames.clear();
      ByValParams.clear();
      intrinsicPrototypesAlreadyGenerated.clear();
      return false;
    }

    void writeOperand(Value *Operand, bool Static = false);

  private :
    void printFunction(Function &);
    void printBasicBlock(BasicBlock *BB);
    void printLoop(Loop *L);

    void printConstant(Constant *CPV, bool Static, raw_ostream &Out);
    void printConstant(Constant *CPV, bool Static);
    void printConstantArray(ConstantArray *CPA, bool Static);
    void printConstantVector(ConstantVector *CV, bool Static);

    void writeOperands(User::const_op_iterator OI,
		       User::const_op_iterator OE);
    void writeOperands(const Instruction &I);

    // Converts an APFloat to a string via a double
    static inline std::string apfToStr(const APFloat& V) {
      double Double;
      if (&V.getSemantics() == &APFloat::IEEEdouble) {
	Double = V.convertToDouble();
      }
      else if (&V.getSemantics() == &APFloat::IEEEsingle) {
	Double = (double) V.convertToFloat();
      }
      char Buffer[200];
      snprintf(Buffer, 200, "%g", Double);
      return Buffer;
    }
    
    std::string GetValueName(const Value *Operand);
  };
}

char JsWriter::ID = 0;


static std::string JsBEMangle(const std::string &S) {
  std::string Result;
  
  for (unsigned i = 0, e = S.size(); i != e; ++i)
    if (isalnum(S[i]) || S[i] == '_') {
      Result += S[i];
    } else {
      Result += '_';
      Result += 'A'+(S[i]&15);
      Result += 'A'+((S[i]>>4)&15);
      Result += '_';
    }
  return Result;
}


/// This method inserts names for any unnamed structure types that are used by
/// the program, and removes names from structure types that are not used by the
/// program.
///
bool JsBackendNameAllUsedStructsAndMergeFunctions::runOnModule(Module &M) {
  // Get a set of types that are used by the program...
  std::set<const Type *> UT = getAnalysis<FindUsedTypes>().getTypes();

  // Loop over the module symbol table, removing types from UT that are
  // already named, and removing names for types that are not used.
  //
  TypeSymbolTable &TST = M.getTypeSymbolTable();
  for (TypeSymbolTable::iterator TI = TST.begin(), TE = TST.end();
       TI != TE; ) {
    TypeSymbolTable::iterator I = TI++;
    
    // If this isn't a struct or array type, remove it from our set of types
    // to name. This simplifies emission later.
    if (!I->second->isStructTy() && !I->second->isOpaqueTy() &&
        !I->second->isArrayTy()) {
      TST.remove(I);
    } else {
      // If this is not used, remove it from the symbol table.
      std::set<const Type *>::iterator UTI = UT.find(I->second);
      if (UTI == UT.end())
        TST.remove(I);
      else
        UT.erase(UTI);    // Only keep one name for this type.
    }
  }

  // UT now contains types that are not named.  Loop over it, naming
  // structure types.
  //
  bool Changed = false;
  unsigned RenameCounter = 0;
  for (std::set<const Type *>::const_iterator I = UT.begin(), E = UT.end();
       I != E; ++I)
    if ((*I)->isStructTy() || (*I)->isArrayTy()) {
      while (M.addTypeName("unnamed"+utostr(RenameCounter), *I))
        ++RenameCounter;
      Changed = true;
    }
      
      
  // Loop over all external functions and globals.  If we have two with
  // identical names, merge them.
  std::map<std::string, GlobalValue*> ExtSymbols;
  for (Module::iterator I = M.begin(), E = M.end(); I != E;) {
    Function *GV = I++;
    if (GV->isDeclaration() && GV->hasName()) {
      std::pair<std::map<std::string, GlobalValue*>::iterator, bool> X
        = ExtSymbols.insert(std::make_pair(GV->getName(), GV));
      if (!X.second) {
        // Found a conflict, replace this global with the previous one.
        GlobalValue *OldGV = X.first->second;
        GV->replaceAllUsesWith(ConstantExpr::getBitCast(OldGV, GV->getType()));
        GV->eraseFromParent();
        Changed = true;
      }
    }
  }
  // Do the same for globals.
  for (Module::global_iterator I = M.global_begin(), E = M.global_end();
       I != E;) {
    GlobalVariable *GV = I++;
    if (GV->isDeclaration() && GV->hasName()) {
      std::pair<std::map<std::string, GlobalValue*>::iterator, bool> X
        = ExtSymbols.insert(std::make_pair(GV->getName(), GV));
      if (!X.second) {
        // Found a conflict, replace this global with the previous one.
        GlobalValue *OldGV = X.first->second;
        GV->replaceAllUsesWith(ConstantExpr::getBitCast(OldGV, GV->getType()));
        GV->eraseFromParent();
        Changed = true;
      }
    }
  }
  
  return Changed;
}

void JsWriter::printConstantArray(ConstantArray *CPA, bool Static) {

  // As a special case, print the array as a string if it is an array of
  // ubytes or an array of sbytes with positive values.
  //
  const Type *ETy = CPA->getType()->getElementType();
  bool isString = (ETy == Type::getInt8Ty(CPA->getContext()) ||
                   ETy == Type::getInt8Ty(CPA->getContext()));

  // Make sure the last character is a null char, as automatically added by C
  if (isString && (CPA->getNumOperands() == 0 ||
                   !cast<Constant>(*(CPA->op_end()-1))->isNullValue()))
    isString = false;

  if (isString) {
    Out << '\"';
    // Keep track of whether the last number was a hexadecimal escape
    bool LastWasHex = false;

    // Do not include the last character, which we know is null
    for (unsigned i = 0, e = CPA->getNumOperands()-1; i != e; ++i) {
      unsigned char C = cast<ConstantInt>(CPA->getOperand(i))->getZExtValue();

      // Print it out literally if it is a printable character.  The only thing
      // to be careful about is when the last letter output was a hex escape
      // code, in which case we have to be careful not to print out hex digits
      // explicitly (the C compiler thinks it is a continuation of the previous
      // character, sheesh...)
      //
      if (isprint(C) && (!LastWasHex || !isxdigit(C))) {
        LastWasHex = false;
        if (C == '"' || C == '\\')
          Out << "\\" << (char)C;
        else
          Out << (char)C;
      } else {
        LastWasHex = false;
        switch (C) {
        case '\n': Out << "\\n"; break;
        case '\t': Out << "\\t"; break;
        case '\r': Out << "\\r"; break;
        case '\v': Out << "\\v"; break;
        case '\a': Out << "\\a"; break;
        case '\"': Out << "\\\""; break;
        case '\'': Out << "\\\'"; break;
        default:
          Out << "\\x";
          Out << (char)(( C/16  < 10) ? ( C/16 +'0') : ( C/16 -10+'A'));
          Out << (char)(((C&15) < 10) ? ((C&15)+'0') : ((C&15)-10+'A'));
          LastWasHex = true;
          break;
        }
      }
    }
    Out << '\"';
    return;
  } 
  Out << '[';
  if (CPA->getNumOperands()) {
    printConstant(cast<Constant>(CPA->getOperand(0)), Static);
    for (unsigned i = 1, e = CPA->getNumOperands(); i != e; ++i) {
      Out << ", ";
      printConstant(cast<Constant>(CPA->getOperand(i)), Static);
    }
  }
  Out << "]";
}

void JsWriter::printConstantVector(ConstantVector *CP, bool Static) {
  Out << '[';
  if (CP->getNumOperands()) {
    Out << ' ';
    printConstant(cast<Constant>(CP->getOperand(0)), Static);
    for (unsigned i = 1, e = CP->getNumOperands(); i != e; ++i) {
      Out << ", ";
      printConstant(cast<Constant>(CP->getOperand(i)), Static);
    }
  }
  Out << " ]";
}

void JsWriter::printConstant(Constant *CPV, bool Static, raw_ostream &Out) {
  if (const ConstantExpr *CE = dyn_cast<ConstantExpr>(CPV)) {
    switch (CE->getOpcode()) {
    case Instruction::Trunc:
    case Instruction::ZExt:
    case Instruction::SExt:
    case Instruction::FPTrunc:
    case Instruction::FPExt:
    case Instruction::UIToFP:
    case Instruction::SIToFP:
    case Instruction::FPToUI:
    case Instruction::FPToSI:
    case Instruction::PtrToInt:
    case Instruction::IntToPtr:
    case Instruction::BitCast:
      Out << "\"(";
      if (CE->getOpcode() == Instruction::SExt &&
          CE->getOperand(0)->getType() == Type::getInt1Ty(CPV->getContext())) {
        // Make sure we really sext from bool here by subtracting from 0
        Out << "0-";
      }
      printConstant(CE->getOperand(0), Static);
      if (CE->getType() == Type::getInt1Ty(CPV->getContext()) &&
          (CE->getOpcode() == Instruction::Trunc ||
           CE->getOpcode() == Instruction::FPToUI ||
           CE->getOpcode() == Instruction::FPToSI ||
           CE->getOpcode() == Instruction::PtrToInt)) {
        // Make sure we really truncate to bool here by anding with 1
        Out << "&1u";
      }
      Out << ")\"";
      return;

    case Instruction::GetElementPtr:
      // If there are no indices, just print out the pointer.
      if (CE->getNumOperands() <= 1) {
	writeOperand(CE->getOperand(0));
	return;
      }

      Out << "{ \"intertype\": \"getelementptr\", ";
      Out << "\"operands\": ";
      writeOperands(CE->op_begin(), CE->op_end());
      return;
    case Instruction::Select:
      Out << "\"(";
      printConstant(CE->getOperand(0), Static);
      Out << '?';
      printConstant(CE->getOperand(1), Static);
      Out << ':';
      printConstant(CE->getOperand(2), Static);
      Out << ")\"";
      return;
    case Instruction::Add:
    case Instruction::FAdd:
    case Instruction::Sub:
    case Instruction::FSub:
    case Instruction::Mul:
    case Instruction::FMul:
    case Instruction::SDiv:
    case Instruction::UDiv:
    case Instruction::FDiv:
    case Instruction::URem:
    case Instruction::SRem:
    case Instruction::FRem:
    case Instruction::And:
    case Instruction::Or:
    case Instruction::Xor:
    case Instruction::ICmp:
    case Instruction::Shl:
    case Instruction::LShr:
    case Instruction::AShr:
    {
      Out << "\"(";
      printConstant(CE->getOperand(0), Static);
      switch (CE->getOpcode()) {
      case Instruction::Add:
      case Instruction::FAdd: Out << " + "; break;
      case Instruction::Sub:
      case Instruction::FSub: Out << " - "; break;
      case Instruction::Mul:
      case Instruction::FMul: Out << " * "; break;
      case Instruction::URem:
      case Instruction::SRem: 
      case Instruction::FRem: Out << " % "; break;
      case Instruction::UDiv: 
      case Instruction::SDiv: 
      case Instruction::FDiv: Out << " / "; break;
      case Instruction::And: Out << " & "; break;
      case Instruction::Or:  Out << " | "; break;
      case Instruction::Xor: Out << " ^ "; break;
      case Instruction::Shl: Out << " << "; break;
      case Instruction::LShr: Out << " >>> "; break;
      case Instruction::AShr: Out << " >> "; break;
      case Instruction::ICmp:
        switch (CE->getPredicate()) {
          case ICmpInst::ICMP_EQ: Out << " == "; break;
          case ICmpInst::ICMP_NE: Out << " != "; break;
          case ICmpInst::ICMP_SLT: 
          case ICmpInst::ICMP_ULT: Out << " < "; break;
          case ICmpInst::ICMP_SLE:
          case ICmpInst::ICMP_ULE: Out << " <= "; break;
          case ICmpInst::ICMP_SGT:
          case ICmpInst::ICMP_UGT: Out << " > "; break;
          case ICmpInst::ICMP_SGE:
          case ICmpInst::ICMP_UGE: Out << " >= "; break;
          default: llvm_unreachable("Illegal ICmp predicate");
        }
        break;
      default: llvm_unreachable("Illegal opcode here!");
      }
      printConstant(CE->getOperand(1), Static);
      Out << ")\"";
      return;
    }
    case Instruction::FCmp: {
      Out << "\"("; 
      if (CE->getPredicate() == FCmpInst::FCMP_FALSE)
        Out << "0";
      else if (CE->getPredicate() == FCmpInst::FCMP_TRUE)
        Out << "1";
      else {
        const char* op = 0;
        switch (CE->getPredicate()) {
        default: llvm_unreachable("Illegal FCmp predicate");
        case FCmpInst::FCMP_ORD: op = "ord"; break;
        case FCmpInst::FCMP_UNO: op = "uno"; break;
        case FCmpInst::FCMP_UEQ: op = "ueq"; break;
        case FCmpInst::FCMP_UNE: op = "une"; break;
        case FCmpInst::FCMP_ULT: op = "ult"; break;
        case FCmpInst::FCMP_ULE: op = "ule"; break;
        case FCmpInst::FCMP_UGT: op = "ugt"; break;
        case FCmpInst::FCMP_UGE: op = "uge"; break;
        case FCmpInst::FCMP_OEQ: op = "oeq"; break;
        case FCmpInst::FCMP_ONE: op = "one"; break;
        case FCmpInst::FCMP_OLT: op = "olt"; break;
        case FCmpInst::FCMP_OLE: op = "ole"; break;
        case FCmpInst::FCMP_OGT: op = "ogt"; break;
        case FCmpInst::FCMP_OGE: op = "oge"; break;
        }
        Out << "llvm_fcmp_" << op << "(";
        printConstant(CE->getOperand(0), Static);
        Out << ", ";
        printConstant(CE->getOperand(1), Static);
        Out << ")";
      }
      Out << ")\"";
      return;
    }
    default:
#ifndef NDEBUG
      errs() << "JsWriter Error: Unhandled constant expression: "
           << *CE << "\n";
#endif
      llvm_unreachable(0);
    }
  } else if (isa<UndefValue>(CPV) && CPV->getType()->isSingleValueType()) {
    if (!CPV->getType()->isVectorTy()) {
      Out << "null";
    } else {
      Out << "[]";
    }
    return;
  }

  if (ConstantInt *CI = dyn_cast<ConstantInt>(CPV)) {
    const Type* Ty = CI->getType();
    if (Ty == Type::getInt1Ty(CPV->getContext()))
      Out << (CI->getZExtValue() ? "true" : "false");
    else if (Ty == Type::getInt32Ty(CPV->getContext()))
      Out << CI->getSExtValue();
    else if (CI->isMinValue(true)) {
      Out << CI->getZExtValue();
    } else {
      Out << CI->getSExtValue();
    }
    return;
  } 

  switch (CPV->getType()->getTypeID()) {
  case Type::FloatTyID:
  case Type::DoubleTyID: {
    ConstantFP *FPC = cast<ConstantFP>(CPV);
    double V;
    if (FPC->getType() == Type::getFloatTy(CPV->getContext())) {
      V = FPC->getValueAPF().convertToFloat();
    } else {
      V = FPC->getValueAPF().convertToDouble();
    }
      
    if (IsNAN(V)) {
      Out << "NaN";
    } else if (IsInf(V)) {
      Out << "Infinity";
    } else {
      Out << apfToStr(FPC->getValueAPF());
    }
    break;
  }

  case Type::ArrayTyID:
    if (ConstantArray *CA = dyn_cast<ConstantArray>(CPV)) {
      printConstantArray(CA, Static);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const ArrayType *AT = cast<ArrayType>(CPV->getType());
      Out << '[';
      if (AT->getNumElements()) {
        Out << ' ';
        Constant *CZ = Constant::getNullValue(AT->getElementType());
        printConstant(CZ, Static);
        for (unsigned i = 1, e = AT->getNumElements(); i != e; ++i) {
          Out << ", ";
          printConstant(CZ, Static);
        }
      }
      Out << " ]";
    }
    break;

  case Type::VectorTyID:
    if (ConstantVector *CV = dyn_cast<ConstantVector>(CPV)) {
      printConstantVector(CV, Static);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const VectorType *VT = cast<VectorType>(CPV->getType());
      Out << "[ ";
      Constant *CZ = Constant::getNullValue(VT->getElementType());
      printConstant(CZ, Static);
      for (unsigned i = 1, e = VT->getNumElements(); i != e; ++i) {
        Out << ", ";
        printConstant(CZ, Static);
      }
      Out << " ]";
    }
    break;

  case Type::StructTyID:
    if (isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV)) {
      const StructType *ST = cast<StructType>(CPV->getType());
      Out << '[';
      if (ST->getNumElements()) {
        Out << ' ';
        printConstant(Constant::getNullValue(ST->getElementType(0)), Static);
        for (unsigned i = 1, e = ST->getNumElements(); i != e; ++i) {
          Out << ", ";
          printConstant(Constant::getNullValue(ST->getElementType(i)), Static);
        }
      }
      Out << " ]";
      break;
    }
    Out << '[';
    if (CPV->getNumOperands()) {
      printConstant(cast<Constant>(CPV->getOperand(0)), Static);
      for (unsigned i = 1, e = CPV->getNumOperands(); i != e; ++i) {
	Out << ", ";
	printConstant(cast<Constant>(CPV->getOperand(i)), Static);
      }
    }
    Out << "]";
    break;

  case Type::PointerTyID:
    if (isa<ConstantPointerNull>(CPV)) {
      Out << "null";
      break;
    } else if (GlobalValue *GV = dyn_cast<GlobalValue>(CPV)) {
      writeOperand(GV, Static);
      break;
    }
  // FALL THROUGH
  default:
#ifndef NDEBUG
    errs() << "Unknown constant type: " << *CPV << "\n";
#endif
    llvm_unreachable(0);
  }
}

// printConstant - The LLVM Constant to C Constant converter.
void inline JsWriter::printConstant(Constant *CPV, bool Static) {
  printConstant(CPV, Static, Out);
}

std::string JsWriter::GetValueName(const Value *Operand) {
  // Mangle globals with the standard mangler interface for LLC compatibility.
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
    SmallString<128> Str;
    Mang->getNameWithPrefix(Str, GV, false);
    return JsBEMangle(Str.str().str());
  }
    
  std::string Name = Operand->getName();
    
  if (Name.empty()) { // Assign unique names to local temporaries.
    std::pair<DenseMapIterator<const Value*, unsigned>, bool> Lookup = AnonValueNumbers.insert(std::pair<const Value*, unsigned>(Operand, NextAnonValueNumber));
    if(Lookup.second) {
      NextAnonValueNumber++;
    }
    unsigned &No = Lookup.first->second;
    Name = utostr(No);
  }
    
  std::string VarName;
  VarName.reserve(Name.capacity());

  for (std::string::iterator I = Name.begin(), E = Name.end();
       I != E; ++I) {
    char ch = *I;

    if (!((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
          (ch >= '0' && ch <= '9') || ch == '_')) {
      char buffer[5];
      sprintf(buffer, "_%x_", ch);
      VarName += buffer;
    } else
      VarName += ch;
  }

  return "v" + VarName;
}

// writeOperands - Outputs a javascript array of operand objects for the
// specified Instruction.
void JsWriter::writeOperands(User::const_op_iterator OI, User::const_op_iterator OE) {
  Out << "[";
  if(OI != OE) {
    Out << "{ \"value\": ";
    writeOperand(*OI);
    Out << ", \"type\": \"";
    Out << OI->get()->getType()->getDescription() << "\" }";
    ++OI;
    for(; OI != OE; ++OI) {
      Out << ", { \"value\": ";
      writeOperand(*OI);
      Out << ", \"type\": \"";
      Out << OI->get()->getType()->getDescription() << "\" }";
    }
  }
  Out << "]";
}

// writeOperands - Outputs a javascript array of operand objects for the
// specified Instruction.
void JsWriter::writeOperands(const Instruction &I) {
  writeOperands(I.op_begin(), I.op_end());
}

// writeOperand - Outputs a javascript object that specifies the given Operand.
void JsWriter::writeOperand(Value *Operand, bool Static) {
  Constant* CPV = dyn_cast<Constant>(Operand);

  if (CPV && !isa<GlobalValue>(CPV)) {
    printConstant(CPV, Static);
  } else {
    Out << "\"" << GetValueName(Operand) << "\"";
  }
}

bool JsWriter::doInitialization(Module &M) {
  FunctionPass::doInitialization(M);
  
  // Initialize
  TheModule = &M;

  TD = new TargetData(&M);
  IL = new IntrinsicLowering(*TD);
  IL->AddPrototypes(M);

  TAsm = new JsBEMCAsmInfo();
  TCtx = new MCContext(*TAsm);
  Mang = new Mangler(*TCtx, *TD);
  Out << "[";
  if(M.global_empty()) {
    return false;
  }
  Module::global_iterator I = M.global_begin(), E = M.global_end();
  for(; I != E; ++I) {
    if (!I->isDeclaration() &&
	(I->hasLocalLinkage() || I->hasHiddenVisibility())) {
      Out << "{ \"ident\": \"" << GetValueName(I);
      Out << "\", \"intertype\": \"globalVariable\", ";
      Out << "\"lineNum\": " << LineNumber++ << ", ";
      Out << "\"type\": \"" << I->getType()->getDescription() << "\", ";
      Out << "\"value\": { \"text\": ";
      writeOperand(I->getInitializer(), true);
      Out << " }}";
      initialized = true;
      ++I;
      break;
    }
  }
  for(; I != E; ++I) {
    if (!I->isDeclaration() &&
	(I->hasLocalLinkage() || I->hasHiddenVisibility())) {
      writeOperand(I->getInitializer(), true);
    }
  }
  return false;
}

void JsWriter::printFunction(Function &F) {
  if(initialized) {
    Out << ",\n";
  } else {
    initialized = true;
  }
  Out << "{ \"ident\": \"" << GetValueName(&F) << "\", ";
  Out << "\"intertype\": \"function\", \"lineNum\": " << LineNumber++;
  Out << ", \"params\": [";
  if(!F.arg_empty()) {
    Function::const_arg_iterator AI = F.arg_begin(), AE = F.arg_end();
    Out << "{ \"item\": \"" << GetValueName(AI) << "\", ";
    Out << "\"intertype\": \"\" }";
    ++AI;
    for(; AI != AE; ++AI) {
      Out << ", { \"item\": \"" << GetValueName(AI) << "\", ";
      Out << "\"intertype\": \"\" }";
    }
  }
  Out << "]";
  Out << ", \"returnType\": \"" << F.getReturnType()->getDescription() << "\" }";

  // print the basic blocks
  Function::iterator BB = F.begin(), E = F.end();
  if(BB != E) {
    if (Loop *L = LI->getLoopFor(BB)) {
      if (L->getHeader() == BB && L->getParentLoop() == 0)
        printLoop(L);
    } else {
      printBasicBlock(BB);
    }
    ++BB;
  }
  for(; BB != E; ++BB) {
    if (Loop *L = LI->getLoopFor(BB)) {
      if (L->getHeader() == BB && L->getParentLoop() == 0)
        printLoop(L);
    } else {
      printBasicBlock(BB);
    }
  }
  Out << ",\n{ \"intertype\": \"functionEnd\", \"lineNum\": ";
  Out << LineNumber++ <<" }";
}

void JsWriter::printLoop(Loop *L) {
  for (unsigned i = 0, e = L->getBlocks().size(); i != e; ++i) {
    BasicBlock *BB = L->getBlocks()[i];
    Loop *BBLoop = LI->getLoopFor(BB);
    if (BBLoop == L)
      printBasicBlock(BB);
    else if (BB == BBLoop->getHeader() && BBLoop->getParentLoop() == L)
      printLoop(BBLoop);
  }
}

void JsWriter::printBasicBlock(BasicBlock *BB) {
  // Output all of the instructions in the basic block...
  for (BasicBlock::iterator II = BB->begin(), E = --BB->end(); II != E;
       ++II, LineNumber++) {
    Out << ",\n{ \"ident\": \"" << GetValueName(II) << "\", ";
    Out << "\"intertype\": \"" << II->getOpcodeName() << "\", ";
    Out << "\"lineNum\": " << LineNumber << ", ";
    Out << "\"operands\": ";
    writeOperands(*II);
  }
  const TerminatorInst *terminator = BB->getTerminator();
  Out << ",\n{ \"intertype\": \"" << terminator->getOpcodeName() << "\", ";
  Out << "\"lineNum\": " << LineNumber++ << ", ";
  Out << "\"type\": \"" << terminator->getType()->getDescription() << "\", ";
  Out << "\"operands\": ";
  writeOperands(*terminator);
  Out << "}";
}

//===----------------------------------------------------------------------===//
//                       External Interface declaration
//===----------------------------------------------------------------------===//

bool JsTargetMachine::addPassesToEmitFile(PassManagerBase &PM,
					  formatted_raw_ostream &o,
					  CodeGenFileType FileType,
					  CodeGenOpt::Level OptLevel,
					  bool DisableVerify) {
  if (FileType != TargetMachine::CGFT_AssemblyFile) return true;
  switch(OptLevel) {
  case CodeGenOpt::None:
    PM.add(new JsBackendNameAllUsedStructsAndMergeFunctions());
    PM.add(new JsWriter(o));
    break;
  default:
    PM.add(createGCLoweringPass());
    PM.add(createLowerInvokePass());
    PM.add(createCFGSimplificationPass());   // clean up after lower invoke.
    PM.add(new JsBackendNameAllUsedStructsAndMergeFunctions());
    PM.add(new JsWriter(o));
    PM.add(createGCInfoDeleter());
  }

  return false;
}
