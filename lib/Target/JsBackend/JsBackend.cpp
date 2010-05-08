//===-- JsBackend.cpp - Library for converting LLVM code to Javascript ----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This library converts LLVM code to Javascript code, executable in a web
// browser.
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
#include "llvm/Support/InstVisitor.h"
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
  class CBEMCAsmInfo : public MCAsmInfo {
  public:
    CBEMCAsmInfo() {
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
      : ModulePass(&ID) {}
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
  class JsWriter : public FunctionPass, public InstVisitor<JsWriter> {
    formatted_raw_ostream &Out;
    IntrinsicLowering *IL;
    Mangler *Mang;
    LoopInfo *LI;
    const Module *TheModule;
    const MCAsmInfo* TAsm;
    MCContext *TCtx;
    const TargetData* TD;
    std::map<const Type *, std::string> TypeNames;
    std::map<const ConstantFP *, unsigned> FPConstantMap;
    std::set<Function*> intrinsicPrototypesAlreadyGenerated;
    std::set<const Argument*> ByValParams;
    unsigned FPCounter;
    unsigned OpaqueCounter;
    DenseMap<const Value*, unsigned> AnonValueNumbers;
    unsigned NextAnonValueNumber;

  public:
    static char ID;
    explicit JsWriter(formatted_raw_ostream &o)
      : FunctionPass(&ID), Out(o), IL(0), Mang(0), LI(0), 
        TheModule(0), TAsm(0), TCtx(0), TD(0), OpaqueCounter(0),
        NextAnonValueNumber(0) {
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

      // Get rid of intrinsics we can't handle.
      lowerIntrinsics(F);

      // Output all floating point constants that cannot be printed accurately.
      printFloatingPointConstants(F);

      printFunction(F);
      return false;
    }

    virtual bool doFinalization(Module &M) {
      Function *Main = M.getFunction("main");
      if(Main) {
	Out << "_.main();\n";
      }
      Out << "})(window);\n";
      if(Main) {
	Out << "</script>\n";
	Out << "</body>\n";
	Out << "</html>";
      }
      // Free memory...
      delete IL;
      delete TD;
      delete Mang;
      delete TCtx;
      delete TAsm;
      FPConstantMap.clear();
      TypeNames.clear();
      ByValParams.clear();
      intrinsicPrototypesAlreadyGenerated.clear();
      return false;
    }

    raw_ostream &printType(raw_ostream &Out, const Type *Ty,
                           bool isSigned = false,
                           const std::string &VariableName = "",
                           bool IgnoreName = false,
                           const AttrListPtr &PAL = AttrListPtr());
    raw_ostream &printSimpleType(raw_ostream &Out, const Type *Ty,
                                 bool isSigned,
                                 const std::string &NameSoFar = "");

    void printStructReturnPointerFunctionType(raw_ostream &Out,
                                              const AttrListPtr &PAL,
                                              const PointerType *Ty);

    /// writeOperandDeref - Print the result of dereferencing the specified
    /// operand with '*'.  This is equivalent to printing '*' then using
    /// writeOperand, but avoids excess syntax in some cases.
    void writeOperandDeref(Value *Operand) {
      if (isAddressExposed(Operand)) {
        // Already something with an address exposed.
        writeOperandInternal(Operand);
      } else {
        Out << "*(";
        writeOperand(Operand);
        Out << ")";
      }
    }
    
    void writeOperand(Value *Operand, bool Static = false);
    void writeInstComputationInline(Instruction &I);
    void writeOperandInternal(Value *Operand, bool Static = false);
    void writeOperandWithCast(Value* Operand, unsigned Opcode);
    void writeOperandWithCast(Value* Operand, const ICmpInst &I);
    bool writeInstructionCast(const Instruction &I);

    void writeMemoryAccess(Value *Operand, const Type *OperandType,
                           bool IsVolatile, unsigned Alignment);

  private :
    std::string InterpretASMConstraint(InlineAsm::ConstraintInfo& c);

    void lowerIntrinsics(Function &F);

    void printModule(Module *M);
    void printModuleTypes(const TypeSymbolTable &ST);
    std::string getOperand(Value *Operand, bool Static = false);
    void printContainedStructs(const Type *Ty, std::set<const Type *> &);
    void printFloatingPointConstants(Function &F);
    void printFloatingPointConstants(const Constant *C);
    void printFunctionSignature(const Function *F, bool Prototype);

    void printFunction(Function &);
    void printBasicBlock(BasicBlock *BB);
    void printLoop(Loop *L);

    void printCast(unsigned opcode, const Type *SrcTy, const Type *DstTy);
    void printConstant(Constant *CPV, bool Static, raw_ostream &Out);
    void printConstant(Constant *CPV, bool Static);
    void printConstantWithCast(Constant *CPV, unsigned Opcode);
    bool printConstExprCast(const ConstantExpr *CE, bool Static);
    void printConstantArray(ConstantArray *CPA, bool Static);
    void printConstantVector(ConstantVector *CV, bool Static);

    /// isAddressExposed - Return true if the specified value's name needs to
    /// have its address taken in order to get a C value of the correct type.
    /// This happens for global variables, byval parameters, and direct allocas.
    bool isAddressExposed(const Value *V) const {
      if (const Argument *A = dyn_cast<Argument>(V))
        return ByValParams.count(A);
      return isa<GlobalVariable>(V) || isDirectAlloca(V);
    }
    
    // isInlinableInst - Attempt to inline instructions into their uses to build
    // trees as much as possible.  To do this, we have to consistently decide
    // what is acceptable to inline, so that variable declarations don't get
    // printed and an extra copy of the expr is not emitted.
    //
    static bool isInlinableInst(const Instruction &I) {
      // Always inline cmp instructions, even if they are shared by multiple
      // expressions.  GCC generates horrible code if we don't.
      if (isa<CmpInst>(I)) 
        return true;

      // Must be an expression, must be used exactly once.  If it is dead, we
      // emit it inline where it would go.
      if (I.getType() == Type::getVoidTy(I.getContext()) || !I.hasOneUse() ||
          isa<TerminatorInst>(I) || isa<CallInst>(I) || isa<PHINode>(I) ||
          isa<LoadInst>(I) || isa<VAArgInst>(I) || isa<InsertElementInst>(I) ||
          isa<InsertValueInst>(I))
        // Don't inline a load across a store or other bad things!
        return false;

      // Must not be used in inline asm, extractelement, or shufflevector.
      if (I.hasOneUse()) {
        const Instruction &User = cast<Instruction>(*I.use_back());
        if (isInlineAsm(User) || isa<ExtractElementInst>(User) ||
            isa<ShuffleVectorInst>(User))
          return false;
      }

      // Only inline instruction it if it's use is in the same BB as the inst.
      return I.getParent() == cast<Instruction>(I.use_back())->getParent();
    }

    // isDirectAlloca - Define fixed sized allocas in the entry block as direct
    // variables which are accessed with the & operator.  This causes GCC to
    // generate significantly better code than to emit alloca calls directly.
    //
    static const AllocaInst *isDirectAlloca(const Value *V) {
      const AllocaInst *AI = dyn_cast<AllocaInst>(V);
      if (!AI) return false;
      if (AI->isArrayAllocation())
        return 0;   // FIXME: we can also inline fixed size array allocas!
      if (AI->getParent() != &AI->getParent()->getParent()->getEntryBlock())
        return 0;
      return AI;
    }
    
    // isInlineAsm - Check if the instruction is a call to an inline asm chunk
    static bool isInlineAsm(const Instruction& I) {
      if (const CallInst *CI = dyn_cast<CallInst>(&I))
        return isa<InlineAsm>(CI->getCalledValue());
      return false;
    }
    
    // Instruction visitation functions
    friend class InstVisitor<JsWriter>;

    void visitReturnInst(ReturnInst &I);
    void visitBranchInst(BranchInst &I);
    void visitSwitchInst(SwitchInst &I);
    void visitIndirectBrInst(IndirectBrInst &I);
    void visitInvokeInst(InvokeInst &I) {
      llvm_unreachable("Lowerinvoke pass didn't work!");
    }

    void visitUnwindInst(UnwindInst &I);
    void visitUnreachableInst(UnreachableInst &I);

    void visitPHINode(PHINode &I);
    void visitBinaryOperator(Instruction &I);
    void visitICmpInst(ICmpInst &I);
    void visitFCmpInst(FCmpInst &I);

    void visitCastInst (CastInst &I);
    void visitSelectInst(SelectInst &I);
    void visitCallInst (CallInst &I);
    void visitInlineAsm(CallInst &I);
    bool visitBuiltinCall(CallInst &I, Intrinsic::ID ID, bool &WroteCallee);

    void visitAllocaInst(AllocaInst &I);
    void visitLoadInst  (LoadInst   &I);
    void visitStoreInst (StoreInst  &I);
    void visitGetElementPtrInst(GetElementPtrInst &I);
    void visitVAArgInst (VAArgInst &I);
    
    void visitInsertElementInst(InsertElementInst &I);
    void visitExtractElementInst(ExtractElementInst &I);
    void visitShuffleVectorInst(ShuffleVectorInst &SVI);

    void visitInsertValueInst(InsertValueInst &I);
    void visitExtractValueInst(ExtractValueInst &I);

    void visitInstruction(Instruction &I) {
#ifndef NDEBUG
      errs() << "Javascript Writer does not know about " << I;
#endif
      llvm_unreachable(0);
    }

    void outputLValue(Instruction *I) {
      Out << GetValueName(I) << " = ";
    }

    bool isGotoCodeNecessary(BasicBlock *From, BasicBlock *To);
    void printPHICopiesForSuccessor(BasicBlock *CurBlock,
                                    BasicBlock *Successor, unsigned Indent);
    void printBranchToBlock(BasicBlock *CurBlock, BasicBlock *SuccBlock,
                            unsigned Indent);
    void printGEPExpression(Value *Ptr, gep_type_iterator I,
                            gep_type_iterator E, bool Static);

    std::string GetValueName(const Value *Operand);
  };
}

char JsWriter::ID = 0;


static std::string CBEMangle(const std::string &S) {
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
  // FIXME: This code should disappear when we don't allow values with the same
  // names when they have different types!
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

/// printStructReturnPointerFunctionType - This is like printType for a struct
/// return type, except, instead of printing the type as void (*)(Struct*, ...)
/// print it as "Struct (*)(...)", for struct return functions.
void JsWriter::printStructReturnPointerFunctionType(raw_ostream &Out,
                                                   const AttrListPtr &PAL,
                                                   const PointerType *TheTy) {
  const FunctionType *FTy = cast<FunctionType>(TheTy->getElementType());
  std::string tstr;
  raw_string_ostream FunctionInnards(tstr);
  FunctionInnards << " (*) (";
  bool PrintedType = false;

  FunctionType::param_iterator I = FTy->param_begin(), E = FTy->param_end();
  const Type *RetTy = cast<PointerType>(I->get())->getElementType();
  unsigned Idx = 1;
  for (++I, ++Idx; I != E; ++I, ++Idx) {
    if (PrintedType)
      FunctionInnards << ", ";
    const Type *ArgTy = *I;
    if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
      assert(ArgTy->isPointerTy());
      ArgTy = cast<PointerType>(ArgTy)->getElementType();
    }
    printType(FunctionInnards, ArgTy,
        /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt), "");
    PrintedType = true;
  }
  if (FTy->isVarArg()) {
    if (!PrintedType)
      FunctionInnards << " int"; //dummy argument for empty vararg functs
    FunctionInnards << ", ...";
  } else if (!PrintedType) {
    FunctionInnards << "void";
  }
  FunctionInnards << ')';
  printType(Out, RetTy, 
      /*isSigned=*/PAL.paramHasAttr(0, Attribute::SExt), FunctionInnards.str());
}

raw_ostream &
JsWriter::printSimpleType(raw_ostream &Out, const Type *Ty, bool isSigned,
                         const std::string &NameSoFar) {
  assert((Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isVectorTy()) && 
         "Invalid type for printSimpleType");
  switch (Ty->getTypeID()) {
  case Type::VoidTyID:   return Out << "void " << NameSoFar;
  case Type::IntegerTyID: {
    unsigned NumBits = cast<IntegerType>(Ty)->getBitWidth();
    if (NumBits == 1) 
      return Out << "bool " << NameSoFar;
    else if (NumBits <= 8)
      return Out << (isSigned?"signed":"unsigned") << " char " << NameSoFar;
    else if (NumBits <= 16)
      return Out << (isSigned?"signed":"unsigned") << " short " << NameSoFar;
    else if (NumBits <= 32)
      return Out << (isSigned?"signed":"unsigned") << " int " << NameSoFar;
    else if (NumBits <= 64)
      return Out << (isSigned?"signed":"unsigned") << " long long "<< NameSoFar;
    else { 
      assert(NumBits <= 128 && "Bit widths > 128 not implemented yet");
      return Out << (isSigned?"llvmInt128":"llvmUInt128") << " " << NameSoFar;
    }
  }
  case Type::FloatTyID:  return Out << "float "   << NameSoFar;
  case Type::DoubleTyID: return Out << "double "  << NameSoFar;
  // Lacking emulation of FP80 on PPC, etc., we assume whichever of these is
  // present matches host 'long double'.
  case Type::X86_FP80TyID:
  case Type::PPC_FP128TyID:
  case Type::FP128TyID:  return Out << "long double " << NameSoFar;
      
  case Type::VectorTyID: {
    const VectorType *VTy = cast<VectorType>(Ty);
    return printSimpleType(Out, VTy->getElementType(), isSigned,
                     " __attribute__((vector_size(" +
                     utostr(TD->getTypeAllocSize(VTy)) + " ))) " + NameSoFar);
  }
    
  default:
#ifndef NDEBUG
    errs() << "Unknown primitive type: " << *Ty << "\n";
#endif
    llvm_unreachable(0);
  }
}

// Pass the Type* and the variable name and this prints out the variable
// declaration.
//
raw_ostream &JsWriter::printType(raw_ostream &Out, const Type *Ty,
                                bool isSigned, const std::string &NameSoFar,
                                bool IgnoreName, const AttrListPtr &PAL) {
  if (Ty->isPrimitiveType() || Ty->isIntegerTy() || Ty->isVectorTy()) {
    printSimpleType(Out, Ty, isSigned, NameSoFar);
    return Out;
  }

  // Check to see if the type is named.
  if (!IgnoreName || Ty->isOpaqueTy()) {
    std::map<const Type *, std::string>::iterator I = TypeNames.find(Ty);
    if (I != TypeNames.end()) return Out << I->second << ' ' << NameSoFar;
  }

  switch (Ty->getTypeID()) {
  case Type::FunctionTyID: {
    const FunctionType *FTy = cast<FunctionType>(Ty);
    std::string tstr;
    raw_string_ostream FunctionInnards(tstr);
    FunctionInnards << " (" << NameSoFar << ") (";
    unsigned Idx = 1;
    for (FunctionType::param_iterator I = FTy->param_begin(),
           E = FTy->param_end(); I != E; ++I) {
      const Type *ArgTy = *I;
      if (PAL.paramHasAttr(Idx, Attribute::ByVal)) {
        assert(ArgTy->isPointerTy());
        ArgTy = cast<PointerType>(ArgTy)->getElementType();
      }
      if (I != FTy->param_begin())
        FunctionInnards << ", ";
      printType(FunctionInnards, ArgTy,
        /*isSigned=*/PAL.paramHasAttr(Idx, Attribute::SExt), "");
      ++Idx;
    }
    if (FTy->isVarArg()) {
      if (!FTy->getNumParams())
        FunctionInnards << " int"; //dummy argument for empty vaarg functs
      FunctionInnards << ", ...";
    } else if (!FTy->getNumParams()) {
      FunctionInnards << "void";
    }
    FunctionInnards << ')';
    printType(Out, FTy->getReturnType(), 
      /*isSigned=*/PAL.paramHasAttr(0, Attribute::SExt), FunctionInnards.str());
    return Out;
  }
  case Type::StructTyID: {
    const StructType *STy = cast<StructType>(Ty);
    Out << NameSoFar + " {\n";
    unsigned Idx = 0;
    for (StructType::element_iterator I = STy->element_begin(),
           E = STy->element_end(); I != E; ++I) {
      Out << "  ";
      printType(Out, *I, false, "field" + utostr(Idx++));
      Out << ";\n";
    }
    Out << '}';
    if (STy->isPacked())
      Out << " __attribute__ ((packed))";
    return Out;
  }

  case Type::PointerTyID: {
    const PointerType *PTy = cast<PointerType>(Ty);
    std::string ptrName = "*" + NameSoFar;

    if (PTy->getElementType()->isArrayTy() ||
        PTy->getElementType()->isVectorTy())
      ptrName = "(" + ptrName + ")";

    if (!PAL.isEmpty())
      // Must be a function ptr cast!
      return printType(Out, PTy->getElementType(), false, ptrName, true, PAL);
    return printType(Out, PTy->getElementType(), false, ptrName);
  }

  case Type::ArrayTyID: {
    const ArrayType *ATy = cast<ArrayType>(Ty);
    unsigned NumElements = ATy->getNumElements();
    if (NumElements == 0) NumElements = 1;
    // Arrays are wrapped in structs to allow them to have normal
    // value semantics (avoiding the array "decay").
    Out << NameSoFar << " { ";
    printType(Out, ATy->getElementType(), false,
              "array[" + utostr(NumElements) + "]");
    return Out << "; }";
  }

  case Type::OpaqueTyID: {
    std::string TyName = "struct opaque_" + itostr(OpaqueCounter++);
    assert(TypeNames.find(Ty) == TypeNames.end());
    TypeNames[Ty] = TyName;
    return Out << TyName << ' ' << NameSoFar;
  }
  default:
    llvm_unreachable("Unhandled case in getTypeProps!");
  }

  return Out;
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
  } else {
    Out << '{';
    if (CPA->getNumOperands()) {
      Out << ' ';
      printConstant(cast<Constant>(CPA->getOperand(0)), Static);
      for (unsigned i = 1, e = CPA->getNumOperands(); i != e; ++i) {
        Out << ", ";
        printConstant(cast<Constant>(CPA->getOperand(i)), Static);
      }
    }
    Out << " }";
  }
}

void JsWriter::printConstantVector(ConstantVector *CP, bool Static) {
  Out << '{';
  if (CP->getNumOperands()) {
    Out << ' ';
    printConstant(cast<Constant>(CP->getOperand(0)), Static);
    for (unsigned i = 1, e = CP->getNumOperands(); i != e; ++i) {
      Out << ", ";
      printConstant(cast<Constant>(CP->getOperand(i)), Static);
    }
  }
  Out << " }";
}

// isFPCSafeToPrint - Returns true if we may assume that CFP may be written out
// textually as a double (rather than as a reference to a stack-allocated
// variable). We decide this by converting CFP to a string and back into a
// double, and then checking whether the conversion results in a bit-equal
// double to the original value of CFP. This depends on us and the target C
// compiler agreeing on the conversion process (which is pretty likely since we
// only deal in IEEE FP).
//
static bool isFPCSafeToPrint(const ConstantFP *CFP) {
  bool ignored;
  // Do long doubles in hex for now.
  if (CFP->getType() != Type::getFloatTy(CFP->getContext()) &&
      CFP->getType() != Type::getDoubleTy(CFP->getContext()))
    return false;
  APFloat APF = APFloat(CFP->getValueAPF());  // copy
  if (CFP->getType() == Type::getFloatTy(CFP->getContext()))
    APF.convert(APFloat::IEEEdouble, APFloat::rmNearestTiesToEven, &ignored);
#if HAVE_PRINTF_A && ENABLE_CBE_PRINTF_A
  char Buffer[100];
  sprintf(Buffer, "%a", APF.convertToDouble());
  if (!strncmp(Buffer, "0x", 2) ||
      !strncmp(Buffer, "-0x", 3) ||
      !strncmp(Buffer, "+0x", 3))
    return APF.bitwiseIsEqual(APFloat(atof(Buffer)));
  return false;
#else
  std::string StrVal = ftostr(APF);

  while (StrVal[0] == ' ')
    StrVal.erase(StrVal.begin());

  // Check to make sure that the stringized number is not some string like "Inf"
  // or NaN.  Check that the string matches the "[-+]?[0-9]" regex.
  if ((StrVal[0] >= '0' && StrVal[0] <= '9') ||
      ((StrVal[0] == '-' || StrVal[0] == '+') &&
       (StrVal[1] >= '0' && StrVal[1] <= '9')))
    // Reparse stringized version!
    return APF.bitwiseIsEqual(APFloat(atof(StrVal.c_str())));
  return false;
#endif
}

/// Print out the casting for a cast operation. This does the double casting
/// necessary for conversion to the destination type, if necessary. 
/// @brief Print a cast
void JsWriter::printCast(unsigned opc, const Type *SrcTy, const Type *DstTy) {
  // Print the destination type cast
  switch (opc) {
    case Instruction::UIToFP:
    case Instruction::SIToFP:
    case Instruction::IntToPtr:
    case Instruction::Trunc:
    case Instruction::BitCast:
    case Instruction::FPExt:
    case Instruction::FPTrunc: // For these the DstTy sign doesn't matter
      Out << '(';
      printType(Out, DstTy);
      Out << ')';
      break;
    case Instruction::ZExt:
    case Instruction::PtrToInt:
    case Instruction::FPToUI: // For these, make sure we get an unsigned dest
      Out << '(';
      printSimpleType(Out, DstTy, false);
      Out << ')';
      break;
    case Instruction::SExt: 
    case Instruction::FPToSI: // For these, make sure we get a signed dest
      Out << '(';
      printSimpleType(Out, DstTy, true);
      Out << ')';
      break;
    default:
      llvm_unreachable("Invalid cast opcode");
  }

  // Print the source type cast
  switch (opc) {
    case Instruction::UIToFP:
    case Instruction::ZExt:
      Out << '(';
      printSimpleType(Out, SrcTy, false);
      Out << ')';
      break;
    case Instruction::SIToFP:
    case Instruction::SExt:
      Out << '(';
      printSimpleType(Out, SrcTy, true); 
      Out << ')';
      break;
    case Instruction::IntToPtr:
    case Instruction::PtrToInt:
      // Avoid "cast to pointer from integer of different size" warnings
      Out << "(unsigned long)";
      break;
    case Instruction::Trunc:
    case Instruction::BitCast:
    case Instruction::FPExt:
    case Instruction::FPTrunc:
    case Instruction::FPToSI:
    case Instruction::FPToUI:
      break; // These don't need a source cast.
    default:
      llvm_unreachable("Invalid cast opcode");
      break;
  }
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
      Out << "(";
      printCast(CE->getOpcode(), CE->getOperand(0)->getType(), CE->getType());
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
      Out << ')';
      return;

    case Instruction::GetElementPtr:
      Out << "(";
      printGEPExpression(CE->getOperand(0), gep_type_begin(CPV),
                         gep_type_end(CPV), Static);
      Out << ")";
      return;
    case Instruction::Select:
      Out << '(';
      printConstant(CE->getOperand(0), Static);
      Out << '?';
      printConstant(CE->getOperand(1), Static);
      Out << ':';
      printConstant(CE->getOperand(2), Static);
      Out << ')';
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
      Out << '(';
      bool NeedsClosingParens = printConstExprCast(CE, Static); 
      printConstantWithCast(CE->getOperand(0), CE->getOpcode());
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
      case Instruction::LShr:
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
      printConstantWithCast(CE->getOperand(1), CE->getOpcode());
      if (NeedsClosingParens)
        Out << "))";
      Out << ')';
      return;
    }
    case Instruction::FCmp: {
      Out << '('; 
      bool NeedsClosingParens = printConstExprCast(CE, Static); 
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
        printConstantWithCast(CE->getOperand(0), CE->getOpcode());
        Out << ", ";
        printConstantWithCast(CE->getOperand(1), CE->getOpcode());
        Out << ")";
      }
      if (NeedsClosingParens)
        Out << "))";
      Out << ')';
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
    Out << "((";
    printType(Out, CPV->getType()); // sign doesn't matter
    Out << ")/*UNDEF*/";
    if (!CPV->getType()->isVectorTy()) {
      Out << "0)";
    } else {
      Out << "{})";
    }
    return;
  }

  if (ConstantInt *CI = dyn_cast<ConstantInt>(CPV)) {
    const Type* Ty = CI->getType();
    if (Ty == Type::getInt1Ty(CPV->getContext()))
      Out << (CI->getZExtValue() ? '1' : '0');
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
  case Type::DoubleTyID: 
  case Type::X86_FP80TyID:
  case Type::PPC_FP128TyID:
  case Type::FP128TyID: {
    ConstantFP *FPC = cast<ConstantFP>(CPV);
    std::map<const ConstantFP*, unsigned>::iterator I = FPConstantMap.find(FPC);
    if (I != FPConstantMap.end()) {
      // Because of FP precision problems we must load from a stack allocated
      // value that holds the value in hex.
      Out << "(*(" << (FPC->getType() == Type::getFloatTy(CPV->getContext()) ?
                       "float" : 
                       FPC->getType() == Type::getDoubleTy(CPV->getContext()) ? 
                       "double" :
                       "long double")
          << "*)&FPConstant" << I->second << ')';
    } else {
      double V;
      if (FPC->getType() == Type::getFloatTy(CPV->getContext()))
        V = FPC->getValueAPF().convertToFloat();
      else if (FPC->getType() == Type::getDoubleTy(CPV->getContext()))
        V = FPC->getValueAPF().convertToDouble();
      else {
        // Long double.  Convert the number to double, discarding precision.
        // This is not awesome, but it at least makes the CBE output somewhat
        // useful.
        APFloat Tmp = FPC->getValueAPF();
        bool LosesInfo;
        Tmp.convert(APFloat::IEEEdouble, APFloat::rmTowardZero, &LosesInfo);
        V = Tmp.convertToDouble();
      }
      
      if (IsNAN(V)) {
        // The value is NaN

        // FIXME the actual NaN bits should be emitted.
        // The prefix for a quiet NaN is 0x7FF8. For a signalling NaN,
        // it's 0x7ff4.
        const unsigned long QuietNaN = 0x7ff8UL;
        //const unsigned long SignalNaN = 0x7ff4UL;

        // We need to grab the first part of the FP #
        char Buffer[100];

        uint64_t ll = DoubleToBits(V);
        sprintf(Buffer, "0x%llx", static_cast<long long>(ll));

        std::string Num(&Buffer[0], &Buffer[6]);
        unsigned long Val = strtoul(Num.c_str(), 0, 16);

        if (FPC->getType() == Type::getFloatTy(FPC->getContext()))
          Out << "LLVM_NAN" << (Val == QuietNaN ? "" : "S") << "F(\""
              << Buffer << "\") /*nan*/ ";
        else
          Out << "LLVM_NAN" << (Val == QuietNaN ? "" : "S") << "(\""
              << Buffer << "\") /*nan*/ ";
      } else if (IsInf(V)) {
        // The value is Inf
        if (V < 0) Out << '-';
        Out << "LLVM_INF" <<
            (FPC->getType() == Type::getFloatTy(FPC->getContext()) ? "F" : "")
            << " /*inf*/ ";
      } else {
        std::string Num;
#if HAVE_PRINTF_A && ENABLE_CBE_PRINTF_A
        // Print out the constant as a floating point number.
        char Buffer[100];
        sprintf(Buffer, "%a", V);
        Num = Buffer;
#else
        Num = ftostr(FPC->getValueAPF());
#endif
       Out << Num;
      }
    }
    break;
  }

  case Type::ArrayTyID:
    // Use C99 compound expression literal initializer syntax.
    if (!Static) {
      Out << "(";
      printType(Out, CPV->getType());
      Out << ")";
    }
    Out << "["; // wrap arrays in an array to simulate pointers
    if (ConstantArray *CA = dyn_cast<ConstantArray>(CPV)) {
      printConstantArray(CA, Static);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const ArrayType *AT = cast<ArrayType>(CPV->getType());
      Out << '{';
      if (AT->getNumElements()) {
        Out << ' ';
        Constant *CZ = Constant::getNullValue(AT->getElementType());
        printConstant(CZ, Static);
        for (unsigned i = 1, e = AT->getNumElements(); i != e; ++i) {
          Out << ", ";
          printConstant(CZ, Static);
        }
      }
      Out << " }";
    }
    Out << "]"; // wrap arrays in an array to simulate pointers
    break;

  case Type::VectorTyID:
    // Use C99 compound expression literal initializer syntax.
    if (!Static) {
      Out << "(";
      printType(Out, CPV->getType());
      Out << ")";
    }
    if (ConstantVector *CV = dyn_cast<ConstantVector>(CPV)) {
      printConstantVector(CV, Static);
    } else {
      assert(isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV));
      const VectorType *VT = cast<VectorType>(CPV->getType());
      Out << "{ ";
      Constant *CZ = Constant::getNullValue(VT->getElementType());
      printConstant(CZ, Static);
      for (unsigned i = 1, e = VT->getNumElements(); i != e; ++i) {
        Out << ", ";
        printConstant(CZ, Static);
      }
      Out << " }";
    }
    break;

  case Type::StructTyID:
    // Use C99 compound expression literal initializer syntax.
    if (!Static) {
      Out << "(";
      printType(Out, CPV->getType());
      Out << ")";
    }
    if (isa<ConstantAggregateZero>(CPV) || isa<UndefValue>(CPV)) {
      const StructType *ST = cast<StructType>(CPV->getType());
      Out << '{';
      if (ST->getNumElements()) {
        Out << ' ';
        printConstant(Constant::getNullValue(ST->getElementType(0)), Static);
        for (unsigned i = 1, e = ST->getNumElements(); i != e; ++i) {
          Out << ", ";
          printConstant(Constant::getNullValue(ST->getElementType(i)), Static);
        }
      }
      Out << " }";
    } else {
      Out << '{';
      if (CPV->getNumOperands()) {
        Out << ' ';
        printConstant(cast<Constant>(CPV->getOperand(0)), Static);
        for (unsigned i = 1, e = CPV->getNumOperands(); i != e; ++i) {
          Out << ", ";
          printConstant(cast<Constant>(CPV->getOperand(i)), Static);
        }
      }
      Out << " }";
    }
    break;

  case Type::PointerTyID:
    if (isa<ConstantPointerNull>(CPV)) {
      Out << "((";
      printType(Out, CPV->getType()); // sign doesn't matter
      Out << ")/*NULL*/0)";
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
void JsWriter::printConstant(Constant *CPV, bool Static) {
  printConstant(CPV, Static, Out);
}

// Some constant expressions need to be casted back to the original types
// because their operands were casted to the expected type. This function takes
// care of detecting that case and printing the cast for the ConstantExpr.
bool JsWriter::printConstExprCast(const ConstantExpr* CE, bool Static) {
  bool NeedsExplicitCast = false;
  const Type *Ty = CE->getOperand(0)->getType();
  bool TypeIsSigned = false;
  switch (CE->getOpcode()) {
  case Instruction::Add:
  case Instruction::Sub:
  case Instruction::Mul:
    // We need to cast integer arithmetic so that it is always performed
    // as unsigned, to avoid undefined behavior on overflow.
  case Instruction::LShr:
  case Instruction::URem: 
  case Instruction::UDiv: NeedsExplicitCast = true; break;
  case Instruction::AShr:
  case Instruction::SRem: 
  case Instruction::SDiv: NeedsExplicitCast = true; TypeIsSigned = true; break;
  case Instruction::SExt:
    Ty = CE->getType();
    NeedsExplicitCast = true;
    TypeIsSigned = true;
    break;
  case Instruction::ZExt:
  case Instruction::Trunc:
  case Instruction::FPTrunc:
  case Instruction::FPExt:
  case Instruction::UIToFP:
  case Instruction::SIToFP:
  case Instruction::FPToUI:
  case Instruction::FPToSI:
  case Instruction::PtrToInt:
  case Instruction::IntToPtr:
  case Instruction::BitCast:
    Ty = CE->getType();
    NeedsExplicitCast = true;
    break;
  default: break;
  }
  if (NeedsExplicitCast) {
    Out << "((";
    if (Ty->isIntegerTy() && Ty != Type::getInt1Ty(Ty->getContext()))
      printSimpleType(Out, Ty, TypeIsSigned);
    else
      printType(Out, Ty); // not integer, sign doesn't matter
    Out << ")(";
  }
  return NeedsExplicitCast;
}

//  Print a constant assuming that it is the operand for a given Opcode. The
//  opcodes that care about sign need to cast their operands to the expected
//  type before the operation proceeds. This function does the casting.
void JsWriter::printConstantWithCast(Constant* CPV, unsigned Opcode) {

  // Extract the operand's type, we'll need it.
  const Type* OpTy = CPV->getType();

  // Indicate whether to do the cast or not.
  bool shouldCast = false;
  bool typeIsSigned = false;

  // Based on the Opcode for which this Constant is being written, determine
  // the new type to which the operand should be casted by setting the value
  // of OpTy. If we change OpTy, also set shouldCast to true so it gets
  // casted below.
  switch (Opcode) {
    default:
      // for most instructions, it doesn't matter
      break; 
    case Instruction::Add:
    case Instruction::Sub:
    case Instruction::Mul:
      // We need to cast integer arithmetic so that it is always performed
      // as unsigned, to avoid undefined behavior on overflow.
    case Instruction::LShr:
    case Instruction::UDiv:
    case Instruction::URem:
      shouldCast = true;
      break;
    case Instruction::AShr:
    case Instruction::SDiv:
    case Instruction::SRem:
      shouldCast = true;
      typeIsSigned = true;
      break;
  }

  // Write out the casted constant if we should, otherwise just write the
  // operand.
  if (shouldCast) {
    Out << "((";
    printSimpleType(Out, OpTy, typeIsSigned);
    Out << ")";
    printConstant(CPV, false);
    Out << ")";
  } else 
    printConstant(CPV, false);
}

std::string JsWriter::GetValueName(const Value *Operand) {
  // Mangle globals with the standard mangler interface for LLC compatibility.
  if (const GlobalValue *GV = dyn_cast<GlobalValue>(Operand)) {
    SmallString<128> Str;
    Mang->getNameWithPrefix(Str, GV, false);
    return CBEMangle(Str.str().str());
  }
    
  std::string Name = Operand->getName();
    
  if (Name.empty()) { // Assign unique names to local temporaries.
    unsigned &No = AnonValueNumbers[Operand];
    if (No == 0)
      No = ++NextAnonValueNumber;
    Name = "tmp__" + utostr(No);
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

  return "llvm_cbe_" + VarName;
}

/// writeInstComputationInline - Emit the computation for the specified
/// instruction inline, with no destination provided.
void JsWriter::writeInstComputationInline(Instruction &I) {
  // We can't currently support integer types other than 1, 8, 16, 32, 64.
  // Validate this.
  const Type *Ty = I.getType();
  if (Ty->isIntegerTy() && (Ty!=Type::getInt1Ty(I.getContext()) &&
        Ty!=Type::getInt8Ty(I.getContext()) && 
        Ty!=Type::getInt16Ty(I.getContext()) &&
        Ty!=Type::getInt32Ty(I.getContext()) &&
        Ty!=Type::getInt64Ty(I.getContext()))) {
      report_fatal_error("The Javascript backend does not currently support integer "
                        "types of widths other than 1, 8, 16, 32, 64.\n"
                        "This is being tracked as PR 4158.");
  }

  // If this is a non-trivial bool computation, make sure to truncate down to
  // a 1 bit value.  This is important because we want "add i1 x, y" to return
  // "0" when x and y are true, not "2" for example.
  bool NeedBoolTrunc = false;
  if (I.getType() == Type::getInt1Ty(I.getContext()) &&
      !isa<ICmpInst>(I) && !isa<FCmpInst>(I))
    NeedBoolTrunc = true;
  
  if (NeedBoolTrunc)
    Out << "((";
  
  visit(I);
  
  if (NeedBoolTrunc)
    Out << ")&1)";
}


void JsWriter::writeOperandInternal(Value *Operand, bool Static) {
  if (Instruction *I = dyn_cast<Instruction>(Operand))
    // Should we inline this instruction to build a tree?
    if (isInlinableInst(*I) && !isDirectAlloca(I)) {
      Out << '(';
      writeInstComputationInline(*I);
      Out << ')';
      return;
    }

  Constant* CPV = dyn_cast<Constant>(Operand);

  if (CPV && !isa<GlobalValue>(CPV))
    printConstant(CPV, Static);
  else
    Out << GetValueName(Operand);
}

void JsWriter::writeOperand(Value *Operand, bool Static) {
  writeOperandInternal(Operand, Static);
}

// Some instructions need to have their result value casted back to the 
// original types because their operands were casted to the expected type. 
// This function takes care of detecting that case and printing the cast 
// for the Instruction.
bool JsWriter::writeInstructionCast(const Instruction &I) {
  const Type *Ty = I.getOperand(0)->getType();
  switch (I.getOpcode()) {
  case Instruction::Add:
  case Instruction::Sub:
  case Instruction::Mul:
    // We need to cast integer arithmetic so that it is always performed
    // as unsigned, to avoid undefined behavior on overflow.
  case Instruction::LShr:
  case Instruction::URem: 
  case Instruction::UDiv: 
    Out << "((";
    printSimpleType(Out, Ty, false);
    Out << ")(";
    return true;
  case Instruction::AShr:
  case Instruction::SRem: 
  case Instruction::SDiv: 
    Out << "((";
    printSimpleType(Out, Ty, true);
    Out << ")(";
    return true;
  default: break;
  }
  return false;
}

// Write the operand with a cast to another type based on the Opcode being used.
// This will be used in cases where an instruction has specific type
// requirements (usually signedness) for its operands. 
void JsWriter::writeOperandWithCast(Value* Operand, unsigned Opcode) {

  // Extract the operand's type, we'll need it.
  const Type* OpTy = Operand->getType();

  // Indicate whether to do the cast or not.
  bool shouldCast = false;

  // Indicate whether the cast should be to a signed type or not.
  bool castIsSigned = false;

  // Based on the Opcode for which this Operand is being written, determine
  // the new type to which the operand should be casted by setting the value
  // of OpTy. If we change OpTy, also set shouldCast to true.
  switch (Opcode) {
    default:
      // for most instructions, it doesn't matter
      break; 
    case Instruction::Add:
    case Instruction::Sub:
    case Instruction::Mul:
      // We need to cast integer arithmetic so that it is always performed
      // as unsigned, to avoid undefined behavior on overflow.
    case Instruction::LShr:
    case Instruction::UDiv:
    case Instruction::URem: // Cast to unsigned first
      shouldCast = true;
      castIsSigned = false;
      break;
    case Instruction::GetElementPtr:
    case Instruction::AShr:
    case Instruction::SDiv:
    case Instruction::SRem: // Cast to signed first
      shouldCast = true;
      castIsSigned = true;
      break;
  }

  // Write out the casted operand if we should, otherwise just write the
  // operand.
  if (shouldCast) {
    Out << "((";
    printSimpleType(Out, OpTy, castIsSigned);
    Out << ")";
    writeOperand(Operand);
    Out << ")";
  } else 
    writeOperand(Operand);
}

// Write the operand with a cast to another type based on the icmp predicate 
// being used. 
void JsWriter::writeOperandWithCast(Value* Operand, const ICmpInst &Cmp) {
  // This has to do a cast to ensure the operand has the right signedness. 
  // Also, if the operand is a pointer, we make sure to cast to an integer when
  // doing the comparison both for signedness and so that the C compiler doesn't
  // optimize things like "p < NULL" to false (p may contain an integer value
  // f.e.).
  bool shouldCast = Cmp.isRelational();

  // Write out the casted operand if we should, otherwise just write the
  // operand.
  if (!shouldCast) {
    writeOperand(Operand);
    return;
  }
  
  // Should this be a signed comparison?  If so, convert to signed.
  bool castIsSigned = Cmp.isSigned();

  // If the operand was a pointer, convert to a large integer type.
  const Type* OpTy = Operand->getType();
  if (OpTy->isPointerTy())
    OpTy = TD->getIntPtrType(Operand->getContext());
  
  Out << "((";
  printSimpleType(Out, OpTy, castIsSigned);
  Out << ")";
  writeOperand(Operand);
  Out << ")";
}

/// FindStaticTors - Given a static ctor/dtor list, unpack its contents into
/// the StaticTors set.
static void FindStaticTors(GlobalVariable *GV, std::set<Function*> &StaticTors){
  ConstantArray *InitList = dyn_cast<ConstantArray>(GV->getInitializer());
  if (!InitList) return;
  
  for (unsigned i = 0, e = InitList->getNumOperands(); i != e; ++i)
    if (ConstantStruct *CS = dyn_cast<ConstantStruct>(InitList->getOperand(i))){
      if (CS->getNumOperands() != 2) return;  // Not array of 2-element structs.
      
      if (CS->getOperand(1)->isNullValue())
        return;  // Found a null terminator, exit printing.
      Constant *FP = CS->getOperand(1);
      if (ConstantExpr *CE = dyn_cast<ConstantExpr>(FP))
        if (CE->isCast())
          FP = CE->getOperand(0);
      if (Function *F = dyn_cast<Function>(FP))
        StaticTors.insert(F);
    }
}

enum SpecialGlobalClass {
  NotSpecial = 0,
  GlobalCtors, GlobalDtors,
  NotPrinted
};

/// getGlobalVariableClass - If this is a global that is specially recognized
/// by LLVM, return a code that indicates how we should handle it.
static SpecialGlobalClass getGlobalVariableClass(const GlobalVariable *GV) {
  // If this is a global ctors/dtors list, handle it now.
  if (GV->hasAppendingLinkage() && GV->use_empty()) {
    if (GV->getName() == "llvm.global_ctors")
      return GlobalCtors;
    else if (GV->getName() == "llvm.global_dtors")
      return GlobalDtors;
  }
  
  // Otherwise, if it is other metadata, don't print it.  This catches things
  // like debug information.
  if (GV->getSection() == "llvm.metadata")
    return NotPrinted;
  
  return NotSpecial;
}

// PrintEscapedString - Print each character of the specified string, escaping
// it if it is not printable or if it is an escape char.
static void PrintEscapedString(const char *Str, unsigned Length,
                               raw_ostream &Out) {
  for (unsigned i = 0; i != Length; ++i) {
    unsigned char C = Str[i];
    if (isprint(C) && C != '\\' && C != '"')
      Out << C;
    else if (C == '\\')
      Out << "\\\\";
    else if (C == '\"')
      Out << "\\\"";
    else if (C == '\t')
      Out << "\\t";
    else
      Out << "\\x" << hexdigit(C >> 4) << hexdigit(C & 0x0F);
  }
}

// PrintEscapedString - Print each character of the specified string, escaping
// it if it is not printable or if it is an escape char.
static void PrintEscapedString(const std::string &Str, raw_ostream &Out) {
  PrintEscapedString(Str.c_str(), Str.size(), Out);
}

bool JsWriter::doInitialization(Module &M) {
  FunctionPass::doInitialization(M);
  
  // Initialize
  TheModule = &M;

  TD = new TargetData(&M);
  IL = new IntrinsicLowering(*TD);
  IL->AddPrototypes(M);

  TAsm = new CBEMCAsmInfo();
  TCtx = new MCContext(*TAsm);
  Mang = new Mangler(*TCtx, *TD);

  if(M.getFunction("main")) {
    Out << "<!doctype html>\n";
    Out << "<html>\n";
    Out << "<head>\n";
    Out << "<title>" << M.getModuleIdentifier() << "</title>\n";
    Out << "</head>\n";
    Out << "<body>\n";
    Out << "<script>\n";
  }
  Out << "(function($w) {\n";
  Out << "$w[\"" << M.getModuleIdentifier() << "\"] = {};\n";
  Out << "var _ = $w[\"" << M.getModuleIdentifier() << "\"];\n";

  // Keep track of which functions are static ctors/dtors so they can have
  // an attribute added to their prototypes.
  std::set<Function*> StaticCtors, StaticDtors;
  for (Module::global_iterator I = M.global_begin(), E = M.global_end();
       I != E; ++I) {
    switch (getGlobalVariableClass(I)) {
    default: break;
    case GlobalCtors:
      FindStaticTors(I, StaticCtors);
      break;
    case GlobalDtors:
      FindStaticTors(I, StaticDtors);
      break;
    }
  }
  
  // First output all the declarations for the program, because C requires
  // Functions & globals to be declared before they are used.
  //
  if (!M.getModuleInlineAsm().empty()) {
    Out << "/* Module asm statements */\n"
        << "asm(";

    // Split the string into lines, to make it easier to read the .ll file.
    std::string Asm = M.getModuleInlineAsm();
    size_t CurPos = 0;
    size_t NewLine = Asm.find_first_of('\n', CurPos);
    while (NewLine != std::string::npos) {
      // We found a newline, print the portion of the asm string from the
      // last newline up to this newline.
      Out << "\"";
      PrintEscapedString(std::string(Asm.begin()+CurPos, Asm.begin()+NewLine),
                         Out);
      Out << "\\n\"\n";
      CurPos = NewLine+1;
      NewLine = Asm.find_first_of('\n', CurPos);
    }
    Out << "\"";
    PrintEscapedString(std::string(Asm.begin()+CurPos, Asm.end()), Out);
    Out << "\");\n"
        << "/* End Module asm statements */\n";
  }

  // Loop over the symbol table, emitting all named constants...
  printModuleTypes(M.getTypeSymbolTable());

  // Global variable declarations...
  if (!M.global_empty()) {
    Out << "\n/* External Global Variable Declarations */\n";
    for (Module::global_iterator I = M.global_begin(), E = M.global_end();
         I != E; ++I) {

      if (I->hasExternalLinkage() || I->hasExternalWeakLinkage() || 
          I->hasCommonLinkage())
        Out << "extern ";
      else if (I->hasDLLImportLinkage())
        Out << "__declspec(dllimport) ";
      else
        continue; // Internal Global

      // Thread Local Storage
      if (I->isThreadLocal())
        Out << "__thread ";

      printType(Out, I->getType()->getElementType(), false, GetValueName(I));

      if (I->hasExternalWeakLinkage())
         Out << " __EXTERNAL_WEAK__";
      Out << ";\n";
    }
  }

  // Output the module-level locals
  if (!M.global_empty()) {
    Module::global_iterator I = M.global_begin(), E = M.global_end();
    bool Found = false;
    for(;I != E; ++I) {
      if (!I->isDeclaration() && !getGlobalVariableClass(I) && (I->hasLocalLinkage() || I->hasHiddenVisibility())) {
	Out << "\n/* Module Local Variables */\n";
	Out << "var " << GetValueName(I) << " = ";
	writeOperand(I->getInitializer(), true);
	Found = true;
	continue;
      }
    }
    for (; I != E; ++I) {
      if (!I->isDeclaration() && !getGlobalVariableClass(I) && (I->hasLocalLinkage() || I->hasHiddenVisibility())) {      
	Out << ", " << GetValueName(I) << " = ";
	writeOperand(I->getInitializer(), true);
	continue;
      }
    }
    if(Found) {
      Out << ";\n";
    }
  }
  
  // Output the module members
  if (!M.global_empty()) {
    Module::global_iterator I = M.global_begin(), E = M.global_end();
    bool Found = false;
    for(;I != E; ++I) {
      if (!I->isDeclaration() && !getGlobalVariableClass(I) && !(I->hasLocalLinkage() || I->hasHiddenVisibility())) {
	if (I->hasDLLImportLinkage()) {
	  // can't import from DLLs
	  llvm_unreachable(0);
	}
	if (I->hasDLLExportLinkage()) {
	  // can't export to DLLs
	  llvm_unreachable(0);
	}
        if (I->isThreadLocal()) {
	  // no support for thread locals
	  llvm_unreachable(0);
	}
	Out << "\n/* Module Members */\n";

	Out << "_." << GetValueName(I) << " = ";
	writeOperand(I->getInitializer(), true);
	Found = true;
	continue;
      }
    }
    for (; I != E; ++I) {
      if (!I->isDeclaration() && !getGlobalVariableClass(I) && !(I->hasLocalLinkage() || I->hasHiddenVisibility())) {
	if (I->hasDLLImportLinkage()) {
	  // can't import from DLLs
	  llvm_unreachable(0);
	}
	if (I->hasDLLExportLinkage()) {
	  // can't export to DLLs
	  llvm_unreachable(0);
	}
        if (I->isThreadLocal()) {
	  // no support for thread locals
	  llvm_unreachable(0);
	}

	Out << ", _." << GetValueName(I) << " = ";
	writeOperand(I->getInitializer(), true);
	continue;
      }
    }
    if(Found) {
      Out << ";\n";
    }
  }

  if (!M.empty()) {
    Out << "\n/* Function Bodies */\n";
  }
  return false;
}


/// Output all floating point constants that cannot be printed accurately...
void JsWriter::printFloatingPointConstants(Function &F) {
  // Scan the module for floating point constants.  If any FP constant is used
  // in the function, we want to redirect it here so that we do not depend on
  // the precision of the printed form, unless the printed form preserves
  // precision.
  //
  for (constant_iterator I = constant_begin(&F), E = constant_end(&F);
       I != E; ++I)
    printFloatingPointConstants(*I);

  Out << '\n';
}

void JsWriter::printFloatingPointConstants(const Constant *C) {
  // If this is a constant expression, recursively check for constant fp values.
  if (const ConstantExpr *CE = dyn_cast<ConstantExpr>(C)) {
    for (unsigned i = 0, e = CE->getNumOperands(); i != e; ++i)
      printFloatingPointConstants(CE->getOperand(i));
    return;
  }
    
  // Otherwise, check for a FP constant that we need to print.
  const ConstantFP *FPC = dyn_cast<ConstantFP>(C);
  if (FPC == 0 ||
      // Do not put in FPConstantMap if safe.
      isFPCSafeToPrint(FPC) ||
      // Already printed this constant?
      FPConstantMap.count(FPC))
    return;

  FPConstantMap[FPC] = FPCounter;  // Number the FP constants
  
  if (FPC->getType() == Type::getDoubleTy(FPC->getContext())) {
    double Val = FPC->getValueAPF().convertToDouble();
    uint64_t i = FPC->getValueAPF().bitcastToAPInt().getZExtValue();
    Out << "static const ConstantDoubleTy FPConstant" << FPCounter++
    << " = 0x" << utohexstr(i)
    << "ULL;    /* " << Val << " */\n";
  } else if (FPC->getType() == Type::getFloatTy(FPC->getContext())) {
    float Val = FPC->getValueAPF().convertToFloat();
    uint32_t i = (uint32_t)FPC->getValueAPF().bitcastToAPInt().
    getZExtValue();
    Out << "static const ConstantFloatTy FPConstant" << FPCounter++
    << " = 0x" << utohexstr(i)
    << "U;    /* " << Val << " */\n";
  } else if (FPC->getType() == Type::getX86_FP80Ty(FPC->getContext())) {
    // api needed to prevent premature destruction
    APInt api = FPC->getValueAPF().bitcastToAPInt();
    const uint64_t *p = api.getRawData();
    Out << "static const ConstantFP80Ty FPConstant" << FPCounter++
    << " = { 0x" << utohexstr(p[0]) 
    << "ULL, 0x" << utohexstr((uint16_t)p[1]) << ",{0,0,0}"
    << "}; /* Long double constant */\n";
  } else if (FPC->getType() == Type::getPPC_FP128Ty(FPC->getContext()) ||
             FPC->getType() == Type::getFP128Ty(FPC->getContext())) {
    APInt api = FPC->getValueAPF().bitcastToAPInt();
    const uint64_t *p = api.getRawData();
    Out << "static const ConstantFP128Ty FPConstant" << FPCounter++
    << " = { 0x"
    << utohexstr(p[0]) << ", 0x" << utohexstr(p[1])
    << "}; /* Long double constant */\n";
    
  } else {
    llvm_unreachable("Unknown float type!");
  }
}



/// printSymbolTable - Run through symbol table looking for type names.  If a
/// type name is found, emit its declaration...
///
void JsWriter::printModuleTypes(const TypeSymbolTable &TST) {
  // No need to explicitly define types
}

// Push the struct onto the stack and recursively push all structs
// this one depends on.
//
// TODO:  Make this work properly with vector types
//
void JsWriter::printContainedStructs(const Type *Ty,
                                    std::set<const Type*> &StructPrinted) {
  // Don't walk through pointers.
  if (Ty->isPointerTy() || Ty->isPrimitiveType() || Ty->isIntegerTy())
    return;
  
  // Print all contained types first.
  for (Type::subtype_iterator I = Ty->subtype_begin(),
       E = Ty->subtype_end(); I != E; ++I)
    printContainedStructs(*I, StructPrinted);
  
  if (Ty->isStructTy() || Ty->isArrayTy()) {
    // Check to see if we have already printed this struct.
    if (StructPrinted.insert(Ty).second) {
      // Print structure type out.
      std::string Name = TypeNames[Ty];
      printType(Out, Ty, false, Name, true);
      Out << ";\n\n";
    }
  }
}

void JsWriter::printFunctionSignature(const Function *F, bool Prototype) {
  if(Prototype || F->isDeclaration()) {
    Out << "var " << GetValueName(F);
    return;
  } 
  Out << "_." << GetValueName(F) << " = function " << GetValueName(F) << "(";
  if(!F->arg_empty()) {
    // print out arguments
    Function::const_arg_iterator AI = F->arg_begin(), AE = F->arg_end();
    Out << GetValueName(AI);
    ++AI;
    for(; AI != AE; ++AI) {
      Out << ", " << GetValueName(AI);
    }
  }
  Out << ")";
}

static inline bool isFPIntBitCast(const Instruction &I) {
  if (!isa<BitCastInst>(I))
    return false;
  const Type *SrcTy = I.getOperand(0)->getType();
  const Type *DstTy = I.getType();
  return (SrcTy->isFloatingPointTy() && DstTy->isIntegerTy()) ||
         (DstTy->isFloatingPointTy() && SrcTy->isIntegerTy());
}

void JsWriter::printFunction(Function &F) {
  printFunctionSignature(&F, false);
  Out << " {\n";
  
  bool PrintedVar = false;
  
  // print local variable information for the function
  for (inst_iterator I = inst_begin(&F), E = inst_end(&F); I != E; ++I) {
    if (isDirectAlloca(&*I) || (I->getType() != Type::getVoidTy(F.getContext()) && 
						       !isInlinableInst(*I))) {
      if(!PrintedVar) {
	Out << "  var " << GetValueName(&*I);
	PrintedVar = true;
      } else {
	Out << ", " << GetValueName(&*I);
      }
      if(isa<PHINode>(*I)) {
	Out << ", " << GetValueName(&*I) << "_";
      }
    }
    // We need a temporary for the BitCast to use so it can pluck a value out
    // of a union to do the BitCast. This is separate from the need for a
    // variable to hold the result of the BitCast. 
    if (isFPIntBitCast(*I)) {
      Out << "  llvmBitCastUnion " << GetValueName(&*I)
          << "__BITCAST_TEMPORARY;\n";
      PrintedVar = true;
    }
  }

  if (PrintedVar) {
    Out << ";\n";
  }

  std::string cls = "";
  raw_string_ostream Closing(cls);

  // print the basic blocks
  Function::iterator BB = F.begin(), E = F.end();
  if(BB != E) {
    Out << "  var _ = '" << GetValueName(BB) << "'; /* jump variable */\n";
    Out << "  while(1) {\n";
    Out << "    switch(_) {\n";
    if (Loop *L = LI->getLoopFor(BB)) {
      if (L->getHeader() == BB && L->getParentLoop() == 0)
        printLoop(L);
    } else {
      printBasicBlock(BB);
    }
    
    Closing << "    }\n";
    Closing << "  }\n";
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

  Out << Closing.str();
  Out << "};\n";
  Out << "\n";
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
  Out << "      case '" << GetValueName(BB) << "':\n";
  // Output all of the instructions in the basic block...
  for (BasicBlock::iterator II = BB->begin(), E = --BB->end(); II != E;
       ++II) {
    if (!isInlinableInst(*II) && !isDirectAlloca(II)) {
      Out << "        ";
      if (II->getType() != Type::getVoidTy(BB->getContext()) &&
          !isInlineAsm(*II)) {
	outputLValue(II);
      }
      writeInstComputationInline(*II);
      Out << ";\n";
    }
  }

  // Don't emit prefix or suffix for the terminator.
  visit(*BB->getTerminator());
}


// Specific Instruction type classes... note that all of the casts are
// necessary because we use the instruction classes as opaque types...
//
void JsWriter::visitReturnInst(ReturnInst &I) {
  // If this is a struct return function, return the temporary struct.
  bool isStructReturn = I.getParent()->getParent()->hasStructRetAttr();

  if (isStructReturn) {
    Out << "        return StructReturn;\n";
    return;
  }
  
  if (I.getNumOperands() > 1) {
    Out << "  return [\n";
    for (unsigned i = 0, e = I.getNumOperands(); i != e; ++i) {
      Out << "      ";
      writeOperand(I.getOperand(i));
      if (i != e - 1)
        Out << ",";
      Out << "\n";
    }
    Out << "    ];\n";
    Out << "  }\n";
    return;
  }

  Out << "        return";
  if (I.getNumOperands()) {
    Out << ' ';
    writeOperand(I.getOperand(0));
  }
  Out << ";\n";
}

void JsWriter::visitSwitchInst(SwitchInst &SI) {
  Out << "        switch (";
  writeOperand(SI.getOperand(0));
  Out << ") {\n          default:\n";
  printPHICopiesForSuccessor (SI.getParent(), SI.getDefaultDest(), 12);
  printBranchToBlock(SI.getParent(), SI.getDefaultDest(), 12);
  for (unsigned i = 2, e = SI.getNumOperands(); i != e; i += 2) {
    Out << "          case ";
    writeOperand(SI.getOperand(i));
    Out << ":\n";
    BasicBlock *Succ = cast<BasicBlock>(SI.getOperand(i+1));
    printPHICopiesForSuccessor (SI.getParent(), Succ, 12);
    printBranchToBlock(SI.getParent(), Succ, 12);
  }
  Out << "        }\n";
}

void JsWriter::visitIndirectBrInst(IndirectBrInst &IBI) {
  Out << "  goto *(void*)(";
  writeOperand(IBI.getOperand(0));
  Out << ");\n";
}

void JsWriter::visitUnreachableInst(UnreachableInst &I) {
  Out << "        throw 'unreachable code';\n";
}

void JsWriter::visitUnwindInst(UnwindInst &I) {
  Out << "        throw 'unwind';\n";
}

bool JsWriter::isGotoCodeNecessary(BasicBlock *From, BasicBlock *To) {
  /// FIXME: This should be reenabled, but loop reordering safe!!
  return true;

  if (llvm::next(Function::iterator(From)) != Function::iterator(To))
    return true;  // Not the direct successor, we need a goto.

  //isa<SwitchInst>(From->getTerminator())

  if (LI->getLoopFor(From) != LI->getLoopFor(To))
    return true;
  return false;
}

void JsWriter::printPHICopiesForSuccessor (BasicBlock *CurBlock,
                                          BasicBlock *Successor,
                                          unsigned Indent) {
  for (BasicBlock::iterator I = Successor->begin(); isa<PHINode>(I); ++I) {
    PHINode *PN = cast<PHINode>(I);
    // Now we have to do the printing.
    Value *IV = PN->getIncomingValueForBlock(CurBlock);
    if (!isa<UndefValue>(IV)) {
      Out << std::string(Indent, ' ');
      Out << "  " << GetValueName(I) << "_ = ";
      writeOperand(IV);
      Out << ";   /* for PHI node */\n";
    }
  }
}

void JsWriter::printBranchToBlock(BasicBlock *CurBB, BasicBlock *Succ,
				  unsigned Indent) {
  std::string Leading(Indent, ' ');
  if (isGotoCodeNecessary(CurBB, Succ)) {
    Out << Leading << "_ = '" << GetValueName(Succ) << "';\n";
    Out << Leading << "continue;\n";
  }
}

// Branch instruction printing - Avoid printing out a branch to a basic block
// that immediately succeeds the current one.
//
void JsWriter::visitBranchInst(BranchInst &I) {

  if (I.isConditional()) {
    if (isGotoCodeNecessary(I.getParent(), I.getSuccessor(0))) {
      Out << "        if (";
      writeOperand(I.getCondition());
      Out << ") {\n";

      printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0), 8);
      printBranchToBlock(I.getParent(), I.getSuccessor(0), 10);

      if (isGotoCodeNecessary(I.getParent(), I.getSuccessor(1))) {
        Out << "        } else {\n";
        printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(1), 8);
        printBranchToBlock(I.getParent(), I.getSuccessor(1), 10);
      }
    } else {
      // First goto not necessary, assume second one is...
      Out << "        if (!";
      writeOperand(I.getCondition());
      Out << ") {\n";

      printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(1), 8);
      printBranchToBlock(I.getParent(), I.getSuccessor(1), 10);
    }

    Out << "        }\n";
  } else {
    printPHICopiesForSuccessor (I.getParent(), I.getSuccessor(0), 6);
    printBranchToBlock(I.getParent(), I.getSuccessor(0), 8);
  }
  Out << "\n";
}

// PHI nodes get copied into temporary values at the end of predecessor basic
// blocks.  We now need to copy these temporary values into the REAL value for
// the PHI.
void JsWriter::visitPHINode(PHINode &I) {
  writeOperand(&I);
  Out << "_";
}


void JsWriter::visitBinaryOperator(Instruction &I) {
  // binary instructions, shift instructions, setCond instructions.
  assert(!I.getType()->isPointerTy());

  // We must cast the results of binary operations which might be promoted.
  bool needsCast = false;
  if ((I.getType() == Type::getInt8Ty(I.getContext())) ||
      (I.getType() == Type::getInt16Ty(I.getContext())) 
      || (I.getType() == Type::getFloatTy(I.getContext()))) {
    needsCast = true;
    Out << "((";
    printType(Out, I.getType(), false);
    Out << ")(";
  }

  // If this is a negation operation, print it out as such.  For FP, we don't
  // want to print "-0.0 - X".
  if (BinaryOperator::isNeg(&I)) {
    Out << "-(";
    writeOperand(BinaryOperator::getNegArgument(cast<BinaryOperator>(&I)));
    Out << ")";
  } else if (BinaryOperator::isFNeg(&I)) {
    Out << "-(";
    writeOperand(BinaryOperator::getFNegArgument(cast<BinaryOperator>(&I)));
    Out << ")";
  } else {

    // Certain instructions require the operand to be forced to a specific type
    // so we use writeOperandWithCast here instead of writeOperand. Similarly
    // below for operand 1
    writeOperand(I.getOperand(0), I.getOpcode());

    switch (I.getOpcode()) {
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
    case Instruction::And:  Out << " & "; break;
    case Instruction::Or:   Out << " | "; break;
    case Instruction::Xor:  Out << " ^ "; break;
    case Instruction::Shl : Out << " << "; break;
    case Instruction::LShr:
    case Instruction::AShr: Out << " >> "; break;
    default: 
#ifndef NDEBUG
       errs() << "Invalid operator type!" << I;
#endif
       llvm_unreachable(0);
    }

    writeOperand(I.getOperand(1), I.getOpcode());
  }

  if (needsCast) {
    Out << "))";
  }
}

void JsWriter::visitICmpInst(ICmpInst &I) {
  // We must cast the results of icmp which might be promoted.
  bool needsCast = false;

  // Write out the cast of the instruction's value back to the proper type
  // if necessary.
  bool NeedsClosingParens = writeInstructionCast(I);

  // Certain icmp predicate require the operand to be forced to a specific type
  // so we use writeOperandWithCast here instead of writeOperand. Similarly
  // below for operand 1
  writeOperandWithCast(I.getOperand(0), I);

  switch (I.getPredicate()) {
  case ICmpInst::ICMP_EQ:  Out << " == "; break;
  case ICmpInst::ICMP_NE:  Out << " != "; break;
  case ICmpInst::ICMP_ULE:
  case ICmpInst::ICMP_SLE: Out << " <= "; break;
  case ICmpInst::ICMP_UGE:
  case ICmpInst::ICMP_SGE: Out << " >= "; break;
  case ICmpInst::ICMP_ULT:
  case ICmpInst::ICMP_SLT: Out << " < "; break;
  case ICmpInst::ICMP_UGT:
  case ICmpInst::ICMP_SGT: Out << " > "; break;
  default:
#ifndef NDEBUG
    errs() << "Invalid icmp predicate!" << I; 
#endif
    llvm_unreachable(0);
  }

  writeOperandWithCast(I.getOperand(1), I);
  if (NeedsClosingParens)
    Out << "))";

  if (needsCast) {
    Out << "))";
  }
}

void JsWriter::visitFCmpInst(FCmpInst &I) {
  if (I.getPredicate() == FCmpInst::FCMP_FALSE) {
    Out << "0";
    return;
  }
  if (I.getPredicate() == FCmpInst::FCMP_TRUE) {
    Out << "1";
    return;
  }

  FCmpInst::Predicate Pred = I.getPredicate();
  bool done = false;
  while(!done) {
    Out << "(";
    switch (Pred) {
    default: llvm_unreachable("Illegal FCmp predicate");
    case FCmpInst::FCMP_ORD:
      Out << I.getOperand(0) << " == " << I.getOperand(0);
      Out << " && " << I.getOperand(1)<< " == " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_UNO:
      Out << I.getOperand(0)<< " != " << I.getOperand(0) << " || ";
      Out << I.getOperand(1) << " != " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_UEQ:
      Out << I.getOperand(0) << " == " << I.getOperand(1) << " || ";
      Pred = FCmpInst::FCMP_UNO;
      break;
    case FCmpInst::FCMP_UNE:
      Out << I.getOperand(0) << " != " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_ULT:
      Out << I.getOperand(0) <<" <  " << I.getOperand(1) << " || ";
      Pred = FCmpInst::FCMP_UNO;
      break;
    case FCmpInst::FCMP_ULE:
      Out << I.getOperand(0) << " >  " << I.getOperand(1) << " || ";
      Pred = FCmpInst::FCMP_UNO;
      break;
    case FCmpInst::FCMP_UGT:
      Out << I.getOperand(0) << " <= " << I.getOperand(1) << " || ";
      Pred = FCmpInst::FCMP_UNO;
      break;
    case FCmpInst::FCMP_UGE:
      Out << I.getOperand(0) << " >= " << I.getOperand(1) << " || ";
      Pred = FCmpInst::FCMP_UNO;
      break;
    case FCmpInst::FCMP_OEQ:
      Out << I.getOperand(0) << " == " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_ONE:
      Out << I.getOperand(0) << " != " << I.getOperand(1) << " && ";
      Pred = FCmpInst::FCMP_ORD;
      break;
    case FCmpInst::FCMP_OLT:
      Out << I.getOperand(0) << " <  " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_OLE:
      Out << I.getOperand(0) << " >  " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_OGT:
      Out << I.getOperand(0) << " <= " << I.getOperand(1);
      done = true;
      break;
    case FCmpInst::FCMP_OGE:
      Out << I.getOperand(0) << " >= " << I.getOperand(1);
      done = true;
      break;
    }
    Out << ")";
  }
}

static const char * getFloatBitCastField(const Type *Ty) {
  switch (Ty->getTypeID()) {
    default: llvm_unreachable("Invalid Type");
    case Type::FloatTyID:  return "Float";
    case Type::DoubleTyID: return "Double";
    case Type::IntegerTyID: {
      unsigned NumBits = cast<IntegerType>(Ty)->getBitWidth();
      if (NumBits <= 32)
        return "Int32";
      else
        return "Int64";
    }
  }
}

void JsWriter::visitCastInst(CastInst &I) {
  const Type *DstTy = I.getType();
  const Type *SrcTy = I.getOperand(0)->getType();
  if (isFPIntBitCast(I)) {
    Out << '(';
    // These int<->float and long<->double casts need to be handled specially
    Out << GetValueName(&I) << "__BITCAST_TEMPORARY." 
        << getFloatBitCastField(I.getOperand(0)->getType()) << " = ";
    writeOperand(I.getOperand(0));
    Out << ", " << GetValueName(&I) << "__BITCAST_TEMPORARY."
        << getFloatBitCastField(I.getType());
    Out << ')';
    return;
  }
  
  Out << '(';
  printCast(I.getOpcode(), SrcTy, DstTy);

  // Make a sext from i1 work by subtracting the i1 from 0 (an int).
  if (SrcTy == Type::getInt1Ty(I.getContext()) &&
      I.getOpcode() == Instruction::SExt)
    Out << "0-";
  
  writeOperand(I.getOperand(0));
    
  if (DstTy == Type::getInt1Ty(I.getContext()) && 
      (I.getOpcode() == Instruction::Trunc ||
       I.getOpcode() == Instruction::FPToUI ||
       I.getOpcode() == Instruction::FPToSI ||
       I.getOpcode() == Instruction::PtrToInt)) {
    // Make sure we really get a trunc to bool by anding the operand with 1 
    Out << "&1u";
  }
  Out << ')';
}

void JsWriter::visitSelectInst(SelectInst &I) {
  Out << "((";
  writeOperand(I.getCondition());
  Out << ") ? (";
  writeOperand(I.getTrueValue());
  Out << ") : (";
  writeOperand(I.getFalseValue());
  Out << "))";
}


void JsWriter::lowerIntrinsics(Function &F) {
  // This is used to keep track of intrinsics that get generated to a lowered
  // function. We must generate the prototypes before the function body which
  // will only be expanded on first use (by the loop below).
  std::vector<Function*> prototypesToGen;

  // Examine all the instructions in this function to find the intrinsics that
  // need to be lowered.
  for (Function::iterator BB = F.begin(), EE = F.end(); BB != EE; ++BB)
    for (BasicBlock::iterator I = BB->begin(), E = BB->end(); I != E; )
      if (CallInst *CI = dyn_cast<CallInst>(I++))
        if (Function *F = CI->getCalledFunction())
          switch (F->getIntrinsicID()) {
          case Intrinsic::not_intrinsic:
          case Intrinsic::memory_barrier:
          case Intrinsic::vastart:
          case Intrinsic::vacopy:
          case Intrinsic::vaend:
          case Intrinsic::returnaddress:
          case Intrinsic::frameaddress:
          case Intrinsic::setjmp:
          case Intrinsic::longjmp:
          case Intrinsic::prefetch:
          case Intrinsic::powi:
          case Intrinsic::x86_sse_cmp_ss:
          case Intrinsic::x86_sse_cmp_ps:
          case Intrinsic::x86_sse2_cmp_sd:
          case Intrinsic::x86_sse2_cmp_pd:
          case Intrinsic::ppc_altivec_lvsl:
              // We directly implement these intrinsics
            break;
          default:
            // If this is an intrinsic that directly corresponds to a GCC
            // builtin, we handle it.
            const char *BuiltinName = "";
#define GET_GCC_BUILTIN_NAME
#include "llvm/Intrinsics.gen"
#undef GET_GCC_BUILTIN_NAME
            // If we handle it, don't lower it.
            if (BuiltinName[0]) break;
            
            // All other intrinsic calls we must lower.
            Instruction *Before = 0;
            if (CI != &BB->front())
              Before = prior(BasicBlock::iterator(CI));

            IL->LowerIntrinsicCall(CI);
            if (Before) {        // Move iterator to instruction after call
              I = Before; ++I;
            } else {
              I = BB->begin();
            }
            // If the intrinsic got lowered to another call, and that call has
            // a definition then we need to make sure its prototype is emitted
            // before any calls to it.
            if (CallInst *Call = dyn_cast<CallInst>(I))
              if (Function *NewF = Call->getCalledFunction())
                if (!NewF->isDeclaration())
                  prototypesToGen.push_back(NewF);

            break;
          }

  // We may have collected some prototypes to emit in the loop above. 
  // Emit them now, before the function that uses them is emitted. But,
  // be careful not to emit them twice.
  std::vector<Function*>::iterator I = prototypesToGen.begin();
  std::vector<Function*>::iterator E = prototypesToGen.end();
  for ( ; I != E; ++I) {
    if (intrinsicPrototypesAlreadyGenerated.insert(*I).second) {
      Out << '\n';
      printFunctionSignature(*I, true);
      Out << ";\n";
    }
  }
}

void JsWriter::visitCallInst(CallInst &I) {
  if (isa<InlineAsm>(I.getCalledValue()))
    return visitInlineAsm(I);

  bool WroteCallee = false;

  // Handle intrinsic function calls first...
  if (Function *F = I.getCalledFunction())
    if (Intrinsic::ID ID = (Intrinsic::ID)F->getIntrinsicID())
      if (visitBuiltinCall(I, ID, WroteCallee))
        return;

  Value *Callee = I.getCalledValue();

  const PointerType  *PTy   = cast<PointerType>(Callee->getType());
  const FunctionType *FTy   = cast<FunctionType>(PTy->getElementType());

  // If this is a call to a struct-return function, assign to the first
  // parameter instead of passing it to the call.
  const AttrListPtr &PAL = I.getAttributes();
  bool hasByVal = I.hasByValArgument();
  bool isStructRet = I.hasStructRetAttr();
  if (isStructRet) {
    writeOperandDeref(I.getOperand(1));
    Out << " = ";
  }
  
  if (I.isTailCall()) Out << " /*tail*/ ";
  
  if (!WroteCallee) {
    // If this is an indirect call to a struct return function, we need to cast
    // the pointer. Ditto for indirect calls with byval arguments.
    bool NeedsCast = (hasByVal || isStructRet) && !isa<Function>(Callee);

    // GCC is a real PITA.  It does not permit codegening casts of functions to
    // function pointers if they are in a call (it generates a trap instruction
    // instead!).  We work around this by inserting a cast to void* in between
    // the function and the function pointer cast.  Unfortunately, we can't just
    // form the constant expression here, because the folder will immediately
    // nuke it.
    //
    // Note finally, that this is completely unsafe.  ANSI C does not guarantee
    // that void* and function pointers have the same size. :( To deal with this
    // in the common case, we handle casts where the number of arguments passed
    // match exactly.
    //
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(Callee))
      if (CE->isCast())
        if (Function *RF = dyn_cast<Function>(CE->getOperand(0))) {
          NeedsCast = true;
          Callee = RF;
        }
  
    if (NeedsCast) {
      // Ok, just cast the pointer type.
      Out << "((";
      if (isStructRet)
        printStructReturnPointerFunctionType(Out, PAL,
                             cast<PointerType>(I.getCalledValue()->getType()));
      else if (hasByVal)
        printType(Out, I.getCalledValue()->getType(), false, "", true, PAL);
      else
        printType(Out, I.getCalledValue()->getType());
      Out << ")(void*)";
    }
    writeOperand(Callee);
    if (NeedsCast) Out << ')';
  }

  Out << '(';

  bool PrintedArg = false;
  if(FTy->isVarArg() && !FTy->getNumParams()) {
    Out << "0 /*dummy arg*/";
    PrintedArg = true;
  }

  unsigned NumDeclaredParams = FTy->getNumParams();

  CallSite::arg_iterator AI = I.op_begin()+1, AE = I.op_end();
  unsigned ArgNo = 0;
  if (isStructRet) {   // Skip struct return argument.
    ++AI;
    ++ArgNo;
  }
      

  for (; AI != AE; ++AI, ++ArgNo) {
    if (PrintedArg) Out << ", ";
    if (ArgNo < NumDeclaredParams &&
        (*AI)->getType() != FTy->getParamType(ArgNo)) {
      Out << '(';
      printType(Out, FTy->getParamType(ArgNo), 
            /*isSigned=*/PAL.paramHasAttr(ArgNo+1, Attribute::SExt));
      Out << ')';
    }
    // Check if the argument is expected to be passed by value.
    if (I.paramHasAttr(ArgNo+1, Attribute::ByVal))
      writeOperandDeref(*AI);
    else
      writeOperand(*AI);
    PrintedArg = true;
  }
  Out << ')';
}

/// visitBuiltinCall - Handle the call to the specified builtin.  Returns true
/// if the entire call is handled, return false if it wasn't handled, and
/// optionally set 'WroteCallee' if the callee has already been printed out.
bool JsWriter::visitBuiltinCall(CallInst &I, Intrinsic::ID ID,
                               bool &WroteCallee) {
  switch (ID) {
  default: {
    // If this is an intrinsic that directly corresponds to a GCC
    // builtin, we emit it here.
    const char *BuiltinName = "";
    Function *F = I.getCalledFunction();
#define GET_GCC_BUILTIN_NAME
#include "llvm/Intrinsics.gen"
#undef GET_GCC_BUILTIN_NAME
    assert(BuiltinName[0] && "Unknown LLVM intrinsic!");
    
    Out << BuiltinName;
    WroteCallee = true;
    return false;
  }
  case Intrinsic::memory_barrier:
    Out << "__sync_synchronize()";
    return true;
  case Intrinsic::vastart:
    Out << "0; ";
      
    Out << "va_start(*(va_list*)";
    writeOperand(I.getOperand(1));
    Out << ", ";
    // Output the last argument to the enclosing function.
    if (I.getParent()->getParent()->arg_empty())
      Out << "vararg_dummy_arg";
    else
      writeOperand(--I.getParent()->getParent()->arg_end());
    Out << ')';
    return true;
  case Intrinsic::vaend:
    if (!isa<ConstantPointerNull>(I.getOperand(1))) {
      Out << "0; va_end(*(va_list*)";
      writeOperand(I.getOperand(1));
      Out << ')';
    } else {
      Out << "va_end(*(va_list*)0)";
    }
    return true;
  case Intrinsic::vacopy:
    Out << "0; ";
    Out << "va_copy(*(va_list*)";
    writeOperand(I.getOperand(1));
    Out << ", *(va_list*)";
    writeOperand(I.getOperand(2));
    Out << ')';
    return true;
  case Intrinsic::returnaddress:
    Out << "__builtin_return_address(";
    writeOperand(I.getOperand(1));
    Out << ')';
    return true;
  case Intrinsic::frameaddress:
    Out << "__builtin_frame_address(";
    writeOperand(I.getOperand(1));
    Out << ')';
    return true;
  case Intrinsic::powi:
    Out << "__builtin_powi(";
    writeOperand(I.getOperand(1));
    Out << ", ";
    writeOperand(I.getOperand(2));
    Out << ')';
    return true;
  case Intrinsic::setjmp:
    Out << "setjmp(*(jmp_buf*)";
    writeOperand(I.getOperand(1));
    Out << ')';
    return true;
  case Intrinsic::longjmp:
    Out << "longjmp(*(jmp_buf*)";
    writeOperand(I.getOperand(1));
    Out << ", ";
    writeOperand(I.getOperand(2));
    Out << ')';
    return true;
  case Intrinsic::prefetch:
    Out << "LLVM_PREFETCH((const void *)";
    writeOperand(I.getOperand(1));
    Out << ", ";
    writeOperand(I.getOperand(2));
    Out << ", ";
    writeOperand(I.getOperand(3));
    Out << ")";
    return true;
  case Intrinsic::stacksave:
    // Emit this as: Val = 0; *((void**)&Val) = __builtin_stack_save()
    // to work around GCC bugs (see PR1809).
    Out << "0; *((void**)&" << GetValueName(&I)
        << ") = __builtin_stack_save()";
    return true;
  case Intrinsic::x86_sse_cmp_ss:
  case Intrinsic::x86_sse_cmp_ps:
  case Intrinsic::x86_sse2_cmp_sd:
  case Intrinsic::x86_sse2_cmp_pd:
    Out << '(';
    printType(Out, I.getType());
    Out << ')';  
    // Multiple GCC builtins multiplex onto this intrinsic.
    switch (cast<ConstantInt>(I.getOperand(3))->getZExtValue()) {
    default: llvm_unreachable("Invalid llvm.x86.sse.cmp!");
    case 0: Out << "__builtin_ia32_cmpeq"; break;
    case 1: Out << "__builtin_ia32_cmplt"; break;
    case 2: Out << "__builtin_ia32_cmple"; break;
    case 3: Out << "__builtin_ia32_cmpunord"; break;
    case 4: Out << "__builtin_ia32_cmpneq"; break;
    case 5: Out << "__builtin_ia32_cmpnlt"; break;
    case 6: Out << "__builtin_ia32_cmpnle"; break;
    case 7: Out << "__builtin_ia32_cmpord"; break;
    }
    if (ID == Intrinsic::x86_sse_cmp_ps || ID == Intrinsic::x86_sse2_cmp_pd)
      Out << 'p';
    else
      Out << 's';
    if (ID == Intrinsic::x86_sse_cmp_ss || ID == Intrinsic::x86_sse_cmp_ps)
      Out << 's';
    else
      Out << 'd';
      
    Out << "(";
    writeOperand(I.getOperand(1));
    Out << ", ";
    writeOperand(I.getOperand(2));
    Out << ")";
    return true;
  case Intrinsic::ppc_altivec_lvsl:
    Out << '(';
    printType(Out, I.getType());
    Out << ')';  
    Out << "__builtin_altivec_lvsl(0, (void*)";
    writeOperand(I.getOperand(1));
    Out << ")";
    return true;
  }
}

//This converts the llvm constraint string to something gcc is expecting.
//TODO: work out platform independent constraints and factor those out
//      of the per target tables
//      handle multiple constraint codes
std::string JsWriter::InterpretASMConstraint(InlineAsm::ConstraintInfo& c) {
  assert(c.Codes.size() == 1 && "Too many asm constraint codes to handle");

  // Grab the translation table from MCAsmInfo if it exists.
  const MCAsmInfo *TargetAsm;
  std::string Triple = TheModule->getTargetTriple();
  if (Triple.empty())
    Triple = llvm::sys::getHostTriple();
  
  std::string E;
  if (const Target *Match = TargetRegistry::lookupTarget(Triple, E))
    TargetAsm = Match->createAsmInfo(Triple);
  else
    return c.Codes[0];
  
  const char *const *table = TargetAsm->getAsmCBE();

  // Search the translation table if it exists.
  for (int i = 0; table && table[i]; i += 2)
    if (c.Codes[0] == table[i]) {
      delete TargetAsm;
      return table[i+1];
    }

  // Default is identity.
  delete TargetAsm;
  return c.Codes[0];
}

//TODO: import logic from AsmPrinter.cpp
static std::string gccifyAsm(std::string asmstr) {
  for (std::string::size_type i = 0; i != asmstr.size(); ++i)
    if (asmstr[i] == '\n')
      asmstr.replace(i, 1, "\\n");
    else if (asmstr[i] == '\t')
      asmstr.replace(i, 1, "\\t");
    else if (asmstr[i] == '$') {
      if (asmstr[i + 1] == '{') {
        std::string::size_type a = asmstr.find_first_of(':', i + 1);
        std::string::size_type b = asmstr.find_first_of('}', i + 1);
        std::string n = "%" + 
          asmstr.substr(a + 1, b - a - 1) +
          asmstr.substr(i + 2, a - i - 2);
        asmstr.replace(i, b - i + 1, n);
        i += n.size() - 1;
      } else
        asmstr.replace(i, 1, "%");
    }
    else if (asmstr[i] == '%')//grr
      { asmstr.replace(i, 1, "%%"); ++i;}
  
  return asmstr;
}

//TODO: assumptions about what consume arguments from the call are likely wrong
//      handle communitivity
void JsWriter::visitInlineAsm(CallInst &CI) {
  InlineAsm* as = cast<InlineAsm>(CI.getCalledValue());
  std::vector<InlineAsm::ConstraintInfo> Constraints = as->ParseConstraints();
  
  std::vector<std::pair<Value*, int> > ResultVals;
  if (CI.getType() == Type::getVoidTy(CI.getContext()))
    ;
  else if (const StructType *ST = dyn_cast<StructType>(CI.getType())) {
    for (unsigned i = 0, e = ST->getNumElements(); i != e; ++i)
      ResultVals.push_back(std::make_pair(&CI, (int)i));
  } else {
    ResultVals.push_back(std::make_pair(&CI, -1));
  }
  
  // Fix up the asm string for gcc and emit it.
  Out << "__asm__ volatile (\"" << gccifyAsm(as->getAsmString()) << "\"\n";
  Out << "        :";

  unsigned ValueCount = 0;
  bool IsFirst = true;
  
  // Convert over all the output constraints.
  for (std::vector<InlineAsm::ConstraintInfo>::iterator I = Constraints.begin(),
       E = Constraints.end(); I != E; ++I) {
    
    if (I->Type != InlineAsm::isOutput) {
      ++ValueCount;
      continue;  // Ignore non-output constraints.
    }
    
    assert(I->Codes.size() == 1 && "Too many asm constraint codes to handle");
    std::string C = InterpretASMConstraint(*I);
    if (C.empty()) continue;
    
    if (!IsFirst) {
      Out << ", ";
      IsFirst = false;
    }

    // Unpack the dest.
    Value *DestVal;
    int DestValNo = -1;
    
    if (ValueCount < ResultVals.size()) {
      DestVal = ResultVals[ValueCount].first;
      DestValNo = ResultVals[ValueCount].second;
    } else
      DestVal = CI.getOperand(ValueCount-ResultVals.size()+1);

    if (I->isEarlyClobber)
      C = "&"+C;
      
    Out << "\"=" << C << "\"(" << GetValueName(DestVal);
    if (DestValNo != -1)
      Out << ".field" << DestValNo; // Multiple retvals.
    Out << ")";
    ++ValueCount;
  }
  
  
  // Convert over all the input constraints.
  Out << "\n        :";
  IsFirst = true;
  ValueCount = 0;
  for (std::vector<InlineAsm::ConstraintInfo>::iterator I = Constraints.begin(),
       E = Constraints.end(); I != E; ++I) {
    if (I->Type != InlineAsm::isInput) {
      ++ValueCount;
      continue;  // Ignore non-input constraints.
    }
    
    assert(I->Codes.size() == 1 && "Too many asm constraint codes to handle");
    std::string C = InterpretASMConstraint(*I);
    if (C.empty()) continue;
    
    if (!IsFirst) {
      Out << ", ";
      IsFirst = false;
    }
    
    assert(ValueCount >= ResultVals.size() && "Input can't refer to result");
    Value *SrcVal = CI.getOperand(ValueCount-ResultVals.size()+1);
    
    Out << "\"" << C << "\"(";
    if (!I->isIndirect)
      writeOperand(SrcVal);
    else
      writeOperandDeref(SrcVal);
    Out << ")";
  }
  
  // Convert over the clobber constraints.
  IsFirst = true;
  for (std::vector<InlineAsm::ConstraintInfo>::iterator I = Constraints.begin(),
       E = Constraints.end(); I != E; ++I) {
    if (I->Type != InlineAsm::isClobber)
      continue;  // Ignore non-input constraints.

    assert(I->Codes.size() == 1 && "Too many asm constraint codes to handle");
    std::string C = InterpretASMConstraint(*I);
    if (C.empty()) continue;
    
    if (!IsFirst) {
      Out << ", ";
      IsFirst = false;
    }
    
    Out << '\"' << C << '"';
  }
  
  Out << ")";
}

void JsWriter::visitAllocaInst(AllocaInst &I) {
  Out << '(';
  printType(Out, I.getType());
  Out << ") alloca(sizeof(";
  printType(Out, I.getType()->getElementType());
  Out << ')';
  if (I.isArrayAllocation()) {
    Out << " * " ;
    writeOperand(I.getOperand(0));
  }
  Out << ')';
}

void JsWriter::printGEPExpression(Value *Ptr, gep_type_iterator I,
                                 gep_type_iterator E, bool Static) {
  
  // If there are no indices, just print out the pointer.
  if (I == E) {
    writeOperand(Ptr);
    return;
  }
  Out << "(";
  gep_type_iterator N = I;
  ++N;
  writeOperand(Ptr);
  for(; N != E; ++I, ++N) {
    Out << "[";
    writeOperand(I.getOperand());
    Out << "]";
  }
  // unrolled last iteration
  Value *LastOp = I.getOperand();
  if(isa<Constant>(LastOp) && cast<Constant>(LastOp)->isNullValue()) {
    // forall i: &i == &i[0] so drop the last index
    Out << ")";
    return;
  }
  Out << "[";
  writeOperand(I.getOperand());
  Out << "])";
}

void JsWriter::writeMemoryAccess(Value *Operand, const Type *OperandType,
                                bool IsVolatile, unsigned Alignment) {

  bool IsUnaligned = Alignment &&
    Alignment < TD->getABITypeAlignment(OperandType);

  if (!IsUnaligned)
    Out << '*';
  if (IsVolatile || IsUnaligned) {
    Out << "((";
    if (IsUnaligned)
      Out << "struct __attribute__ ((packed, aligned(" << Alignment << "))) {";
    printType(Out, OperandType, false, IsUnaligned ? "data" : "volatile*");
    if (IsUnaligned) {
      Out << "; } ";
      if (IsVolatile) Out << "volatile ";
      Out << "*";
    }
    Out << ")";
  }

  writeOperand(Operand);

  if (IsVolatile || IsUnaligned) {
    Out << ')';
    if (IsUnaligned)
      Out << "->data";
  }
}

void JsWriter::visitLoadInst(LoadInst &I) {
  writeMemoryAccess(I.getOperand(0), I.getType(), I.isVolatile(),
                    I.getAlignment());

}

void JsWriter::visitStoreInst(StoreInst &I) {
  writeMemoryAccess(I.getPointerOperand(), I.getOperand(0)->getType(),
                    I.isVolatile(), I.getAlignment());
  Out << " = ";
  Value *Operand = I.getOperand(0);
  Constant *BitMask = 0;
  if (const IntegerType* ITy = dyn_cast<IntegerType>(Operand->getType()))
    if (!ITy->isPowerOf2ByteWidth())
      // We have a bit width that doesn't match an even power-of-2 byte
      // size. Consequently we must & the value with the type's bit mask
      BitMask = ConstantInt::get(ITy, ITy->getBitMask());
  if (BitMask)
    Out << "((";
  writeOperand(Operand);
  if (BitMask) {
    Out << ") & ";
    printConstant(BitMask, false);
    Out << ")"; 
  }
}

void JsWriter::visitGetElementPtrInst(GetElementPtrInst &I) {
  printGEPExpression(I.getPointerOperand(), gep_type_begin(I),
                     gep_type_end(I), false);
}

void JsWriter::visitVAArgInst(VAArgInst &I) {
  Out << "va_arg(*(va_list*)";
  writeOperand(I.getOperand(0));
  Out << ", ";
  printType(Out, I.getType());
  Out << ");\n ";
}

void JsWriter::visitInsertElementInst(InsertElementInst &I) {
  const Type *EltTy = I.getType()->getElementType();
  writeOperand(I.getOperand(0));
  Out << ";\n  ";
  Out << "((";
  printType(Out, PointerType::getUnqual(EltTy));
  Out << ")(&" << GetValueName(&I) << "))[";
  writeOperand(I.getOperand(2));
  Out << "] = (";
  writeOperand(I.getOperand(1));
  Out << ")";
}

void JsWriter::visitExtractElementInst(ExtractElementInst &I) {
  // We know that our operand is not inlined.
  Out << "((";
  const Type *EltTy = 
    cast<VectorType>(I.getOperand(0)->getType())->getElementType();
  printType(Out, PointerType::getUnqual(EltTy));
  Out << ")(&" << GetValueName(I.getOperand(0)) << "))[";
  writeOperand(I.getOperand(1));
  Out << "]";
}

void JsWriter::visitShuffleVectorInst(ShuffleVectorInst &SVI) {
  Out << "(";
  printType(Out, SVI.getType());
  Out << "){ ";
  const VectorType *VT = SVI.getType();
  unsigned NumElts = VT->getNumElements();
  const Type *EltTy = VT->getElementType();

  for (unsigned i = 0; i != NumElts; ++i) {
    if (i) Out << ", ";
    int SrcVal = SVI.getMaskValue(i);
    if ((unsigned)SrcVal >= NumElts*2) {
      Out << " 0/*undef*/ ";
    } else {
      Value *Op = SVI.getOperand((unsigned)SrcVal >= NumElts);
      if (isa<Instruction>(Op)) {
        // Do an extractelement of this value from the appropriate input.
        Out << "((";
        printType(Out, PointerType::getUnqual(EltTy));
        Out << ")(&" << GetValueName(Op)
            << "))[" << (SrcVal & (NumElts-1)) << "]";
      } else if (isa<ConstantAggregateZero>(Op) || isa<UndefValue>(Op)) {
        Out << "0";
      } else {
        printConstant(cast<ConstantVector>(Op)->getOperand(SrcVal &
                                                           (NumElts-1)),
                      false);
      }
    }
  }
  Out << "}";
}

void JsWriter::visitInsertValueInst(InsertValueInst &IVI) {
  // Start by copying the entire aggregate value into the result variable.
  writeOperand(IVI.getOperand(0));
  Out << ";\n  ";

  // Then do the insert to update the field.
  Out << GetValueName(&IVI);
  for (const unsigned *b = IVI.idx_begin(), *i = b, *e = IVI.idx_end();
       i != e; ++i) {
    const Type *IndexedTy =
      ExtractValueInst::getIndexedType(IVI.getOperand(0)->getType(), b, i+1);
    if (IndexedTy->isArrayTy())
      Out << ".array[" << *i << "]";
    else
      Out << ".field" << *i;
  }
  Out << " = ";
  writeOperand(IVI.getOperand(1));
}

void JsWriter::visitExtractValueInst(ExtractValueInst &EVI) {
  Out << "(";
  if (isa<UndefValue>(EVI.getOperand(0))) {
    Out << "(";
    printType(Out, EVI.getType());
    Out << ") 0/*UNDEF*/";
  } else {
    Out << GetValueName(EVI.getOperand(0));
    for (const unsigned *b = EVI.idx_begin(), *i = b, *e = EVI.idx_end();
         i != e; ++i) {
      const Type *IndexedTy =
        ExtractValueInst::getIndexedType(EVI.getOperand(0)->getType(), b, i+1);
      if (IndexedTy->isArrayTy())
        Out << ".array[" << *i << "]";
      else
        Out << ".field" << *i;
    }
  }
  Out << ")";
}

//===----------------------------------------------------------------------===//
//                       External Interface declaration
//===----------------------------------------------------------------------===//

bool JsTargetMachine::addPassesToEmitWholeFile(PassManager &PM,
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
