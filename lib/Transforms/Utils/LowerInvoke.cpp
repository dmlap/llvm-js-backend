//===- LowerInvoke.cpp - Eliminate Invoke & Unwind instructions -----------===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This transformation is designed for use by code generators which do not yet
// support stack unwinding.  This pass gives them the ability to execute any
// program which does not throw an exception, by turning 'invoke' instructions
// into calls and by turning 'unwind' instructions into calls to abort().
//
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/Scalar.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Module.h"
#include "llvm/Pass.h"
#include "llvm/iTerminators.h"
#include "llvm/iOther.h"
#include "Support/Statistic.h"
using namespace llvm;

namespace {
  Statistic<> NumLowered("lowerinvoke", "Number of invoke & unwinds replaced");

  class LowerInvoke : public FunctionPass {
    Value *AbortMessage;
    unsigned AbortMessageLength;
    Function *WriteFn;
    Function *AbortFn;
  public:
    bool doInitialization(Module &M);
    bool runOnFunction(Function &F);
  };

  RegisterOpt<LowerInvoke>
  X("lowerinvoke", "Lower invoke and unwind, for unwindless code generators");
}

// Public Interface To the LowerInvoke pass.
FunctionPass *llvm::createLowerInvokePass() { return new LowerInvoke(); }

// doInitialization - Make sure that there is a prototype for abort in the
// current module.
bool LowerInvoke::doInitialization(Module &M) {
  Constant *Msg =
    ConstantArray::get("Exception handler needed, but not available.\n");
  AbortMessageLength = Msg->getNumOperands()-1;  // don't include \0
  
  GlobalVariable *MsgGV =
    new GlobalVariable(Msg->getType(), true, GlobalValue::InternalLinkage, Msg,
                       "", &M);
  std::vector<Constant*> GEPIdx(2, Constant::getNullValue(Type::LongTy));
  AbortMessage =
    ConstantExpr::getGetElementPtr(ConstantPointerRef::get(MsgGV), GEPIdx);

  AbortFn = M.getOrInsertFunction("abort", Type::VoidTy, 0);
  WriteFn = M.getOrInsertFunction("write", Type::VoidTy, Type::IntTy,
                                  PointerType::get(Type::SByteTy), Type::IntTy,
                                  0);
  return true;
}

bool LowerInvoke::runOnFunction(Function &F) {
  bool Changed = false;
  for (Function::iterator BB = F.begin(), E = F.end(); BB != E; ++BB)
    if (InvokeInst *II = dyn_cast<InvokeInst>(BB->getTerminator())) {
      // Insert a normal call instruction...
      std::string Name = II->getName(); II->setName("");
      Value *NewCall = new CallInst(II->getCalledValue(),
                                    std::vector<Value*>(II->op_begin()+3,
                                                        II->op_end()), Name,II);
      II->replaceAllUsesWith(NewCall);
      
      // Insert an unconditional branch to the normal destination.
      new BranchInst(II->getNormalDest(), II);

      // Remove any PHI node entries from the exception destination.
      II->getExceptionalDest()->removePredecessor(BB);

      // Remove the invoke instruction now.
      BB->getInstList().erase(II);

      ++NumLowered; Changed = true;
    } else if (UnwindInst *UI = dyn_cast<UnwindInst>(BB->getTerminator())) {
      // Insert a new call to write(2, AbortMessage, AbortMessageLength);
      std::vector<Value*> Args;
      Args.push_back(ConstantInt::get(Type::IntTy, 2));
      Args.push_back(AbortMessage);
      Args.push_back(ConstantInt::get(Type::IntTy, AbortMessageLength));
      new CallInst(WriteFn, Args, "", UI);

      // Insert a call to abort()
      new CallInst(AbortFn, std::vector<Value*>(), "", UI);

      // Insert a return instruction.
      new ReturnInst(F.getReturnType() == Type::VoidTy ? 0 :
                            Constant::getNullValue(F.getReturnType()), UI);

      // Remove the unwind instruction now.
      BB->getInstList().erase(UI);

      ++NumLowered; Changed = true;
    }
  return Changed;
}
