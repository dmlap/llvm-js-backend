//===- FunctionInlining.cpp - Code to perform function inlining -----------===//
//
// This file implements inlining of functions.
//
// Specifically, this:
//   * Exports functionality to inline any function call
//   * Inlines functions that consist of a single basic block
//   * Is able to inline ANY function call
//   . Has a smart heuristic for when to inline a function
//
// FIXME: This pass should transform alloca instructions in the called function
//        into malloc/free pairs!  Or perhaps it should refuse to inline them!
//
//===----------------------------------------------------------------------===//

#include "llvm/Transforms/IPO.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Module.h"
#include "llvm/Pass.h"
#include "llvm/iTerminators.h"
#include "llvm/iPHINode.h"
#include "llvm/iOther.h"
#include "llvm/Type.h"
#include "Support/Statistic.h"
#include <algorithm>

static Statistic<> NumInlined("inline", "Number of functions inlined");
using std::cerr;

// InlineFunction - This function forcibly inlines the called function into the
// basic block of the caller.  This returns false if it is not possible to
// inline this call.  The program is still in a well defined state if this 
// occurs though.
//
// Note that this only does one level of inlining.  For example, if the 
// instruction 'call B' is inlined, and 'B' calls 'C', then the call to 'C' now 
// exists in the instruction stream.  Similiarly this will inline a recursive
// function by one level.
//
bool InlineFunction(CallInst *CI) {
  assert(isa<CallInst>(CI) && "InlineFunction only works on CallInst nodes");
  assert(CI->getParent() && "Instruction not embedded in basic block!");
  assert(CI->getParent()->getParent() && "Instruction not in function!");

  const Function *CalledFunc = CI->getCalledFunction();
  if (CalledFunc == 0 ||   // Can't inline external function or indirect call!
      CalledFunc->isExternal()) return false;

  //cerr << "Inlining " << CalledFunc->getName() << " into " 
  //     << CurrentMeth->getName() << "\n";

  BasicBlock *OrigBB = CI->getParent();

  // Call splitBasicBlock - The original basic block now ends at the instruction
  // immediately before the call.  The original basic block now ends with an
  // unconditional branch to NewBB, and NewBB starts with the call instruction.
  //
  BasicBlock *NewBB = OrigBB->splitBasicBlock(CI);
  NewBB->setName("InlinedFunctionReturnNode");

  // Remove (unlink) the CallInst from the start of the new basic block.  
  NewBB->getInstList().remove(CI);

  // If we have a return value generated by this call, convert it into a PHI 
  // node that gets values from each of the old RET instructions in the original
  // function.
  //
  PHINode *PHI = 0;
  if (!CI->use_empty()) {
    // The PHI node should go at the front of the new basic block to merge all 
    // possible incoming values.
    //
    PHI = new PHINode(CalledFunc->getReturnType(), CI->getName(),
                      NewBB->begin());

    // Anything that used the result of the function call should now use the PHI
    // node as their operand.
    //
    CI->replaceAllUsesWith(PHI);
  }

  // Get a pointer to the last basic block in the function, which will have the
  // new function inlined after it.
  //
  Function::iterator LastBlock = &OrigBB->getParent()->back();

  // Calculate the vector of arguments to pass into the function cloner...
  std::map<const Value*, Value*> ValueMap;
  assert((unsigned)std::distance(CalledFunc->abegin(), CalledFunc->aend()) == 
         CI->getNumOperands()-1 && "No varargs calls can be inlined yet!");

  unsigned i = 1;
  for (Function::const_aiterator I = CalledFunc->abegin(), E=CalledFunc->aend();
       I != E; ++I, ++i)
    ValueMap[I] = CI->getOperand(i);

  // Since we are now done with the CallInst, we can delete it.
  delete CI;

  // Make a vector to capture the return instructions in the cloned function...
  std::vector<ReturnInst*> Returns;

  // Populate the value map with all of the globals in the program.
  Module &M = *OrigBB->getParent()->getParent();
  for (Module::iterator I = M.begin(), E = M.end(); I != E; ++I)
    ValueMap[I] = I;
  for (Module::giterator I = M.gbegin(), E = M.gend(); I != E; ++I)
    ValueMap[I] = I;

  // Do all of the hard part of cloning the callee into the caller...
  CloneFunctionInto(OrigBB->getParent(), CalledFunc, ValueMap, Returns, ".i");

  // Loop over all of the return instructions, turning them into unconditional
  // branches to the merge point now...
  for (unsigned i = 0, e = Returns.size(); i != e; ++i) {
    ReturnInst *RI = Returns[i];
    BasicBlock *BB = RI->getParent();

    // Add a branch to the merge point where the PHI node would live...
    new BranchInst(NewBB, RI);

    if (PHI) {   // The PHI node should include this value!
      assert(RI->getReturnValue() && "Ret should have value!");
      assert(RI->getReturnValue()->getType() == PHI->getType() && 
             "Ret value not consistent in function!");
      PHI->addIncoming(RI->getReturnValue(), BB);
    }

    // Delete the return instruction now
    BB->getInstList().erase(RI);
  }

  // Check to see if the PHI node only has one argument.  This is a common
  // case resulting from there only being a single return instruction in the
  // function call.  Because this is so common, eliminate the PHI node.
  //
  if (PHI && PHI->getNumIncomingValues() == 1) {
    PHI->replaceAllUsesWith(PHI->getIncomingValue(0));
    PHI->getParent()->getInstList().erase(PHI);
  }

  // Change the branch that used to go to NewBB to branch to the first basic 
  // block of the inlined function.
  //
  TerminatorInst *Br = OrigBB->getTerminator();
  assert(Br && Br->getOpcode() == Instruction::Br && 
	 "splitBasicBlock broken!");
  Br->setOperand(0, ++LastBlock);
  return true;
}

static inline bool ShouldInlineFunction(const CallInst *CI, const Function *F) {
  assert(CI->getParent() && CI->getParent()->getParent() && 
	 "Call not embedded into a function!");

  // Don't inline a recursive call.
  if (CI->getParent()->getParent() == F) return false;

  // Don't inline something too big.  This is a really crappy heuristic
  if (F->size() > 3) return false;

  // Don't inline into something too big. This is a **really** crappy heuristic
  if (CI->getParent()->getParent()->size() > 10) return false;

  // Go ahead and try just about anything else.
  return true;
}


static inline bool DoFunctionInlining(BasicBlock *BB) {
  for (BasicBlock::iterator I = BB->begin(); I != BB->end(); ++I) {
    if (CallInst *CI = dyn_cast<CallInst>(I)) {
      // Check to see if we should inline this function
      Function *F = CI->getCalledFunction();
      if (F && ShouldInlineFunction(CI, F)) {
	return InlineFunction(CI);
      }
    }
  }
  return false;
}

// doFunctionInlining - Use a heuristic based approach to inline functions that
// seem to look good.
//
static bool doFunctionInlining(Function &F) {
  bool Changed = false;

  // Loop through now and inline instructions a basic block at a time...
  for (Function::iterator I = F.begin(); I != F.end(); )
    if (DoFunctionInlining(I)) {
      ++NumInlined;
      Changed = true;
    } else {
      ++I;
    }

  return Changed;
}

namespace {
  struct FunctionInlining : public FunctionPass {
    virtual bool runOnFunction(Function &F) {
      return doFunctionInlining(F);
    }
  };
  RegisterOpt<FunctionInlining> X("inline", "Function Integration/Inlining");
}

Pass *createFunctionInliningPass() { return new FunctionInlining(); }
