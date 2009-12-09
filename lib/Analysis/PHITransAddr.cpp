//===- PHITransAddr.cpp - PHI Translation for Addresses -------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the PHITransAddr class.
//
//===----------------------------------------------------------------------===//

#include "llvm/Analysis/PHITransAddr.h"
#include "llvm/Analysis/Dominators.h"
#include "llvm/Analysis/InstructionSimplify.h"
#include "llvm/Support/raw_ostream.h"
using namespace llvm;

static bool CanPHITrans(Instruction *Inst) {
  if (isa<PHINode>(Inst) ||
      isa<BitCastInst>(Inst) ||
      isa<GetElementPtrInst>(Inst))
    return true;
  
  if (Inst->getOpcode() == Instruction::Add &&
      isa<ConstantInt>(Inst->getOperand(1)))
    return true;
  
  //   cerr << "MEMDEP: Could not PHI translate: " << *Pointer;
  //   if (isa<BitCastInst>(PtrInst) || isa<GetElementPtrInst>(PtrInst))
  //     cerr << "OP:\t\t\t\t" << *PtrInst->getOperand(0);
  return false;
}

void PHITransAddr::dump() const {
  if (Addr == 0) {
    errs() << "PHITransAddr: null\n";
    return;
  }
  errs() << "PHITransAddr: " << *Addr << "\n";
  for (unsigned i = 0, e = InstInputs.size(); i != e; ++i)
    errs() << "  Input #" << i << " is " << *InstInputs[i] << "\n";
}


static bool VerifySubExpr(Value *Expr,
                          SmallVectorImpl<Instruction*> &InstInputs) {
  // If this is a non-instruction value, there is nothing to do.
  Instruction *I = dyn_cast<Instruction>(Expr);
  if (I == 0) return true;
  
  // If it's an instruction, it is either in Tmp or its operands recursively
  // are.
  SmallVectorImpl<Instruction*>::iterator Entry =
    std::find(InstInputs.begin(), InstInputs.end(), I);
  if (Entry != InstInputs.end()) {
    InstInputs.erase(Entry);
    return true;
  }
  
  // If it isn't in the InstInputs list it is a subexpr incorporated into the
  // address.  Sanity check that it is phi translatable.
  if (!CanPHITrans(I)) {
    errs() << "Non phi translatable instruction found in PHITransAddr, either "
              "something is missing from InstInputs or CanPHITrans is wrong:\n";
    errs() << *I << '\n';
    return false;
  }
  
  // Validate the operands of the instruction.
  for (unsigned i = 0, e = I->getNumOperands(); i != e; ++i)
    if (!VerifySubExpr(I->getOperand(i), InstInputs))
      return false;

  return true;
}

/// Verify - Check internal consistency of this data structure.  If the
/// structure is valid, it returns true.  If invalid, it prints errors and
/// returns false.
bool PHITransAddr::Verify() const {
  if (Addr == 0) return true;
  
  SmallVector<Instruction*, 8> Tmp(InstInputs.begin(), InstInputs.end());  
  
  if (!VerifySubExpr(Addr, Tmp))
    return false;
  
  if (!Tmp.empty()) {
    errs() << "PHITransAddr inconsistent, contains extra instructions:\n";
    for (unsigned i = 0, e = InstInputs.size(); i != e; ++i)
      errs() << "  InstInput #" << i << " is " << *InstInputs[i] << "\n";
    return false;
  }
  
  // a-ok.
  return true;
}


/// IsPotentiallyPHITranslatable - If this needs PHI translation, return true
/// if we have some hope of doing it.  This should be used as a filter to
/// avoid calling PHITranslateValue in hopeless situations.
bool PHITransAddr::IsPotentiallyPHITranslatable() const {
  // If the input value is not an instruction, or if it is not defined in CurBB,
  // then we don't need to phi translate it.
  Instruction *Inst = dyn_cast<Instruction>(Addr);
  return Inst == 0 || CanPHITrans(Inst);
}


static void RemoveInstInputs(Instruction *I, 
                             SmallVectorImpl<Instruction*> &InstInputs) {
  // If the instruction is in the InstInputs list, remove it.
  SmallVectorImpl<Instruction*>::iterator Entry =
    std::find(InstInputs.begin(), InstInputs.end(), I);
  if (Entry != InstInputs.end()) {
    InstInputs.erase(Entry);
    return;
  }
  
  // Otherwise, it must have instruction inputs itself.  Zap them recursively.
  bool HadInstInputs = false;
  for (unsigned i = 0, e = I->getNumOperands(); i != e; ++i) {
    if (Instruction *Op = dyn_cast<Instruction>(I->getOperand(i))) {
      RemoveInstInputs(Op, InstInputs);
      HadInstInputs = true;
    }
  }

  // This instruction had to have operands in the instinputs list or it should
  // have been in the list itself.  If not, the list is broken.
  assert(HadInstInputs && "InstInputs list inconsistent!");
}

/// ReplaceInstWithValue - Remove any instruction inputs in the InstInputs
/// array that are due to the specified instruction that is about to be
/// removed from the address, and add any corresponding to V.  This returns V.
Value *PHITransAddr::ReplaceInstWithValue(Instruction *I, Value *V) {
  // Remove the old instruction from InstInputs.
  RemoveInstInputs(I, InstInputs);
  
  // If V is an instruction, it is now an input.
  if (Instruction *VI = dyn_cast<Instruction>(V))
    InstInputs.push_back(VI);
  return V;
}


Value *PHITransAddr::PHITranslateSubExpr(Value *V, BasicBlock *CurBB,
                                         BasicBlock *PredBB) {
  // If this is a non-instruction value, it can't require PHI translation.
  Instruction *Inst = dyn_cast<Instruction>(V);
  if (Inst == 0) return V;
  
  // Determine whether 'Inst' is an input to our PHI translatable expression.
  bool isInput = std::count(InstInputs.begin(), InstInputs.end(), Inst);

  // Handle inputs instructions if needed.
  if (isInput) {
    if (Inst->getParent() != CurBB) {
      // If it is an input defined in a different block, then it remains an
      // input.
      return Inst;
    }
  
    // If 'Inst' is defined in this block, it must be an input that needs to be
    // phi translated or an intermediate expression that needs to be incorporated
    // into the expression.
    
    // If this is a PHI, go ahead and translate it.
    if (PHINode *PN = dyn_cast<PHINode>(Inst))
      return PN->getIncomingValueForBlock(PredBB);

    
    // If this is a non-phi value, and it is analyzable, we can incorporate it
    // into the expression by making all instruction operands be inputs.
    if (!CanPHITrans(Inst))
      return 0;
    
    // Okay, we can incorporate it, this instruction is no longer an input.
    InstInputs.erase(std::find(InstInputs.begin(), InstInputs.end(), Inst));
    
    // All instruction operands are now inputs (and of course, they may also be
    // defined in this block, so they may need to be phi translated themselves.
    for (unsigned i = 0, e = Inst->getNumOperands(); i != e; ++i)
      if (Instruction *Op = dyn_cast<Instruction>(Inst->getOperand(i)))
        InstInputs.push_back(Op);
  }

  // Ok, it must be an intermediate result (either because it started that way
  // or because we just incorporated it into the expression).  See if its
  // operands need to be phi translated, and if so, reconstruct it.
  
  if (BitCastInst *BC = dyn_cast<BitCastInst>(Inst)) {
    Value *PHIIn = PHITranslateSubExpr(BC->getOperand(0), CurBB, PredBB);
    if (PHIIn == 0) return 0;
    if (PHIIn == BC->getOperand(0))
      return BC;
    
    // Find an available version of this cast.
    
    // Constants are trivial to find.
    if (Constant *C = dyn_cast<Constant>(PHIIn))
      return ReplaceInstWithValue(BC, ConstantExpr::getBitCast(C,
                                                               BC->getType()));
    
    // Otherwise we have to see if a bitcasted version of the incoming pointer
    // is available.  If so, we can use it, otherwise we have to fail.
    for (Value::use_iterator UI = PHIIn->use_begin(), E = PHIIn->use_end();
         UI != E; ++UI) {
      if (BitCastInst *BCI = dyn_cast<BitCastInst>(*UI))
        if (BCI->getType() == BC->getType())
          return BCI;
    }
    return 0;
  }
  
  // Handle getelementptr with at least one PHI translatable operand.
  if (GetElementPtrInst *GEP = dyn_cast<GetElementPtrInst>(Inst)) {
    SmallVector<Value*, 8> GEPOps;
    BasicBlock *CurBB = GEP->getParent();
    bool AnyChanged = false;
    for (unsigned i = 0, e = GEP->getNumOperands(); i != e; ++i) {
      Value *GEPOp = PHITranslateSubExpr(GEP->getOperand(i), CurBB, PredBB);
      if (GEPOp == 0) return 0;
      
      AnyChanged |= GEPOp != GEP->getOperand(i);
      GEPOps.push_back(GEPOp);
    }
    
    if (!AnyChanged)
      return GEP;
    
    // Simplify the GEP to handle 'gep x, 0' -> x etc.
    if (Value *V = SimplifyGEPInst(&GEPOps[0], GEPOps.size(), TD))
      return ReplaceInstWithValue(GEP, V);
    
    // Scan to see if we have this GEP available.
    Value *APHIOp = GEPOps[0];
    for (Value::use_iterator UI = APHIOp->use_begin(), E = APHIOp->use_end();
         UI != E; ++UI) {
      if (GetElementPtrInst *GEPI = dyn_cast<GetElementPtrInst>(*UI))
        if (GEPI->getType() == GEP->getType() &&
            GEPI->getNumOperands() == GEPOps.size() &&
            GEPI->getParent()->getParent() == CurBB->getParent()) {
          bool Mismatch = false;
          for (unsigned i = 0, e = GEPOps.size(); i != e; ++i)
            if (GEPI->getOperand(i) != GEPOps[i]) {
              Mismatch = true;
              break;
            }
          if (!Mismatch)
            return GEPI;
        }
    }
    return 0;
  }
  
  // Handle add with a constant RHS.
  if (Inst->getOpcode() == Instruction::Add &&
      isa<ConstantInt>(Inst->getOperand(1))) {
    // PHI translate the LHS.
    Constant *RHS = cast<ConstantInt>(Inst->getOperand(1));
    bool isNSW = cast<BinaryOperator>(Inst)->hasNoSignedWrap();
    bool isNUW = cast<BinaryOperator>(Inst)->hasNoUnsignedWrap();
    
    Value *LHS = PHITranslateSubExpr(Inst->getOperand(0), CurBB, PredBB);
    if (LHS == 0) return 0;
    
    // If the PHI translated LHS is an add of a constant, fold the immediates.
    if (BinaryOperator *BOp = dyn_cast<BinaryOperator>(LHS))
      if (BOp->getOpcode() == Instruction::Add)
        if (ConstantInt *CI = dyn_cast<ConstantInt>(BOp->getOperand(1))) {
          LHS = BOp->getOperand(0);
          RHS = ConstantExpr::getAdd(RHS, CI);
          isNSW = isNUW = false;
        }
    
    // See if the add simplifies away.
    if (Value *Res = SimplifyAddInst(LHS, RHS, isNSW, isNUW, TD))
      return ReplaceInstWithValue(Inst, Res);
    
    // Otherwise, see if we have this add available somewhere.
    for (Value::use_iterator UI = LHS->use_begin(), E = LHS->use_end();
         UI != E; ++UI) {
      if (BinaryOperator *BO = dyn_cast<BinaryOperator>(*UI))
        if (BO->getOperand(0) == LHS && BO->getOperand(1) == RHS &&
            BO->getParent()->getParent() == CurBB->getParent())
          return BO;
    }
    
    return 0;
  }
  
  // Otherwise, we failed.
  return 0;
}


/// PHITranslateValue - PHI translate the current address up the CFG from
/// CurBB to Pred, updating our state the reflect any needed changes.  This
/// returns true on failure and sets Addr to null.
bool PHITransAddr::PHITranslateValue(BasicBlock *CurBB, BasicBlock *PredBB) {
  assert(Verify() && "Invalid PHITransAddr!");
  Addr = PHITranslateSubExpr(Addr, CurBB, PredBB);
  assert(Verify() && "Invalid PHITransAddr!");
  return Addr == 0;
}

/// GetAvailablePHITranslatedSubExpr - Return the value computed by
/// PHITranslateSubExpr if it dominates PredBB, otherwise return null.
Value *PHITransAddr::
GetAvailablePHITranslatedSubExpr(Value *V, BasicBlock *CurBB,BasicBlock *PredBB,
                                 const DominatorTree &DT) const {
  PHITransAddr Tmp(V, TD);
  Tmp.PHITranslateValue(CurBB, PredBB);
  
  // See if PHI translation succeeds.
  V = Tmp.getAddr();
  
  // Make sure the value is live in the predecessor.
  if (Instruction *Inst = dyn_cast_or_null<Instruction>(V))
    if (!DT.dominates(Inst->getParent(), PredBB))
      return 0;
  return V;
}


/// PHITranslateWithInsertion - PHI translate this value into the specified
/// predecessor block, inserting a computation of the value if it is
/// unavailable.
///
/// All newly created instructions are added to the NewInsts list.  This
/// returns null on failure.
///
Value *PHITransAddr::
PHITranslateWithInsertion(BasicBlock *CurBB, BasicBlock *PredBB,
                          const DominatorTree &DT,
                          SmallVectorImpl<Instruction*> &NewInsts) {
  unsigned NISize = NewInsts.size();
  
  // Attempt to PHI translate with insertion.
  Addr = InsertPHITranslatedSubExpr(Addr, CurBB, PredBB, DT, NewInsts);
  
  // If successful, return the new value.
  if (Addr) return Addr;
  
  // If not, destroy any intermediate instructions inserted.
  while (NewInsts.size() != NISize)
    NewInsts.pop_back_val()->eraseFromParent();
  return 0;
}


/// InsertPHITranslatedPointer - Insert a computation of the PHI translated
/// version of 'V' for the edge PredBB->CurBB into the end of the PredBB
/// block.  All newly created instructions are added to the NewInsts list.
/// This returns null on failure.
///
Value *PHITransAddr::
InsertPHITranslatedSubExpr(Value *InVal, BasicBlock *CurBB,
                           BasicBlock *PredBB, const DominatorTree &DT,
                           SmallVectorImpl<Instruction*> &NewInsts) {
  // See if we have a version of this value already available and dominating
  // PredBB.  If so, there is no need to insert a new instance of it.
  if (Value *Res = GetAvailablePHITranslatedSubExpr(InVal, CurBB, PredBB, DT))
    return Res;

  // If we don't have an available version of this value, it must be an
  // instruction.
  Instruction *Inst = cast<Instruction>(InVal);
  
  // Handle bitcast of PHI translatable value.
  if (BitCastInst *BC = dyn_cast<BitCastInst>(Inst)) {
    Value *OpVal = InsertPHITranslatedSubExpr(BC->getOperand(0),
                                              CurBB, PredBB, DT, NewInsts);
    if (OpVal == 0) return 0;
    
    // Otherwise insert a bitcast at the end of PredBB.
    BitCastInst *New = new BitCastInst(OpVal, InVal->getType(),
                                       InVal->getName()+".phi.trans.insert",
                                       PredBB->getTerminator());
    NewInsts.push_back(New);
    return New;
  }
  
  // Handle getelementptr with at least one PHI operand.
  if (GetElementPtrInst *GEP = dyn_cast<GetElementPtrInst>(Inst)) {
    SmallVector<Value*, 8> GEPOps;
    BasicBlock *CurBB = GEP->getParent();
    for (unsigned i = 0, e = GEP->getNumOperands(); i != e; ++i) {
      Value *OpVal = InsertPHITranslatedSubExpr(GEP->getOperand(i),
                                                CurBB, PredBB, DT, NewInsts);
      if (OpVal == 0) return 0;
      GEPOps.push_back(OpVal);
    }
    
    GetElementPtrInst *Result = 
    GetElementPtrInst::Create(GEPOps[0], GEPOps.begin()+1, GEPOps.end(),
                              InVal->getName()+".phi.trans.insert",
                              PredBB->getTerminator());
    Result->setIsInBounds(GEP->isInBounds());
    NewInsts.push_back(Result);
    return Result;
  }
  
#if 0
  // FIXME: This code works, but it is unclear that we actually want to insert
  // a big chain of computation in order to make a value available in a block.
  // This needs to be evaluated carefully to consider its cost trade offs.
  
  // Handle add with a constant RHS.
  if (Inst->getOpcode() == Instruction::Add &&
      isa<ConstantInt>(Inst->getOperand(1))) {
    // PHI translate the LHS.
    Value *OpVal = InsertPHITranslatedSubExpr(Inst->getOperand(0),
                                              CurBB, PredBB, DT, NewInsts);
    if (OpVal == 0) return 0;
    
    BinaryOperator *Res = BinaryOperator::CreateAdd(OpVal, Inst->getOperand(1),
                                           InVal->getName()+".phi.trans.insert",
                                                    PredBB->getTerminator());
    Res->setHasNoSignedWrap(cast<BinaryOperator>(Inst)->hasNoSignedWrap());
    Res->setHasNoUnsignedWrap(cast<BinaryOperator>(Inst)->hasNoUnsignedWrap());
    NewInsts.push_back(Res);
    return Res;
  }
#endif
  
  return 0;
}
