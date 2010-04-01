//===-- DebugLoc.cpp - Implement DebugLoc class ---------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/Support/DebugLoc.h"
#include "LLVMContextImpl.h"
using namespace llvm;

//===----------------------------------------------------------------------===//
// DebugLoc Implementation
//===----------------------------------------------------------------------===//

MDNode *NewDebugLoc::getScope(const LLVMContext &Ctx) const {
  if (ScopeIdx == 0) return 0;
  
  if (ScopeIdx > 0) {
    // Positive ScopeIdx is an index into ScopeRecords, which has no inlined-at
    // position specified.
    assert(unsigned(ScopeIdx) <= Ctx.pImpl->ScopeRecords.size() &&
           "Invalid ScopeIdx!");
    return Ctx.pImpl->ScopeRecords[ScopeIdx-1].get();
  }
  
  // Otherwise, the index is in the ScopeInlinedAtRecords array.
  assert(unsigned(-ScopeIdx) <= Ctx.pImpl->ScopeInlinedAtRecords.size() &&
         "Invalid ScopeIdx");
  return Ctx.pImpl->ScopeInlinedAtRecords[-ScopeIdx-1].first.get();
}

MDNode *NewDebugLoc::getInlinedAt(const LLVMContext &Ctx) const {
  // Positive ScopeIdx is an index into ScopeRecords, which has no inlined-at
  // position specified.  Zero is invalid.
  if (ScopeIdx >= 0) return 0;
  
  // Otherwise, the index is in the ScopeInlinedAtRecords array.
  assert(unsigned(-ScopeIdx) <= Ctx.pImpl->ScopeInlinedAtRecords.size() &&
         "Invalid ScopeIdx");
  return Ctx.pImpl->ScopeInlinedAtRecords[-ScopeIdx-1].second.get();
}

/// Return both the Scope and the InlinedAt values.
void NewDebugLoc::getScopeAndInlinedAt(MDNode *&Scope, MDNode *&IA,
                                       const LLVMContext &Ctx) const {
  if (ScopeIdx == 0) {
    Scope = IA = 0;
    return;
  }
  
  if (ScopeIdx > 0) {
    // Positive ScopeIdx is an index into ScopeRecords, which has no inlined-at
    // position specified.
    assert(unsigned(ScopeIdx) <= Ctx.pImpl->ScopeRecords.size() &&
           "Invalid ScopeIdx!");
    Scope = Ctx.pImpl->ScopeRecords[ScopeIdx-1].get();
    IA = 0;
    return;
  }
  
  // Otherwise, the index is in the ScopeInlinedAtRecords array.
  assert(unsigned(-ScopeIdx) <= Ctx.pImpl->ScopeInlinedAtRecords.size() &&
         "Invalid ScopeIdx");
  Scope = Ctx.pImpl->ScopeInlinedAtRecords[-ScopeIdx-1].first.get();
  IA    = Ctx.pImpl->ScopeInlinedAtRecords[-ScopeIdx-1].second.get();
}


NewDebugLoc NewDebugLoc::get(unsigned Line, unsigned Col,
                             MDNode *Scope, MDNode *InlinedAt) {
  NewDebugLoc Result;
  
  // If no scope is available, this is an unknown location.
  if (Scope == 0) return Result;
  
  // Saturate line and col to "unknown".
  if (Col > 255) Col = 0;
  if (Line >= (1 << 24)) Line = 0;
  Result.LineCol = Line | (Col << 24);
  
  LLVMContext &Ctx = Scope->getContext();
  
  // If there is no inlined-at location, use the ScopeRecords array.
  if (InlinedAt == 0)
    Result.ScopeIdx = Ctx.pImpl->getOrAddScopeRecordIdxEntry(Scope, 0);
  else
    Result.ScopeIdx = Ctx.pImpl->getOrAddScopeInlinedAtIdxEntry(Scope,
                                                                InlinedAt, 0);

  return Result;
}

/// getAsMDNode - This method converts the compressed DebugLoc node into a
/// DILocation compatible MDNode.
MDNode *NewDebugLoc::getAsMDNode(const LLVMContext &Ctx) const {
  if (isUnknown()) return 0;
  
  MDNode *Scope, *IA;
  getScopeAndInlinedAt(Scope, IA, Ctx);
  assert(Scope && "If scope is null, this should be isUnknown()");
  
  LLVMContext &Ctx2 = Scope->getContext();
  const Type *Int32 = Type::getInt32Ty(Ctx2);
  Value *Elts[] = {
    ConstantInt::get(Int32, getLine()), ConstantInt::get(Int32, getCol()),
    Scope, IA
  };
  return MDNode::get(Ctx2, &Elts[0], 4);
}

/// getFromDILocation - Translate the DILocation quad into a NewDebugLoc.
NewDebugLoc NewDebugLoc::getFromDILocation(MDNode *N) {
  if (N == 0 || N->getNumOperands() != 4) return NewDebugLoc();
  
  MDNode *Scope = dyn_cast_or_null<MDNode>(N->getOperand(2));
  if (Scope == 0) return NewDebugLoc();
  
  unsigned LineNo = 0, ColNo = 0;
  if (ConstantInt *Line = dyn_cast_or_null<ConstantInt>(N->getOperand(0)))
    LineNo = Line->getZExtValue();
  if (ConstantInt *Col = dyn_cast_or_null<ConstantInt>(N->getOperand(1)))
    ColNo = Col->getZExtValue();
  
  return get(LineNo, ColNo, Scope, dyn_cast_or_null<MDNode>(N->getOperand(3)));
}

//===----------------------------------------------------------------------===//
// LLVMContextImpl Implementation
//===----------------------------------------------------------------------===//

int LLVMContextImpl::getOrAddScopeRecordIdxEntry(MDNode *Scope,
                                                 int ExistingIdx) {
  // If we already have an entry for this scope, return it.
  int &Idx = ScopeRecordIdx[Scope];
  if (Idx) return Idx;
  
  // If we don't have an entry, but ExistingIdx is specified, use it.
  if (ExistingIdx)
    return Idx = ExistingIdx;
  
  // Otherwise add a new entry.
  
  // Start out ScopeRecords with a minimal reasonable size to avoid
  // excessive reallocation starting out.
  if (ScopeRecords.empty())
    ScopeRecords.reserve(128);
  
  // Index is biased by 1 for index.
  Idx = ScopeRecords.size()+1;
  ScopeRecords.push_back(DebugRecVH(Scope, this, Idx));
  return Idx;
}

int LLVMContextImpl::getOrAddScopeInlinedAtIdxEntry(MDNode *Scope, MDNode *IA,
                                                    int ExistingIdx) {
  // If we already have an entry, return it.
  int &Idx = ScopeInlinedAtIdx[std::make_pair(Scope, IA)];
  if (Idx) return Idx;
  
  // If we don't have an entry, but ExistingIdx is specified, use it.
  if (ExistingIdx)
    return Idx = ExistingIdx;
  
  // Start out ScopeInlinedAtRecords with a minimal reasonable size to avoid
  // excessive reallocation starting out.
  if (ScopeInlinedAtRecords.empty())
    ScopeInlinedAtRecords.reserve(128);
    
  // Index is biased by 1 and negated.
  Idx = -ScopeInlinedAtRecords.size()-1;
  ScopeInlinedAtRecords.push_back(std::make_pair(DebugRecVH(Scope, this, Idx),
                                                 DebugRecVH(IA, this, Idx)));
  return Idx;
}


//===----------------------------------------------------------------------===//
// DebugRecVH Implementation
//===----------------------------------------------------------------------===//

/// deleted - The MDNode this is pointing to got deleted, so this pointer needs
/// to drop to null and we need remove our entry from the DenseMap.
void DebugRecVH::deleted() {
  // If this is a  non-canonical reference, just drop the value to null, we know
  // it doesn't have a map entry.
  if (Idx == 0) {
    setValPtr(0);
    return;
  }
    
  MDNode *Cur = get();
  
  // If the index is positive, it is an entry in ScopeRecords.
  if (Idx > 0) {
    assert(Ctx->ScopeRecordIdx[Cur] == Idx && "Mapping out of date!");
    Ctx->ScopeRecordIdx.erase(Cur);
    // Reset this VH to null and we're done.
    setValPtr(0);
    Idx = 0;
    return;
  }
  
  // Otherwise, it is an entry in ScopeInlinedAtRecords, we don't know if it
  // is the scope or the inlined-at record entry.
  assert(unsigned(-Idx-1) < Ctx->ScopeInlinedAtRecords.size());
  std::pair<DebugRecVH, DebugRecVH> &Entry = Ctx->ScopeInlinedAtRecords[-Idx-1];
  assert((this == &Entry.first || this == &Entry.second) &&
         "Mapping out of date!");
  
  MDNode *OldScope = Entry.first.get();
  MDNode *OldInlinedAt = Entry.second.get();
  assert(OldScope != 0 && OldInlinedAt != 0 &&
         "Entry should be non-canonical if either val dropped to null");

  // Otherwise, we do have an entry in it, nuke it and we're done.
  assert(Ctx->ScopeInlinedAtIdx[std::make_pair(OldScope, OldInlinedAt)] == Idx&&
         "Mapping out of date");
  Ctx->ScopeInlinedAtIdx.erase(std::make_pair(OldScope, OldInlinedAt));
  
  // Reset this VH to null.  Drop both 'Idx' values to null to indicate that
  // we're in non-canonical form now.
  setValPtr(0);
  Entry.first.Idx = Entry.second.Idx = 0;
}

void DebugRecVH::allUsesReplacedWith(Value *NewVa) {
  // If being replaced with a non-mdnode value (e.g. undef) handle this as if
  // the mdnode got deleted.
  MDNode *NewVal = dyn_cast<MDNode>(NewVa);
  if (NewVal == 0) return deleted();
  
  // If this is a non-canonical reference, just change it, we know it already
  // doesn't have a map entry.
  if (Idx == 0) {
    setValPtr(NewVa);
    return;
  }
  
  MDNode *OldVal = get();
  assert(OldVal != NewVa && "Node replaced with self?");
  
  // If the index is positive, it is an entry in ScopeRecords.
  if (Idx > 0) {
    assert(Ctx->ScopeRecordIdx[OldVal] == Idx && "Mapping out of date!");
    Ctx->ScopeRecordIdx.erase(OldVal);
    setValPtr(NewVal);

    int NewEntry = Ctx->getOrAddScopeRecordIdxEntry(NewVal, Idx);
    
    // If NewVal already has an entry, this becomes a non-canonical reference,
    // just drop Idx to 0 to signify this.
    if (NewEntry != Idx)
      Idx = 0;
    return;
  }
  
  // Otherwise, it is an entry in ScopeInlinedAtRecords, we don't know if it
  // is the scope or the inlined-at record entry.
  assert(unsigned(-Idx-1) < Ctx->ScopeInlinedAtRecords.size());
  std::pair<DebugRecVH, DebugRecVH> &Entry = Ctx->ScopeInlinedAtRecords[-Idx-1];
  assert((this == &Entry.first || this == &Entry.second) &&
         "Mapping out of date!");
  
  MDNode *OldScope = Entry.first.get();
  MDNode *OldInlinedAt = Entry.second.get();
  assert(OldScope != 0 && OldInlinedAt != 0 &&
         "Entry should be non-canonical if either val dropped to null");
  
  // Otherwise, we do have an entry in it, nuke it and we're done.
  assert(Ctx->ScopeInlinedAtIdx[std::make_pair(OldScope, OldInlinedAt)] == Idx&&
         "Mapping out of date");
  Ctx->ScopeInlinedAtIdx.erase(std::make_pair(OldScope, OldInlinedAt));
  
  // Reset this VH to the new value.
  setValPtr(NewVal);

  int NewIdx = Ctx->getOrAddScopeInlinedAtIdxEntry(Entry.first.get(),
                                                   Entry.second.get(), Idx);
  // If NewVal already has an entry, this becomes a non-canonical reference,
  // just drop Idx to 0 to signify this.
  if (NewIdx != Idx) {
    std::pair<DebugRecVH, DebugRecVH> &Entry=Ctx->ScopeInlinedAtRecords[-Idx-1];
    Entry.first.Idx = Entry.second.Idx = 0;
  }
}