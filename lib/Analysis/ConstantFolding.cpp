//===-- ConstantFolding.cpp - Fold instructions into constants ------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines routines for folding instructions into constants.
//
// Also, to supplement the basic VMCore ConstantExpr simplifications,
// this file defines some additional folding routines that can make use of
// TargetData information. These functions cannot go in VMCore due to library
// dependency issues.
//
//===----------------------------------------------------------------------===//

#include "llvm/Analysis/ConstantFolding.h"
#include "llvm/Constants.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Function.h"
#include "llvm/GlobalVariable.h"
#include "llvm/Instructions.h"
#include "llvm/Intrinsics.h"
#include "llvm/LLVMContext.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/Target/TargetData.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"
#include "llvm/Support/MathExtras.h"
#include <cerrno>
#include <cmath>
using namespace llvm;

//===----------------------------------------------------------------------===//
// Constant Folding internal helper functions
//===----------------------------------------------------------------------===//

/// IsConstantOffsetFromGlobal - If this constant is actually a constant offset
/// from a global, return the global and the constant.  Because of
/// constantexprs, this function is recursive.
static bool IsConstantOffsetFromGlobal(Constant *C, GlobalValue *&GV,
                                       int64_t &Offset, const TargetData &TD) {
  // Trivial case, constant is the global.
  if ((GV = dyn_cast<GlobalValue>(C))) {
    Offset = 0;
    return true;
  }
  
  // Otherwise, if this isn't a constant expr, bail out.
  ConstantExpr *CE = dyn_cast<ConstantExpr>(C);
  if (!CE) return false;
  
  // Look through ptr->int and ptr->ptr casts.
  if (CE->getOpcode() == Instruction::PtrToInt ||
      CE->getOpcode() == Instruction::BitCast)
    return IsConstantOffsetFromGlobal(CE->getOperand(0), GV, Offset, TD);
  
  // i32* getelementptr ([5 x i32]* @a, i32 0, i32 5)    
  if (CE->getOpcode() == Instruction::GetElementPtr) {
    // Cannot compute this if the element type of the pointer is missing size
    // info.
    if (!cast<PointerType>(CE->getOperand(0)->getType())
                 ->getElementType()->isSized())
      return false;
    
    // If the base isn't a global+constant, we aren't either.
    if (!IsConstantOffsetFromGlobal(CE->getOperand(0), GV, Offset, TD))
      return false;
    
    // Otherwise, add any offset that our operands provide.
    gep_type_iterator GTI = gep_type_begin(CE);
    for (User::const_op_iterator i = CE->op_begin() + 1, e = CE->op_end();
         i != e; ++i, ++GTI) {
      ConstantInt *CI = dyn_cast<ConstantInt>(*i);
      if (!CI) return false;  // Index isn't a simple constant?
      if (CI->getZExtValue() == 0) continue;  // Not adding anything.
      
      if (const StructType *ST = dyn_cast<StructType>(*GTI)) {
        // N = N + Offset
        Offset += TD.getStructLayout(ST)->getElementOffset(CI->getZExtValue());
      } else {
        const SequentialType *SQT = cast<SequentialType>(*GTI);
        Offset += TD.getTypeAllocSize(SQT->getElementType())*CI->getSExtValue();
      }
    }
    return true;
  }
  
  return false;
}

/// ReadDataFromGlobal - Recursive helper to read bits out of global.  C is the
/// constant being copied out of. ByteOffset is an offset into C.  CurPtr is the
/// pointer to copy results into and BytesLeft is the number of bytes left in
/// the CurPtr buffer.  TD is the target data.
static bool ReadDataFromGlobal(Constant *C, uint64_t ByteOffset,
                               unsigned char *CurPtr, unsigned BytesLeft,
                               const TargetData &TD) {
  assert(ByteOffset <= TD.getTypeAllocSize(C->getType()) &&
         "Out of range access");
  
  if (isa<ConstantAggregateZero>(C) || isa<UndefValue>(C))
    return true;
  
  if (ConstantInt *CI = dyn_cast<ConstantInt>(C)) {
    if (CI->getBitWidth() > 64 ||
        (CI->getBitWidth() & 7) != 0)
      return false;
    
    uint64_t Val = CI->getZExtValue();
    unsigned IntBytes = unsigned(CI->getBitWidth()/8);
    
    for (unsigned i = 0; i != BytesLeft && ByteOffset != IntBytes; ++i) {
      CurPtr[i] = (unsigned char)(Val >> ByteOffset * 8);
      ++ByteOffset;
    }
    return true;
  }
  
  if (ConstantFP *CFP = dyn_cast<ConstantFP>(C)) {
    if (CFP->getType()->isDoubleTy()) {
      C = ConstantExpr::getBitCast(C, Type::getInt64Ty(C->getContext()));
      return ReadDataFromGlobal(C, ByteOffset, CurPtr, BytesLeft, TD);
    }
    if (CFP->getType()->isFloatTy()){
      C = ConstantExpr::getBitCast(C, Type::getInt32Ty(C->getContext()));
      return ReadDataFromGlobal(C, ByteOffset, CurPtr, BytesLeft, TD);
    }
  }

  if (ConstantStruct *CS = dyn_cast<ConstantStruct>(C)) {
    const StructLayout *SL = TD.getStructLayout(CS->getType());
    unsigned Index = SL->getElementContainingOffset(ByteOffset);
    uint64_t CurEltOffset = SL->getElementOffset(Index);
    ByteOffset -= CurEltOffset;
    
    while (1) {
      // If the element access is to the element itself and not to tail padding,
      // read the bytes from the element.
      uint64_t EltSize = TD.getTypeAllocSize(CS->getOperand(Index)->getType());

      if (ByteOffset < EltSize &&
          !ReadDataFromGlobal(CS->getOperand(Index), ByteOffset, CurPtr,
                              BytesLeft, TD))
        return false;
      
      ++Index;
      
      // Check to see if we read from the last struct element, if so we're done.
      if (Index == CS->getType()->getNumElements())
        return true;

      // If we read all of the bytes we needed from this element we're done.
      uint64_t NextEltOffset = SL->getElementOffset(Index);

      if (BytesLeft <= NextEltOffset-CurEltOffset-ByteOffset)
        return true;

      // Move to the next element of the struct.
      BytesLeft -= NextEltOffset-CurEltOffset-ByteOffset;
      ByteOffset = 0;
      CurEltOffset = NextEltOffset;
    }
    // not reached.
  }

  if (ConstantArray *CA = dyn_cast<ConstantArray>(C)) {
    uint64_t EltSize = TD.getTypeAllocSize(CA->getType()->getElementType());
    uint64_t Index = ByteOffset / EltSize;
    uint64_t Offset = ByteOffset - Index * EltSize;
    for (; Index != CA->getType()->getNumElements(); ++Index) {
      if (!ReadDataFromGlobal(CA->getOperand(Index), Offset, CurPtr,
                              BytesLeft, TD))
        return false;
      if (EltSize >= BytesLeft)
        return true;
      
      Offset = 0;
      BytesLeft -= EltSize;
      CurPtr += EltSize;
    }
    return true;
  }
  
  if (ConstantVector *CV = dyn_cast<ConstantVector>(C)) {
    uint64_t EltSize = TD.getTypeAllocSize(CV->getType()->getElementType());
    uint64_t Index = ByteOffset / EltSize;
    uint64_t Offset = ByteOffset - Index * EltSize;
    for (; Index != CV->getType()->getNumElements(); ++Index) {
      if (!ReadDataFromGlobal(CV->getOperand(Index), Offset, CurPtr,
                              BytesLeft, TD))
        return false;
      if (EltSize >= BytesLeft)
        return true;
      
      Offset = 0;
      BytesLeft -= EltSize;
      CurPtr += EltSize;
    }
    return true;
  }
  
  // Otherwise, unknown initializer type.
  return false;
}

static Constant *FoldReinterpretLoadFromConstPtr(Constant *C,
                                                 const TargetData &TD) {
  const Type *LoadTy = cast<PointerType>(C->getType())->getElementType();
  const IntegerType *IntType = dyn_cast<IntegerType>(LoadTy);
  
  // If this isn't an integer load we can't fold it directly.
  if (!IntType) {
    // If this is a float/double load, we can try folding it as an int32/64 load
    // and then bitcast the result.  This can be useful for union cases.  Note
    // that address spaces don't matter here since we're not going to result in
    // an actual new load.
    const Type *MapTy;
    if (LoadTy->isFloatTy())
      MapTy = Type::getInt32PtrTy(C->getContext());
    else if (LoadTy->isDoubleTy())
      MapTy = Type::getInt64PtrTy(C->getContext());
    else if (isa<VectorType>(LoadTy)) {
      MapTy = IntegerType::get(C->getContext(),
                               TD.getTypeAllocSizeInBits(LoadTy));
      MapTy = PointerType::getUnqual(MapTy);
    } else
      return 0;

    C = ConstantExpr::getBitCast(C, MapTy);
    if (Constant *Res = FoldReinterpretLoadFromConstPtr(C, TD))
      return ConstantExpr::getBitCast(Res, LoadTy);
    return 0;
  }
  
  unsigned BytesLoaded = (IntType->getBitWidth() + 7) / 8;
  if (BytesLoaded > 32 || BytesLoaded == 0) return 0;
  
  GlobalValue *GVal;
  int64_t Offset;
  if (!IsConstantOffsetFromGlobal(C, GVal, Offset, TD))
    return 0;
  
  GlobalVariable *GV = dyn_cast<GlobalVariable>(GVal);
  if (!GV || !GV->isConstant() || !GV->hasInitializer() ||
      !GV->hasDefinitiveInitializer() ||
      !GV->getInitializer()->getType()->isSized())
    return 0;

  // If we're loading off the beginning of the global, some bytes may be valid,
  // but we don't try to handle this.
  if (Offset < 0) return 0;
  
  // If we're not accessing anything in this constant, the result is undefined.
  if (uint64_t(Offset) >= TD.getTypeAllocSize(GV->getInitializer()->getType()))
    return UndefValue::get(IntType);
  
  unsigned char RawBytes[32] = {0};
  if (!ReadDataFromGlobal(GV->getInitializer(), Offset, RawBytes,
                          BytesLoaded, TD))
    return 0;

  APInt ResultVal(IntType->getBitWidth(), 0);
  for (unsigned i = 0; i != BytesLoaded; ++i) {
    ResultVal <<= 8;
    ResultVal |= APInt(IntType->getBitWidth(), RawBytes[BytesLoaded-1-i]);
  }

  return ConstantInt::get(IntType->getContext(), ResultVal);
}

/// ConstantFoldLoadFromConstPtr - Return the value that a load from C would
/// produce if it is constant and determinable.  If this is not determinable,
/// return null.
Constant *llvm::ConstantFoldLoadFromConstPtr(Constant *C,
                                             const TargetData *TD) {
  // First, try the easy cases:
  if (GlobalVariable *GV = dyn_cast<GlobalVariable>(C))
    if (GV->isConstant() && GV->hasDefinitiveInitializer())
      return GV->getInitializer();

  // If the loaded value isn't a constant expr, we can't handle it.
  ConstantExpr *CE = dyn_cast<ConstantExpr>(C);
  if (!CE) return 0;
  
  if (CE->getOpcode() == Instruction::GetElementPtr) {
    if (GlobalVariable *GV = dyn_cast<GlobalVariable>(CE->getOperand(0)))
      if (GV->isConstant() && GV->hasDefinitiveInitializer())
        if (Constant *V = 
             ConstantFoldLoadThroughGEPConstantExpr(GV->getInitializer(), CE))
          return V;
  }
  
  // Instead of loading constant c string, use corresponding integer value
  // directly if string length is small enough.
  std::string Str;
  if (TD && GetConstantStringInfo(CE->getOperand(0), Str) && !Str.empty()) {
    unsigned StrLen = Str.length();
    const Type *Ty = cast<PointerType>(CE->getType())->getElementType();
    unsigned NumBits = Ty->getPrimitiveSizeInBits();
    // Replace LI with immediate integer store.
    if ((NumBits >> 3) == StrLen + 1) {
      APInt StrVal(NumBits, 0);
      APInt SingleChar(NumBits, 0);
      if (TD->isLittleEndian()) {
        for (signed i = StrLen-1; i >= 0; i--) {
          SingleChar = (uint64_t) Str[i] & UCHAR_MAX;
          StrVal = (StrVal << 8) | SingleChar;
        }
      } else {
        for (unsigned i = 0; i < StrLen; i++) {
          SingleChar = (uint64_t) Str[i] & UCHAR_MAX;
          StrVal = (StrVal << 8) | SingleChar;
        }
        // Append NULL at the end.
        SingleChar = 0;
        StrVal = (StrVal << 8) | SingleChar;
      }
      return ConstantInt::get(CE->getContext(), StrVal);
    }
  }
  
  // If this load comes from anywhere in a constant global, and if the global
  // is all undef or zero, we know what it loads.
  if (GlobalVariable *GV = dyn_cast<GlobalVariable>(CE->getUnderlyingObject())){
    if (GV->isConstant() && GV->hasDefinitiveInitializer()) {
      const Type *ResTy = cast<PointerType>(C->getType())->getElementType();
      if (GV->getInitializer()->isNullValue())
        return Constant::getNullValue(ResTy);
      if (isa<UndefValue>(GV->getInitializer()))
        return UndefValue::get(ResTy);
    }
  }
  
  // Try hard to fold loads from bitcasted strange and non-type-safe things.  We
  // currently don't do any of this for big endian systems.  It can be
  // generalized in the future if someone is interested.
  if (TD && TD->isLittleEndian())
    return FoldReinterpretLoadFromConstPtr(CE, *TD);
  return 0;
}

static Constant *ConstantFoldLoadInst(const LoadInst *LI, const TargetData *TD){
  if (LI->isVolatile()) return 0;
  
  if (Constant *C = dyn_cast<Constant>(LI->getOperand(0)))
    return ConstantFoldLoadFromConstPtr(C, TD);

  return 0;
}

/// SymbolicallyEvaluateBinop - One of Op0/Op1 is a constant expression.
/// Attempt to symbolically evaluate the result of a binary operator merging
/// these together.  If target data info is available, it is provided as TD, 
/// otherwise TD is null.
static Constant *SymbolicallyEvaluateBinop(unsigned Opc, Constant *Op0,
                                           Constant *Op1, const TargetData *TD,
                                           LLVMContext &Context){
  // SROA
  
  // Fold (and 0xffffffff00000000, (shl x, 32)) -> shl.
  // Fold (lshr (or X, Y), 32) -> (lshr [X/Y], 32) if one doesn't contribute
  // bits.
  
  
  // If the constant expr is something like &A[123] - &A[4].f, fold this into a
  // constant.  This happens frequently when iterating over a global array.
  if (Opc == Instruction::Sub && TD) {
    GlobalValue *GV1, *GV2;
    int64_t Offs1, Offs2;
    
    if (IsConstantOffsetFromGlobal(Op0, GV1, Offs1, *TD))
      if (IsConstantOffsetFromGlobal(Op1, GV2, Offs2, *TD) &&
          GV1 == GV2) {
        // (&GV+C1) - (&GV+C2) -> C1-C2, pointer arithmetic cannot overflow.
        return ConstantInt::get(Op0->getType(), Offs1-Offs2);
      }
  }
    
  return 0;
}

/// SymbolicallyEvaluateGEP - If we can symbolically evaluate the specified GEP
/// constant expression, do so.
static Constant *SymbolicallyEvaluateGEP(Constant* const* Ops, unsigned NumOps,
                                         const Type *ResultTy,
                                         LLVMContext &Context,
                                         const TargetData *TD) {
  Constant *Ptr = Ops[0];
  if (!TD || !cast<PointerType>(Ptr->getType())->getElementType()->isSized())
    return 0;

  unsigned BitWidth = TD->getTypeSizeInBits(TD->getIntPtrType(Context));
  APInt BasePtr(BitWidth, 0);
  bool BaseIsInt = true;
  if (!Ptr->isNullValue()) {
    // If this is a inttoptr from a constant int, we can fold this as the base,
    // otherwise we can't.
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(Ptr))
      if (CE->getOpcode() == Instruction::IntToPtr)
        if (ConstantInt *Base = dyn_cast<ConstantInt>(CE->getOperand(0))) {
          BasePtr = Base->getValue();
          BasePtr.zextOrTrunc(BitWidth);
        }
    
    if (BasePtr == 0)
      BaseIsInt = false;
  }

  // If this is a constant expr gep that is effectively computing an
  // "offsetof", fold it into 'cast int Size to T*' instead of 'gep 0, 0, 12'
  for (unsigned i = 1; i != NumOps; ++i)
    if (!isa<ConstantInt>(Ops[i]))
      return 0;
  
  APInt Offset = APInt(BitWidth,
                       TD->getIndexedOffset(Ptr->getType(),
                                            (Value**)Ops+1, NumOps-1));
  // If the base value for this address is a literal integer value, fold the
  // getelementptr to the resulting integer value casted to the pointer type.
  if (BaseIsInt) {
    Constant *C = ConstantInt::get(Context, Offset+BasePtr);
    return ConstantExpr::getIntToPtr(C, ResultTy);
  }

  // Otherwise form a regular getelementptr. Recompute the indices so that
  // we eliminate over-indexing of the notional static type array bounds.
  // This makes it easy to determine if the getelementptr is "inbounds".
  // Also, this helps GlobalOpt do SROA on GlobalVariables.
  const Type *Ty = Ptr->getType();
  SmallVector<Constant*, 32> NewIdxs;
  do {
    if (const SequentialType *ATy = dyn_cast<SequentialType>(Ty)) {
      // The only pointer indexing we'll do is on the first index of the GEP.
      if (isa<PointerType>(ATy) && !NewIdxs.empty())
        break;
      // Determine which element of the array the offset points into.
      APInt ElemSize(BitWidth, TD->getTypeAllocSize(ATy->getElementType()));
      if (ElemSize == 0)
        return 0;
      APInt NewIdx = Offset.udiv(ElemSize);
      Offset -= NewIdx * ElemSize;
      NewIdxs.push_back(ConstantInt::get(TD->getIntPtrType(Context), NewIdx));
      Ty = ATy->getElementType();
    } else if (const StructType *STy = dyn_cast<StructType>(Ty)) {
      // Determine which field of the struct the offset points into. The
      // getZExtValue is at least as safe as the StructLayout API because we
      // know the offset is within the struct at this point.
      const StructLayout &SL = *TD->getStructLayout(STy);
      unsigned ElIdx = SL.getElementContainingOffset(Offset.getZExtValue());
      NewIdxs.push_back(ConstantInt::get(Type::getInt32Ty(Context), ElIdx));
      Offset -= APInt(BitWidth, SL.getElementOffset(ElIdx));
      Ty = STy->getTypeAtIndex(ElIdx);
    } else {
      // We've reached some non-indexable type.
      break;
    }
  } while (Ty != cast<PointerType>(ResultTy)->getElementType());

  // If we haven't used up the entire offset by descending the static
  // type, then the offset is pointing into the middle of an indivisible
  // member, so we can't simplify it.
  if (Offset != 0)
    return 0;

  // Create a GEP.
  Constant *C =
    ConstantExpr::getGetElementPtr(Ptr, &NewIdxs[0], NewIdxs.size());
  assert(cast<PointerType>(C->getType())->getElementType() == Ty &&
         "Computed GetElementPtr has unexpected type!");

  // If we ended up indexing a member with a type that doesn't match
  // the type of what the original indices indexed, add a cast.
  if (Ty != cast<PointerType>(ResultTy)->getElementType())
    C = ConstantExpr::getBitCast(C, ResultTy);

  return C;
}

/// FoldBitCast - Constant fold bitcast, symbolically evaluating it with 
/// targetdata.  Return 0 if unfoldable.
static Constant *FoldBitCast(Constant *C, const Type *DestTy,
                             const TargetData &TD, LLVMContext &Context) {
  // If this is a bitcast from constant vector -> vector, fold it.
  if (ConstantVector *CV = dyn_cast<ConstantVector>(C)) {
    if (const VectorType *DestVTy = dyn_cast<VectorType>(DestTy)) {
      // If the element types match, VMCore can fold it.
      unsigned NumDstElt = DestVTy->getNumElements();
      unsigned NumSrcElt = CV->getNumOperands();
      if (NumDstElt == NumSrcElt)
        return 0;
      
      const Type *SrcEltTy = CV->getType()->getElementType();
      const Type *DstEltTy = DestVTy->getElementType();
      
      // Otherwise, we're changing the number of elements in a vector, which 
      // requires endianness information to do the right thing.  For example,
      //    bitcast (<2 x i64> <i64 0, i64 1> to <4 x i32>)
      // folds to (little endian):
      //    <4 x i32> <i32 0, i32 0, i32 1, i32 0>
      // and to (big endian):
      //    <4 x i32> <i32 0, i32 0, i32 0, i32 1>
      
      // First thing is first.  We only want to think about integer here, so if
      // we have something in FP form, recast it as integer.
      if (DstEltTy->isFloatingPoint()) {
        // Fold to an vector of integers with same size as our FP type.
        unsigned FPWidth = DstEltTy->getPrimitiveSizeInBits();
        const Type *DestIVTy = VectorType::get(
                                 IntegerType::get(Context, FPWidth), NumDstElt);
        // Recursively handle this integer conversion, if possible.
        C = FoldBitCast(C, DestIVTy, TD, Context);
        if (!C) return 0;
        
        // Finally, VMCore can handle this now that #elts line up.
        return ConstantExpr::getBitCast(C, DestTy);
      }
      
      // Okay, we know the destination is integer, if the input is FP, convert
      // it to integer first.
      if (SrcEltTy->isFloatingPoint()) {
        unsigned FPWidth = SrcEltTy->getPrimitiveSizeInBits();
        const Type *SrcIVTy = VectorType::get(
                                 IntegerType::get(Context, FPWidth), NumSrcElt);
        // Ask VMCore to do the conversion now that #elts line up.
        C = ConstantExpr::getBitCast(C, SrcIVTy);
        CV = dyn_cast<ConstantVector>(C);
        if (!CV) return 0;  // If VMCore wasn't able to fold it, bail out.
      }
      
      // Now we know that the input and output vectors are both integer vectors
      // of the same size, and that their #elements is not the same.  Do the
      // conversion here, which depends on whether the input or output has
      // more elements.
      bool isLittleEndian = TD.isLittleEndian();
      
      SmallVector<Constant*, 32> Result;
      if (NumDstElt < NumSrcElt) {
        // Handle: bitcast (<4 x i32> <i32 0, i32 1, i32 2, i32 3> to <2 x i64>)
        Constant *Zero = Constant::getNullValue(DstEltTy);
        unsigned Ratio = NumSrcElt/NumDstElt;
        unsigned SrcBitSize = SrcEltTy->getPrimitiveSizeInBits();
        unsigned SrcElt = 0;
        for (unsigned i = 0; i != NumDstElt; ++i) {
          // Build each element of the result.
          Constant *Elt = Zero;
          unsigned ShiftAmt = isLittleEndian ? 0 : SrcBitSize*(Ratio-1);
          for (unsigned j = 0; j != Ratio; ++j) {
            Constant *Src = dyn_cast<ConstantInt>(CV->getOperand(SrcElt++));
            if (!Src) return 0;  // Reject constantexpr elements.
            
            // Zero extend the element to the right size.
            Src = ConstantExpr::getZExt(Src, Elt->getType());
            
            // Shift it to the right place, depending on endianness.
            Src = ConstantExpr::getShl(Src, 
                             ConstantInt::get(Src->getType(), ShiftAmt));
            ShiftAmt += isLittleEndian ? SrcBitSize : -SrcBitSize;
            
            // Mix it in.
            Elt = ConstantExpr::getOr(Elt, Src);
          }
          Result.push_back(Elt);
        }
      } else {
        // Handle: bitcast (<2 x i64> <i64 0, i64 1> to <4 x i32>)
        unsigned Ratio = NumDstElt/NumSrcElt;
        unsigned DstBitSize = DstEltTy->getPrimitiveSizeInBits();
        
        // Loop over each source value, expanding into multiple results.
        for (unsigned i = 0; i != NumSrcElt; ++i) {
          Constant *Src = dyn_cast<ConstantInt>(CV->getOperand(i));
          if (!Src) return 0;  // Reject constantexpr elements.

          unsigned ShiftAmt = isLittleEndian ? 0 : DstBitSize*(Ratio-1);
          for (unsigned j = 0; j != Ratio; ++j) {
            // Shift the piece of the value into the right place, depending on
            // endianness.
            Constant *Elt = ConstantExpr::getLShr(Src, 
                            ConstantInt::get(Src->getType(), ShiftAmt));
            ShiftAmt += isLittleEndian ? DstBitSize : -DstBitSize;

            // Truncate and remember this piece.
            Result.push_back(ConstantExpr::getTrunc(Elt, DstEltTy));
          }
        }
      }
      
      return ConstantVector::get(Result.data(), Result.size());
    }
  }
  
  return 0;
}


//===----------------------------------------------------------------------===//
// Constant Folding public APIs
//===----------------------------------------------------------------------===//


/// ConstantFoldInstruction - Attempt to constant fold the specified
/// instruction.  If successful, the constant result is returned, if not, null
/// is returned.  Note that this function can only fail when attempting to fold
/// instructions like loads and stores, which have no constant expression form.
///
Constant *llvm::ConstantFoldInstruction(Instruction *I, LLVMContext &Context,
                                        const TargetData *TD) {
  if (PHINode *PN = dyn_cast<PHINode>(I)) {
    if (PN->getNumIncomingValues() == 0)
      return UndefValue::get(PN->getType());

    Constant *Result = dyn_cast<Constant>(PN->getIncomingValue(0));
    if (Result == 0) return 0;

    // Handle PHI nodes specially here...
    for (unsigned i = 1, e = PN->getNumIncomingValues(); i != e; ++i)
      if (PN->getIncomingValue(i) != Result && PN->getIncomingValue(i) != PN)
        return 0;   // Not all the same incoming constants...

    // If we reach here, all incoming values are the same constant.
    return Result;
  }

  // Scan the operand list, checking to see if they are all constants, if so,
  // hand off to ConstantFoldInstOperands.
  SmallVector<Constant*, 8> Ops;
  for (User::op_iterator i = I->op_begin(), e = I->op_end(); i != e; ++i)
    if (Constant *Op = dyn_cast<Constant>(*i))
      Ops.push_back(Op);
    else
      return 0;  // All operands not constant!

  if (const CmpInst *CI = dyn_cast<CmpInst>(I))
    return ConstantFoldCompareInstOperands(CI->getPredicate(),
                                           Ops.data(), Ops.size(), 
                                           Context, TD);
  
  if (const LoadInst *LI = dyn_cast<LoadInst>(I))
    return ConstantFoldLoadInst(LI, TD);
  
  return ConstantFoldInstOperands(I->getOpcode(), I->getType(),
                                  Ops.data(), Ops.size(), Context, TD);
}

/// ConstantFoldConstantExpression - Attempt to fold the constant expression
/// using the specified TargetData.  If successful, the constant result is
/// result is returned, if not, null is returned.
Constant *llvm::ConstantFoldConstantExpression(ConstantExpr *CE,
                                               LLVMContext &Context,
                                               const TargetData *TD) {
  SmallVector<Constant*, 8> Ops;
  for (User::op_iterator i = CE->op_begin(), e = CE->op_end(); i != e; ++i)
    Ops.push_back(cast<Constant>(*i));

  if (CE->isCompare())
    return ConstantFoldCompareInstOperands(CE->getPredicate(),
                                           Ops.data(), Ops.size(), 
                                           Context, TD);
  return ConstantFoldInstOperands(CE->getOpcode(), CE->getType(),
                                  Ops.data(), Ops.size(), Context, TD);
}

/// ConstantFoldInstOperands - Attempt to constant fold an instruction with the
/// specified opcode and operands.  If successful, the constant result is
/// returned, if not, null is returned.  Note that this function can fail when
/// attempting to fold instructions like loads and stores, which have no
/// constant expression form.
///
Constant *llvm::ConstantFoldInstOperands(unsigned Opcode, const Type *DestTy, 
                                         Constant* const* Ops, unsigned NumOps,
                                         LLVMContext &Context,
                                         const TargetData *TD) {
  // Handle easy binops first.
  if (Instruction::isBinaryOp(Opcode)) {
    if (isa<ConstantExpr>(Ops[0]) || isa<ConstantExpr>(Ops[1]))
      if (Constant *C = SymbolicallyEvaluateBinop(Opcode, Ops[0], Ops[1], TD,
                                                  Context))
        return C;
    
    return ConstantExpr::get(Opcode, Ops[0], Ops[1]);
  }
  
  switch (Opcode) {
  default: return 0;
  case Instruction::Call:
    if (Function *F = dyn_cast<Function>(Ops[0]))
      if (canConstantFoldCallTo(F))
        return ConstantFoldCall(F, Ops+1, NumOps-1);
    return 0;
  case Instruction::ICmp:
  case Instruction::FCmp:
    llvm_unreachable("This function is invalid for compares: no predicate specified");
  case Instruction::PtrToInt:
    // If the input is a inttoptr, eliminate the pair.  This requires knowing
    // the width of a pointer, so it can't be done in ConstantExpr::getCast.
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(Ops[0])) {
      if (TD && CE->getOpcode() == Instruction::IntToPtr) {
        Constant *Input = CE->getOperand(0);
        unsigned InWidth = Input->getType()->getScalarSizeInBits();
        if (TD->getPointerSizeInBits() < InWidth) {
          Constant *Mask = 
            ConstantInt::get(Context, APInt::getLowBitsSet(InWidth,
                                                  TD->getPointerSizeInBits()));
          Input = ConstantExpr::getAnd(Input, Mask);
        }
        // Do a zext or trunc to get to the dest size.
        return ConstantExpr::getIntegerCast(Input, DestTy, false);
      }
    }
    return ConstantExpr::getCast(Opcode, Ops[0], DestTy);
  case Instruction::IntToPtr:
    // If the input is a ptrtoint, turn the pair into a ptr to ptr bitcast if
    // the int size is >= the ptr size.  This requires knowing the width of a
    // pointer, so it can't be done in ConstantExpr::getCast.
    if (ConstantExpr *CE = dyn_cast<ConstantExpr>(Ops[0])) {
      if (TD &&
          TD->getPointerSizeInBits() <=
          CE->getType()->getScalarSizeInBits()) {
        if (CE->getOpcode() == Instruction::PtrToInt) {
          Constant *Input = CE->getOperand(0);
          Constant *C = FoldBitCast(Input, DestTy, *TD, Context);
          return C ? C : ConstantExpr::getBitCast(Input, DestTy);
        }
        // If there's a constant offset added to the integer value before
        // it is casted back to a pointer, see if the expression can be
        // converted into a GEP.
        if (CE->getOpcode() == Instruction::Add)
          if (ConstantInt *L = dyn_cast<ConstantInt>(CE->getOperand(0)))
            if (ConstantExpr *R = dyn_cast<ConstantExpr>(CE->getOperand(1)))
              if (R->getOpcode() == Instruction::PtrToInt)
                if (GlobalVariable *GV =
                      dyn_cast<GlobalVariable>(R->getOperand(0))) {
                  const PointerType *GVTy = cast<PointerType>(GV->getType());
                  if (const ArrayType *AT =
                        dyn_cast<ArrayType>(GVTy->getElementType())) {
                    const Type *ElTy = AT->getElementType();
                    uint64_t AllocSize = TD->getTypeAllocSize(ElTy);
                    APInt PSA(L->getValue().getBitWidth(), AllocSize);
                    if (ElTy == cast<PointerType>(DestTy)->getElementType() &&
                        L->getValue().urem(PSA) == 0) {
                      APInt ElemIdx = L->getValue().udiv(PSA);
                      if (ElemIdx.ult(APInt(ElemIdx.getBitWidth(),
                                            AT->getNumElements()))) {
                        Constant *Index[] = {
                          Constant::getNullValue(CE->getType()),
                          ConstantInt::get(Context, ElemIdx)
                        };
                        return
                        ConstantExpr::getGetElementPtr(GV, &Index[0], 2);
                      }
                    }
                  }
                }
      }
    }
    return ConstantExpr::getCast(Opcode, Ops[0], DestTy);
  case Instruction::Trunc:
  case Instruction::ZExt:
  case Instruction::SExt:
  case Instruction::FPTrunc:
  case Instruction::FPExt:
  case Instruction::UIToFP:
  case Instruction::SIToFP:
  case Instruction::FPToUI:
  case Instruction::FPToSI:
      return ConstantExpr::getCast(Opcode, Ops[0], DestTy);
  case Instruction::BitCast:
    if (TD)
      if (Constant *C = FoldBitCast(Ops[0], DestTy, *TD, Context))
        return C;
    return ConstantExpr::getBitCast(Ops[0], DestTy);
  case Instruction::Select:
    return ConstantExpr::getSelect(Ops[0], Ops[1], Ops[2]);
  case Instruction::ExtractElement:
    return ConstantExpr::getExtractElement(Ops[0], Ops[1]);
  case Instruction::InsertElement:
    return ConstantExpr::getInsertElement(Ops[0], Ops[1], Ops[2]);
  case Instruction::ShuffleVector:
    return ConstantExpr::getShuffleVector(Ops[0], Ops[1], Ops[2]);
  case Instruction::GetElementPtr:
    if (Constant *C = SymbolicallyEvaluateGEP(Ops, NumOps, DestTy, Context, TD))
      return C;
    
    return ConstantExpr::getGetElementPtr(Ops[0], Ops+1, NumOps-1);
  }
}

/// ConstantFoldCompareInstOperands - Attempt to constant fold a compare
/// instruction (icmp/fcmp) with the specified operands.  If it fails, it
/// returns a constant expression of the specified operands.
///
Constant *llvm::ConstantFoldCompareInstOperands(unsigned Predicate,
                                                Constant*const * Ops, 
                                                unsigned NumOps,
                                                LLVMContext &Context,
                                                const TargetData *TD) {
  // fold: icmp (inttoptr x), null         -> icmp x, 0
  // fold: icmp (ptrtoint x), 0            -> icmp x, null
  // fold: icmp (inttoptr x), (inttoptr y) -> icmp trunc/zext x, trunc/zext y
  // fold: icmp (ptrtoint x), (ptrtoint y) -> icmp x, y
  //
  // ConstantExpr::getCompare cannot do this, because it doesn't have TD
  // around to know if bit truncation is happening.
  if (ConstantExpr *CE0 = dyn_cast<ConstantExpr>(Ops[0])) {
    if (TD && Ops[1]->isNullValue()) {
      const Type *IntPtrTy = TD->getIntPtrType(Context);
      if (CE0->getOpcode() == Instruction::IntToPtr) {
        // Convert the integer value to the right size to ensure we get the
        // proper extension or truncation.
        Constant *C = ConstantExpr::getIntegerCast(CE0->getOperand(0),
                                                   IntPtrTy, false);
        Constant *NewOps[] = { C, Constant::getNullValue(C->getType()) };
        return ConstantFoldCompareInstOperands(Predicate, NewOps, 2,
                                               Context, TD);
      }
      
      // Only do this transformation if the int is intptrty in size, otherwise
      // there is a truncation or extension that we aren't modeling.
      if (CE0->getOpcode() == Instruction::PtrToInt && 
          CE0->getType() == IntPtrTy) {
        Constant *C = CE0->getOperand(0);
        Constant *NewOps[] = { C, Constant::getNullValue(C->getType()) };
        // FIXME!
        return ConstantFoldCompareInstOperands(Predicate, NewOps, 2,
                                               Context, TD);
      }
    }
    
    if (ConstantExpr *CE1 = dyn_cast<ConstantExpr>(Ops[1])) {
      if (TD && CE0->getOpcode() == CE1->getOpcode()) {
        const Type *IntPtrTy = TD->getIntPtrType(Context);

        if (CE0->getOpcode() == Instruction::IntToPtr) {
          // Convert the integer value to the right size to ensure we get the
          // proper extension or truncation.
          Constant *C0 = ConstantExpr::getIntegerCast(CE0->getOperand(0),
                                                      IntPtrTy, false);
          Constant *C1 = ConstantExpr::getIntegerCast(CE1->getOperand(0),
                                                      IntPtrTy, false);
          Constant *NewOps[] = { C0, C1 };
          return ConstantFoldCompareInstOperands(Predicate, NewOps, 2, 
                                                 Context, TD);
        }

        // Only do this transformation if the int is intptrty in size, otherwise
        // there is a truncation or extension that we aren't modeling.
        if ((CE0->getOpcode() == Instruction::PtrToInt &&
             CE0->getType() == IntPtrTy &&
             CE0->getOperand(0)->getType() == CE1->getOperand(0)->getType())) {
          Constant *NewOps[] = { 
            CE0->getOperand(0), CE1->getOperand(0) 
          };
          return ConstantFoldCompareInstOperands(Predicate, NewOps, 2, 
                                                 Context, TD);
        }
      }
    }
  }
  return ConstantExpr::getCompare(Predicate, Ops[0], Ops[1]);
}


/// ConstantFoldLoadThroughGEPConstantExpr - Given a constant and a
/// getelementptr constantexpr, return the constant value being addressed by the
/// constant expression, or null if something is funny and we can't decide.
Constant *llvm::ConstantFoldLoadThroughGEPConstantExpr(Constant *C, 
                                                       ConstantExpr *CE) {
  if (CE->getOperand(1) != Constant::getNullValue(CE->getOperand(1)->getType()))
    return 0;  // Do not allow stepping over the value!
  
  // Loop over all of the operands, tracking down which value we are
  // addressing...
  gep_type_iterator I = gep_type_begin(CE), E = gep_type_end(CE);
  for (++I; I != E; ++I)
    if (const StructType *STy = dyn_cast<StructType>(*I)) {
      ConstantInt *CU = cast<ConstantInt>(I.getOperand());
      assert(CU->getZExtValue() < STy->getNumElements() &&
             "Struct index out of range!");
      unsigned El = (unsigned)CU->getZExtValue();
      if (ConstantStruct *CS = dyn_cast<ConstantStruct>(C)) {
        C = CS->getOperand(El);
      } else if (isa<ConstantAggregateZero>(C)) {
        C = Constant::getNullValue(STy->getElementType(El));
      } else if (isa<UndefValue>(C)) {
        C = UndefValue::get(STy->getElementType(El));
      } else {
        return 0;
      }
    } else if (ConstantInt *CI = dyn_cast<ConstantInt>(I.getOperand())) {
      if (const ArrayType *ATy = dyn_cast<ArrayType>(*I)) {
        if (CI->getZExtValue() >= ATy->getNumElements())
         return 0;
        if (ConstantArray *CA = dyn_cast<ConstantArray>(C))
          C = CA->getOperand(CI->getZExtValue());
        else if (isa<ConstantAggregateZero>(C))
          C = Constant::getNullValue(ATy->getElementType());
        else if (isa<UndefValue>(C))
          C = UndefValue::get(ATy->getElementType());
        else
          return 0;
      } else if (const VectorType *VTy = dyn_cast<VectorType>(*I)) {
        if (CI->getZExtValue() >= VTy->getNumElements())
          return 0;
        if (ConstantVector *CP = dyn_cast<ConstantVector>(C))
          C = CP->getOperand(CI->getZExtValue());
        else if (isa<ConstantAggregateZero>(C))
          C = Constant::getNullValue(VTy->getElementType());
        else if (isa<UndefValue>(C))
          C = UndefValue::get(VTy->getElementType());
        else
          return 0;
      } else {
        return 0;
      }
    } else {
      return 0;
    }
  return C;
}


//===----------------------------------------------------------------------===//
//  Constant Folding for Calls
//

/// canConstantFoldCallTo - Return true if its even possible to fold a call to
/// the specified function.
bool
llvm::canConstantFoldCallTo(const Function *F) {
  switch (F->getIntrinsicID()) {
  case Intrinsic::sqrt:
  case Intrinsic::powi:
  case Intrinsic::bswap:
  case Intrinsic::ctpop:
  case Intrinsic::ctlz:
  case Intrinsic::cttz:
  case Intrinsic::uadd_with_overflow:
  case Intrinsic::usub_with_overflow:
  case Intrinsic::sadd_with_overflow:
  case Intrinsic::ssub_with_overflow:
    return true;
  default:
    return false;
  case 0: break;
  }

  if (!F->hasName()) return false;
  StringRef Name = F->getName();
  
  // In these cases, the check of the length is required.  We don't want to
  // return true for a name like "cos\0blah" which strcmp would return equal to
  // "cos", but has length 8.
  switch (Name[0]) {
  default: return false;
  case 'a':
    return Name == "acos" || Name == "asin" || 
      Name == "atan" || Name == "atan2";
  case 'c':
    return Name == "cos" || Name == "ceil" || Name == "cosf" || Name == "cosh";
  case 'e':
    return Name == "exp";
  case 'f':
    return Name == "fabs" || Name == "fmod" || Name == "floor";
  case 'l':
    return Name == "log" || Name == "log10";
  case 'p':
    return Name == "pow";
  case 's':
    return Name == "sin" || Name == "sinh" || Name == "sqrt" ||
      Name == "sinf" || Name == "sqrtf";
  case 't':
    return Name == "tan" || Name == "tanh";
  }
}

static Constant *ConstantFoldFP(double (*NativeFP)(double), double V, 
                                const Type *Ty, LLVMContext &Context) {
  errno = 0;
  V = NativeFP(V);
  if (errno != 0) {
    errno = 0;
    return 0;
  }
  
  if (Ty->isFloatTy())
    return ConstantFP::get(Context, APFloat((float)V));
  if (Ty->isDoubleTy())
    return ConstantFP::get(Context, APFloat(V));
  llvm_unreachable("Can only constant fold float/double");
  return 0; // dummy return to suppress warning
}

static Constant *ConstantFoldBinaryFP(double (*NativeFP)(double, double),
                                      double V, double W,
                                      const Type *Ty,
                                      LLVMContext &Context) {
  errno = 0;
  V = NativeFP(V, W);
  if (errno != 0) {
    errno = 0;
    return 0;
  }
  
  if (Ty->isFloatTy())
    return ConstantFP::get(Context, APFloat((float)V));
  if (Ty->isDoubleTy())
    return ConstantFP::get(Context, APFloat(V));
  llvm_unreachable("Can only constant fold float/double");
  return 0; // dummy return to suppress warning
}

/// ConstantFoldCall - Attempt to constant fold a call to the specified function
/// with the specified arguments, returning null if unsuccessful.
Constant *
llvm::ConstantFoldCall(Function *F, 
                       Constant *const *Operands, unsigned NumOperands) {
  if (!F->hasName()) return 0;
  LLVMContext &Context = F->getContext();
  StringRef Name = F->getName();

  const Type *Ty = F->getReturnType();
  if (NumOperands == 1) {
    if (ConstantFP *Op = dyn_cast<ConstantFP>(Operands[0])) {
      if (!Ty->isFloatTy() && !Ty->isDoubleTy())
        return 0;
      /// Currently APFloat versions of these functions do not exist, so we use
      /// the host native double versions.  Float versions are not called
      /// directly but for all these it is true (float)(f((double)arg)) ==
      /// f(arg).  Long double not supported yet.
      double V = Ty->isFloatTy() ? (double)Op->getValueAPF().convertToFloat() :
                                     Op->getValueAPF().convertToDouble();
      switch (Name[0]) {
      case 'a':
        if (Name == "acos")
          return ConstantFoldFP(acos, V, Ty, Context);
        else if (Name == "asin")
          return ConstantFoldFP(asin, V, Ty, Context);
        else if (Name == "atan")
          return ConstantFoldFP(atan, V, Ty, Context);
        break;
      case 'c':
        if (Name == "ceil")
          return ConstantFoldFP(ceil, V, Ty, Context);
        else if (Name == "cos")
          return ConstantFoldFP(cos, V, Ty, Context);
        else if (Name == "cosh")
          return ConstantFoldFP(cosh, V, Ty, Context);
        else if (Name == "cosf")
          return ConstantFoldFP(cos, V, Ty, Context);
        break;
      case 'e':
        if (Name == "exp")
          return ConstantFoldFP(exp, V, Ty, Context);
        break;
      case 'f':
        if (Name == "fabs")
          return ConstantFoldFP(fabs, V, Ty, Context);
        else if (Name == "floor")
          return ConstantFoldFP(floor, V, Ty, Context);
        break;
      case 'l':
        if (Name == "log" && V > 0)
          return ConstantFoldFP(log, V, Ty, Context);
        else if (Name == "log10" && V > 0)
          return ConstantFoldFP(log10, V, Ty, Context);
        else if (Name == "llvm.sqrt.f32" ||
                 Name == "llvm.sqrt.f64") {
          if (V >= -0.0)
            return ConstantFoldFP(sqrt, V, Ty, Context);
          else // Undefined
            return Constant::getNullValue(Ty);
        }
        break;
      case 's':
        if (Name == "sin")
          return ConstantFoldFP(sin, V, Ty, Context);
        else if (Name == "sinh")
          return ConstantFoldFP(sinh, V, Ty, Context);
        else if (Name == "sqrt" && V >= 0)
          return ConstantFoldFP(sqrt, V, Ty, Context);
        else if (Name == "sqrtf" && V >= 0)
          return ConstantFoldFP(sqrt, V, Ty, Context);
        else if (Name == "sinf")
          return ConstantFoldFP(sin, V, Ty, Context);
        break;
      case 't':
        if (Name == "tan")
          return ConstantFoldFP(tan, V, Ty, Context);
        else if (Name == "tanh")
          return ConstantFoldFP(tanh, V, Ty, Context);
        break;
      default:
        break;
      }
      return 0;
    }
    
    
    if (ConstantInt *Op = dyn_cast<ConstantInt>(Operands[0])) {
      if (Name.startswith("llvm.bswap"))
        return ConstantInt::get(Context, Op->getValue().byteSwap());
      else if (Name.startswith("llvm.ctpop"))
        return ConstantInt::get(Ty, Op->getValue().countPopulation());
      else if (Name.startswith("llvm.cttz"))
        return ConstantInt::get(Ty, Op->getValue().countTrailingZeros());
      else if (Name.startswith("llvm.ctlz"))
        return ConstantInt::get(Ty, Op->getValue().countLeadingZeros());
      return 0;
    }
    
    return 0;
  }
  
  if (NumOperands == 2) {
    if (ConstantFP *Op1 = dyn_cast<ConstantFP>(Operands[0])) {
      if (!Ty->isFloatTy() && !Ty->isDoubleTy())
        return 0;
      double Op1V = Ty->isFloatTy() ? 
                      (double)Op1->getValueAPF().convertToFloat() :
                      Op1->getValueAPF().convertToDouble();
      if (ConstantFP *Op2 = dyn_cast<ConstantFP>(Operands[1])) {
        if (Op2->getType() != Op1->getType())
          return 0;
        
        double Op2V = Ty->isFloatTy() ? 
                      (double)Op2->getValueAPF().convertToFloat():
                      Op2->getValueAPF().convertToDouble();

        if (Name == "pow")
          return ConstantFoldBinaryFP(pow, Op1V, Op2V, Ty, Context);
        if (Name == "fmod")
          return ConstantFoldBinaryFP(fmod, Op1V, Op2V, Ty, Context);
        if (Name == "atan2")
          return ConstantFoldBinaryFP(atan2, Op1V, Op2V, Ty, Context);
      } else if (ConstantInt *Op2C = dyn_cast<ConstantInt>(Operands[1])) {
        if (Name == "llvm.powi.f32")
          return ConstantFP::get(Context, APFloat((float)std::pow((float)Op1V,
                                                 (int)Op2C->getZExtValue())));
        if (Name == "llvm.powi.f64")
          return ConstantFP::get(Context, APFloat((double)std::pow((double)Op1V,
                                                 (int)Op2C->getZExtValue())));
      }
      return 0;
    }
    
    
    if (ConstantInt *Op1 = dyn_cast<ConstantInt>(Operands[0])) {
      if (ConstantInt *Op2 = dyn_cast<ConstantInt>(Operands[1])) {
        switch (F->getIntrinsicID()) {
        default: break;
        case Intrinsic::uadd_with_overflow: {
          Constant *Res = ConstantExpr::getAdd(Op1, Op2);           // result.
          Constant *Ops[] = {
            Res, ConstantExpr::getICmp(CmpInst::ICMP_ULT, Res, Op1) // overflow.
          };
          return ConstantStruct::get(F->getContext(), Ops, 2, false);
        }
        case Intrinsic::usub_with_overflow: {
          Constant *Res = ConstantExpr::getSub(Op1, Op2);           // result.
          Constant *Ops[] = {
            Res, ConstantExpr::getICmp(CmpInst::ICMP_UGT, Res, Op1) // overflow.
          };
          return ConstantStruct::get(F->getContext(), Ops, 2, false);
        }
        case Intrinsic::sadd_with_overflow: {
          Constant *Res = ConstantExpr::getAdd(Op1, Op2);           // result.
          Constant *Overflow = ConstantExpr::getSelect(
              ConstantExpr::getICmp(CmpInst::ICMP_SGT,
                ConstantInt::get(Op1->getType(), 0), Op1),
              ConstantExpr::getICmp(CmpInst::ICMP_SGT, Res, Op2), 
              ConstantExpr::getICmp(CmpInst::ICMP_SLT, Res, Op2)); // overflow.

          Constant *Ops[] = { Res, Overflow };
          return ConstantStruct::get(F->getContext(), Ops, 2, false);
        }
        case Intrinsic::ssub_with_overflow: {
          Constant *Res = ConstantExpr::getSub(Op1, Op2);           // result.
          Constant *Overflow = ConstantExpr::getSelect(
              ConstantExpr::getICmp(CmpInst::ICMP_SGT,
                ConstantInt::get(Op2->getType(), 0), Op2),
              ConstantExpr::getICmp(CmpInst::ICMP_SLT, Res, Op1), 
              ConstantExpr::getICmp(CmpInst::ICMP_SGT, Res, Op1)); // overflow.

          Constant *Ops[] = { Res, Overflow };
          return ConstantStruct::get(F->getContext(), Ops, 2, false);
        }
        }
      }
      
      return 0;
    }
    return 0;
  }
  return 0;
}

