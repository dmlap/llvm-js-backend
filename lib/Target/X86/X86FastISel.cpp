//===-- X86FastISel.cpp - X86 FastISel implementation ---------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines the X86-specific support for the FastISel class. Much
// of the target-specific code is generated by tablegen in the file
// X86GenFastISel.inc, which is #included here.
//
//===----------------------------------------------------------------------===//

#include "X86.h"
#include "X86InstrBuilder.h"
#include "X86ISelLowering.h"
#include "X86RegisterInfo.h"
#include "X86Subtarget.h"
#include "X86TargetMachine.h"
#include "llvm/CallingConv.h"
#include "llvm/DerivedTypes.h"
#include "llvm/Instructions.h"
#include "llvm/CodeGen/FastISel.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CallSite.h"
#include "llvm/Support/GetElementPtrTypeIterator.h"

using namespace llvm;

class X86FastISel : public FastISel {
  /// Subtarget - Keep a pointer to the X86Subtarget around so that we can
  /// make the right decision when generating code for different targets.
  const X86Subtarget *Subtarget;

  /// StackPtr - Register used as the stack pointer.
  ///
  unsigned StackPtr;

  /// X86ScalarSSEf32, X86ScalarSSEf64 - Select between SSE or x87 
  /// floating point ops.
  /// When SSE is available, use it for f32 operations.
  /// When SSE2 is available, use it for f64 operations.
  bool X86ScalarSSEf64;
  bool X86ScalarSSEf32;

public:
  explicit X86FastISel(MachineFunction &mf,
                       DenseMap<const Value *, unsigned> &vm,
                       DenseMap<const BasicBlock *, MachineBasicBlock *> &bm,
                       DenseMap<const AllocaInst *, int> &am)
    : FastISel(mf, vm, bm, am) {
    Subtarget = &TM.getSubtarget<X86Subtarget>();
    StackPtr = Subtarget->is64Bit() ? X86::RSP : X86::ESP;
    X86ScalarSSEf64 = Subtarget->hasSSE2();
    X86ScalarSSEf32 = Subtarget->hasSSE1();
  }

  virtual bool TargetSelectInstruction(Instruction *I);

#include "X86GenFastISel.inc"

private:
  bool X86FastEmitLoad(MVT VT, const X86AddressMode &AM, unsigned &RR);

  bool X86FastEmitStore(MVT VT, unsigned Val,
                        const X86AddressMode &AM);

  bool X86FastEmitExtend(ISD::NodeType Opc, MVT DstVT, unsigned Src, MVT SrcVT,
                         unsigned &ResultReg);
  
  bool X86SelectAddress(Value *V, X86AddressMode &AM, bool isCall);

  bool X86SelectLoad(Instruction *I);
  
  bool X86SelectStore(Instruction *I);

  bool X86SelectCmp(Instruction *I);

  bool X86SelectZExt(Instruction *I);

  bool X86SelectBranch(Instruction *I);

  bool X86SelectShift(Instruction *I);

  bool X86SelectSelect(Instruction *I);

  bool X86SelectTrunc(Instruction *I);

  bool X86SelectFPExt(Instruction *I);
  bool X86SelectFPTrunc(Instruction *I);

  bool X86SelectCall(Instruction *I);

  CCAssignFn *CCAssignFnForCall(unsigned CC, bool isTailCall = false);

  unsigned TargetMaterializeConstant(Constant *C);

  unsigned TargetMaterializeAlloca(AllocaInst *C);

  /// isScalarFPTypeInSSEReg - Return true if the specified scalar FP type is
  /// computed in an SSE register, not on the X87 floating point stack.
  bool isScalarFPTypeInSSEReg(MVT VT) const {
    return (VT == MVT::f64 && X86ScalarSSEf64) || // f64 is when SSE2
      (VT == MVT::f32 && X86ScalarSSEf32);   // f32 is when SSE1
  }

};

static bool isTypeLegal(const Type *Ty, const TargetLowering &TLI, MVT &VT,
                        bool AllowI1 = false) {
  VT = MVT::getMVT(Ty, /*HandleUnknown=*/true);
  if (VT == MVT::Other || !VT.isSimple())
    // Unhandled type. Halt "fast" selection and bail.
    return false;
  if (VT == MVT::iPTR)
    // Use pointer type.
    VT = TLI.getPointerTy();
  // We only handle legal types. For example, on x86-32 the instruction
  // selector contains all of the 64-bit instructions from x86-64,
  // under the assumption that i64 won't be used if the target doesn't
  // support it.
  return (AllowI1 && VT == MVT::i1) || TLI.isTypeLegal(VT);
}

#include "X86GenCallingConv.inc"

/// CCAssignFnForCall - Selects the correct CCAssignFn for a given calling
/// convention.
CCAssignFn *X86FastISel::CCAssignFnForCall(unsigned CC, bool isTaillCall) {
  if (Subtarget->is64Bit()) {
    if (Subtarget->isTargetWin64())
      return CC_X86_Win64_C;
    else if (CC == CallingConv::Fast && isTaillCall)
      return CC_X86_64_TailCall;
    else
      return CC_X86_64_C;
  }

  if (CC == CallingConv::X86_FastCall)
    return CC_X86_32_FastCall;
  else if (CC == CallingConv::Fast && isTaillCall)
    return CC_X86_32_TailCall;
  else if (CC == CallingConv::Fast)
    return CC_X86_32_FastCC;
  else
    return CC_X86_32_C;
}

/// X86FastEmitLoad - Emit a machine instruction to load a value of type VT.
/// The address is either pre-computed, i.e. Ptr, or a GlobalAddress, i.e. GV.
/// Return true and the result register by reference if it is possible.
bool X86FastISel::X86FastEmitLoad(MVT VT, const X86AddressMode &AM,
                                  unsigned &ResultReg) {
  // Get opcode and regclass of the output for the given load instruction.
  unsigned Opc = 0;
  const TargetRegisterClass *RC = NULL;
  switch (VT.getSimpleVT()) {
  default: return false;
  case MVT::i8:
    Opc = X86::MOV8rm;
    RC  = X86::GR8RegisterClass;
    break;
  case MVT::i16:
    Opc = X86::MOV16rm;
    RC  = X86::GR16RegisterClass;
    break;
  case MVT::i32:
    Opc = X86::MOV32rm;
    RC  = X86::GR32RegisterClass;
    break;
  case MVT::i64:
    // Must be in x86-64 mode.
    Opc = X86::MOV64rm;
    RC  = X86::GR64RegisterClass;
    break;
  case MVT::f32:
    if (Subtarget->hasSSE1()) {
      Opc = X86::MOVSSrm;
      RC  = X86::FR32RegisterClass;
    } else {
      Opc = X86::LD_Fp32m;
      RC  = X86::RFP32RegisterClass;
    }
    break;
  case MVT::f64:
    if (Subtarget->hasSSE2()) {
      Opc = X86::MOVSDrm;
      RC  = X86::FR64RegisterClass;
    } else {
      Opc = X86::LD_Fp64m;
      RC  = X86::RFP64RegisterClass;
    }
    break;
  case MVT::f80:
    Opc = X86::LD_Fp80m;
    RC  = X86::RFP80RegisterClass;
    break;
  }

  ResultReg = createResultReg(RC);
  addFullAddress(BuildMI(MBB, TII.get(Opc), ResultReg), AM);
  return true;
}

/// X86FastEmitStore - Emit a machine instruction to store a value Val of
/// type VT. The address is either pre-computed, consisted of a base ptr, Ptr
/// and a displacement offset, or a GlobalAddress,
/// i.e. V. Return true if it is possible.
bool
X86FastISel::X86FastEmitStore(MVT VT, unsigned Val,
                              const X86AddressMode &AM) {
  // Get opcode and regclass of the output for the given store instruction.
  unsigned Opc = 0;
  const TargetRegisterClass *RC = NULL;
  switch (VT.getSimpleVT()) {
  default: return false;
  case MVT::i8:
    Opc = X86::MOV8mr;
    RC  = X86::GR8RegisterClass;
    break;
  case MVT::i16:
    Opc = X86::MOV16mr;
    RC  = X86::GR16RegisterClass;
    break;
  case MVT::i32:
    Opc = X86::MOV32mr;
    RC  = X86::GR32RegisterClass;
    break;
  case MVT::i64:
    // Must be in x86-64 mode.
    Opc = X86::MOV64mr;
    RC  = X86::GR64RegisterClass;
    break;
  case MVT::f32:
    if (Subtarget->hasSSE1()) {
      Opc = X86::MOVSSmr;
      RC  = X86::FR32RegisterClass;
    } else {
      Opc = X86::ST_Fp32m;
      RC  = X86::RFP32RegisterClass;
    }
    break;
  case MVT::f64:
    if (Subtarget->hasSSE2()) {
      Opc = X86::MOVSDmr;
      RC  = X86::FR64RegisterClass;
    } else {
      Opc = X86::ST_Fp64m;
      RC  = X86::RFP64RegisterClass;
    }
    break;
  case MVT::f80:
    Opc = X86::ST_FP80m;
    RC  = X86::RFP80RegisterClass;
    break;
  }

  addFullAddress(BuildMI(MBB, TII.get(Opc)), AM).addReg(Val);
  return true;
}

/// X86FastEmitExtend - Emit a machine instruction to extend a value Src of
/// type SrcVT to type DstVT using the specified extension opcode Opc (e.g.
/// ISD::SIGN_EXTEND).
bool X86FastISel::X86FastEmitExtend(ISD::NodeType Opc, MVT DstVT,
                                    unsigned Src, MVT SrcVT,
                                    unsigned &ResultReg) {
  unsigned RR = FastEmit_r(SrcVT.getSimpleVT(), DstVT.getSimpleVT(), Opc, Src);
  
  if (RR != 0) {
    ResultReg = RR;
    return true;
  } else
    return false;
}

/// X86SelectAddress - Attempt to fill in an address from the given value.
///
bool X86FastISel::X86SelectAddress(Value *V, X86AddressMode &AM, bool isCall) {
  User *U;
  unsigned Opcode = Instruction::UserOp1;
  if (Instruction *I = dyn_cast<Instruction>(V)) {
    Opcode = I->getOpcode();
    U = I;
  } else if (ConstantExpr *C = dyn_cast<ConstantExpr>(V)) {
    Opcode = C->getOpcode();
    U = C;
  }

  switch (Opcode) {
  default: break;
  case Instruction::BitCast:
    // Look past bitcasts.
    return X86SelectAddress(U->getOperand(0), AM, isCall);

  case Instruction::IntToPtr:
    // Look past no-op inttoptrs.
    if (TLI.getValueType(U->getOperand(0)->getType()) == TLI.getPointerTy())
      return X86SelectAddress(U->getOperand(0), AM, isCall);

  case Instruction::PtrToInt:
    // Look past no-op ptrtoints.
    if (TLI.getValueType(U->getType()) == TLI.getPointerTy())
      return X86SelectAddress(U->getOperand(0), AM, isCall);

  case Instruction::Alloca: {
    if (isCall) break;
    // Do static allocas.
    const AllocaInst *A = cast<AllocaInst>(V);
    DenseMap<const AllocaInst*, int>::iterator SI = StaticAllocaMap.find(A);
    if (SI == StaticAllocaMap.end())
      return false;
    AM.BaseType = X86AddressMode::FrameIndexBase;
    AM.Base.FrameIndex = SI->second;
    return true;
  }

  case Instruction::Add: {
    if (isCall) break;
    // Adds of constants are common and easy enough.
    if (ConstantInt *CI = dyn_cast<ConstantInt>(U->getOperand(1))) {
      AM.Disp += CI->getZExtValue();
      return X86SelectAddress(U->getOperand(0), AM, isCall);
    }
    break;
  }

  case Instruction::GetElementPtr: {
    if (isCall) break;
    // Pattern-match simple GEPs.
    uint64_t Disp = AM.Disp;
    unsigned IndexReg = AM.IndexReg;
    unsigned Scale = AM.Scale;
    gep_type_iterator GTI = gep_type_begin(U);
    // Look at all but the last index. Constants can be folded,
    // and one dynamic index can be handled, if the scale is supported.
    for (User::op_iterator i = U->op_begin() + 1, e = U->op_end();
         i != e; ++i, ++GTI) {
      Value *Op = *i;
      if (const StructType *STy = dyn_cast<StructType>(*GTI)) {
        const StructLayout *SL = TD.getStructLayout(STy);
        unsigned Idx = cast<ConstantInt>(Op)->getZExtValue();
        Disp += SL->getElementOffset(Idx);
      } else {
        uint64_t S = TD.getABITypeSize(GTI.getIndexedType());
        if (ConstantInt *CI = dyn_cast<ConstantInt>(Op)) {
          // Constant-offset addressing.
          Disp += CI->getZExtValue() * S;
        } else if (IndexReg == 0 &&
                   (S == 1 || S == 2 || S == 4 || S == 8)) {
          // Scaled-index addressing.
          Scale = S;
          IndexReg = getRegForValue(Op);
          if (IndexReg == 0)
            return false;
        } else
          // Unsupported.
          goto unsupported_gep;
      }
    }
    // Ok, the GEP indices were covered by constant-offset and scaled-index
    // addressing. Update the address state and move on to examining the base.
    AM.IndexReg = IndexReg;
    AM.Scale = Scale;
    AM.Disp = Disp;
    return X86SelectAddress(U->getOperand(0), AM, isCall);
  unsupported_gep:
    // Ok, the GEP indices weren't all covered.
    break;
  }
  }

  // Handle constant address.
  if (GlobalValue *GV = dyn_cast<GlobalValue>(V)) {
    if (Subtarget->GVRequiresExtraLoad(GV, TM, isCall)) {
      // Check to see if we've already materialized this
      // value in a register in this block.
      if (unsigned Reg = LocalValueMap[V])
        return Reg;
      // Issue load from stub if necessary.
      unsigned Opc = 0;
      const TargetRegisterClass *RC = NULL;
      if (TLI.getPointerTy() == MVT::i32) {
        Opc = X86::MOV32rm;
        RC  = X86::GR32RegisterClass;
      } else {
        Opc = X86::MOV64rm;
        RC  = X86::GR64RegisterClass;
      }
      AM.Base.Reg = createResultReg(RC);
      X86AddressMode LocalAM;
      LocalAM.GV = GV;
      addFullAddress(BuildMI(MBB, TII.get(Opc), AM.Base.Reg), LocalAM);
      // Prevent loading GV stub multiple times in same MBB.
      LocalValueMap[V] = AM.Base.Reg;
    } else {
      AM.GV = GV;
    }
    return true;
  }

  // If all else fails, just materialize the value in a register.
  AM.Base.Reg = getRegForValue(V);
  return AM.Base.Reg != 0;
}

/// X86SelectStore - Select and emit code to implement store instructions.
bool X86FastISel::X86SelectStore(Instruction* I) {
  MVT VT;
  if (!isTypeLegal(I->getOperand(0)->getType(), TLI, VT))
    return false;
  unsigned Val = getRegForValue(I->getOperand(0));
  if (Val == 0)
    // Unhandled operand. Halt "fast" selection and bail.
    return false;    

  X86AddressMode AM;
  if (!X86SelectAddress(I->getOperand(1), AM, false))
    return false;

  return X86FastEmitStore(VT, Val, AM);
}

/// X86SelectLoad - Select and emit code to implement load instructions.
///
bool X86FastISel::X86SelectLoad(Instruction *I)  {
  MVT VT;
  if (!isTypeLegal(I->getType(), TLI, VT))
    return false;

  X86AddressMode AM;
  if (!X86SelectAddress(I->getOperand(0), AM, false))
    return false;

  unsigned ResultReg = 0;
  if (X86FastEmitLoad(VT, AM, ResultReg)) {
    UpdateValueMap(I, ResultReg);
    return true;
  }
  return false;
}

bool X86FastISel::X86SelectCmp(Instruction *I) {
  CmpInst *CI = cast<CmpInst>(I);

  MVT VT = TLI.getValueType(I->getOperand(0)->getType());
  if (!TLI.isTypeLegal(VT))
    return false;

  unsigned Op0Reg = getRegForValue(CI->getOperand(0));
  if (Op0Reg == 0) return false;
  unsigned Op1Reg = getRegForValue(CI->getOperand(1));
  if (Op1Reg == 0) return false;

  unsigned Opc;
  switch (VT.getSimpleVT()) {
  case MVT::i8: Opc = X86::CMP8rr; break;
  case MVT::i16: Opc = X86::CMP16rr; break;
  case MVT::i32: Opc = X86::CMP32rr; break;
  case MVT::i64: Opc = X86::CMP64rr; break;
  case MVT::f32: Opc = X86::UCOMISSrr; break;
  case MVT::f64: Opc = X86::UCOMISDrr; break;
  default: return false;
  }

  unsigned ResultReg = createResultReg(&X86::GR8RegClass);
  switch (CI->getPredicate()) {
  case CmpInst::FCMP_OEQ: {
    unsigned EReg = createResultReg(&X86::GR8RegClass);
    unsigned NPReg = createResultReg(&X86::GR8RegClass);
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETEr), EReg);
    BuildMI(MBB, TII.get(X86::SETNPr), NPReg);
    BuildMI(MBB, TII.get(X86::AND8rr), ResultReg).addReg(NPReg).addReg(EReg);
    break;
  }
  case CmpInst::FCMP_UNE: {
    unsigned NEReg = createResultReg(&X86::GR8RegClass);
    unsigned PReg = createResultReg(&X86::GR8RegClass);
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETNEr), NEReg);
    BuildMI(MBB, TII.get(X86::SETPr), PReg);
    BuildMI(MBB, TII.get(X86::OR8rr), ResultReg).addReg(PReg).addReg(NEReg);
    break;
  }
  case CmpInst::FCMP_OGT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETAr), ResultReg);
    break;
  case CmpInst::FCMP_OGE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETAEr), ResultReg);
    break;
  case CmpInst::FCMP_OLT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op1Reg).addReg(Op0Reg);
    BuildMI(MBB, TII.get(X86::SETAr), ResultReg);
    break;
  case CmpInst::FCMP_OLE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op1Reg).addReg(Op0Reg);
    BuildMI(MBB, TII.get(X86::SETAEr), ResultReg);
    break;
  case CmpInst::FCMP_ONE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETNEr), ResultReg);
    break;
  case CmpInst::FCMP_ORD:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETNPr), ResultReg);
    break;
  case CmpInst::FCMP_UNO:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETPr), ResultReg);
    break;
  case CmpInst::FCMP_UEQ:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETEr), ResultReg);
    break;
  case CmpInst::FCMP_UGT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op1Reg).addReg(Op0Reg);
    BuildMI(MBB, TII.get(X86::SETBr), ResultReg);
    break;
  case CmpInst::FCMP_UGE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op1Reg).addReg(Op0Reg);
    BuildMI(MBB, TII.get(X86::SETBEr), ResultReg);
    break;
  case CmpInst::FCMP_ULT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETBr), ResultReg);
    break;
  case CmpInst::FCMP_ULE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETBEr), ResultReg);
    break;
  case CmpInst::ICMP_EQ:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETEr), ResultReg);
    break;
  case CmpInst::ICMP_NE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETNEr), ResultReg);
    break;
  case CmpInst::ICMP_UGT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETAr), ResultReg);
    break;
  case CmpInst::ICMP_UGE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETAEr), ResultReg);
    break;
  case CmpInst::ICMP_ULT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETBr), ResultReg);
    break;
  case CmpInst::ICMP_ULE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETBEr), ResultReg);
    break;
  case CmpInst::ICMP_SGT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETGr), ResultReg);
    break;
  case CmpInst::ICMP_SGE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETGEr), ResultReg);
    break;
  case CmpInst::ICMP_SLT:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETLr), ResultReg);
    break;
  case CmpInst::ICMP_SLE:
    BuildMI(MBB, TII.get(Opc)).addReg(Op0Reg).addReg(Op1Reg);
    BuildMI(MBB, TII.get(X86::SETLEr), ResultReg);
    break;
  default:
    return false;
  }

  UpdateValueMap(I, ResultReg);
  return true;
}

bool X86FastISel::X86SelectZExt(Instruction *I) {
  // Special-case hack: The only i1 values we know how to produce currently
  // set the upper bits of an i8 value to zero.
  if (I->getType() == Type::Int8Ty &&
      I->getOperand(0)->getType() == Type::Int1Ty) {
    unsigned ResultReg = getRegForValue(I->getOperand(0));
    if (ResultReg == 0) return false;
    UpdateValueMap(I, ResultReg);
    return true;
  }

  return false;
}

bool X86FastISel::X86SelectBranch(Instruction *I) {
  BranchInst *BI = cast<BranchInst>(I);
  // Unconditional branches are selected by tablegen-generated code.
  unsigned OpReg = getRegForValue(BI->getCondition());
  if (OpReg == 0) return false;
  MachineBasicBlock *TrueMBB = MBBMap[BI->getSuccessor(0)];
  MachineBasicBlock *FalseMBB = MBBMap[BI->getSuccessor(1)];

  BuildMI(MBB, TII.get(X86::TEST8rr)).addReg(OpReg).addReg(OpReg);
  BuildMI(MBB, TII.get(X86::JNE)).addMBB(TrueMBB);
  BuildMI(MBB, TII.get(X86::JMP)).addMBB(FalseMBB);

  MBB->addSuccessor(TrueMBB);
  MBB->addSuccessor(FalseMBB);

  return true;
}

bool X86FastISel::X86SelectShift(Instruction *I) {
  unsigned CReg = 0;
  unsigned Opc = 0;
  const TargetRegisterClass *RC = NULL;
  if (I->getType() == Type::Int8Ty) {
    CReg = X86::CL;
    RC = &X86::GR8RegClass;
    switch (I->getOpcode()) {
    case Instruction::LShr: Opc = X86::SHR8rCL; break;
    case Instruction::AShr: Opc = X86::SAR8rCL; break;
    case Instruction::Shl:  Opc = X86::SHL8rCL; break;
    default: return false;
    }
  } else if (I->getType() == Type::Int16Ty) {
    CReg = X86::CX;
    RC = &X86::GR16RegClass;
    switch (I->getOpcode()) {
    case Instruction::LShr: Opc = X86::SHR16rCL; break;
    case Instruction::AShr: Opc = X86::SAR16rCL; break;
    case Instruction::Shl:  Opc = X86::SHL16rCL; break;
    default: return false;
    }
  } else if (I->getType() == Type::Int32Ty) {
    CReg = X86::ECX;
    RC = &X86::GR32RegClass;
    switch (I->getOpcode()) {
    case Instruction::LShr: Opc = X86::SHR32rCL; break;
    case Instruction::AShr: Opc = X86::SAR32rCL; break;
    case Instruction::Shl:  Opc = X86::SHL32rCL; break;
    default: return false;
    }
  } else if (I->getType() == Type::Int64Ty) {
    CReg = X86::RCX;
    RC = &X86::GR64RegClass;
    switch (I->getOpcode()) {
    case Instruction::LShr: Opc = X86::SHR64rCL; break;
    case Instruction::AShr: Opc = X86::SAR64rCL; break;
    case Instruction::Shl:  Opc = X86::SHL64rCL; break;
    default: return false;
    }
  } else {
    return false;
  }

  MVT VT = MVT::getMVT(I->getType(), /*HandleUnknown=*/true);
  if (VT == MVT::Other || !TLI.isTypeLegal(VT))
    return false;

  unsigned Op0Reg = getRegForValue(I->getOperand(0));
  if (Op0Reg == 0) return false;
  unsigned Op1Reg = getRegForValue(I->getOperand(1));
  if (Op1Reg == 0) return false;
  TII.copyRegToReg(*MBB, MBB->end(), CReg, Op1Reg, RC, RC);
  unsigned ResultReg = createResultReg(RC);
  BuildMI(MBB, TII.get(Opc), ResultReg).addReg(Op0Reg);
  UpdateValueMap(I, ResultReg);
  return true;
}

bool X86FastISel::X86SelectSelect(Instruction *I) {
  const Type *Ty = I->getType();
  if (isa<PointerType>(Ty))
    Ty = TD.getIntPtrType();

  unsigned Opc = 0;
  const TargetRegisterClass *RC = NULL;
  if (Ty == Type::Int16Ty) {
    Opc = X86::CMOVE16rr;
    RC = &X86::GR16RegClass;
  } else if (Ty == Type::Int32Ty) {
    Opc = X86::CMOVE32rr;
    RC = &X86::GR32RegClass;
  } else if (Ty == Type::Int64Ty) {
    Opc = X86::CMOVE64rr;
    RC = &X86::GR64RegClass;
  } else {
    return false; 
  }

  MVT VT = MVT::getMVT(Ty, /*HandleUnknown=*/true);
  if (VT == MVT::Other || !TLI.isTypeLegal(VT))
    return false;

  unsigned Op0Reg = getRegForValue(I->getOperand(0));
  if (Op0Reg == 0) return false;
  unsigned Op1Reg = getRegForValue(I->getOperand(1));
  if (Op1Reg == 0) return false;
  unsigned Op2Reg = getRegForValue(I->getOperand(2));
  if (Op2Reg == 0) return false;

  BuildMI(MBB, TII.get(X86::TEST8rr)).addReg(Op0Reg).addReg(Op0Reg);
  unsigned ResultReg = createResultReg(RC);
  BuildMI(MBB, TII.get(Opc), ResultReg).addReg(Op1Reg).addReg(Op2Reg);
  UpdateValueMap(I, ResultReg);
  return true;
}

bool X86FastISel::X86SelectFPExt(Instruction *I) {
  if (Subtarget->hasSSE2()) {
    if (I->getType() == Type::DoubleTy) {
      Value *V = I->getOperand(0);
      if (V->getType() == Type::FloatTy) {
        unsigned OpReg = getRegForValue(V);
        if (OpReg == 0) return false;
        unsigned ResultReg = createResultReg(X86::FR64RegisterClass);
        BuildMI(MBB, TII.get(X86::CVTSS2SDrr), ResultReg).addReg(OpReg);
        UpdateValueMap(I, ResultReg);
        return true;
      }
    }
  }

  return false;
}

bool X86FastISel::X86SelectFPTrunc(Instruction *I) {
  if (Subtarget->hasSSE2()) {
    if (I->getType() == Type::FloatTy) {
      Value *V = I->getOperand(0);
      if (V->getType() == Type::DoubleTy) {
        unsigned OpReg = getRegForValue(V);
        if (OpReg == 0) return false;
        unsigned ResultReg = createResultReg(X86::FR32RegisterClass);
        BuildMI(MBB, TII.get(X86::CVTSD2SSrr), ResultReg).addReg(OpReg);
        UpdateValueMap(I, ResultReg);
        return true;
      }
    }
  }

  return false;
}

bool X86FastISel::X86SelectTrunc(Instruction *I) {
  if (Subtarget->is64Bit())
    // All other cases should be handled by the tblgen generated code.
    return false;
  MVT SrcVT = TLI.getValueType(I->getOperand(0)->getType());
  MVT DstVT = TLI.getValueType(I->getType());
  if (DstVT != MVT::i8)
    // All other cases should be handled by the tblgen generated code.
    return false;
  if (SrcVT != MVT::i16 && SrcVT != MVT::i32)
    // All other cases should be handled by the tblgen generated code.
    return false;

  unsigned InputReg = getRegForValue(I->getOperand(0));
  if (!InputReg)
    // Unhandled operand.  Halt "fast" selection and bail.
    return false;

  // First issue a copy to GR16_ or GR32_.
  unsigned CopyOpc = (SrcVT == MVT::i16) ? X86::MOV16to16_ : X86::MOV32to32_;
  const TargetRegisterClass *CopyRC = (SrcVT == MVT::i16)
    ? X86::GR16_RegisterClass : X86::GR32_RegisterClass;
  unsigned CopyReg = createResultReg(CopyRC);
  BuildMI(MBB, TII.get(CopyOpc), CopyReg).addReg(InputReg);

  // Then issue an extract_subreg.
  unsigned ResultReg = FastEmitInst_extractsubreg(CopyReg,1); // x86_subreg_8bit
  if (!ResultReg)
    return false;

  UpdateValueMap(I, ResultReg);
  return true;
}

bool X86FastISel::X86SelectCall(Instruction *I) {
  CallInst *CI = cast<CallInst>(I);
  Value *Callee = I->getOperand(0);

  // Can't handle inline asm yet.
  if (isa<InlineAsm>(Callee))
    return false;

  // FIXME: Handle some intrinsics.
  if (Function *F = CI->getCalledFunction()) {
    if (F->isDeclaration() &&F->getIntrinsicID())
      return false;
  }

  // Handle only C and fastcc calling conventions for now.
  CallSite CS(CI);
  unsigned CC = CS.getCallingConv();
  if (CC != CallingConv::C &&
      CC != CallingConv::Fast &&
      CC != CallingConv::X86_FastCall)
    return false;

  // Let SDISel handle vararg functions.
  const PointerType *PT = cast<PointerType>(CS.getCalledValue()->getType());
  const FunctionType *FTy = cast<FunctionType>(PT->getElementType());
  if (FTy->isVarArg())
    return false;

  // Handle *simple* calls for now.
  const Type *RetTy = CS.getType();
  MVT RetVT;
  if (RetTy == Type::VoidTy)
    RetVT = MVT::isVoid;
  else if (!isTypeLegal(RetTy, TLI, RetVT, true))
    return false;

  // Materialize callee address in a register. FIXME: GV address can be
  // handled with a CALLpcrel32 instead.
  X86AddressMode CalleeAM;
  if (!X86SelectAddress(Callee, CalleeAM, true))
    return false;
  unsigned CalleeOp = 0;
  GlobalValue *GV = 0;
  if (CalleeAM.Base.Reg != 0) {
    assert(CalleeAM.GV == 0);
    CalleeOp = CalleeAM.Base.Reg;
  } else if (CalleeAM.GV != 0) {
    assert(CalleeAM.GV != 0);
    GV = CalleeAM.GV;
  } else
    return false;

  // Allow calls which produce i1 results.
  bool AndToI1 = false;
  if (RetVT == MVT::i1) {
    RetVT = MVT::i8;
    AndToI1 = true;
  }

  // Deal with call operands first.
  SmallVector<unsigned, 4> Args;
  SmallVector<MVT, 4> ArgVTs;
  SmallVector<ISD::ArgFlagsTy, 4> ArgFlags;
  Args.reserve(CS.arg_size());
  ArgVTs.reserve(CS.arg_size());
  ArgFlags.reserve(CS.arg_size());
  for (CallSite::arg_iterator i = CS.arg_begin(), e = CS.arg_end();
       i != e; ++i) {
    unsigned Arg = getRegForValue(*i);
    if (Arg == 0)
      return false;
    ISD::ArgFlagsTy Flags;
    unsigned AttrInd = i - CS.arg_begin() + 1;
    if (CS.paramHasAttr(AttrInd, ParamAttr::SExt))
      Flags.setSExt();
    if (CS.paramHasAttr(AttrInd, ParamAttr::ZExt))
      Flags.setZExt();

    // FIXME: Only handle *easy* calls for now.
    if (CS.paramHasAttr(AttrInd, ParamAttr::InReg) ||
        CS.paramHasAttr(AttrInd, ParamAttr::StructRet) ||
        CS.paramHasAttr(AttrInd, ParamAttr::Nest) ||
        CS.paramHasAttr(AttrInd, ParamAttr::ByVal))
      return false;

    const Type *ArgTy = (*i)->getType();
    MVT ArgVT;
    if (!isTypeLegal(ArgTy, TLI, ArgVT))
      return false;
    unsigned OriginalAlignment = TD.getABITypeAlignment(ArgTy);
    Flags.setOrigAlign(OriginalAlignment);

    Args.push_back(Arg);
    ArgVTs.push_back(ArgVT);
    ArgFlags.push_back(Flags);
  }

  // Analyze operands of the call, assigning locations to each operand.
  SmallVector<CCValAssign, 16> ArgLocs;
  CCState CCInfo(CC, false, TM, ArgLocs);
  CCInfo.AnalyzeCallOperands(ArgVTs, ArgFlags, CCAssignFnForCall(CC));

  // Get a count of how many bytes are to be pushed on the stack.
  unsigned NumBytes = CCInfo.getNextStackOffset();

  // Issue CALLSEQ_START
  BuildMI(MBB, TII.get(X86::ADJCALLSTACKDOWN)).addImm(NumBytes);

  // Process argumenet: walk the register/memloc assignments, inserting
  // copies / loads.
  SmallVector<unsigned, 4> RegArgs;
  for (unsigned i = 0, e = ArgLocs.size(); i != e; ++i) {
    CCValAssign &VA = ArgLocs[i];
    unsigned Arg = Args[VA.getValNo()];
    MVT ArgVT = ArgVTs[VA.getValNo()];
  
    // Promote the value if needed.
    switch (VA.getLocInfo()) {
    default: assert(0 && "Unknown loc info!");
    case CCValAssign::Full: break;
    case CCValAssign::SExt: {
      bool Emitted = X86FastEmitExtend(ISD::SIGN_EXTEND, VA.getLocVT(),
                                       Arg, ArgVT, Arg);
      assert(Emitted && "Failed to emit a sext!");
      ArgVT = VA.getLocVT();
      break;
    }
    case CCValAssign::ZExt: {
      bool Emitted = X86FastEmitExtend(ISD::ZERO_EXTEND, VA.getLocVT(),
                                       Arg, ArgVT, Arg);
      assert(Emitted && "Failed to emit a zext!");
      ArgVT = VA.getLocVT();
      break;
    }
    case CCValAssign::AExt: {
      bool Emitted = X86FastEmitExtend(ISD::ANY_EXTEND, VA.getLocVT(),
                                       Arg, ArgVT, Arg);
      if (!Emitted)
        Emitted = X86FastEmitExtend(ISD::ZERO_EXTEND, VA.getLocVT(),
                                         Arg, ArgVT, Arg);
      if (!Emitted)
        Emitted = X86FastEmitExtend(ISD::SIGN_EXTEND, VA.getLocVT(),
                                    Arg, ArgVT, Arg);
      
      assert(Emitted && "Failed to emit a aext!");
      ArgVT = VA.getLocVT();
      break;
    }
    }
    
    if (VA.isRegLoc()) {
      TargetRegisterClass* RC = TLI.getRegClassFor(ArgVT);
      bool Emitted = TII.copyRegToReg(*MBB, MBB->end(), VA.getLocReg(),
                                      Arg, RC, RC);
      assert(Emitted && "Failed to emit a copy instruction!");
      RegArgs.push_back(VA.getLocReg());
    } else {
      unsigned LocMemOffset = VA.getLocMemOffset();
      X86AddressMode AM;
      AM.Base.Reg = StackPtr;
      AM.Disp = LocMemOffset;
      X86FastEmitStore(ArgVT, Arg, AM);
    }
  }

  // Issue the call.
  unsigned CallOpc = CalleeOp
    ? (Subtarget->is64Bit() ? X86::CALL64r       : X86::CALL32r)
    : (Subtarget->is64Bit() ? X86::CALL64pcrel32 : X86::CALLpcrel32);
  MachineInstrBuilder MIB = CalleeOp
    ? BuildMI(MBB, TII.get(CallOpc)).addReg(CalleeOp)
    : BuildMI(MBB, TII.get(CallOpc)).addGlobalAddress(GV);
  // Add implicit physical register uses to the call.
  while (!RegArgs.empty()) {
    MIB.addReg(RegArgs.back());
    RegArgs.pop_back();
  }

  // Issue CALLSEQ_END
  BuildMI(MBB, TII.get(X86::ADJCALLSTACKUP)).addImm(NumBytes).addImm(0);

  // Now handle call return value (if any).
  if (RetVT.getSimpleVT() != MVT::isVoid) {
    SmallVector<CCValAssign, 16> RVLocs;
    CCState CCInfo(CC, false, TM, RVLocs);
    CCInfo.AnalyzeCallResult(RetVT, RetCC_X86);

    // Copy all of the result registers out of their specified physreg.
    assert(RVLocs.size() == 1 && "Can't handle multi-value calls!");
    MVT CopyVT = RVLocs[0].getValVT();
    TargetRegisterClass* DstRC = TLI.getRegClassFor(CopyVT);
    TargetRegisterClass *SrcRC = DstRC;
    
    // If this is a call to a function that returns an fp value on the x87 fp
    // stack, but where we prefer to use the value in xmm registers, copy it
    // out as F80 and use a truncate to move it from fp stack reg to xmm reg.
    if ((RVLocs[0].getLocReg() == X86::ST0 ||
         RVLocs[0].getLocReg() == X86::ST1) &&
        isScalarFPTypeInSSEReg(RVLocs[0].getValVT())) {
      CopyVT = MVT::f80;
      SrcRC = X86::RSTRegisterClass;
      DstRC = X86::RFP80RegisterClass;
    }

    unsigned ResultReg = createResultReg(DstRC);
    bool Emitted = TII.copyRegToReg(*MBB, MBB->end(), ResultReg,
                                    RVLocs[0].getLocReg(), DstRC, SrcRC);
    assert(Emitted && "Failed to emit a copy instruction!");
    if (CopyVT != RVLocs[0].getValVT()) {
      // Round the F80 the right size, which also moves to the appropriate xmm
      // register. This is accomplished by storing the F80 value in memory and
      // then loading it back. Ewww...
      MVT ResVT = RVLocs[0].getValVT();
      unsigned Opc = ResVT == MVT::f32 ? X86::ST_Fp80m32 : X86::ST_Fp80m64;
      unsigned MemSize = ResVT.getSizeInBits()/8;
      int FI = MFI.CreateStackObject(MemSize, MemSize);
      addFrameReference(BuildMI(MBB, TII.get(Opc)), FI).addReg(ResultReg);
      DstRC = ResVT == MVT::f32
        ? X86::FR32RegisterClass : X86::FR64RegisterClass;
      Opc = ResVT == MVT::f32 ? X86::MOVSSrm : X86::MOVSDrm;
      ResultReg = createResultReg(DstRC);
      addFrameReference(BuildMI(MBB, TII.get(Opc), ResultReg), FI);
    }

    if (AndToI1) {
      // Mask out all but lowest bit for some call which produces an i1.
      unsigned AndResult = createResultReg(X86::GR8RegisterClass);
      BuildMI(MBB, TII.get(X86::AND8ri), AndResult).addReg(ResultReg).addImm(1);
      ResultReg = AndResult;
    }

    UpdateValueMap(I, ResultReg);
  }

  return true;
}


bool
X86FastISel::TargetSelectInstruction(Instruction *I)  {
  switch (I->getOpcode()) {
  default: break;
  case Instruction::Load:
    return X86SelectLoad(I);
  case Instruction::Store:
    return X86SelectStore(I);
  case Instruction::ICmp:
  case Instruction::FCmp:
    return X86SelectCmp(I);
  case Instruction::ZExt:
    return X86SelectZExt(I);
  case Instruction::Br:
    return X86SelectBranch(I);
  case Instruction::Call:
    return X86SelectCall(I);
  case Instruction::LShr:
  case Instruction::AShr:
  case Instruction::Shl:
    return X86SelectShift(I);
  case Instruction::Select:
    return X86SelectSelect(I);
  case Instruction::Trunc:
    return X86SelectTrunc(I);
  case Instruction::FPExt:
    return X86SelectFPExt(I);
  case Instruction::FPTrunc:
    return X86SelectFPTrunc(I);
  }

  return false;
}

unsigned X86FastISel::TargetMaterializeConstant(Constant *C) {
  // Can't handle PIC-mode yet.
  if (TM.getRelocationModel() == Reloc::PIC_)
    return 0;
  
  MVT VT;
  if (!isTypeLegal(C->getType(), TLI, VT))
    return false;
  
  // Get opcode and regclass of the output for the given load instruction.
  unsigned Opc = 0;
  const TargetRegisterClass *RC = NULL;
  switch (VT.getSimpleVT()) {
  default: return false;
  case MVT::i8:
    Opc = X86::MOV8rm;
    RC  = X86::GR8RegisterClass;
    break;
  case MVT::i16:
    Opc = X86::MOV16rm;
    RC  = X86::GR16RegisterClass;
    break;
  case MVT::i32:
    Opc = X86::MOV32rm;
    RC  = X86::GR32RegisterClass;
    break;
  case MVT::i64:
    // Must be in x86-64 mode.
    Opc = X86::MOV64rm;
    RC  = X86::GR64RegisterClass;
    break;
  case MVT::f32:
    if (Subtarget->hasSSE1()) {
      Opc = X86::MOVSSrm;
      RC  = X86::FR32RegisterClass;
    } else {
      Opc = X86::LD_Fp32m;
      RC  = X86::RFP32RegisterClass;
    }
    break;
  case MVT::f64:
    if (Subtarget->hasSSE2()) {
      Opc = X86::MOVSDrm;
      RC  = X86::FR64RegisterClass;
    } else {
      Opc = X86::LD_Fp64m;
      RC  = X86::RFP64RegisterClass;
    }
    break;
  case MVT::f80:
    Opc = X86::LD_Fp80m;
    RC  = X86::RFP80RegisterClass;
    break;
  }
  
  // Materialize addresses with LEA instructions.
  if (isa<GlobalValue>(C)) {
    X86AddressMode AM;
    if (X86SelectAddress(C, AM, false)) {
      if (TLI.getPointerTy() == MVT::i32)
        Opc = X86::LEA32r;
      else
        Opc = X86::LEA64r;
      unsigned ResultReg = createResultReg(RC);
      addFullAddress(BuildMI(MBB, TII.get(Opc), ResultReg), AM);
      return ResultReg;
    }
    return 0;
  }
  
  // MachineConstantPool wants an explicit alignment.
  unsigned Align = TD.getPreferredTypeAlignmentShift(C->getType());
  if (Align == 0) {
    // Alignment of vector types.  FIXME!
    Align = TD.getABITypeSize(C->getType());
    Align = Log2_64(Align);
  }
  
  unsigned MCPOffset = MCP.getConstantPoolIndex(C, Align);
  unsigned ResultReg = createResultReg(RC);
  addConstantPoolReference(BuildMI(MBB, TII.get(Opc), ResultReg), MCPOffset);
  return ResultReg;
}

unsigned X86FastISel::TargetMaterializeAlloca(AllocaInst *C) {
  X86AddressMode AM;
  if (!X86SelectAddress(C, AM, false))
    return 0;
  unsigned Opc = Subtarget->is64Bit() ? X86::LEA64r : X86::LEA32r;
  TargetRegisterClass* RC = TLI.getRegClassFor(TLI.getPointerTy());
  unsigned ResultReg = createResultReg(RC);
  addFullAddress(BuildMI(MBB, TII.get(Opc), ResultReg), AM);
  return ResultReg;
}

namespace llvm {
  llvm::FastISel *X86::createFastISel(MachineFunction &mf,
                        DenseMap<const Value *, unsigned> &vm,
                        DenseMap<const BasicBlock *, MachineBasicBlock *> &bm,
                        DenseMap<const AllocaInst *, int> &am) {
    return new X86FastISel(mf, vm, bm, am);
  }
}
