//===-- X86ISelPattern.cpp - A pattern matching inst selector for X86 -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file defines a pattern matching instruction selector for X86.
//
//===----------------------------------------------------------------------===//

#include "X86.h"
#include "X86InstrBuilder.h"
#include "X86RegisterInfo.h"
#include "llvm/Constants.h"                   // FIXME: REMOVE
#include "llvm/Function.h"
#include "llvm/CodeGen/MachineConstantPool.h" // FIXME: REMOVE
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/SelectionDAGISel.h"
#include "llvm/CodeGen/SSARegMap.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetLowering.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/ADT/Statistic.h"
#include <set>
#include <algorithm>
using namespace llvm;

//===----------------------------------------------------------------------===//
//  X86TargetLowering - X86 Implementation of the TargetLowering interface
namespace {
  class X86TargetLowering : public TargetLowering {
    int VarArgsFrameIndex;            // FrameIndex for start of varargs area.
    int ReturnAddrIndex;              // FrameIndex for return slot.
  public:
    X86TargetLowering(TargetMachine &TM) : TargetLowering(TM) {
      // Set up the TargetLowering object.

      // X86 is wierd, it always uses i8 for shift amounts and setcc results.
      setShiftAmountType(MVT::i8);
      setSetCCResultType(MVT::i8);

      // Set up the register classes.
      addRegisterClass(MVT::i8, X86::R8RegisterClass);
      addRegisterClass(MVT::i16, X86::R16RegisterClass);
      addRegisterClass(MVT::i32, X86::R32RegisterClass);
      addRegisterClass(MVT::f64, X86::RFPRegisterClass);
      
      // FIXME: Eliminate these two classes when legalize can handle promotions
      // well.
/**/  addRegisterClass(MVT::i1, X86::R8RegisterClass);

      setOperationAction(ISD::MEMMOVE          , MVT::Other, Expand);
      setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i16  , Expand);
      setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i16  , Expand);
      setOperationAction(ISD::SIGN_EXTEND_INREG, MVT::i1   , Expand);
      setOperationAction(ISD::ZERO_EXTEND_INREG, MVT::i1   , Expand);
      setOperationAction(ISD::FP_ROUND_INREG   , MVT::f32  , Expand);
      setOperationAction(ISD::SEXTLOAD         , MVT::i1   , Expand);
      setOperationAction(ISD::SREM             , MVT::f64  , Expand);
      
      // These should be promoted to a larger select which is supported.
/**/  setOperationAction(ISD::SELECT           , MVT::i1   , Promote);
      setOperationAction(ISD::SELECT           , MVT::i8   , Promote);
      
      computeRegisterProperties();
      
      addLegalFPImmediate(+0.0); // FLD0
      addLegalFPImmediate(+1.0); // FLD1
      addLegalFPImmediate(-0.0); // FLD0/FCHS
      addLegalFPImmediate(-1.0); // FLD1/FCHS
    }

    /// LowerArguments - This hook must be implemented to indicate how we should
    /// lower the arguments for the specified function, into the specified DAG.
    virtual std::vector<SDOperand>
    LowerArguments(Function &F, SelectionDAG &DAG);

    /// LowerCallTo - This hook lowers an abstract call to a function into an
    /// actual call.
    virtual std::pair<SDOperand, SDOperand>
    LowerCallTo(SDOperand Chain, const Type *RetTy, SDOperand Callee,
                ArgListTy &Args, SelectionDAG &DAG);

    virtual std::pair<SDOperand, SDOperand>
    LowerVAStart(SDOperand Chain, SelectionDAG &DAG);

    virtual std::pair<SDOperand,SDOperand>
    LowerVAArgNext(bool isVANext, SDOperand Chain, SDOperand VAList,
                   const Type *ArgTy, SelectionDAG &DAG);

    virtual std::pair<SDOperand, SDOperand>
    LowerFrameReturnAddress(bool isFrameAddr, SDOperand Chain, unsigned Depth,
                            SelectionDAG &DAG);
  };
}


std::vector<SDOperand>
X86TargetLowering::LowerArguments(Function &F, SelectionDAG &DAG) {
  std::vector<SDOperand> ArgValues;

  // Add DAG nodes to load the arguments...  On entry to a function on the X86,
  // the stack frame looks like this:
  //
  // [ESP] -- return address
  // [ESP + 4] -- first argument (leftmost lexically)
  // [ESP + 8] -- second argument, if first argument is four bytes in size
  //    ... 
  //
  MachineFunction &MF = DAG.getMachineFunction();
  MachineFrameInfo *MFI = MF.getFrameInfo();
  
  unsigned ArgOffset = 0;   // Frame mechanisms handle retaddr slot
  for (Function::aiterator I = F.abegin(), E = F.aend(); I != E; ++I) {
    MVT::ValueType ObjectVT = getValueType(I->getType());
    unsigned ArgIncrement = 4;
    unsigned ObjSize;
    switch (ObjectVT) {
    default: assert(0 && "Unhandled argument type!");
    case MVT::i1:
    case MVT::i8:  ObjSize = 1;                break;
    case MVT::i16: ObjSize = 2;                break;
    case MVT::i32: ObjSize = 4;                break;
    case MVT::i64: ObjSize = ArgIncrement = 8; break;
    case MVT::f32: ObjSize = 4;                break;
    case MVT::f64: ObjSize = ArgIncrement = 8; break;
    }
    // Create the frame index object for this incoming parameter...
    int FI = MFI->CreateFixedObject(ObjSize, ArgOffset);
    
    // Create the SelectionDAG nodes corresponding to a load from this parameter
    SDOperand FIN = DAG.getFrameIndex(FI, MVT::i32);

    // Don't codegen dead arguments.  FIXME: remove this check when we can nuke
    // dead loads.
    SDOperand ArgValue;
    if (!I->use_empty())
      ArgValue = DAG.getLoad(ObjectVT, DAG.getEntryNode(), FIN);
    else {
      if (MVT::isInteger(ObjectVT))
        ArgValue = DAG.getConstant(0, ObjectVT);
      else
        ArgValue = DAG.getConstantFP(0, ObjectVT);
    }
    ArgValues.push_back(ArgValue);

    ArgOffset += ArgIncrement;   // Move on to the next argument...
  }

  // If the function takes variable number of arguments, make a frame index for
  // the start of the first vararg value... for expansion of llvm.va_start.
  if (F.isVarArg())
    VarArgsFrameIndex = MFI->CreateFixedObject(1, ArgOffset);
  ReturnAddrIndex = 0;  // No return address slot generated yet.
  return ArgValues;
}

std::pair<SDOperand, SDOperand>
X86TargetLowering::LowerCallTo(SDOperand Chain,
                               const Type *RetTy, SDOperand Callee,
                               ArgListTy &Args, SelectionDAG &DAG) {
  // Count how many bytes are to be pushed on the stack.
  unsigned NumBytes = 0;

  if (Args.empty()) {
    // Save zero bytes.
    Chain = DAG.getNode(ISD::ADJCALLSTACKDOWN, MVT::Other, Chain,
                        DAG.getConstant(0, getPointerTy()));
  } else {
    for (unsigned i = 0, e = Args.size(); i != e; ++i)
      switch (getValueType(Args[i].second)) {
      default: assert(0 && "Unknown value type!");
      case MVT::i1:
      case MVT::i8:
      case MVT::i16:
      case MVT::i32:
      case MVT::f32:
        NumBytes += 4;
        break;
      case MVT::i64:
      case MVT::f64:
        NumBytes += 8;
        break;
      }

    Chain = DAG.getNode(ISD::ADJCALLSTACKDOWN, MVT::Other, Chain,
                        DAG.getConstant(NumBytes, getPointerTy()));

    // Arguments go on the stack in reverse order, as specified by the ABI.
    unsigned ArgOffset = 0;
    SDOperand StackPtr = DAG.getCopyFromReg(X86::ESP, MVT::i32,
                                            DAG.getEntryNode());
    for (unsigned i = 0, e = Args.size(); i != e; ++i) {
      unsigned ArgReg;
      SDOperand PtrOff = DAG.getConstant(ArgOffset, getPointerTy());
      PtrOff = DAG.getNode(ISD::ADD, MVT::i32, StackPtr, PtrOff);

      switch (getValueType(Args[i].second)) {
      default: assert(0 && "Unexpected ValueType for argument!");
      case MVT::i1:
      case MVT::i8:
      case MVT::i16:
        // Promote the integer to 32 bits.  If the input type is signed use a
        // sign extend, otherwise use a zero extend.
        if (Args[i].second->isSigned())
          Args[i].first =DAG.getNode(ISD::SIGN_EXTEND, MVT::i32, Args[i].first);
        else
          Args[i].first =DAG.getNode(ISD::ZERO_EXTEND, MVT::i32, Args[i].first);

        // FALL THROUGH
      case MVT::i32:
      case MVT::f32:
        // FIXME: Note that all of these stores are independent of each other.
        Chain = DAG.getNode(ISD::STORE, MVT::Other, Chain,
                            Args[i].first, PtrOff);
        ArgOffset += 4;
        break;
      case MVT::i64:
      case MVT::f64:
        // FIXME: Note that all of these stores are independent of each other.
        Chain = DAG.getNode(ISD::STORE, MVT::Other, Chain,
                            Args[i].first, PtrOff);
        ArgOffset += 8;
        break;
      }
    }
  }

  std::vector<MVT::ValueType> RetVals;
  MVT::ValueType RetTyVT = getValueType(RetTy);
  if (RetTyVT != MVT::isVoid)
    RetVals.push_back(RetTyVT);
  RetVals.push_back(MVT::Other);

  SDOperand TheCall = SDOperand(DAG.getCall(RetVals, Chain, Callee), 0);
  Chain = TheCall.getValue(RetTyVT != MVT::isVoid);
  Chain = DAG.getNode(ISD::ADJCALLSTACKUP, MVT::Other, Chain,
                      DAG.getConstant(NumBytes, getPointerTy()));
  return std::make_pair(TheCall, Chain);
}

std::pair<SDOperand, SDOperand>
X86TargetLowering::LowerVAStart(SDOperand Chain, SelectionDAG &DAG) {
  // vastart just returns the address of the VarArgsFrameIndex slot.
  return std::make_pair(DAG.getFrameIndex(VarArgsFrameIndex, MVT::i32), Chain);
}

std::pair<SDOperand,SDOperand> X86TargetLowering::
LowerVAArgNext(bool isVANext, SDOperand Chain, SDOperand VAList,
               const Type *ArgTy, SelectionDAG &DAG) {
  MVT::ValueType ArgVT = getValueType(ArgTy);
  SDOperand Result;
  if (!isVANext) {
    Result = DAG.getLoad(ArgVT, DAG.getEntryNode(), VAList);
  } else {
    unsigned Amt;
    if (ArgVT == MVT::i32)
      Amt = 4;
    else {
      assert((ArgVT == MVT::i64 || ArgVT == MVT::f64) &&
             "Other types should have been promoted for varargs!");
      Amt = 8;
    }
    Result = DAG.getNode(ISD::ADD, VAList.getValueType(), VAList,
                         DAG.getConstant(Amt, VAList.getValueType()));
  }
  return std::make_pair(Result, Chain);
}
               

std::pair<SDOperand, SDOperand> X86TargetLowering::
LowerFrameReturnAddress(bool isFrameAddress, SDOperand Chain, unsigned Depth,
                        SelectionDAG &DAG) {
  SDOperand Result;
  if (Depth)        // Depths > 0 not supported yet!
    Result = DAG.getConstant(0, getPointerTy());
  else {
    if (ReturnAddrIndex == 0) {
      // Set up a frame object for the return address.
      MachineFunction &MF = DAG.getMachineFunction();
      ReturnAddrIndex = MF.getFrameInfo()->CreateFixedObject(4, -4);
    }
    
    SDOperand RetAddrFI = DAG.getFrameIndex(ReturnAddrIndex, MVT::i32);

    if (!isFrameAddress)
      // Just load the return address
      Result = DAG.getLoad(MVT::i32, DAG.getEntryNode(), RetAddrFI);
    else
      Result = DAG.getNode(ISD::SUB, MVT::i32, RetAddrFI,
                           DAG.getConstant(4, MVT::i32));
  }
  return std::make_pair(Result, Chain);
}


namespace {
  /// X86ISelAddressMode - This corresponds to X86AddressMode, but uses
  /// SDOperand's instead of register numbers for the leaves of the matched
  /// tree.
  struct X86ISelAddressMode {
    enum {
      RegBase,
      FrameIndexBase,
    } BaseType;
    
    struct {            // This is really a union, discriminated by BaseType!
      SDOperand Reg;
      int FrameIndex;
    } Base;
    
    unsigned Scale;
    SDOperand IndexReg;
    unsigned Disp;
    GlobalValue *GV;
    
    X86ISelAddressMode()
      : BaseType(RegBase), Scale(1), IndexReg(), Disp(), GV(0) {
    }
  };
}


namespace {
  Statistic<>
  NumFPKill("x86-codegen", "Number of FP_REG_KILL instructions added");

  //===--------------------------------------------------------------------===//
  /// ISel - X86 specific code to select X86 machine instructions for
  /// SelectionDAG operations.
  ///
  class ISel : public SelectionDAGISel {
    /// ContainsFPCode - Every instruction we select that uses or defines a FP
    /// register should set this to true.
    bool ContainsFPCode;

    /// X86Lowering - This object fully describes how to lower LLVM code to an
    /// X86-specific SelectionDAG.
    X86TargetLowering X86Lowering;

    /// RegPressureMap - This keeps an approximate count of the number of
    /// registers required to evaluate each node in the graph.
    std::map<SDNode*, unsigned> RegPressureMap;

    /// ExprMap - As shared expressions are codegen'd, we keep track of which
    /// vreg the value is produced in, so we only emit one copy of each compiled
    /// tree.
    std::map<SDOperand, unsigned> ExprMap;

  public:
    ISel(TargetMachine &TM) : SelectionDAGISel(X86Lowering), X86Lowering(TM) {
    }

    unsigned getRegPressure(SDOperand O) {
      return RegPressureMap[O.Val];
    }
    unsigned ComputeRegPressure(SDOperand O);

    /// InstructionSelectBasicBlock - This callback is invoked by
    /// SelectionDAGISel when it has created a SelectionDAG for us to codegen.
    virtual void InstructionSelectBasicBlock(SelectionDAG &DAG);

    bool isFoldableLoad(SDOperand Op, SDOperand OtherOp);
    void EmitFoldedLoad(SDOperand Op, X86AddressMode &AM);
    bool TryToFoldLoadOpStore(SDNode *Node);

    void EmitCMP(SDOperand LHS, SDOperand RHS, bool isOnlyUse);
    bool EmitBranchCC(MachineBasicBlock *Dest, SDOperand Chain, SDOperand Cond);
    void EmitSelectCC(SDOperand Cond, MVT::ValueType SVT,
                      unsigned RTrue, unsigned RFalse, unsigned RDest);
    unsigned SelectExpr(SDOperand N);

    X86AddressMode SelectAddrExprs(const X86ISelAddressMode &IAM);
    bool MatchAddress(SDOperand N, X86ISelAddressMode &AM);
    void SelectAddress(SDOperand N, X86AddressMode &AM);
    void Select(SDOperand N);
  };
}

/// InstructionSelectBasicBlock - This callback is invoked by SelectionDAGISel
/// when it has created a SelectionDAG for us to codegen.
void ISel::InstructionSelectBasicBlock(SelectionDAG &DAG) {
  // While we're doing this, keep track of whether we see any FP code for
  // FP_REG_KILL insertion.
  ContainsFPCode = false;

  // Scan the PHI nodes that already are inserted into this basic block.  If any
  // of them is a PHI of a floating point value, we need to insert an
  // FP_REG_KILL.
  SSARegMap *RegMap = BB->getParent()->getSSARegMap();
  for (MachineBasicBlock::iterator I = BB->begin(), E = BB->end();
       I != E; ++I) {
    assert(I->getOpcode() == X86::PHI &&
           "Isn't just PHI nodes?");
    if (RegMap->getRegClass(I->getOperand(0).getReg()) ==
        X86::RFPRegisterClass) {
      ContainsFPCode = true;
      break;
    }
  }

  // Compute the RegPressureMap, which is an approximation for the number of
  // registers required to compute each node.
  ComputeRegPressure(DAG.getRoot());

  // Codegen the basic block.
  Select(DAG.getRoot());

  // Finally, look at all of the successors of this block.  If any contain a PHI
  // node of FP type, we need to insert an FP_REG_KILL in this block.
  for (MachineBasicBlock::succ_iterator SI = BB->succ_begin(),
         E = BB->succ_end(); SI != E && !ContainsFPCode; ++SI)
    for (MachineBasicBlock::iterator I = (*SI)->begin(), E = (*SI)->end();
         I != E && I->getOpcode() == X86::PHI; ++I) {
      if (RegMap->getRegClass(I->getOperand(0).getReg()) ==
          X86::RFPRegisterClass) {
        ContainsFPCode = true;
        break;
      }
    }
  
  // Insert FP_REG_KILL instructions into basic blocks that need them.  This
  // only occurs due to the floating point stackifier not being aggressive
  // enough to handle arbitrary global stackification.
  //
  // Currently we insert an FP_REG_KILL instruction into each block that uses or
  // defines a floating point virtual register.
  //
  // When the global register allocators (like linear scan) finally update live
  // variable analysis, we can keep floating point values in registers across
  // basic blocks.  This will be a huge win, but we are waiting on the global
  // allocators before we can do this.
  //
  if (ContainsFPCode && BB->succ_size()) {
    BuildMI(*BB, BB->getFirstTerminator(), X86::FP_REG_KILL, 0);
    ++NumFPKill;
  }
  
  // Clear state used for selection.
  ExprMap.clear();
  RegPressureMap.clear();
}


// ComputeRegPressure - Compute the RegPressureMap, which is an approximation
// for the number of registers required to compute each node.  This is basically
// computing a generalized form of the Sethi-Ullman number for each node.
unsigned ISel::ComputeRegPressure(SDOperand O) {
  SDNode *N = O.Val;
  unsigned &Result = RegPressureMap[N];
  if (Result) return Result;

  // FIXME: Should operations like CALL (which clobber lots o regs) have a
  // higher fixed cost??

  if (N->getNumOperands() == 0) {
    Result = 1;
  } else {
    unsigned MaxRegUse = 0;
    unsigned NumExtraMaxRegUsers = 0;
    for (unsigned i = 0, e = N->getNumOperands(); i != e; ++i) {
      unsigned Regs;
      if (N->getOperand(i).getOpcode() == ISD::Constant)
        Regs = 0;
      else
        Regs = ComputeRegPressure(N->getOperand(i));
      if (Regs > MaxRegUse) {
        MaxRegUse = Regs;
        NumExtraMaxRegUsers = 0;
      } else if (Regs == MaxRegUse &&
                 N->getOperand(i).getValueType() != MVT::Other) {
        ++NumExtraMaxRegUsers;
      }
    }

    if (O.getOpcode() != ISD::TokenFactor)
      Result = MaxRegUse+NumExtraMaxRegUsers;
    else
      Result = MaxRegUse == 1 ? 0 : MaxRegUse-1;
  }

  //std::cerr << " WEIGHT: " << Result << " ";  N->dump(); std::cerr << "\n";
  return Result;
}

X86AddressMode ISel::SelectAddrExprs(const X86ISelAddressMode &IAM) {
  X86AddressMode Result;

  // If we need to emit two register operands, emit the one with the highest
  // register pressure first.
  if (IAM.BaseType == X86ISelAddressMode::RegBase &&
      IAM.Base.Reg.Val && IAM.IndexReg.Val) {
    if (getRegPressure(IAM.Base.Reg) > getRegPressure(IAM.IndexReg)) {
      Result.Base.Reg = SelectExpr(IAM.Base.Reg);
      Result.IndexReg = SelectExpr(IAM.IndexReg);
    } else {
      Result.IndexReg = SelectExpr(IAM.IndexReg);
      Result.Base.Reg = SelectExpr(IAM.Base.Reg);
    }
  } else if (IAM.BaseType == X86ISelAddressMode::RegBase && IAM.Base.Reg.Val) {
    Result.Base.Reg = SelectExpr(IAM.Base.Reg);
  } else if (IAM.IndexReg.Val) {
    Result.IndexReg = SelectExpr(IAM.IndexReg);
  }
             
  switch (IAM.BaseType) {
  case X86ISelAddressMode::RegBase:
    Result.BaseType = X86AddressMode::RegBase;
    break;
  case X86ISelAddressMode::FrameIndexBase:
    Result.BaseType = X86AddressMode::FrameIndexBase;
    Result.Base.FrameIndex = IAM.Base.FrameIndex;
    break;
  default:
    assert(0 && "Unknown base type!");
    break;
  }
  Result.Scale = IAM.Scale;
  Result.Disp = IAM.Disp;
  Result.GV = IAM.GV;
  return Result;
}

/// SelectAddress - Pattern match the maximal addressing mode for this node and
/// emit all of the leaf registers.
void ISel::SelectAddress(SDOperand N, X86AddressMode &AM) {
  X86ISelAddressMode IAM;
  MatchAddress(N, IAM);
  AM = SelectAddrExprs(IAM);
}

/// MatchAddress - Add the specified node to the specified addressing mode,
/// returning true if it cannot be done.  This just pattern matches for the
/// addressing mode, it does not cause any code to be emitted.  For that, use
/// SelectAddress.
bool ISel::MatchAddress(SDOperand N, X86ISelAddressMode &AM) {
  switch (N.getOpcode()) {
  default: break;
  case ISD::FrameIndex:
    if (AM.BaseType == X86ISelAddressMode::RegBase && AM.Base.Reg.Val == 0) {
      AM.BaseType = X86ISelAddressMode::FrameIndexBase;
      AM.Base.FrameIndex = cast<FrameIndexSDNode>(N)->getIndex();
      return false;
    }
    break;
  case ISD::GlobalAddress:
    if (AM.GV == 0) {
      AM.GV = cast<GlobalAddressSDNode>(N)->getGlobal();
      return false;
    }
    break;
  case ISD::Constant:
    AM.Disp += cast<ConstantSDNode>(N)->getValue();
    return false;
  case ISD::SHL:
    // We might have folded the load into this shift, so don't regen the value
    // if so.
    if (ExprMap.count(N)) break;

    if (AM.IndexReg.Val == 0 && AM.Scale == 1)
      if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.Val->getOperand(1))) {
        unsigned Val = CN->getValue();
        if (Val == 1 || Val == 2 || Val == 3) {
          AM.Scale = 1 << Val;
          SDOperand ShVal = N.Val->getOperand(0);

          // Okay, we know that we have a scale by now.  However, if the scaled
          // value is an add of something and a constant, we can fold the
          // constant into the disp field here.
          if (ShVal.Val->getOpcode() == ISD::ADD && ShVal.hasOneUse() &&
              isa<ConstantSDNode>(ShVal.Val->getOperand(1))) {
            AM.IndexReg = ShVal.Val->getOperand(0);
            ConstantSDNode *AddVal =
              cast<ConstantSDNode>(ShVal.Val->getOperand(1));
            AM.Disp += AddVal->getValue() << Val;
          } else {
            AM.IndexReg = ShVal;
          }
          return false;
        }
      }
    break;
  case ISD::MUL:
    // We might have folded the load into this mul, so don't regen the value if
    // so.
    if (ExprMap.count(N)) break;

    // X*[3,5,9] -> X+X*[2,4,8]
    if (AM.IndexReg.Val == 0 && AM.BaseType == X86ISelAddressMode::RegBase &&
        AM.Base.Reg.Val == 0)
      if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.Val->getOperand(1)))
        if (CN->getValue() == 3 || CN->getValue() == 5 || CN->getValue() == 9) {
          AM.Scale = unsigned(CN->getValue())-1;

          SDOperand MulVal = N.Val->getOperand(0);
          SDOperand Reg;

          // Okay, we know that we have a scale by now.  However, if the scaled
          // value is an add of something and a constant, we can fold the
          // constant into the disp field here.
          if (MulVal.Val->getOpcode() == ISD::ADD && MulVal.hasOneUse() &&
              isa<ConstantSDNode>(MulVal.Val->getOperand(1))) {
            Reg = MulVal.Val->getOperand(0);
            ConstantSDNode *AddVal =
              cast<ConstantSDNode>(MulVal.Val->getOperand(1));
            AM.Disp += AddVal->getValue() * CN->getValue();
          } else {          
            Reg = N.Val->getOperand(0);
          }

          AM.IndexReg = AM.Base.Reg = Reg;
          return false;
        }
    break;

  case ISD::ADD: {
    // We might have folded the load into this mul, so don't regen the value if
    // so.
    if (ExprMap.count(N)) break;

    X86ISelAddressMode Backup = AM;
    if (!MatchAddress(N.Val->getOperand(0), AM) &&
        !MatchAddress(N.Val->getOperand(1), AM))
      return false;
    AM = Backup;
    if (!MatchAddress(N.Val->getOperand(1), AM) &&
        !MatchAddress(N.Val->getOperand(0), AM))
      return false;
    AM = Backup;
    break;
  }
  }

  // Is the base register already occupied?
  if (AM.BaseType != X86ISelAddressMode::RegBase || AM.Base.Reg.Val) {
    // If so, check to see if the scale index register is set.
    if (AM.IndexReg.Val == 0) {
      AM.IndexReg = N;
      AM.Scale = 1;
      return false;
    }

    // Otherwise, we cannot select it.
    return true;
  }

  // Default, generate it as a register.
  AM.BaseType = X86ISelAddressMode::RegBase;
  AM.Base.Reg = N;
  return false;
}

/// Emit2SetCCsAndLogical - Emit the following sequence of instructions,
/// assuming that the temporary registers are in the 8-bit register class.
///
///  Tmp1 = setcc1
///  Tmp2 = setcc2
///  DestReg = logicalop Tmp1, Tmp2
///
static void Emit2SetCCsAndLogical(MachineBasicBlock *BB, unsigned SetCC1,
                                  unsigned SetCC2, unsigned LogicalOp,
                                  unsigned DestReg) {
  SSARegMap *RegMap = BB->getParent()->getSSARegMap();
  unsigned Tmp1 = RegMap->createVirtualRegister(X86::R8RegisterClass);
  unsigned Tmp2 = RegMap->createVirtualRegister(X86::R8RegisterClass);
  BuildMI(BB, SetCC1, 0, Tmp1);
  BuildMI(BB, SetCC2, 0, Tmp2);
  BuildMI(BB, LogicalOp, 2, DestReg).addReg(Tmp1).addReg(Tmp2);
}

/// EmitSetCC - Emit the code to set the specified 8-bit register to 1 if the
/// condition codes match the specified SetCCOpcode.  Note that some conditions
/// require multiple instructions to generate the correct value.
static void EmitSetCC(MachineBasicBlock *BB, unsigned DestReg,
                      ISD::CondCode SetCCOpcode, bool isFP) {
  unsigned Opc;
  if (!isFP) {
    switch (SetCCOpcode) {
    default: assert(0 && "Illegal integer SetCC!");
    case ISD::SETEQ: Opc = X86::SETEr; break;
    case ISD::SETGT: Opc = X86::SETGr; break;
    case ISD::SETGE: Opc = X86::SETGEr; break;
    case ISD::SETLT: Opc = X86::SETLr; break;
    case ISD::SETLE: Opc = X86::SETLEr; break;
    case ISD::SETNE: Opc = X86::SETNEr; break;
    case ISD::SETULT: Opc = X86::SETBr; break;
    case ISD::SETUGT: Opc = X86::SETAr; break;
    case ISD::SETULE: Opc = X86::SETBEr; break;
    case ISD::SETUGE: Opc = X86::SETAEr; break;
    }
  } else {
    // On a floating point condition, the flags are set as follows:
    // ZF  PF  CF   op
    //  0 | 0 | 0 | X > Y
    //  0 | 0 | 1 | X < Y
    //  1 | 0 | 0 | X == Y
    //  1 | 1 | 1 | unordered
    //
    switch (SetCCOpcode) {
    default: assert(0 && "Invalid FP setcc!");
    case ISD::SETUEQ:
    case ISD::SETEQ:
      Opc = X86::SETEr;    // True if ZF = 1
      break;
    case ISD::SETOGT:
    case ISD::SETGT:
      Opc = X86::SETAr;    // True if CF = 0 and ZF = 0
      break;
    case ISD::SETOGE:
    case ISD::SETGE:
      Opc = X86::SETAEr;   // True if CF = 0
      break;
    case ISD::SETULT:
    case ISD::SETLT:
      Opc = X86::SETBr;    // True if CF = 1
      break;
    case ISD::SETULE:
    case ISD::SETLE:
      Opc = X86::SETBEr;   // True if CF = 1 or ZF = 1
      break;
    case ISD::SETONE:
    case ISD::SETNE:
      Opc = X86::SETNEr;   // True if ZF = 0
      break;
    case ISD::SETUO:
      Opc = X86::SETPr;    // True if PF = 1
      break;
    case ISD::SETO:
      Opc = X86::SETNPr;   // True if PF = 0
      break;
    case ISD::SETOEQ:      // !PF & ZF
      Emit2SetCCsAndLogical(BB, X86::SETNPr, X86::SETEr, X86::AND8rr, DestReg);
      return;
    case ISD::SETOLT:      // !PF & CF
      Emit2SetCCsAndLogical(BB, X86::SETNPr, X86::SETBr, X86::AND8rr, DestReg);
      return;
    case ISD::SETOLE:      // !PF & (CF || ZF)
      Emit2SetCCsAndLogical(BB, X86::SETNPr, X86::SETBEr, X86::AND8rr, DestReg);
      return;
    case ISD::SETUGT:      // PF | (!ZF & !CF)
      Emit2SetCCsAndLogical(BB, X86::SETPr, X86::SETAr, X86::OR8rr, DestReg);
      return;
    case ISD::SETUGE:      // PF | !CF
      Emit2SetCCsAndLogical(BB, X86::SETPr, X86::SETAEr, X86::OR8rr, DestReg);
      return;
    case ISD::SETUNE:      // PF | !ZF
      Emit2SetCCsAndLogical(BB, X86::SETPr, X86::SETNEr, X86::OR8rr, DestReg);
      return;
    }
  }
  BuildMI(BB, Opc, 0, DestReg);
}


/// EmitBranchCC - Emit code into BB that arranges for control to transfer to
/// the Dest block if the Cond condition is true.  If we cannot fold this
/// condition into the branch, return true.
///
bool ISel::EmitBranchCC(MachineBasicBlock *Dest, SDOperand Chain,
                        SDOperand Cond) {
  // FIXME: Evaluate whether it would be good to emit code like (X < Y) | (A >
  // B) using two conditional branches instead of one condbr, two setcc's, and
  // an or.
  if ((Cond.getOpcode() == ISD::OR ||
       Cond.getOpcode() == ISD::AND) && Cond.Val->hasOneUse()) {
    // And and or set the flags for us, so there is no need to emit a TST of the
    // result.  It is only safe to do this if there is only a single use of the
    // AND/OR though, otherwise we don't know it will be emitted here.
    Select(Chain);
    SelectExpr(Cond);
    BuildMI(BB, X86::JNE, 1).addMBB(Dest);
    return false;
  }

  // Codegen br not C -> JE.
  if (Cond.getOpcode() == ISD::XOR)
    if (ConstantSDNode *NC = dyn_cast<ConstantSDNode>(Cond.Val->getOperand(1)))
      if (NC->isAllOnesValue()) {
        unsigned CondR;
        if (getRegPressure(Chain) > getRegPressure(Cond)) {
          Select(Chain);
          CondR = SelectExpr(Cond.Val->getOperand(0));
        } else {
          CondR = SelectExpr(Cond.Val->getOperand(0));
          Select(Chain);
        }
        BuildMI(BB, X86::TEST8rr, 2).addReg(CondR).addReg(CondR);
        BuildMI(BB, X86::JE, 1).addMBB(Dest);
        return false;
      }

  SetCCSDNode *SetCC = dyn_cast<SetCCSDNode>(Cond);
  if (SetCC == 0)
    return true;                       // Can only handle simple setcc's so far.

  unsigned Opc;

  // Handle integer conditions first.
  if (MVT::isInteger(SetCC->getOperand(0).getValueType())) {
    switch (SetCC->getCondition()) {
    default: assert(0 && "Illegal integer SetCC!");
    case ISD::SETEQ: Opc = X86::JE; break;
    case ISD::SETGT: Opc = X86::JG; break;
    case ISD::SETGE: Opc = X86::JGE; break;
    case ISD::SETLT: Opc = X86::JL; break;
    case ISD::SETLE: Opc = X86::JLE; break;
    case ISD::SETNE: Opc = X86::JNE; break;
    case ISD::SETULT: Opc = X86::JB; break;
    case ISD::SETUGT: Opc = X86::JA; break;
    case ISD::SETULE: Opc = X86::JBE; break;
    case ISD::SETUGE: Opc = X86::JAE; break;
    }
    Select(Chain);
    EmitCMP(SetCC->getOperand(0), SetCC->getOperand(1), SetCC->hasOneUse());
    BuildMI(BB, Opc, 1).addMBB(Dest);
    return false;
  }

  unsigned Opc2 = 0;  // Second branch if needed.

  // On a floating point condition, the flags are set as follows:
  // ZF  PF  CF   op
  //  0 | 0 | 0 | X > Y
  //  0 | 0 | 1 | X < Y
  //  1 | 0 | 0 | X == Y
  //  1 | 1 | 1 | unordered
  //
  switch (SetCC->getCondition()) {
  default: assert(0 && "Invalid FP setcc!");
  case ISD::SETUEQ:
  case ISD::SETEQ:   Opc = X86::JE;  break;     // True if ZF = 1
  case ISD::SETOGT:
  case ISD::SETGT:   Opc = X86::JA;  break;     // True if CF = 0 and ZF = 0
  case ISD::SETOGE:
  case ISD::SETGE:   Opc = X86::JAE; break;     // True if CF = 0
  case ISD::SETULT:
  case ISD::SETLT:   Opc = X86::JB;  break;     // True if CF = 1
  case ISD::SETULE:
  case ISD::SETLE:   Opc = X86::JBE; break;     // True if CF = 1 or ZF = 1
  case ISD::SETONE:
  case ISD::SETNE:   Opc = X86::JNE; break;     // True if ZF = 0
  case ISD::SETUO:   Opc = X86::JP;  break;     // True if PF = 1
  case ISD::SETO:    Opc = X86::JNP; break;     // True if PF = 0
  case ISD::SETUGT:      // PF = 1 | (ZF = 0 & CF = 0)
    Opc = X86::JA;       // ZF = 0 & CF = 0
    Opc2 = X86::JP;      // PF = 1
    break;
  case ISD::SETUGE:      // PF = 1 | CF = 0
    Opc = X86::JAE;      // CF = 0
    Opc2 = X86::JP;      // PF = 1
    break;
  case ISD::SETUNE:      // PF = 1 | ZF = 0
    Opc = X86::JNE;      // ZF = 0
    Opc2 = X86::JP;      // PF = 1
    break;
  case ISD::SETOEQ:      // PF = 0 & ZF = 1
    //X86::JNP, X86::JE
    //X86::AND8rr
    return true;    // FIXME: Emit more efficient code for this branch.
  case ISD::SETOLT:      // PF = 0 & CF = 1
    //X86::JNP, X86::JB
    //X86::AND8rr
    return true;    // FIXME: Emit more efficient code for this branch.
  case ISD::SETOLE:      // PF = 0 & (CF = 1 || ZF = 1)
    //X86::JNP, X86::JBE
    //X86::AND8rr
    return true;    // FIXME: Emit more efficient code for this branch.
  }

  Select(Chain);
  EmitCMP(SetCC->getOperand(0), SetCC->getOperand(1), SetCC->hasOneUse());
  BuildMI(BB, Opc, 1).addMBB(Dest);
  if (Opc2)
    BuildMI(BB, Opc2, 1).addMBB(Dest);
  return false;
}

/// EmitSelectCC - Emit code into BB that performs a select operation between
/// the two registers RTrue and RFalse, generating a result into RDest.  Return
/// true if the fold cannot be performed.
///
void ISel::EmitSelectCC(SDOperand Cond, MVT::ValueType SVT,
                        unsigned RTrue, unsigned RFalse, unsigned RDest) {
  enum Condition {
    EQ, NE, LT, LE, GT, GE, B, BE, A, AE, P, NP,
    NOT_SET
  } CondCode = NOT_SET;

  static const unsigned CMOVTAB16[] = {
    X86::CMOVE16rr,  X86::CMOVNE16rr, X86::CMOVL16rr,  X86::CMOVLE16rr,
    X86::CMOVG16rr,  X86::CMOVGE16rr, X86::CMOVB16rr,  X86::CMOVBE16rr,
    X86::CMOVA16rr,  X86::CMOVAE16rr, X86::CMOVP16rr,  X86::CMOVNP16rr, 
  };
  static const unsigned CMOVTAB32[] = {
    X86::CMOVE32rr,  X86::CMOVNE32rr, X86::CMOVL32rr,  X86::CMOVLE32rr,
    X86::CMOVG32rr,  X86::CMOVGE32rr, X86::CMOVB32rr,  X86::CMOVBE32rr,
    X86::CMOVA32rr,  X86::CMOVAE32rr, X86::CMOVP32rr,  X86::CMOVNP32rr, 
  };
  static const unsigned CMOVTABFP[] = {
    X86::FCMOVE ,  X86::FCMOVNE, /*missing*/0, /*missing*/0,
    /*missing*/0,  /*missing*/0, X86::FCMOVB , X86::FCMOVBE,
    X86::FCMOVA ,  X86::FCMOVAE, X86::FCMOVP , X86::FCMOVNP
  };

  if (SetCCSDNode *SetCC = dyn_cast<SetCCSDNode>(Cond)) {
    if (MVT::isInteger(SetCC->getOperand(0).getValueType())) {
      switch (SetCC->getCondition()) {
      default: assert(0 && "Unknown integer comparison!");
      case ISD::SETEQ:  CondCode = EQ; break;
      case ISD::SETGT:  CondCode = GT; break;
      case ISD::SETGE:  CondCode = GE; break;
      case ISD::SETLT:  CondCode = LT; break;
      case ISD::SETLE:  CondCode = LE; break;
      case ISD::SETNE:  CondCode = NE; break;
      case ISD::SETULT: CondCode = B; break;
      case ISD::SETUGT: CondCode = A; break;
      case ISD::SETULE: CondCode = BE; break;
      case ISD::SETUGE: CondCode = AE; break;
      }
    } else {
      // On a floating point condition, the flags are set as follows:
      // ZF  PF  CF   op
      //  0 | 0 | 0 | X > Y
      //  0 | 0 | 1 | X < Y
      //  1 | 0 | 0 | X == Y
      //  1 | 1 | 1 | unordered
      //
      switch (SetCC->getCondition()) {
      default: assert(0 && "Unknown FP comparison!");
      case ISD::SETUEQ:
      case ISD::SETEQ:  CondCode = EQ; break;     // True if ZF = 1
      case ISD::SETOGT:
      case ISD::SETGT:  CondCode = A;  break;     // True if CF = 0 and ZF = 0
      case ISD::SETOGE:
      case ISD::SETGE:  CondCode = AE; break;     // True if CF = 0
      case ISD::SETULT:
      case ISD::SETLT:  CondCode = B;  break;     // True if CF = 1
      case ISD::SETULE:
      case ISD::SETLE:  CondCode = BE; break;     // True if CF = 1 or ZF = 1
      case ISD::SETONE:
      case ISD::SETNE:  CondCode = NE; break;     // True if ZF = 0
      case ISD::SETUO:  CondCode = P;  break;     // True if PF = 1
      case ISD::SETO:   CondCode = NP; break;     // True if PF = 0
      case ISD::SETUGT:      // PF = 1 | (ZF = 0 & CF = 0)
      case ISD::SETUGE:      // PF = 1 | CF = 0
      case ISD::SETUNE:      // PF = 1 | ZF = 0
      case ISD::SETOEQ:      // PF = 0 & ZF = 1
      case ISD::SETOLT:      // PF = 0 & CF = 1
      case ISD::SETOLE:      // PF = 0 & (CF = 1 || ZF = 1)
        // We cannot emit this comparison as a single cmov.
        break;
      }
    }
  }

  unsigned Opc = 0;
  if (CondCode != NOT_SET) {
    switch (SVT) {
    default: assert(0 && "Cannot select this type!");
    case MVT::i16: Opc = CMOVTAB16[CondCode]; break;
    case MVT::i32: Opc = CMOVTAB32[CondCode]; break;
    case MVT::f64: Opc = CMOVTABFP[CondCode]; break;
    }
  }

  // Finally, if we weren't able to fold this, just emit the condition and test
  // it.
  if (CondCode == NOT_SET || Opc == 0) {
    // Get the condition into the zero flag.
    unsigned CondReg = SelectExpr(Cond);
    BuildMI(BB, X86::TEST8rr, 2).addReg(CondReg).addReg(CondReg);

    switch (SVT) {
    default: assert(0 && "Cannot select this type!");
    case MVT::i16: Opc = X86::CMOVE16rr; break;
    case MVT::i32: Opc = X86::CMOVE32rr; break;
    case MVT::f64: Opc = X86::FCMOVE; break;
    }
  } else {
    // FIXME: CMP R, 0 -> TEST R, R
    EmitCMP(Cond.getOperand(0), Cond.getOperand(1), Cond.Val->hasOneUse());
    std::swap(RTrue, RFalse);
  }
  BuildMI(BB, Opc, 2, RDest).addReg(RTrue).addReg(RFalse);
}

void ISel::EmitCMP(SDOperand LHS, SDOperand RHS, bool HasOneUse) {
  unsigned Opc;
  if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(RHS)) {
    Opc = 0;
    if (HasOneUse && isFoldableLoad(LHS, RHS)) {
      switch (RHS.getValueType()) {
      default: break;
      case MVT::i1:
      case MVT::i8:  Opc = X86::CMP8mi;  break;
      case MVT::i16: Opc = X86::CMP16mi; break;
      case MVT::i32: Opc = X86::CMP32mi; break;
      }
      if (Opc) {
        X86AddressMode AM;
        EmitFoldedLoad(LHS, AM);
        addFullAddress(BuildMI(BB, Opc, 5), AM).addImm(CN->getValue());
        return;
      }
    }

    switch (RHS.getValueType()) {
    default: break;
    case MVT::i1:
    case MVT::i8:  Opc = X86::CMP8ri;  break;
    case MVT::i16: Opc = X86::CMP16ri; break;
    case MVT::i32: Opc = X86::CMP32ri; break;
    }
    if (Opc) {
      unsigned Tmp1 = SelectExpr(LHS);
      BuildMI(BB, Opc, 2).addReg(Tmp1).addImm(CN->getValue());
      return;
    }
  } else if (ConstantFPSDNode *CN = dyn_cast<ConstantFPSDNode>(RHS)) {
    if (CN->isExactlyValue(+0.0) ||
        CN->isExactlyValue(-0.0)) {
      unsigned Reg = SelectExpr(LHS);
      BuildMI(BB, X86::FTST, 1).addReg(Reg);
      BuildMI(BB, X86::FNSTSW8r, 0);
      BuildMI(BB, X86::SAHF, 1);
    }
  }

  Opc = 0;
  if (HasOneUse && isFoldableLoad(LHS, RHS)) {
    switch (RHS.getValueType()) {
    default: break;
    case MVT::i1:
    case MVT::i8:  Opc = X86::CMP8mr;  break;
    case MVT::i16: Opc = X86::CMP16mr; break;
    case MVT::i32: Opc = X86::CMP32mr; break;
    }
    if (Opc) {
      X86AddressMode AM;
      EmitFoldedLoad(LHS, AM);
      unsigned Reg = SelectExpr(RHS);
      addFullAddress(BuildMI(BB, Opc, 5), AM).addReg(Reg);
      return;
    }
  }

  switch (LHS.getValueType()) {
  default: assert(0 && "Cannot compare this value!");
  case MVT::i1:
  case MVT::i8:  Opc = X86::CMP8rr;  break;
  case MVT::i16: Opc = X86::CMP16rr; break;
  case MVT::i32: Opc = X86::CMP32rr; break;
  case MVT::f64: Opc = X86::FUCOMIr; break;
  }
  unsigned Tmp1, Tmp2;
  if (getRegPressure(LHS) > getRegPressure(RHS)) {
    Tmp1 = SelectExpr(LHS);
    Tmp2 = SelectExpr(RHS);
  } else {
    Tmp2 = SelectExpr(RHS);
    Tmp1 = SelectExpr(LHS);
  }
  BuildMI(BB, Opc, 2).addReg(Tmp1).addReg(Tmp2);
}

/// NodeTransitivelyUsesValue - Return true if N or any of its uses uses Op.
/// The DAG cannot have cycles in it, by definition, so the visited set is not
/// needed to prevent infinite loops.  The DAG CAN, however, have unbounded
/// reuse, so it prevents exponential cases.
///
static bool NodeTransitivelyUsesValue(SDOperand N, SDOperand Op,
                                      std::set<SDNode*> &Visited) {
  if (N == Op) return true;                        // Found it.
  SDNode *Node = N.Val;
  if (Node->getNumOperands() == 0) return false;   // Leaf?
  if (!Visited.insert(Node).second) return false;  // Already visited?

  // Recurse for the first N-1 operands.
  for (unsigned i = 1, e = Node->getNumOperands(); i != e; ++i)
    if (NodeTransitivelyUsesValue(Node->getOperand(i), Op, Visited))
      return true;

  // Tail recurse for the last operand.
  return NodeTransitivelyUsesValue(Node->getOperand(0), Op, Visited);
}

/// isFoldableLoad - Return true if this is a load instruction that can safely
/// be folded into an operation that uses it.
bool ISel::isFoldableLoad(SDOperand Op, SDOperand OtherOp) {
  if (Op.getOpcode() != ISD::LOAD ||
      // FIXME: currently can't fold constant pool indexes.
      isa<ConstantPoolSDNode>(Op.getOperand(1)))
    return false;

  // If this load has already been emitted, we clearly can't fold it.
  assert(Op.ResNo == 0 && "Not a use of the value of the load?");
  if (ExprMap.count(Op.getValue(1))) return false;
  assert(!ExprMap.count(Op.getValue(0)) && "Value in map but not token chain?");
  assert(!ExprMap.count(Op.getValue(1))&&"Token lowered but value not in map?");

  // If there is not just one use of its value, we cannot fold.
  if (!Op.Val->hasNUsesOfValue(1, 0)) return false;

  // Finally, we cannot fold the load into the operation if this would induce a
  // cycle into the resultant dag.  To check for this, see if OtherOp (the other
  // operand of the operation we are folding the load into) can possible use the
  // chain node defined by the load.
  if (OtherOp.Val && !Op.Val->hasNUsesOfValue(0, 1)) { // Has uses of chain?
    std::set<SDNode*> Visited;
    if (NodeTransitivelyUsesValue(OtherOp, Op.getValue(1), Visited))
      return false;
  }
  return true;
}


/// EmitFoldedLoad - Ensure that the arguments of the load are code generated,
/// and compute the address being loaded into AM.
void ISel::EmitFoldedLoad(SDOperand Op, X86AddressMode &AM) {
  SDOperand Chain   = Op.getOperand(0);
  SDOperand Address = Op.getOperand(1);

  if (getRegPressure(Chain) > getRegPressure(Address)) {
    Select(Chain);
    SelectAddress(Address, AM);
  } else {
    SelectAddress(Address, AM);
    Select(Chain);
  }

  // The chain for this load is now lowered.
  assert(ExprMap.count(SDOperand(Op.Val, 1)) == 0 &&
         "Load emitted more than once?");
  if (!ExprMap.insert(std::make_pair(Op.getValue(1), 1)).second)
    assert(0 && "Load emitted more than once!");
}

unsigned ISel::SelectExpr(SDOperand N) {
  unsigned Result;
  unsigned Tmp1, Tmp2, Tmp3;
  unsigned Opc = 0;
  SDNode *Node = N.Val;
  SDOperand Op0, Op1;

  if (Node->getOpcode() == ISD::CopyFromReg) {
    // FIXME: Handle copy from physregs!

    // Just use the specified register as our input.
    return dyn_cast<RegSDNode>(Node)->getReg();
  }
  
  unsigned &Reg = ExprMap[N];
  if (Reg) return Reg;
  
  if (N.getOpcode() != ISD::CALL)
    Reg = Result = (N.getValueType() != MVT::Other) ?
      MakeReg(N.getValueType()) : 1;
  else {
    // If this is a call instruction, make sure to prepare ALL of the result
    // values as well as the chain.
    if (Node->getNumValues() == 1)
      Reg = Result = 1;  // Void call, just a chain.
    else {
      Result = MakeReg(Node->getValueType(0));
      ExprMap[N.getValue(0)] = Result;
      for (unsigned i = 1, e = N.Val->getNumValues()-1; i != e; ++i)
        ExprMap[N.getValue(i)] = MakeReg(Node->getValueType(i));
      ExprMap[SDOperand(Node, Node->getNumValues()-1)] = 1;
    }
  }
  
  switch (N.getOpcode()) {
  default:
    Node->dump();
    assert(0 && "Node not handled!\n");
  case ISD::FrameIndex:
    Tmp1 = cast<FrameIndexSDNode>(N)->getIndex();
    addFrameReference(BuildMI(BB, X86::LEA32r, 4, Result), (int)Tmp1);
    return Result;
  case ISD::ConstantPool:
    Tmp1 = cast<ConstantPoolSDNode>(N)->getIndex();
    addConstantPoolReference(BuildMI(BB, X86::LEA32r, 4, Result), Tmp1);
    return Result;
  case ISD::ConstantFP:
    ContainsFPCode = true;
    Tmp1 = Result;   // Intermediate Register
    if (cast<ConstantFPSDNode>(N)->getValue() < 0.0 ||
        cast<ConstantFPSDNode>(N)->isExactlyValue(-0.0))
      Tmp1 = MakeReg(MVT::f64);

    if (cast<ConstantFPSDNode>(N)->isExactlyValue(+0.0) ||
        cast<ConstantFPSDNode>(N)->isExactlyValue(-0.0))
      BuildMI(BB, X86::FLD0, 0, Tmp1);
    else if (cast<ConstantFPSDNode>(N)->isExactlyValue(+1.0) ||
             cast<ConstantFPSDNode>(N)->isExactlyValue(-1.0))
      BuildMI(BB, X86::FLD1, 0, Tmp1);
    else
      assert(0 && "Unexpected constant!");
    if (Tmp1 != Result)
      BuildMI(BB, X86::FCHS, 1, Result).addReg(Tmp1);
    return Result;
  case ISD::Constant:
    switch (N.getValueType()) {
    default: assert(0 && "Cannot use constants of this type!");
    case MVT::i1:
    case MVT::i8:  Opc = X86::MOV8ri;  break;
    case MVT::i16: Opc = X86::MOV16ri; break;
    case MVT::i32: Opc = X86::MOV32ri; break;
    }
    BuildMI(BB, Opc, 1,Result).addImm(cast<ConstantSDNode>(N)->getValue());
    return Result;
  case ISD::GlobalAddress: {
    GlobalValue *GV = cast<GlobalAddressSDNode>(N)->getGlobal();
    BuildMI(BB, X86::MOV32ri, 1, Result).addGlobalAddress(GV);
    return Result;
  }
  case ISD::ExternalSymbol: {
    const char *Sym = cast<ExternalSymbolSDNode>(N)->getSymbol();
    BuildMI(BB, X86::MOV32ri, 1, Result).addExternalSymbol(Sym);
    return Result;
  }
  case ISD::FP_EXTEND:
    Tmp1 = SelectExpr(N.getOperand(0));
    BuildMI(BB, X86::FpMOV, 1, Result).addReg(Tmp1);
    return Result;
  case ISD::ZERO_EXTEND: {
    int DestIs16 = N.getValueType() == MVT::i16;
    int SrcIs16  = N.getOperand(0).getValueType() == MVT::i16;

    // FIXME: This hack is here for zero extension casts from bool to i8.  This
    // would not be needed if bools were promoted by Legalize.
    if (N.getValueType() == MVT::i8) {
      Tmp1 = SelectExpr(N.getOperand(0));
      BuildMI(BB, X86::MOV8rr, 1, Result).addReg(Tmp1);
      return Result;
    }

    if (isFoldableLoad(N.getOperand(0), SDOperand())) {
      static const unsigned Opc[3] = {
        X86::MOVZX32rm8, X86::MOVZX32rm16, X86::MOVZX16rm8
      };

      X86AddressMode AM;
      EmitFoldedLoad(N.getOperand(0), AM);
      addFullAddress(BuildMI(BB, Opc[SrcIs16+DestIs16*2], 4, Result), AM);
                             
      return Result;
    }

    static const unsigned Opc[3] = {
      X86::MOVZX32rr8, X86::MOVZX32rr16, X86::MOVZX16rr8
    };
    Tmp1 = SelectExpr(N.getOperand(0));
    BuildMI(BB, Opc[SrcIs16+DestIs16*2], 1, Result).addReg(Tmp1);
    return Result;
  }    
  case ISD::SIGN_EXTEND: {
    int DestIs16 = N.getValueType() == MVT::i16;
    int SrcIs16  = N.getOperand(0).getValueType() == MVT::i16;

    // FIXME: Legalize should promote bools to i8!
    assert(N.getOperand(0).getValueType() != MVT::i1 &&
           "Sign extend from bool not implemented!");

    if (isFoldableLoad(N.getOperand(0), SDOperand())) {
      static const unsigned Opc[3] = {
        X86::MOVSX32rm8, X86::MOVSX32rm16, X86::MOVSX16rm8
      };

      X86AddressMode AM;
      EmitFoldedLoad(N.getOperand(0), AM);
      addFullAddress(BuildMI(BB, Opc[SrcIs16+DestIs16*2], 4, Result), AM);
      return Result;
    }

    static const unsigned Opc[3] = {
      X86::MOVSX32rr8, X86::MOVSX32rr16, X86::MOVSX16rr8
    };
    Tmp1 = SelectExpr(N.getOperand(0));
    BuildMI(BB, Opc[SrcIs16+DestIs16*2], 1, Result).addReg(Tmp1);
    return Result;
  }
  case ISD::TRUNCATE:
    // Fold TRUNCATE (LOAD P) into a smaller load from P.
    if (isFoldableLoad(N.getOperand(0), SDOperand())) {
      switch (N.getValueType()) {
      default: assert(0 && "Unknown truncate!");
      case MVT::i1:
      case MVT::i8:  Opc = X86::MOV8rm;  break;
      case MVT::i16: Opc = X86::MOV16rm; break;
      }
      X86AddressMode AM;
      EmitFoldedLoad(N.getOperand(0), AM);
      addFullAddress(BuildMI(BB, Opc, 4, Result), AM);
      return Result;
    }

    // Handle cast of LARGER int to SMALLER int using a move to EAX followed by
    // a move out of AX or AL.
    switch (N.getOperand(0).getValueType()) {
    default: assert(0 && "Unknown truncate!");
    case MVT::i8:  Tmp2 = X86::AL;  Opc = X86::MOV8rr;  break;
    case MVT::i16: Tmp2 = X86::AX;  Opc = X86::MOV16rr; break;
    case MVT::i32: Tmp2 = X86::EAX; Opc = X86::MOV32rr; break;
    }
    Tmp1 = SelectExpr(N.getOperand(0));
    BuildMI(BB, Opc, 1, Tmp2).addReg(Tmp1);

    switch (N.getValueType()) {
    default: assert(0 && "Unknown truncate!");
    case MVT::i1:
    case MVT::i8:  Tmp2 = X86::AL;  Opc = X86::MOV8rr;  break;
    case MVT::i16: Tmp2 = X86::AX;  Opc = X86::MOV16rr; break;
    }
    BuildMI(BB, Opc, 1, Result).addReg(Tmp2);
    return Result;

  case ISD::FP_ROUND:
    // Truncate from double to float by storing to memory as float,
    // then reading it back into a register.

    // Create as stack slot to use.
    // FIXME: This should automatically be made by the Legalizer!
    Tmp1 = TLI.getTargetData().getFloatAlignment();
    Tmp2 = BB->getParent()->getFrameInfo()->CreateStackObject(4, Tmp1);

    // Codegen the input.
    Tmp1 = SelectExpr(N.getOperand(0));

    // Emit the store, then the reload.
    addFrameReference(BuildMI(BB, X86::FST32m, 5), Tmp2).addReg(Tmp1);
    addFrameReference(BuildMI(BB, X86::FLD32m, 5, Result), Tmp2);
    return Result;

  case ISD::SINT_TO_FP:
  case ISD::UINT_TO_FP: {
    // FIXME: Most of this grunt work should be done by legalize!
    ContainsFPCode = true;

    // Promote the integer to a type supported by FLD.  We do this because there
    // are no unsigned FLD instructions, so we must promote an unsigned value to
    // a larger signed value, then use FLD on the larger value.
    //
    MVT::ValueType PromoteType = MVT::Other;
    MVT::ValueType SrcTy = N.getOperand(0).getValueType();
    unsigned PromoteOpcode = 0;
    unsigned RealDestReg = Result;
    switch (SrcTy) {
    case MVT::i1:
    case MVT::i8:
      // We don't have the facilities for directly loading byte sized data from
      // memory (even signed).  Promote it to 16 bits.
      PromoteType = MVT::i16;
      PromoteOpcode = Node->getOpcode() == ISD::SINT_TO_FP ?
        X86::MOVSX16rr8 : X86::MOVZX16rr8;
      break;
    case MVT::i16:
      if (Node->getOpcode() == ISD::UINT_TO_FP) {
        PromoteType = MVT::i32;
        PromoteOpcode = X86::MOVZX32rr16;
      }
      break;
    default:
      // Don't fild into the real destination.
      if (Node->getOpcode() == ISD::UINT_TO_FP)
        Result = MakeReg(Node->getValueType(0));
      break;
    }

    Tmp1 = SelectExpr(N.getOperand(0));  // Get the operand register
    
    if (PromoteType != MVT::Other) {
      Tmp2 = MakeReg(PromoteType);
      BuildMI(BB, PromoteOpcode, 1, Tmp2).addReg(Tmp1);
      SrcTy = PromoteType;
      Tmp1 = Tmp2;
    }

    // Spill the integer to memory and reload it from there.
    unsigned Size = MVT::getSizeInBits(SrcTy)/8;
    MachineFunction *F = BB->getParent();
    int FrameIdx = F->getFrameInfo()->CreateStackObject(Size, Size);

    switch (SrcTy) {
    case MVT::i64:
      assert(0 && "Cast ulong to FP not implemented yet!");
      // FIXME: this won't work for cast [u]long to FP
      addFrameReference(BuildMI(BB, X86::MOV32mr, 5),
                        FrameIdx).addReg(Tmp1);
      addFrameReference(BuildMI(BB, X86::MOV32mr, 5),
                        FrameIdx, 4).addReg(Tmp1+1);
      addFrameReference(BuildMI(BB, X86::FILD64m, 5, Result), FrameIdx);
      break;
    case MVT::i32:
      addFrameReference(BuildMI(BB, X86::MOV32mr, 5),
                        FrameIdx).addReg(Tmp1);
      addFrameReference(BuildMI(BB, X86::FILD32m, 5, Result), FrameIdx);
      break;
    case MVT::i16:
      addFrameReference(BuildMI(BB, X86::MOV16mr, 5),
                        FrameIdx).addReg(Tmp1);
      addFrameReference(BuildMI(BB, X86::FILD16m, 5, Result), FrameIdx);
      break;
    default: break; // No promotion required.
    }

    if (Node->getOpcode() == ISD::UINT_TO_FP && Result != RealDestReg) {
      // If this is a cast from uint -> double, we need to be careful when if
      // the "sign" bit is set.  If so, we don't want to make a negative number,
      // we want to make a positive number.  Emit code to add an offset if the
      // sign bit is set.

      // Compute whether the sign bit is set by shifting the reg right 31 bits.
      unsigned IsNeg = MakeReg(MVT::i32);
      BuildMI(BB, X86::SHR32ri, 2, IsNeg).addReg(Tmp1).addImm(31);

      // Create a CP value that has the offset in one word and 0 in the other.
      static ConstantInt *TheOffset = ConstantUInt::get(Type::ULongTy,
                                                        0x4f80000000000000ULL);
      unsigned CPI = F->getConstantPool()->getConstantPoolIndex(TheOffset);
      BuildMI(BB, X86::FADD32m, 5, RealDestReg).addReg(Result)
        .addConstantPoolIndex(CPI).addZImm(4).addReg(IsNeg).addSImm(0);

    } else if (Node->getOpcode() == ISD::UINT_TO_FP && SrcTy == MVT::i64) {
      // We need special handling for unsigned 64-bit integer sources.  If the
      // input number has the "sign bit" set, then we loaded it incorrectly as a
      // negative 64-bit number.  In this case, add an offset value.

      // Emit a test instruction to see if the dynamic input value was signed.
      BuildMI(BB, X86::TEST32rr, 2).addReg(Tmp1+1).addReg(Tmp1+1);

      // If the sign bit is set, get a pointer to an offset, otherwise get a
      // pointer to a zero.
      MachineConstantPool *CP = F->getConstantPool();
      unsigned Zero = MakeReg(MVT::i32);
      Constant *Null = Constant::getNullValue(Type::UIntTy);
      addConstantPoolReference(BuildMI(BB, X86::LEA32r, 5, Zero), 
                               CP->getConstantPoolIndex(Null));
      unsigned Offset = MakeReg(MVT::i32);
      Constant *OffsetCst = ConstantUInt::get(Type::UIntTy, 0x5f800000);
                                             
      addConstantPoolReference(BuildMI(BB, X86::LEA32r, 5, Offset),
                               CP->getConstantPoolIndex(OffsetCst));
      unsigned Addr = MakeReg(MVT::i32);
      BuildMI(BB, X86::CMOVS32rr, 2, Addr).addReg(Zero).addReg(Offset);

      // Load the constant for an add.  FIXME: this could make an 'fadd' that
      // reads directly from memory, but we don't support these yet.
      unsigned ConstReg = MakeReg(MVT::f64);
      addDirectMem(BuildMI(BB, X86::FLD32m, 4, ConstReg), Addr);

      BuildMI(BB, X86::FpADD, 2, RealDestReg).addReg(ConstReg).addReg(Result);
    }
    return RealDestReg;
  }
  case ISD::FP_TO_SINT:
  case ISD::FP_TO_UINT: {
    // FIXME: Most of this grunt work should be done by legalize!
    Tmp1 = SelectExpr(N.getOperand(0));  // Get the operand register

    // Change the floating point control register to use "round towards zero"
    // mode when truncating to an integer value.
    //
    MachineFunction *F = BB->getParent();
    int CWFrameIdx = F->getFrameInfo()->CreateStackObject(2, 2);
    addFrameReference(BuildMI(BB, X86::FNSTCW16m, 4), CWFrameIdx);

    // Load the old value of the high byte of the control word...
    unsigned HighPartOfCW = MakeReg(MVT::i8);
    addFrameReference(BuildMI(BB, X86::MOV8rm, 4, HighPartOfCW),
                      CWFrameIdx, 1);

    // Set the high part to be round to zero...
    addFrameReference(BuildMI(BB, X86::MOV8mi, 5),
                      CWFrameIdx, 1).addImm(12);

    // Reload the modified control word now...
    addFrameReference(BuildMI(BB, X86::FLDCW16m, 4), CWFrameIdx);
    
    // Restore the memory image of control word to original value
    addFrameReference(BuildMI(BB, X86::MOV8mr, 5),
                      CWFrameIdx, 1).addReg(HighPartOfCW);

    // We don't have the facilities for directly storing byte sized data to
    // memory.  Promote it to 16 bits.  We also must promote unsigned values to
    // larger classes because we only have signed FP stores.
    MVT::ValueType StoreClass = Node->getValueType(0);
    if (StoreClass == MVT::i8 || Node->getOpcode() == ISD::FP_TO_UINT)
      switch (StoreClass) {
      case MVT::i8:  StoreClass = MVT::i16; break;
      case MVT::i16: StoreClass = MVT::i32; break;
      case MVT::i32: StoreClass = MVT::i64; break;
        // The following treatment of cLong may not be perfectly right,
        // but it survives chains of casts of the form
        // double->ulong->double.
      case MVT::i64:  StoreClass = MVT::i64;  break;
      default: assert(0 && "Unknown store class!");
      }

    // Spill the integer to memory and reload it from there.
    unsigned Size = MVT::getSizeInBits(StoreClass)/8;
    int FrameIdx = F->getFrameInfo()->CreateStackObject(Size, Size);

    switch (StoreClass) {
    default: assert(0 && "Unknown store class!");
    case MVT::i16:
      addFrameReference(BuildMI(BB, X86::FIST16m, 5), FrameIdx).addReg(Tmp1);
      break;
    case MVT::i32:
      addFrameReference(BuildMI(BB, X86::FIST32m, 5), FrameIdx).addReg(Tmp1);
      break;
    case MVT::i64:
      addFrameReference(BuildMI(BB, X86::FISTP64m, 5), FrameIdx).addReg(Tmp1);
      break;
    }

    switch (Node->getValueType(0)) {
    default:
      assert(0 && "Unknown integer type!");
    case MVT::i64:
      // FIXME: this isn't gunna work.
      assert(0 && "Cast FP to long not implemented yet!");
      addFrameReference(BuildMI(BB, X86::MOV32rm, 4, Result), FrameIdx);
      addFrameReference(BuildMI(BB, X86::MOV32rm, 4, Result+1), FrameIdx, 4);
    case MVT::i32:
      addFrameReference(BuildMI(BB, X86::MOV32rm, 4, Result), FrameIdx);
      break;
    case MVT::i16:
      addFrameReference(BuildMI(BB, X86::MOV16rm, 4, Result), FrameIdx);
      break;
    case MVT::i8:
      addFrameReference(BuildMI(BB, X86::MOV8rm, 4, Result), FrameIdx);
      break;
    }

    // Reload the original control word now.
    addFrameReference(BuildMI(BB, X86::FLDCW16m, 4), CWFrameIdx);
    return Result;
  }
  case ISD::ADD:
    Op0 = N.getOperand(0);
    Op1 = N.getOperand(1);

    if (isFoldableLoad(Op0, Op1)) {
      std::swap(Op0, Op1);
      goto FoldAdd;
    }

    if (isFoldableLoad(Op1, Op0)) {
    FoldAdd:
      switch (N.getValueType()) {
      default: assert(0 && "Cannot add this type!");
      case MVT::i1:
      case MVT::i8:  Opc = X86::ADD8rm;  break;
      case MVT::i16: Opc = X86::ADD16rm; break;
      case MVT::i32: Opc = X86::ADD32rm; break;
      case MVT::f32: Opc = X86::FADD32m; break;
      case MVT::f64: Opc = X86::FADD64m; break;
      }
      X86AddressMode AM;
      EmitFoldedLoad(Op1, AM);
      Tmp1 = SelectExpr(Op0);
      addFullAddress(BuildMI(BB, Opc, 5, Result).addReg(Tmp1), AM);
      return Result;
    }

    // See if we can codegen this as an LEA to fold operations together.
    if (N.getValueType() == MVT::i32) {
      ExprMap.erase(N);
      X86ISelAddressMode AM;
      MatchAddress(N, AM);
      ExprMap[N] = Result;

      // If this is not just an add, emit the LEA.  For a simple add (like
      // reg+reg or reg+imm), we just emit an add.  It might be a good idea to
      // leave this as LEA, then peephole it to 'ADD' after two address elim
      // happens.
      if (AM.Scale != 1 || AM.BaseType == X86ISelAddressMode::FrameIndexBase||
          AM.GV || (AM.Base.Reg.Val && AM.IndexReg.Val && AM.Disp)) {
        X86AddressMode XAM = SelectAddrExprs(AM);
        addFullAddress(BuildMI(BB, X86::LEA32r, 4, Result), XAM);
        return Result;
      }
    }

    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Op1)) {
      Opc = 0;
      if (CN->getValue() == 1) {   // add X, 1 -> inc X
        switch (N.getValueType()) {
        default: assert(0 && "Cannot integer add this type!");
        case MVT::i8:  Opc = X86::INC8r; break;
        case MVT::i16: Opc = X86::INC16r; break;
        case MVT::i32: Opc = X86::INC32r; break;
        }
      } else if (CN->isAllOnesValue()) { // add X, -1 -> dec X
        switch (N.getValueType()) {
        default: assert(0 && "Cannot integer add this type!");
        case MVT::i8:  Opc = X86::DEC8r; break;
        case MVT::i16: Opc = X86::DEC16r; break;
        case MVT::i32: Opc = X86::DEC32r; break;
        }
      }

      if (Opc) {
        Tmp1 = SelectExpr(Op0);
        BuildMI(BB, Opc, 1, Result).addReg(Tmp1);
        return Result;
      }

      switch (N.getValueType()) {
      default: assert(0 && "Cannot add this type!");
      case MVT::i8:  Opc = X86::ADD8ri; break;
      case MVT::i16: Opc = X86::ADD16ri; break;
      case MVT::i32: Opc = X86::ADD32ri; break;
      }
      if (Opc) {
        Tmp1 = SelectExpr(Op0);
        BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addImm(CN->getValue());
        return Result;
      }
    }

    switch (N.getValueType()) {
    default: assert(0 && "Cannot add this type!");
    case MVT::i8:  Opc = X86::ADD8rr; break;
    case MVT::i16: Opc = X86::ADD16rr; break;
    case MVT::i32: Opc = X86::ADD32rr; break;
    case MVT::f64: Opc = X86::FpADD; break;
    }

    if (getRegPressure(Op0) > getRegPressure(Op1)) {
      Tmp1 = SelectExpr(Op0);
      Tmp2 = SelectExpr(Op1);
    } else {
      Tmp2 = SelectExpr(Op1);
      Tmp1 = SelectExpr(Op0);
    }

    BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp2);
    return Result;
  case ISD::SUB:
  case ISD::MUL:
  case ISD::AND:
  case ISD::OR:
  case ISD::XOR: {
    static const unsigned SUBTab[] = {
      X86::SUB8ri, X86::SUB16ri, X86::SUB32ri, 0, 0,
      X86::SUB8rm, X86::SUB16rm, X86::SUB32rm, X86::FSUB32m, X86::FSUB64m,
      X86::SUB8rr, X86::SUB16rr, X86::SUB32rr, X86::FpSUB  , X86::FpSUB,
    };
    static const unsigned MULTab[] = {
      0, X86::IMUL16rri, X86::IMUL32rri, 0, 0,
      0, X86::IMUL16rm , X86::IMUL32rm, X86::FMUL32m, X86::FMUL64m,
      0, X86::IMUL16rr , X86::IMUL32rr, X86::FpMUL  , X86::FpMUL,
    };
    static const unsigned ANDTab[] = {
      X86::AND8ri, X86::AND16ri, X86::AND32ri, 0, 0,
      X86::AND8rm, X86::AND16rm, X86::AND32rm, 0, 0,
      X86::AND8rr, X86::AND16rr, X86::AND32rr, 0, 0, 
    };
    static const unsigned ORTab[] = {
      X86::OR8ri, X86::OR16ri, X86::OR32ri, 0, 0,
      X86::OR8rm, X86::OR16rm, X86::OR32rm, 0, 0,
      X86::OR8rr, X86::OR16rr, X86::OR32rr, 0, 0,
    };
    static const unsigned XORTab[] = {
      X86::XOR8ri, X86::XOR16ri, X86::XOR32ri, 0, 0,
      X86::XOR8rm, X86::XOR16rm, X86::XOR32rm, 0, 0,
      X86::XOR8rr, X86::XOR16rr, X86::XOR32rr, 0, 0,
    };

    Op0 = Node->getOperand(0);
    Op1 = Node->getOperand(1);

    if (Node->getOpcode() == ISD::SUB && MVT::isInteger(N.getValueType()))
      if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(0)))
        if (CN->isNullValue()) {   // 0 - N -> neg N
          switch (N.getValueType()) {
          default: assert(0 && "Cannot sub this type!");
          case MVT::i1:
          case MVT::i8:  Opc = X86::NEG8r;  break;
          case MVT::i16: Opc = X86::NEG16r; break;
          case MVT::i32: Opc = X86::NEG32r; break;
          }
          Tmp1 = SelectExpr(N.getOperand(1));
          BuildMI(BB, Opc, 1, Result).addReg(Tmp1);
          return Result;
        }

    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Op1)) {
      if (CN->isAllOnesValue() && Node->getOpcode() == ISD::XOR) {
        Opc = 0;
        switch (N.getValueType()) {
        default: assert(0 && "Cannot add this type!");
        case MVT::i1:  break;  // Not supported, don't invert upper bits!
        case MVT::i8:  Opc = X86::NOT8r;  break;
        case MVT::i16: Opc = X86::NOT16r; break;
        case MVT::i32: Opc = X86::NOT32r; break;
        }
        if (Opc) {
          Tmp1 = SelectExpr(Op0);
          BuildMI(BB, Opc, 1, Result).addReg(Tmp1);
          return Result;
        }
      }

      // Fold common multiplies into LEA instructions.
      if (Node->getOpcode() == ISD::MUL && N.getValueType() == MVT::i32) {
        switch ((int)CN->getValue()) {
        default: break;
        case 3:
        case 5:
        case 9:
          // Remove N from exprmap so SelectAddress doesn't get confused.
          ExprMap.erase(N);
          X86AddressMode AM;
          SelectAddress(N, AM);
          // Restore it to the map.
          ExprMap[N] = Result;
          addFullAddress(BuildMI(BB, X86::LEA32r, 4, Result), AM);
          return Result;
        }
      }

      switch (N.getValueType()) {
      default: assert(0 && "Cannot xor this type!");
      case MVT::i1:
      case MVT::i8:  Opc = 0; break;
      case MVT::i16: Opc = 1; break;
      case MVT::i32: Opc = 2; break;
      }
      switch (Node->getOpcode()) {
      default: assert(0 && "Unreachable!");
      case ISD::SUB: Opc = SUBTab[Opc]; break;
      case ISD::MUL: Opc = MULTab[Opc]; break;
      case ISD::AND: Opc = ANDTab[Opc]; break;
      case ISD::OR:  Opc =  ORTab[Opc]; break;
      case ISD::XOR: Opc = XORTab[Opc]; break;
      }
      if (Opc) {  // Can't fold MUL:i8 R, imm
        Tmp1 = SelectExpr(Op0);
        BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addImm(CN->getValue());
        return Result;
      }
    }

    if (isFoldableLoad(Op0, Op1))
      if (Node->getOpcode() != ISD::SUB) {
        std::swap(Op0, Op1);
        goto FoldOps;
      } else {
        // Emit 'reverse' subract, with a memory operand.
        switch (N.getValueType()) {
        default: Opc = 0; break;
        case MVT::f32: Opc = X86::FSUBR32m; break;
        case MVT::f64: Opc = X86::FSUBR64m; break;
        }
        if (Opc) {
          X86AddressMode AM;
          EmitFoldedLoad(Op0, AM);
          Tmp1 = SelectExpr(Op1);
          addFullAddress(BuildMI(BB, Opc, 5, Result).addReg(Tmp1), AM);
          return Result;
        }
      }

    if (isFoldableLoad(Op1, Op0)) {
    FoldOps:
      switch (N.getValueType()) {
      default: assert(0 && "Cannot operate on this type!");
      case MVT::i1:
      case MVT::i8:  Opc = 5; break;
      case MVT::i16: Opc = 6; break;
      case MVT::i32: Opc = 7; break;
      case MVT::f32: Opc = 8; break;
      case MVT::f64: Opc = 9; break;
      }
      switch (Node->getOpcode()) {
      default: assert(0 && "Unreachable!");
      case ISD::SUB: Opc = SUBTab[Opc]; break;
      case ISD::MUL: Opc = MULTab[Opc]; break;
      case ISD::AND: Opc = ANDTab[Opc]; break;
      case ISD::OR:  Opc =  ORTab[Opc]; break;
      case ISD::XOR: Opc = XORTab[Opc]; break;
      }

      X86AddressMode AM;
      EmitFoldedLoad(Op1, AM);
      Tmp1 = SelectExpr(Op0);
      if (Opc) {
        addFullAddress(BuildMI(BB, Opc, 5, Result).addReg(Tmp1), AM);
      } else {
        assert(Node->getOpcode() == ISD::MUL &&
               N.getValueType() == MVT::i8 && "Unexpected situation!");
        // Must use the MUL instruction, which forces use of AL.
        BuildMI(BB, X86::MOV8rr, 1, X86::AL).addReg(Tmp1);
        addFullAddress(BuildMI(BB, X86::MUL8m, 1), AM);
        BuildMI(BB, X86::MOV8rr, 1, Result).addReg(X86::AL);
      }
      return Result;
    }

    if (getRegPressure(Op0) > getRegPressure(Op1)) {
      Tmp1 = SelectExpr(Op0);
      Tmp2 = SelectExpr(Op1);
    } else {
      Tmp2 = SelectExpr(Op1);
      Tmp1 = SelectExpr(Op0);
    }

    switch (N.getValueType()) {
    default: assert(0 && "Cannot add this type!");
    case MVT::i1:
    case MVT::i8:  Opc = 10; break;
    case MVT::i16: Opc = 11; break;
    case MVT::i32: Opc = 12; break;
    case MVT::f32: Opc = 13; break;
    case MVT::f64: Opc = 14; break;
    }
    switch (Node->getOpcode()) {
    default: assert(0 && "Unreachable!");
    case ISD::SUB: Opc = SUBTab[Opc]; break;
    case ISD::MUL: Opc = MULTab[Opc]; break;
    case ISD::AND: Opc = ANDTab[Opc]; break;
    case ISD::OR:  Opc =  ORTab[Opc]; break;
    case ISD::XOR: Opc = XORTab[Opc]; break;
    }
    if (Opc) {
      BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp2);
    } else {
      assert(Node->getOpcode() == ISD::MUL &&
             N.getValueType() == MVT::i8 && "Unexpected situation!");
      // Must use the MUL instruction, which forces use of AL.
      BuildMI(BB, X86::MOV8rr, 1, X86::AL).addReg(Tmp1);
      BuildMI(BB, X86::MUL8r, 1).addReg(Tmp2);
      BuildMI(BB, X86::MOV8rr, 1, Result).addReg(X86::AL);
    }
    return Result;
  }
  case ISD::SELECT:
    if (getRegPressure(N.getOperand(1)) > getRegPressure(N.getOperand(2))) {
      Tmp2 = SelectExpr(N.getOperand(1));
      Tmp3 = SelectExpr(N.getOperand(2));
    } else {
      Tmp3 = SelectExpr(N.getOperand(2));
      Tmp2 = SelectExpr(N.getOperand(1));
    }
    EmitSelectCC(N.getOperand(0), N.getValueType(), Tmp2, Tmp3, Result);
    return Result;

  case ISD::SDIV:
  case ISD::UDIV:
  case ISD::SREM:
  case ISD::UREM: {
    assert((N.getOpcode() != ISD::SREM || MVT::isInteger(N.getValueType())) &&
           "We don't support this operator!");

    if (N.getOpcode() == ISD::SDIV)
      if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
        // FIXME: These special cases should be handled by the lowering impl!
        unsigned RHS = CN->getValue();
        bool isNeg = false;
        if ((int)RHS < 0) {
          isNeg = true;
          RHS = -RHS;
        }
        if (RHS && (RHS & (RHS-1)) == 0) {   // Signed division by power of 2?
          unsigned Log = log2(RHS);
          unsigned TmpReg = MakeReg(N.getValueType());
          unsigned SAROpc, SHROpc, ADDOpc, NEGOpc;
          switch (N.getValueType()) {
          default: assert("Unknown type to signed divide!");
          case MVT::i8:
            SAROpc = X86::SAR8ri;
            SHROpc = X86::SHR8ri;
            ADDOpc = X86::ADD8rr;
            NEGOpc = X86::NEG8r;
            break;
          case MVT::i16:
            SAROpc = X86::SAR16ri;
            SHROpc = X86::SHR16ri;
            ADDOpc = X86::ADD16rr;
            NEGOpc = X86::NEG16r;
            break;
          case MVT::i32:
            SAROpc = X86::SAR32ri;
            SHROpc = X86::SHR32ri;
            ADDOpc = X86::ADD32rr;
            NEGOpc = X86::NEG32r;
            break;
          }
          Tmp1 = SelectExpr(N.getOperand(0));
          BuildMI(BB, SAROpc, 2, TmpReg).addReg(Tmp1).addImm(Log-1);
          unsigned TmpReg2 = MakeReg(N.getValueType());
          BuildMI(BB, SHROpc, 2, TmpReg2).addReg(TmpReg).addImm(32-Log);
          unsigned TmpReg3 = MakeReg(N.getValueType());
          BuildMI(BB, ADDOpc, 2, TmpReg3).addReg(Tmp1).addReg(TmpReg2);
          
          unsigned TmpReg4 = isNeg ? MakeReg(N.getValueType()) : Result;
          BuildMI(BB, SAROpc, 2, TmpReg4).addReg(TmpReg3).addImm(Log);
          if (isNeg)
            BuildMI(BB, NEGOpc, 1, Result).addReg(TmpReg4);
          return Result;
        }
      }

    if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
      Tmp1 = SelectExpr(N.getOperand(0));
      Tmp2 = SelectExpr(N.getOperand(1));
    } else {
      Tmp2 = SelectExpr(N.getOperand(1));
      Tmp1 = SelectExpr(N.getOperand(0));
    }

    bool isSigned = N.getOpcode() == ISD::SDIV || N.getOpcode() == ISD::SREM;
    bool isDiv    = N.getOpcode() == ISD::SDIV || N.getOpcode() == ISD::UDIV;
    unsigned LoReg, HiReg, DivOpcode, MovOpcode, ClrOpcode, SExtOpcode;
    switch (N.getValueType()) {
    default: assert(0 && "Cannot sdiv this type!");
    case MVT::i8:
      DivOpcode = isSigned ? X86::IDIV8r : X86::DIV8r;
      LoReg = X86::AL;
      HiReg = X86::AH;
      MovOpcode = X86::MOV8rr;
      ClrOpcode = X86::MOV8ri;
      SExtOpcode = X86::CBW;
      break;
    case MVT::i16:
      DivOpcode = isSigned ? X86::IDIV16r : X86::DIV16r;
      LoReg = X86::AX;
      HiReg = X86::DX;
      MovOpcode = X86::MOV16rr;
      ClrOpcode = X86::MOV16ri;
      SExtOpcode = X86::CWD;
      break;
    case MVT::i32:
      DivOpcode = isSigned ? X86::IDIV32r : X86::DIV32r;
      LoReg = X86::EAX;
      HiReg = X86::EDX;
      MovOpcode = X86::MOV32rr;
      ClrOpcode = X86::MOV32ri;
      SExtOpcode = X86::CDQ;
      break;
    case MVT::f64:
      BuildMI(BB, X86::FpDIV, 2, Result).addReg(Tmp1).addReg(Tmp2);
      return Result;
    }

    // Set up the low part.
    BuildMI(BB, MovOpcode, 1, LoReg).addReg(Tmp1);

    if (isSigned) {
      // Sign extend the low part into the high part.
      BuildMI(BB, SExtOpcode, 0);
    } else {
      // Zero out the high part, effectively zero extending the input.
      BuildMI(BB, ClrOpcode, 1, HiReg).addImm(0);
    }

    // Emit the DIV/IDIV instruction.
    BuildMI(BB, DivOpcode, 1).addReg(Tmp2);    

    // Get the result of the divide or rem.
    BuildMI(BB, MovOpcode, 1, Result).addReg(isDiv ? LoReg : HiReg);
    return Result;
  }

  case ISD::SHL:
    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      if (CN->getValue() == 1) {   // X = SHL Y, 1  -> X = ADD Y, Y
        switch (N.getValueType()) {
        default: assert(0 && "Cannot shift this type!");
        case MVT::i8:  Opc = X86::ADD8rr; break;
        case MVT::i16: Opc = X86::ADD16rr; break;
        case MVT::i32: Opc = X86::ADD32rr; break;
        }
        Tmp1 = SelectExpr(N.getOperand(0));
        BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp1);
        return Result;
      }
      
      switch (N.getValueType()) {
      default: assert(0 && "Cannot shift this type!");
      case MVT::i8:  Opc = X86::SHL8ri; break;
      case MVT::i16: Opc = X86::SHL16ri; break;
      case MVT::i32: Opc = X86::SHL32ri; break;
      }
      Tmp1 = SelectExpr(N.getOperand(0));
      BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addImm(CN->getValue());
      return Result;
    }

    if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
      Tmp1 = SelectExpr(N.getOperand(0));
      Tmp2 = SelectExpr(N.getOperand(1));
    } else {
      Tmp2 = SelectExpr(N.getOperand(1));
      Tmp1 = SelectExpr(N.getOperand(0));
    }

    switch (N.getValueType()) {
    default: assert(0 && "Cannot shift this type!");
    case MVT::i8 : Opc = X86::SHL8rCL; break;
    case MVT::i16: Opc = X86::SHL16rCL; break;
    case MVT::i32: Opc = X86::SHL32rCL; break;
    }
    BuildMI(BB, X86::MOV8rr, 1, X86::CL).addReg(Tmp2);
    BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp2);
    return Result;
  case ISD::SRL:
    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      switch (N.getValueType()) {
      default: assert(0 && "Cannot shift this type!");
      case MVT::i8:  Opc = X86::SHR8ri; break;
      case MVT::i16: Opc = X86::SHR16ri; break;
      case MVT::i32: Opc = X86::SHR32ri; break;
      }
      Tmp1 = SelectExpr(N.getOperand(0));
      BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addImm(CN->getValue());
      return Result;
    }

    if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
      Tmp1 = SelectExpr(N.getOperand(0));
      Tmp2 = SelectExpr(N.getOperand(1));
    } else {
      Tmp2 = SelectExpr(N.getOperand(1));
      Tmp1 = SelectExpr(N.getOperand(0));
    }

    switch (N.getValueType()) {
    default: assert(0 && "Cannot shift this type!");
    case MVT::i8 : Opc = X86::SHR8rCL; break;
    case MVT::i16: Opc = X86::SHR16rCL; break;
    case MVT::i32: Opc = X86::SHR32rCL; break;
    }
    BuildMI(BB, X86::MOV8rr, 1, X86::CL).addReg(Tmp2);
    BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp2);
    return Result;
  case ISD::SRA:
    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      switch (N.getValueType()) {
      default: assert(0 && "Cannot shift this type!");
      case MVT::i8:  Opc = X86::SAR8ri; break;
      case MVT::i16: Opc = X86::SAR16ri; break;
      case MVT::i32: Opc = X86::SAR32ri; break;
      }
      Tmp1 = SelectExpr(N.getOperand(0));
      BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addImm(CN->getValue());
      return Result;
    }

    if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
      Tmp1 = SelectExpr(N.getOperand(0));
      Tmp2 = SelectExpr(N.getOperand(1));
    } else {
      Tmp2 = SelectExpr(N.getOperand(1));
      Tmp1 = SelectExpr(N.getOperand(0));
    }

    switch (N.getValueType()) {
    default: assert(0 && "Cannot shift this type!");
    case MVT::i8 : Opc = X86::SAR8rCL; break;
    case MVT::i16: Opc = X86::SAR16rCL; break;
    case MVT::i32: Opc = X86::SAR32rCL; break;
    }
    BuildMI(BB, X86::MOV8rr, 1, X86::CL).addReg(Tmp2);
    BuildMI(BB, Opc, 2, Result).addReg(Tmp1).addReg(Tmp2);
    return Result;

  case ISD::SETCC:
    EmitCMP(N.getOperand(0), N.getOperand(1), Node->hasOneUse());
    EmitSetCC(BB, Result, cast<SetCCSDNode>(N)->getCondition(),
              MVT::isFloatingPoint(N.getOperand(1).getValueType()));
    return Result;
  case ISD::LOAD:
    // Make sure we generate both values.
    if (Result != 1) {  // Generate the token
      if (!ExprMap.insert(std::make_pair(N.getValue(1), 1)).second)
        assert(0 && "Load already emitted!?");
    } else
      Result = ExprMap[N.getValue(0)] = MakeReg(N.getValue(0).getValueType());

    switch (Node->getValueType(0)) {
    default: assert(0 && "Cannot load this type!");
    case MVT::i1:
    case MVT::i8:  Opc = X86::MOV8rm; break;
    case MVT::i16: Opc = X86::MOV16rm; break;
    case MVT::i32: Opc = X86::MOV32rm; break;
    case MVT::f64: Opc = X86::FLD64m; ContainsFPCode = true; break;
    }

    if (ConstantPoolSDNode *CP = dyn_cast<ConstantPoolSDNode>(N.getOperand(1))){
      Select(N.getOperand(0));
      addConstantPoolReference(BuildMI(BB, Opc, 4, Result), CP->getIndex());
    } else {
      X86AddressMode AM;

      SDOperand Chain   = N.getOperand(0);
      SDOperand Address = N.getOperand(1);
      if (getRegPressure(Chain) > getRegPressure(Address)) {
        Select(Chain);
        SelectAddress(Address, AM);
      } else {
        SelectAddress(Address, AM);
        Select(Chain);
      }

      addFullAddress(BuildMI(BB, Opc, 4, Result), AM);
    }
    return Result;

  case ISD::EXTLOAD:          // Arbitrarily codegen extloads as MOVZX*
  case ISD::ZEXTLOAD: {
    // Make sure we generate both values.
    if (Result != 1)
      ExprMap[N.getValue(1)] = 1;   // Generate the token
    else
      Result = ExprMap[N.getValue(0)] = MakeReg(N.getValue(0).getValueType());

    if (ConstantPoolSDNode *CP = dyn_cast<ConstantPoolSDNode>(N.getOperand(1)))
      if (Node->getValueType(0) == MVT::f64) {
        assert(cast<MVTSDNode>(Node)->getExtraValueType() == MVT::f32 &&
               "Bad EXTLOAD!");
        addConstantPoolReference(BuildMI(BB, X86::FLD32m, 4, Result),
                                 CP->getIndex());
        return Result;
      }

    X86AddressMode AM;
    if (getRegPressure(Node->getOperand(0)) >
           getRegPressure(Node->getOperand(1))) {
      Select(Node->getOperand(0)); // chain
      SelectAddress(Node->getOperand(1), AM);
    } else {
      SelectAddress(Node->getOperand(1), AM);
      Select(Node->getOperand(0)); // chain
    }

    switch (Node->getValueType(0)) {
    default: assert(0 && "Unknown type to sign extend to.");
    case MVT::f64:
      assert(cast<MVTSDNode>(Node)->getExtraValueType() == MVT::f32 &&
             "Bad EXTLOAD!");
      addFullAddress(BuildMI(BB, X86::FLD32m, 5, Result), AM);
      break;
    case MVT::i32:
      switch (cast<MVTSDNode>(Node)->getExtraValueType()) {
      default:
        assert(0 && "Bad zero extend!");
      case MVT::i1:
      case MVT::i8:
        addFullAddress(BuildMI(BB, X86::MOVZX32rm8, 5, Result), AM);
        break;
      case MVT::i16:
        addFullAddress(BuildMI(BB, X86::MOVZX32rm16, 5, Result), AM);
        break;
      }
      break;
    case MVT::i16:
      assert(cast<MVTSDNode>(Node)->getExtraValueType() <= MVT::i8 &&
             "Bad zero extend!");
      addFullAddress(BuildMI(BB, X86::MOVSX16rm8, 5, Result), AM);
      break;
    case MVT::i8:
      assert(cast<MVTSDNode>(Node)->getExtraValueType() == MVT::i1 &&
             "Bad zero extend!");
      addFullAddress(BuildMI(BB, X86::MOV8rm, 5, Result), AM);
      break;
    }
    return Result;
  }
  case ISD::SEXTLOAD: {
    // Make sure we generate both values.
    if (Result != 1)
      ExprMap[N.getValue(1)] = 1;   // Generate the token
    else
      Result = ExprMap[N.getValue(0)] = MakeReg(N.getValue(0).getValueType());

    X86AddressMode AM;
    if (getRegPressure(Node->getOperand(0)) >
           getRegPressure(Node->getOperand(1))) {
      Select(Node->getOperand(0)); // chain
      SelectAddress(Node->getOperand(1), AM);
    } else {
      SelectAddress(Node->getOperand(1), AM);
      Select(Node->getOperand(0)); // chain
    }

    switch (Node->getValueType(0)) {
    case MVT::i8: assert(0 && "Cannot sign extend from bool!");
    default: assert(0 && "Unknown type to sign extend to.");
    case MVT::i32:
      switch (cast<MVTSDNode>(Node)->getExtraValueType()) {
      default:
      case MVT::i1: assert(0 && "Cannot sign extend from bool!");
      case MVT::i8:
        addFullAddress(BuildMI(BB, X86::MOVSX32rm8, 5, Result), AM);
        break;
      case MVT::i16:
        addFullAddress(BuildMI(BB, X86::MOVSX32rm16, 5, Result), AM);
        break;
      }
      break;
    case MVT::i16:
      assert(cast<MVTSDNode>(Node)->getExtraValueType() == MVT::i8 &&
             "Cannot sign extend from bool!");
      addFullAddress(BuildMI(BB, X86::MOVSX16rm8, 5, Result), AM);
      break;
    }
    return Result;
  }

  case ISD::DYNAMIC_STACKALLOC:
    // Generate both result values.
    if (Result != 1)
      ExprMap[N.getValue(1)] = 1;   // Generate the token
    else
      Result = ExprMap[N.getValue(0)] = MakeReg(N.getValue(0).getValueType());

    // FIXME: We are currently ignoring the requested alignment for handling
    // greater than the stack alignment.  This will need to be revisited at some
    // point.  Align = N.getOperand(2);

    if (!isa<ConstantSDNode>(N.getOperand(2)) ||
        cast<ConstantSDNode>(N.getOperand(2))->getValue() != 0) {
      std::cerr << "Cannot allocate stack object with greater alignment than"
                << " the stack alignment yet!";
      abort();
    }
  
    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      Select(N.getOperand(0));
      BuildMI(BB, X86::SUB32ri, 2, X86::ESP).addReg(X86::ESP)
        .addImm(CN->getValue());
    } else {
      if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
        Select(N.getOperand(0));
        Tmp1 = SelectExpr(N.getOperand(1));
      } else {
        Tmp1 = SelectExpr(N.getOperand(1));
        Select(N.getOperand(0));
      }

      // Subtract size from stack pointer, thereby allocating some space.
      BuildMI(BB, X86::SUB32rr, 2, X86::ESP).addReg(X86::ESP).addReg(Tmp1);
    }

    // Put a pointer to the space into the result register, by copying the stack
    // pointer.
    BuildMI(BB, X86::MOV32rr, 1, Result).addReg(X86::ESP);
    return Result;

  case ISD::CALL:
    // The chain for this call is now lowered.
    ExprMap.insert(std::make_pair(N.getValue(Node->getNumValues()-1), 1));

    if (GlobalAddressSDNode *GASD =
               dyn_cast<GlobalAddressSDNode>(N.getOperand(1))) {
      Select(N.getOperand(0));
      BuildMI(BB, X86::CALLpcrel32, 1).addGlobalAddress(GASD->getGlobal(),true);
    } else if (ExternalSymbolSDNode *ESSDN =
               dyn_cast<ExternalSymbolSDNode>(N.getOperand(1))) {
      Select(N.getOperand(0));
      BuildMI(BB, X86::CALLpcrel32,
              1).addExternalSymbol(ESSDN->getSymbol(), true);
    } else {
      if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
        Select(N.getOperand(0));
        Tmp1 = SelectExpr(N.getOperand(1));
      } else {
        Tmp1 = SelectExpr(N.getOperand(1));
        Select(N.getOperand(0));
      }

      BuildMI(BB, X86::CALL32r, 1).addReg(Tmp1);
    }
    switch (Node->getValueType(0)) {
    default: assert(0 && "Unknown value type for call result!");
    case MVT::Other: return 1;
    case MVT::i1:
    case MVT::i8:
      BuildMI(BB, X86::MOV8rr, 1, Result).addReg(X86::AL);
      break;
    case MVT::i16:
      BuildMI(BB, X86::MOV16rr, 1, Result).addReg(X86::AX);
      break;
    case MVT::i32:
      BuildMI(BB, X86::MOV32rr, 1, Result).addReg(X86::EAX);
      if (Node->getValueType(1) == MVT::i32)
        BuildMI(BB, X86::MOV32rr, 1, Result+1).addReg(X86::EDX);
      break;
    case MVT::f64:     // Floating-point return values live in %ST(0)
      ContainsFPCode = true;
      BuildMI(BB, X86::FpGETRESULT, 1, Result);
      break;
    }
    return Result+N.ResNo;
  }

  return 0;
}

/// TryToFoldLoadOpStore - Given a store node, try to fold together a
/// load/op/store instruction.  If successful return true.
bool ISel::TryToFoldLoadOpStore(SDNode *Node) {
  assert(Node->getOpcode() == ISD::STORE && "Can only do this for stores!");
  SDOperand Chain  = Node->getOperand(0);
  SDOperand StVal  = Node->getOperand(1);
  SDOperand StPtr  = Node->getOperand(2);

  // The chain has to be a load, the stored value must be an integer binary
  // operation with one use.
  if (!StVal.Val->hasOneUse() || StVal.Val->getNumOperands() != 2 ||
      MVT::isFloatingPoint(StVal.getValueType()))
    return false;

  // Token chain must either be a factor node or the load to fold.
  if (Chain.getOpcode() != ISD::LOAD && Chain.getOpcode() != ISD::TokenFactor)
    return false;

  SDOperand TheLoad;

  // Check to see if there is a load from the same pointer that we're storing
  // to in either operand of the binop.
  if (StVal.getOperand(0).getOpcode() == ISD::LOAD &&
      StVal.getOperand(0).getOperand(1) == StPtr)
    TheLoad = StVal.getOperand(0);
  else if (StVal.getOperand(1).getOpcode() == ISD::LOAD &&
           StVal.getOperand(1).getOperand(1) == StPtr)
    TheLoad = StVal.getOperand(1);
  else
    return false;  // No matching load operand.

  // We can only fold the load if there are no intervening side-effecting
  // operations.  This means that the store uses the load as its token chain, or
  // there are only token factor nodes in between the store and load.
  if (Chain != TheLoad.getValue(1)) {
    // Okay, the other option is that we have a store referring to (possibly
    // nested) token factor nodes.  For now, just try peeking through one level
    // of token factors to see if this is the case.
    bool ChainOk = false;
    if (Chain.getOpcode() == ISD::TokenFactor) {
      for (unsigned i = 0, e = Chain.getNumOperands(); i != e; ++i)
        if (Chain.getOperand(i) == TheLoad.getValue(1)) {
          ChainOk = true;
          break;
        }
    }

    if (!ChainOk) return false;
  }

  if (TheLoad.getOperand(1) != StPtr)
    return false;

  // Make sure that one of the operands of the binop is the load, and that the
  // load folds into the binop.
  if (((StVal.getOperand(0) != TheLoad ||
        !isFoldableLoad(TheLoad, StVal.getOperand(1))) &&
       (StVal.getOperand(1) != TheLoad ||
        !isFoldableLoad(TheLoad, StVal.getOperand(0)))))
    return false;

  // Finally, check to see if this is one of the ops we can handle!
  static const unsigned ADDTAB[] = {
    X86::ADD8mi, X86::ADD16mi, X86::ADD32mi,
    X86::ADD8mr, X86::ADD16mr, X86::ADD32mr,
  };
  static const unsigned SUBTAB[] = {
    X86::SUB8mi, X86::SUB16mi, X86::SUB32mi,
    X86::SUB8mr, X86::SUB16mr, X86::SUB32mr,
  };
  static const unsigned ANDTAB[] = {
    X86::AND8mi, X86::AND16mi, X86::AND32mi,
    X86::AND8mr, X86::AND16mr, X86::AND32mr,
  };
  static const unsigned ORTAB[] = {
    X86::OR8mi, X86::OR16mi, X86::OR32mi,
    X86::OR8mr, X86::OR16mr, X86::OR32mr,
  };
  static const unsigned XORTAB[] = {
    X86::XOR8mi, X86::XOR16mi, X86::XOR32mi,
    X86::XOR8mr, X86::XOR16mr, X86::XOR32mr,
  };
  static const unsigned SHLTAB[] = {
    X86::SHL8mi, X86::SHL16mi, X86::SHL32mi,
    /*Have to put the reg in CL*/0, 0, 0,
  };
  static const unsigned SARTAB[] = {
    X86::SAR8mi, X86::SAR16mi, X86::SAR32mi,
    /*Have to put the reg in CL*/0, 0, 0,
  };
  static const unsigned SHRTAB[] = {
    X86::SHR8mi, X86::SHR16mi, X86::SHR32mi,
    /*Have to put the reg in CL*/0, 0, 0,
  };
  
  const unsigned *TabPtr = 0;
  switch (StVal.getOpcode()) {
  default:
    std::cerr << "CANNOT [mem] op= val: ";
    StVal.Val->dump(); std::cerr << "\n";
  case ISD::MUL:
  case ISD::SDIV:
  case ISD::UDIV:
  case ISD::SREM:
  case ISD::UREM: return false;
    
  case ISD::ADD: TabPtr = ADDTAB; break;
  case ISD::SUB: TabPtr = SUBTAB; break;
  case ISD::AND: TabPtr = ANDTAB; break;
  case ISD:: OR: TabPtr =  ORTAB; break;
  case ISD::XOR: TabPtr = XORTAB; break;
  case ISD::SHL: TabPtr = SHLTAB; break;
  case ISD::SRA: TabPtr = SARTAB; break;
  case ISD::SRL: TabPtr = SHRTAB; break;
  }
  
  // Handle: [mem] op= CST
  SDOperand Op0 = StVal.getOperand(0);
  SDOperand Op1 = StVal.getOperand(1);
  unsigned Opc;
  if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(Op1)) {
    switch (Op0.getValueType()) { // Use Op0's type because of shifts.
    default: break;
    case MVT::i1:
    case MVT::i8:  Opc = TabPtr[0]; break;
    case MVT::i16: Opc = TabPtr[1]; break;
    case MVT::i32: Opc = TabPtr[2]; break;
    }
    
    if (Opc) {
      if (!ExprMap.insert(std::make_pair(TheLoad.getValue(1), 1)).second)
        assert(0 && "Already emitted?");
      Select(Chain);

      X86AddressMode AM;
      if (getRegPressure(TheLoad.getOperand(0)) >
          getRegPressure(TheLoad.getOperand(1))) {
        Select(TheLoad.getOperand(0));
        SelectAddress(TheLoad.getOperand(1), AM);
      } else {
        SelectAddress(TheLoad.getOperand(1), AM);
        Select(TheLoad.getOperand(0));
      }            

      if (StVal.getOpcode() == ISD::ADD) {
        if (CN->getValue() == 1) {
          switch (Op0.getValueType()) {
          default: break;
          case MVT::i8:
            addFullAddress(BuildMI(BB, X86::INC8m, 4), AM);
            return true;
          case MVT::i16: Opc = TabPtr[1];
            addFullAddress(BuildMI(BB, X86::INC16m, 4), AM);
            return true;
          case MVT::i32: Opc = TabPtr[2];
            addFullAddress(BuildMI(BB, X86::INC32m, 4), AM);
            return true;
          }
        } else if (CN->getValue()+1 == 0) {   // [X] += -1 -> DEC [X]
          switch (Op0.getValueType()) {
          default: break;
          case MVT::i8:
            addFullAddress(BuildMI(BB, X86::DEC8m, 4), AM);
            return true;
          case MVT::i16: Opc = TabPtr[1];
            addFullAddress(BuildMI(BB, X86::DEC16m, 4), AM);
            return true;
          case MVT::i32: Opc = TabPtr[2];
            addFullAddress(BuildMI(BB, X86::DEC32m, 4), AM);
            return true;
          }
        }
      }
      
      addFullAddress(BuildMI(BB, Opc, 4+1),AM).addImm(CN->getValue());
      return true;
    }
  }
  
  // If we have [mem] = V op [mem], try to turn it into:
  // [mem] = [mem] op V.
  if (Op1 == TheLoad && StVal.getOpcode() != ISD::SUB &&
      StVal.getOpcode() != ISD::SHL && StVal.getOpcode() != ISD::SRA &&
      StVal.getOpcode() != ISD::SRL)
    std::swap(Op0, Op1);
  
  if (Op0 != TheLoad) return false;

  switch (Op0.getValueType()) {
  default: return false;
  case MVT::i1:
  case MVT::i8:  Opc = TabPtr[3]; break;
  case MVT::i16: Opc = TabPtr[4]; break;
  case MVT::i32: Opc = TabPtr[5]; break;
  }

  // Table entry doesn't exist?
  if (Opc == 0) return false;

  if (!ExprMap.insert(std::make_pair(TheLoad.getValue(1), 1)).second)
    assert(0 && "Already emitted?");
  Select(Chain);
  Select(TheLoad.getOperand(0));

  X86AddressMode AM;
  SelectAddress(TheLoad.getOperand(1), AM);
  unsigned Reg = SelectExpr(Op1);
  addFullAddress(BuildMI(BB, Opc, 4+1), AM).addReg(Reg);
  return true;
}


void ISel::Select(SDOperand N) {
  unsigned Tmp1, Tmp2, Opc;

  // FIXME: Disable for our current expansion model!
  if (/*!N->hasOneUse() &&*/ !ExprMap.insert(std::make_pair(N, 1)).second)
    return;  // Already selected.

  SDNode *Node = N.Val;

  switch (Node->getOpcode()) {
  default:
    Node->dump(); std::cerr << "\n";
    assert(0 && "Node not handled yet!");
  case ISD::EntryToken: return;  // Noop
  case ISD::TokenFactor:
    if (Node->getNumOperands() == 2) {
      bool OneFirst = 
        getRegPressure(Node->getOperand(1))>getRegPressure(Node->getOperand(0));
      Select(Node->getOperand(OneFirst));
      Select(Node->getOperand(!OneFirst));
    } else {
      std::vector<std::pair<unsigned, unsigned> > OpsP;
      for (unsigned i = 0, e = Node->getNumOperands(); i != e; ++i)
        OpsP.push_back(std::make_pair(getRegPressure(Node->getOperand(i)), i));
      std::sort(OpsP.begin(), OpsP.end());
      std::reverse(OpsP.begin(), OpsP.end());
      for (unsigned i = 0, e = Node->getNumOperands(); i != e; ++i)
        Select(Node->getOperand(OpsP[i].second));
    }
    return;
  case ISD::CopyToReg:
    if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
      Select(N.getOperand(0));
      Tmp1 = SelectExpr(N.getOperand(1));
    } else {
      Tmp1 = SelectExpr(N.getOperand(1));
      Select(N.getOperand(0));
    }
    Tmp2 = cast<RegSDNode>(N)->getReg();
    
    if (Tmp1 != Tmp2) {
      switch (N.getOperand(1).getValueType()) {
      default: assert(0 && "Invalid type for operation!");
      case MVT::i1:
      case MVT::i8:  Opc = X86::MOV8rr; break;
      case MVT::i16: Opc = X86::MOV16rr; break;
      case MVT::i32: Opc = X86::MOV32rr; break;
      case MVT::f64: Opc = X86::FpMOV; ContainsFPCode = true; break;
      }
      BuildMI(BB, Opc, 1, Tmp2).addReg(Tmp1);
    }
    return;
  case ISD::RET:
    switch (N.getNumOperands()) {
    default:
      assert(0 && "Unknown return instruction!");
    case 3:
      assert(N.getOperand(1).getValueType() == MVT::i32 &&
	     N.getOperand(2).getValueType() == MVT::i32 &&
	     "Unknown two-register value!");
      if (getRegPressure(N.getOperand(1)) > getRegPressure(N.getOperand(2))) {
        Tmp1 = SelectExpr(N.getOperand(1));
        Tmp2 = SelectExpr(N.getOperand(2));
      } else {
        Tmp2 = SelectExpr(N.getOperand(2));
        Tmp1 = SelectExpr(N.getOperand(1));
      }
      Select(N.getOperand(0));

      BuildMI(BB, X86::MOV32rr, 1, X86::EAX).addReg(Tmp1);
      BuildMI(BB, X86::MOV32rr, 1, X86::EDX).addReg(Tmp2);
      // Declare that EAX & EDX are live on exit.
      BuildMI(BB, X86::IMPLICIT_USE, 3).addReg(X86::EAX).addReg(X86::EDX)
	.addReg(X86::ESP);
      break;
    case 2:
      if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
        Select(N.getOperand(0));
        Tmp1 = SelectExpr(N.getOperand(1));
      } else {
        Tmp1 = SelectExpr(N.getOperand(1));
        Select(N.getOperand(0));
      }
      switch (N.getOperand(1).getValueType()) {
      default: assert(0 && "All other types should have been promoted!!");
      case MVT::f64:
	BuildMI(BB, X86::FpSETRESULT, 1).addReg(Tmp1);
	// Declare that top-of-stack is live on exit
	BuildMI(BB, X86::IMPLICIT_USE, 2).addReg(X86::ST0).addReg(X86::ESP);
	break;
      case MVT::i32:
	BuildMI(BB, X86::MOV32rr, 1, X86::EAX).addReg(Tmp1);
	BuildMI(BB, X86::IMPLICIT_USE, 2).addReg(X86::EAX).addReg(X86::ESP);
	break;
      }
      break;
    case 1:
      Select(N.getOperand(0));
      break;
    }
    BuildMI(BB, X86::RET, 0); // Just emit a 'ret' instruction
    return;
  case ISD::BR: {
    Select(N.getOperand(0));
    MachineBasicBlock *Dest =
      cast<BasicBlockSDNode>(N.getOperand(1))->getBasicBlock();
    BuildMI(BB, X86::JMP, 1).addMBB(Dest);
    return;
  }

  case ISD::BRCOND: {
    MachineBasicBlock *Dest =
      cast<BasicBlockSDNode>(N.getOperand(2))->getBasicBlock();

    // Try to fold a setcc into the branch.  If this fails, emit a test/jne
    // pair.
    if (EmitBranchCC(Dest, N.getOperand(0), N.getOperand(1))) {
      if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(1))) {
        Select(N.getOperand(0));
        Tmp1 = SelectExpr(N.getOperand(1));
      } else {
        Tmp1 = SelectExpr(N.getOperand(1));
        Select(N.getOperand(0));
      }
      BuildMI(BB, X86::TEST8rr, 2).addReg(Tmp1).addReg(Tmp1);
      BuildMI(BB, X86::JNE, 1).addMBB(Dest);
    }

    return;
  }

  case ISD::LOAD:
    // If this load could be folded into the only using instruction, and if it
    // is safe to emit the instruction here, try to do so now.
    if (Node->hasNUsesOfValue(1, 0)) {
      SDOperand TheVal = N.getValue(0);
      SDNode *User = 0;
      for (SDNode::use_iterator UI = Node->use_begin(); ; ++UI) {
        assert(UI != Node->use_end() && "Didn't find use!");
        SDNode *UN = *UI;
        for (unsigned i = 0, e = UN->getNumOperands(); i != e; ++i)
          if (UN->getOperand(i) == TheVal) {
            User = UN;
            goto FoundIt;
          }
      }
    FoundIt:
      // Only handle unary operators right now.
      if (User->getNumOperands() == 1) {
        ExprMap.erase(N);
        SelectExpr(SDOperand(User, 0));
        return;
      }
    }
    ExprMap.erase(N);
    SelectExpr(N);
    return;

  case ISD::EXTLOAD:
  case ISD::SEXTLOAD:
  case ISD::ZEXTLOAD:
  case ISD::CALL:
  case ISD::DYNAMIC_STACKALLOC:
    ExprMap.erase(N);
    SelectExpr(N);
    return;

  case ISD::TRUNCSTORE: {  // truncstore chain, val, ptr :storety
    // On X86, we can represent all types except for Bool and Float natively.
    X86AddressMode AM;
    MVT::ValueType StoredTy = cast<MVTSDNode>(Node)->getExtraValueType();
    assert((StoredTy == MVT::i1 || StoredTy == MVT::f32 ||
            StoredTy == MVT::i16 /*FIXME: THIS IS JUST FOR TESTING!*/)
           && "Unsupported TRUNCSTORE for this target!");

    if (StoredTy == MVT::i16) {
      // FIXME: This is here just to allow testing.  X86 doesn't really have a
      // TRUNCSTORE i16 operation, but this is required for targets that do not
      // have 16-bit integer registers.  We occasionally disable 16-bit integer
      // registers to test the promotion code.
      Select(N.getOperand(0));
      Tmp1 = SelectExpr(N.getOperand(1));
      SelectAddress(N.getOperand(2), AM);

      BuildMI(BB, X86::MOV32rr, 1, X86::EAX).addReg(Tmp1);
      addFullAddress(BuildMI(BB, X86::MOV16mr, 5), AM).addReg(X86::AX);
      return;
    }

    // Store of constant bool?
    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(2))) {
        Select(N.getOperand(0));
        SelectAddress(N.getOperand(2), AM);
      } else {
        SelectAddress(N.getOperand(2), AM);
        Select(N.getOperand(0));
      }
      addFullAddress(BuildMI(BB, X86::MOV8mi, 5), AM).addImm(CN->getValue());
      return;
    }

    switch (StoredTy) {
    default: assert(0 && "Cannot truncstore this type!");
    case MVT::i1: Opc = X86::MOV8mr; break;
    case MVT::f32: Opc = X86::FST32m; break;
    }
    
    std::vector<std::pair<unsigned, unsigned> > RP;
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(0)), 0));
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(1)), 1));
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(2)), 2));
    std::sort(RP.begin(), RP.end());

    for (unsigned i = 0; i != 3; ++i)
      switch (RP[2-i].second) {
      default: assert(0 && "Unknown operand number!");
      case 0: Select(N.getOperand(0)); break;
      case 1: Tmp1 = SelectExpr(N.getOperand(1)); break;
      case 2: SelectAddress(N.getOperand(2), AM); break;
      }

    addFullAddress(BuildMI(BB, Opc, 4+1), AM).addReg(Tmp1);
    return;
  }
  case ISD::STORE: {
    X86AddressMode AM;

    if (ConstantSDNode *CN = dyn_cast<ConstantSDNode>(N.getOperand(1))) {
      Opc = 0;
      switch (CN->getValueType(0)) {
      default: assert(0 && "Invalid type for operation!");
      case MVT::i1:
      case MVT::i8:  Opc = X86::MOV8mi; break;
      case MVT::i16: Opc = X86::MOV16mi; break;
      case MVT::i32: Opc = X86::MOV32mi; break;
      case MVT::f64: break;
      }
      if (Opc) {
        if (getRegPressure(N.getOperand(0)) > getRegPressure(N.getOperand(2))) {
          Select(N.getOperand(0));
          SelectAddress(N.getOperand(2), AM);
        } else {
          SelectAddress(N.getOperand(2), AM);
          Select(N.getOperand(0));
        }
        addFullAddress(BuildMI(BB, Opc, 4+1), AM).addImm(CN->getValue());
        return;
      }
    }

    // Check to see if this is a load/op/store combination.
    if (TryToFoldLoadOpStore(Node))
      return;

    switch (N.getOperand(1).getValueType()) {
    default: assert(0 && "Cannot store this type!");
    case MVT::i1:
    case MVT::i8:  Opc = X86::MOV8mr; break;
    case MVT::i16: Opc = X86::MOV16mr; break;
    case MVT::i32: Opc = X86::MOV32mr; break;
    case MVT::f64: Opc = X86::FST64m; break;
    }
    
    std::vector<std::pair<unsigned, unsigned> > RP;
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(0)), 0));
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(1)), 1));
    RP.push_back(std::make_pair(getRegPressure(N.getOperand(2)), 2));
    std::sort(RP.begin(), RP.end());

    for (unsigned i = 0; i != 3; ++i)
      switch (RP[2-i].second) {
      default: assert(0 && "Unknown operand number!");
      case 0: Select(N.getOperand(0)); break;
      case 1: Tmp1 = SelectExpr(N.getOperand(1)); break;
      case 2: SelectAddress(N.getOperand(2), AM); break;
      }

    addFullAddress(BuildMI(BB, Opc, 4+1), AM).addReg(Tmp1);
    return;
  }
  case ISD::ADJCALLSTACKDOWN:
  case ISD::ADJCALLSTACKUP:
    Select(N.getOperand(0));
    Tmp1 = cast<ConstantSDNode>(N.getOperand(1))->getValue();
    
    Opc = N.getOpcode() == ISD::ADJCALLSTACKDOWN ? X86::ADJCALLSTACKDOWN :
                                                   X86::ADJCALLSTACKUP;
    BuildMI(BB, Opc, 1).addImm(Tmp1);
    return;
  case ISD::MEMSET: {
    Select(N.getOperand(0));  // Select the chain.
    unsigned Align =
      (unsigned)cast<ConstantSDNode>(Node->getOperand(4))->getValue();
    if (Align == 0) Align = 1;

    // Turn the byte code into # iterations
    unsigned CountReg;
    unsigned Opcode;
    if (ConstantSDNode *ValC = dyn_cast<ConstantSDNode>(Node->getOperand(2))) {
      unsigned Val = ValC->getValue() & 255;

      // If the value is a constant, then we can potentially use larger sets.
      switch (Align & 3) {
      case 2:   // WORD aligned
        CountReg = MakeReg(MVT::i32);
        if (ConstantSDNode *I = dyn_cast<ConstantSDNode>(Node->getOperand(3))) {
          BuildMI(BB, X86::MOV32ri, 1, CountReg).addImm(I->getValue()/2);
        } else {
          unsigned ByteReg = SelectExpr(Node->getOperand(3));
          BuildMI(BB, X86::SHR32ri, 2, CountReg).addReg(ByteReg).addImm(1);
        }
        BuildMI(BB, X86::MOV16ri, 1, X86::AX).addImm((Val << 8) | Val);
        Opcode = X86::REP_STOSW;
        break;
      case 0:   // DWORD aligned
        CountReg = MakeReg(MVT::i32);
        if (ConstantSDNode *I = dyn_cast<ConstantSDNode>(Node->getOperand(3))) {
          BuildMI(BB, X86::MOV32ri, 1, CountReg).addImm(I->getValue()/4);
        } else {
          unsigned ByteReg = SelectExpr(Node->getOperand(3));
          BuildMI(BB, X86::SHR32ri, 2, CountReg).addReg(ByteReg).addImm(2);
        }
        Val = (Val << 8) | Val;
        BuildMI(BB, X86::MOV32ri, 1, X86::EAX).addImm((Val << 16) | Val);
        Opcode = X86::REP_STOSD;
        break;
      default:  // BYTE aligned
        CountReg = SelectExpr(Node->getOperand(3));
        BuildMI(BB, X86::MOV8ri, 1, X86::AL).addImm(Val);
        Opcode = X86::REP_STOSB;
        break;
      }
    } else {
      // If it's not a constant value we are storing, just fall back.  We could
      // try to be clever to form 16 bit and 32 bit values, but we don't yet.
      unsigned ValReg = SelectExpr(Node->getOperand(2));
      BuildMI(BB, X86::MOV8rr, 1, X86::AL).addReg(ValReg);
      CountReg = SelectExpr(Node->getOperand(3));
      Opcode = X86::REP_STOSB;
    }

    // No matter what the alignment is, we put the source in ESI, the
    // destination in EDI, and the count in ECX.
    unsigned TmpReg1 = SelectExpr(Node->getOperand(1));
    BuildMI(BB, X86::MOV32rr, 1, X86::ECX).addReg(CountReg);
    BuildMI(BB, X86::MOV32rr, 1, X86::EDI).addReg(TmpReg1);
    BuildMI(BB, Opcode, 0);
    return;
  }
  case ISD::MEMCPY:
    Select(N.getOperand(0));  // Select the chain.
    unsigned Align =
      (unsigned)cast<ConstantSDNode>(Node->getOperand(4))->getValue();
    if (Align == 0) Align = 1;

    // Turn the byte code into # iterations
    unsigned CountReg;
    unsigned Opcode;
    switch (Align & 3) {
    case 2:   // WORD aligned
      CountReg = MakeReg(MVT::i32);
      if (ConstantSDNode *I = dyn_cast<ConstantSDNode>(Node->getOperand(3))) {
        BuildMI(BB, X86::MOV32ri, 1, CountReg).addImm(I->getValue()/2);
      } else {
        unsigned ByteReg = SelectExpr(Node->getOperand(3));
        BuildMI(BB, X86::SHR32ri, 2, CountReg).addReg(ByteReg).addImm(1);
      }
      Opcode = X86::REP_MOVSW;
      break;
    case 0:   // DWORD aligned
      CountReg = MakeReg(MVT::i32);
      if (ConstantSDNode *I = dyn_cast<ConstantSDNode>(Node->getOperand(3))) {
        BuildMI(BB, X86::MOV32ri, 1, CountReg).addImm(I->getValue()/4);
      } else {
        unsigned ByteReg = SelectExpr(Node->getOperand(3));
        BuildMI(BB, X86::SHR32ri, 2, CountReg).addReg(ByteReg).addImm(2);
      }
      Opcode = X86::REP_MOVSD;
      break;
    default:  // BYTE aligned
      CountReg = SelectExpr(Node->getOperand(3));
      Opcode = X86::REP_MOVSB;
      break;
    }

    // No matter what the alignment is, we put the source in ESI, the
    // destination in EDI, and the count in ECX.
    unsigned TmpReg1 = SelectExpr(Node->getOperand(1));
    unsigned TmpReg2 = SelectExpr(Node->getOperand(2));
    BuildMI(BB, X86::MOV32rr, 1, X86::ECX).addReg(CountReg);
    BuildMI(BB, X86::MOV32rr, 1, X86::EDI).addReg(TmpReg1);
    BuildMI(BB, X86::MOV32rr, 1, X86::ESI).addReg(TmpReg2);
    BuildMI(BB, Opcode, 0);
    return;
  }
  assert(0 && "Should not be reached!");
}


/// createX86PatternInstructionSelector - This pass converts an LLVM function
/// into a machine code representation using pattern matching and a machine
/// description file.
///
FunctionPass *llvm::createX86PatternInstructionSelector(TargetMachine &TM) {
  return new ISel(TM);  
}
