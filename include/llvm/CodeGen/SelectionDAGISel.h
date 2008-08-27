//===-- llvm/CodeGen/SelectionDAGISel.h - Common Base Class------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the SelectionDAGISel class, which is used as the common
// base class for SelectionDAG-based instruction selectors.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_SELECTIONDAG_ISEL_H
#define LLVM_CODEGEN_SELECTIONDAG_ISEL_H

#include "llvm/BasicBlock.h"
#include "llvm/Pass.h"
#include "llvm/Constant.h"
#include "llvm/CodeGen/SelectionDAG.h"

namespace llvm {
  class SelectionDAGLowering;
  class SDValue;
  class MachineRegisterInfo;
  class MachineBasicBlock;
  class MachineFunction;
  class MachineInstr;
  class TargetLowering;
  class FunctionLoweringInfo;
  class HazardRecognizer;
  class GCFunctionInfo;
  class ScheduleDAG;
 
/// SelectionDAGISel - This is the common base class used for SelectionDAG-based
/// pattern-matching instruction selectors.
class SelectionDAGISel : public FunctionPass {
public:
  TargetLowering &TLI;
  MachineRegisterInfo *RegInfo;
  FunctionLoweringInfo *FuncInfo;
  SelectionDAG *CurDAG;
  SelectionDAGLowering *SDL;
  MachineBasicBlock *BB;
  AliasAnalysis *AA;
  GCFunctionInfo *GFI;
  bool Fast;
  std::vector<SDNode*> TopOrder;
  static char ID;

  explicit SelectionDAGISel(TargetLowering &tli, bool fast = false);
  virtual ~SelectionDAGISel();
  
  TargetLowering &getTargetLowering() { return TLI; }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const;

  virtual bool runOnFunction(Function &Fn);

  unsigned MakeReg(MVT VT);

  virtual void EmitFunctionEntryCode(Function &Fn, MachineFunction &MF) {}
  virtual void InstructionSelect() = 0;
  virtual void InstructionSelectPostProcessing() {}
  
  void SelectRootInit() {
    DAGSize = CurDAG->AssignTopologicalOrder(TopOrder);
  }

  /// SelectInlineAsmMemoryOperand - Select the specified address as a target
  /// addressing mode, according to the specified constraint code.  If this does
  /// not match or is not implemented, return true.  The resultant operands
  /// (which will appear in the machine instruction) should be added to the
  /// OutOps vector.
  virtual bool SelectInlineAsmMemoryOperand(const SDValue &Op,
                                            char ConstraintCode,
                                            std::vector<SDValue> &OutOps) {
    return true;
  }

  /// CanBeFoldedBy - Returns true if the specific operand node N of U can be
  /// folded during instruction selection that starts at Root?
  virtual bool CanBeFoldedBy(SDNode *N, SDNode *U, SDNode *Root) const {
    return true;
  }
  
  /// CreateTargetHazardRecognizer - Return a newly allocated hazard recognizer
  /// to use for this target when scheduling the DAG.
  virtual HazardRecognizer *CreateTargetHazardRecognizer();
  
protected:
  /// DAGSize - Size of DAG being instruction selected.
  ///
  unsigned DAGSize;

  /// SelectInlineAsmMemoryOperands - Calls to this are automatically generated
  /// by tblgen.  Others should not call it.
  void SelectInlineAsmMemoryOperands(std::vector<SDValue> &Ops);

  // Calls to these predicates are generated by tblgen.
  bool CheckAndMask(SDValue LHS, ConstantSDNode *RHS,
                    int64_t DesiredMaskS) const;
  bool CheckOrMask(SDValue LHS, ConstantSDNode *RHS,
                    int64_t DesiredMaskS) const;
  
private:
  void SelectAllBasicBlocks(Function &Fn, MachineFunction &MF);
  void FinishBasicBlock();

  void SelectBasicBlock(BasicBlock *LLVMBB,
                        BasicBlock::iterator Begin,
                        BasicBlock::iterator End,
                        bool DoArgs);
  void CodeGenAndEmitDAG();
  void LowerArguments(BasicBlock *BB);
  
  void ComputeLiveOutVRegInfo();

  void HandlePHINodesInSuccessorBlocks(BasicBlock *LLVMBB);

  /// Pick a safe ordering for instructions for each target node in the
  /// graph.
  ScheduleDAG *Schedule();
};

}

#endif /* LLVM_CODEGEN_SELECTIONDAG_ISEL_H */
