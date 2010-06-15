//===-- SimpleRegisterCoalescing.h - Register Coalescing --------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements a simple register copy coalescing phase.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_SIMPLE_REGISTER_COALESCING_H
#define LLVM_CODEGEN_SIMPLE_REGISTER_COALESCING_H

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/LiveIntervalAnalysis.h"
#include "llvm/CodeGen/RegisterCoalescer.h"
#include "llvm/ADT/BitVector.h"

namespace llvm {
  class SimpleRegisterCoalescing;
  class LiveVariables;
  class TargetRegisterInfo;
  class TargetInstrInfo;
  class VirtRegMap;
  class MachineLoopInfo;

  /// CopyRec - Representation for copy instructions in coalescer queue.
  ///
  struct CopyRec {
    MachineInstr *MI;
    unsigned LoopDepth;
    CopyRec(MachineInstr *mi, unsigned depth)
      : MI(mi), LoopDepth(depth) {}
  };

  class SimpleRegisterCoalescing : public MachineFunctionPass,
                                   public RegisterCoalescer {
    MachineFunction* mf_;
    MachineRegisterInfo* mri_;
    const TargetMachine* tm_;
    const TargetRegisterInfo* tri_;
    const TargetInstrInfo* tii_;
    LiveIntervals *li_;
    const MachineLoopInfo* loopInfo;
    AliasAnalysis *AA;
    
    BitVector allocatableRegs_;
    DenseMap<const TargetRegisterClass*, BitVector> allocatableRCRegs_;

    /// JoinedCopies - Keep track of copies eliminated due to coalescing.
    ///
    SmallPtrSet<MachineInstr*, 32> JoinedCopies;

    /// ReMatCopies - Keep track of copies eliminated due to remat.
    ///
    SmallPtrSet<MachineInstr*, 32> ReMatCopies;

    /// ReMatDefs - Keep track of definition instructions which have
    /// been remat'ed.
    SmallPtrSet<MachineInstr*, 8> ReMatDefs;

  public:
    static char ID; // Pass identifcation, replacement for typeid
    SimpleRegisterCoalescing() : MachineFunctionPass(&ID) {}

    struct InstrSlots {
      enum {
        LOAD  = 0,
        USE   = 1,
        DEF   = 2,
        STORE = 3,
        NUM   = 4
      };
    };
    
    virtual void getAnalysisUsage(AnalysisUsage &AU) const;
    virtual void releaseMemory();

    /// runOnMachineFunction - pass entry point
    virtual bool runOnMachineFunction(MachineFunction&);

    bool coalesceFunction(MachineFunction &mf, RegallocQuery &) {
      // This runs as an independent pass, so don't do anything.
      return false;
    }

    /// print - Implement the dump method.
    virtual void print(raw_ostream &O, const Module* = 0) const;

  private:
    /// joinIntervals - join compatible live intervals
    void joinIntervals();

    /// CopyCoalesceInMBB - Coalesce copies in the specified MBB, putting
    /// copies that cannot yet be coalesced into the "TryAgain" list.
    void CopyCoalesceInMBB(MachineBasicBlock *MBB,
                           std::vector<CopyRec> &TryAgain);

    /// JoinCopy - Attempt to join intervals corresponding to SrcReg/DstReg,
    /// which are the src/dst of the copy instruction CopyMI.  This returns true
    /// if the copy was successfully coalesced away. If it is not currently
    /// possible to coalesce this interval, but it may be possible if other
    /// things get coalesced, then it returns true by reference in 'Again'.
    bool JoinCopy(CopyRec &TheCopy, bool &Again);
    
    /// JoinIntervals - Attempt to join these two intervals.  On failure, this
    /// returns false.  Otherwise, if one of the intervals being joined is a
    /// physreg, this method always canonicalizes DestInt to be it.  The output
    /// "SrcInt" will not have been modified, so we can use this information
    /// below to update aliases.
    bool JoinIntervals(LiveInterval &LHS, LiveInterval &RHS, bool &Swapped,
                       CoalescerPair &CP);
    
    /// SimpleJoin - Attempt to join the specified interval into this one. The
    /// caller of this method must guarantee that the RHS only contains a single
    /// value number and that the RHS is not defined by a copy from this
    /// interval.  This returns false if the intervals are not joinable, or it
    /// joins them and returns true.
    bool SimpleJoin(LiveInterval &LHS, LiveInterval &RHS, CoalescerPair &CP);
    
    /// Return true if the two specified registers belong to different register
    /// classes.  The registers may be either phys or virt regs.
    bool differingRegisterClasses(unsigned RegA, unsigned RegB) const;

    /// AdjustCopiesBackFrom - We found a non-trivially-coalescable copy. If
    /// the source value number is defined by a copy from the destination reg
    /// see if we can merge these two destination reg valno# into a single
    /// value number, eliminating a copy.
    bool AdjustCopiesBackFrom(LiveInterval &IntA, LiveInterval &IntB,
                              MachineInstr *CopyMI);

    /// HasOtherReachingDefs - Return true if there are definitions of IntB
    /// other than BValNo val# that can reach uses of AValno val# of IntA.
    bool HasOtherReachingDefs(LiveInterval &IntA, LiveInterval &IntB,
                              VNInfo *AValNo, VNInfo *BValNo);

    /// RemoveCopyByCommutingDef - We found a non-trivially-coalescable copy.
    /// If the source value number is defined by a commutable instruction and
    /// its other operand is coalesced to the copy dest register, see if we
    /// can transform the copy into a noop by commuting the definition.
    bool RemoveCopyByCommutingDef(LiveInterval &IntA, LiveInterval &IntB,
                                  MachineInstr *CopyMI);

    /// TrimLiveIntervalToLastUse - If there is a last use in the same basic
    /// block as the copy instruction, trim the ive interval to the last use
    /// and return true.
    bool TrimLiveIntervalToLastUse(SlotIndex CopyIdx,
                                   MachineBasicBlock *CopyMBB,
                                   LiveInterval &li, const LiveRange *LR);

    /// ReMaterializeTrivialDef - If the source of a copy is defined by a trivial
    /// computation, replace the copy by rematerialize the definition.
    bool ReMaterializeTrivialDef(LiveInterval &SrcInt, unsigned DstReg,
                                 unsigned DstSubIdx, MachineInstr *CopyMI);

    /// CanCoalesceWithImpDef - Returns true if the specified copy instruction
    /// from an implicit def to another register can be coalesced away.
    bool CanCoalesceWithImpDef(MachineInstr *CopyMI,
                               LiveInterval &li, LiveInterval &ImpLi) const;

    /// TurnCopiesFromValNoToImpDefs - The specified value# is defined by an
    /// implicit_def and it is being removed. Turn all copies from this value#
    /// into implicit_defs.
    void TurnCopiesFromValNoToImpDefs(LiveInterval &li, VNInfo *VNI);

    /// isWinToJoinVRWithSrcPhysReg - Return true if it's worth while to join a
    /// a virtual destination register with physical source register.
    bool isWinToJoinVRWithSrcPhysReg(MachineInstr *CopyMI,
                                    MachineBasicBlock *CopyMBB,
                                    LiveInterval &DstInt, LiveInterval &SrcInt);

    /// isWinToJoinVRWithDstPhysReg - Return true if it's worth while to join a
    /// copy from a virtual source register to a physical destination register.
    bool isWinToJoinVRWithDstPhysReg(MachineInstr *CopyMI,
                                    MachineBasicBlock *CopyMBB,
                                    LiveInterval &DstInt, LiveInterval &SrcInt);

    /// isWinToJoinCrossClass - Return true if it's profitable to coalesce
    /// two virtual registers from different register classes.
    bool isWinToJoinCrossClass(unsigned SrcReg,
                               unsigned DstReg,
                               const TargetRegisterClass *SrcRC,
                               const TargetRegisterClass *DstRC,
                               const TargetRegisterClass *NewRC);

    /// HasIncompatibleSubRegDefUse - If we are trying to coalesce a virtual
    /// register with a physical register, check if any of the virtual register
    /// operand is a sub-register use or def. If so, make sure it won't result
    /// in an illegal extract_subreg or insert_subreg instruction.
    bool HasIncompatibleSubRegDefUse(MachineInstr *CopyMI,
                                     unsigned VirtReg, unsigned PhysReg);

    /// CanJoinExtractSubRegToPhysReg - Return true if it's possible to coalesce
    /// an extract_subreg where dst is a physical register, e.g.
    /// cl = EXTRACT_SUBREG reg1024, 1
    bool CanJoinExtractSubRegToPhysReg(unsigned DstReg, unsigned SrcReg,
                                       unsigned SubIdx, unsigned &RealDstReg);

    /// CanJoinInsertSubRegToPhysReg - Return true if it's possible to coalesce
    /// an insert_subreg where src is a physical register, e.g.
    /// reg1024 = INSERT_SUBREG reg1024, c1, 0
    bool CanJoinInsertSubRegToPhysReg(unsigned DstReg, unsigned SrcReg,
                                      unsigned SubIdx, unsigned &RealDstReg);

    /// ValueLiveAt - Return true if the LiveRange pointed to by the given
    /// iterator, or any subsequent range with the same value number,
    /// is live at the given point.
    bool ValueLiveAt(LiveInterval::iterator LRItr, LiveInterval::iterator LREnd, 
                     SlotIndex defPoint) const;                                  

    /// RangeIsDefinedByCopy - Return true if the specified live range of the
    /// specified live interval is defined by a coalescable copy.
    bool RangeIsDefinedByCopy(LiveInterval &li, LiveRange *LR,
                              CoalescerPair &CP);

    /// UpdateRegDefsUses - Replace all defs and uses of SrcReg to DstReg and
    /// update the subregister number if it is not zero. If DstReg is a
    /// physical register and the existing subregister number of the def / use
    /// being updated is not zero, make sure to set it to the correct physical
    /// subregister.
    void UpdateRegDefsUses(unsigned SrcReg, unsigned DstReg, unsigned SubIdx);

    /// ShortenDeadCopyLiveRange - Shorten a live range defined by a dead copy.
    /// Return true if live interval is removed.
    bool ShortenDeadCopyLiveRange(LiveInterval &li, MachineInstr *CopyMI);

    /// ShortenDeadCopyLiveRange - Shorten a live range as it's artificially
    /// extended by a dead copy. Mark the last use (if any) of the val# as kill
    /// as ends the live range there. If there isn't another use, then this
    /// live range is dead. Return true if live interval is removed.
    bool ShortenDeadCopySrcLiveRange(LiveInterval &li, MachineInstr *CopyMI);

    /// RemoveDeadDef - If a def of a live interval is now determined dead,
    /// remove the val# it defines. If the live interval becomes empty, remove
    /// it as well.
    bool RemoveDeadDef(LiveInterval &li, MachineInstr *DefMI);

    /// lastRegisterUse - Returns the last use of the specific register between
    /// cycles Start and End or NULL if there are no uses.
    MachineOperand *lastRegisterUse(SlotIndex Start, SlotIndex End,
                                    unsigned Reg, SlotIndex &LastUseIdx) const;
  };

} // End llvm namespace

#endif
