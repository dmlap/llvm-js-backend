//===-- PhiElimination.cpp - Eliminate PHI nodes by inserting copies ------===//
//
// This pass eliminates machine instruction PHI nodes by inserting copy
// instructions.  This destroys SSA information, but is the desired input for
// some register allocators.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/SSARegMap.h"
#include "llvm/CodeGen/LiveVariables.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/TargetMachine.h"

namespace {
  struct PNE : public MachineFunctionPass {
    bool runOnMachineFunction(MachineFunction &Fn) {
      bool Changed = false;

      // Eliminate PHI instructions by inserting copies into predecessor blocks.
      //
      for (MachineFunction::iterator I = Fn.begin(), E = Fn.end(); I != E; ++I)
	Changed |= EliminatePHINodes(Fn, *I);

      //std::cerr << "AFTER PHI NODE ELIM:\n";
      //Fn.dump();
      return Changed;
    }

    virtual void getAnalysisUsage(AnalysisUsage &AU) const {
      AU.addPreserved<LiveVariables>();
      MachineFunctionPass::getAnalysisUsage(AU);
    }

  private:
    /// EliminatePHINodes - Eliminate phi nodes by inserting copy instructions
    /// in predecessor basic blocks.
    ///
    bool EliminatePHINodes(MachineFunction &MF, MachineBasicBlock &MBB);
  };

  RegisterPass<PNE> X("phi-node-elimination",
		      "Eliminate PHI nodes for register allocation");
}

const PassInfo *PHIEliminationID = X.getPassInfo();

/// EliminatePHINodes - Eliminate phi nodes by inserting copy instructions in
/// predecessor basic blocks.
///
bool PNE::EliminatePHINodes(MachineFunction &MF, MachineBasicBlock &MBB) {
  if (MBB.empty() || MBB.front()->getOpcode() != TargetInstrInfo::PHI)
    return false;   // Quick exit for normal case...

  LiveVariables *LV = getAnalysisToUpdate<LiveVariables>();
  const TargetInstrInfo &MII = MF.getTarget().getInstrInfo();
  const MRegisterInfo *RegInfo = MF.getTarget().getRegisterInfo();

  while (MBB.front()->getOpcode() == TargetInstrInfo::PHI) {
    MachineInstr *MI = MBB.front();
    // Unlink the PHI node from the basic block... but don't delete the PHI yet
    MBB.erase(MBB.begin());

    assert(MI->getOperand(0).isVirtualRegister() &&
           "PHI node doesn't write virt reg?");

    unsigned DestReg = MI->getOperand(0).getAllocatedRegNum();
    
    // Create a new register for the incoming PHI arguments
    const TargetRegisterClass *RC = MF.getSSARegMap()->getRegClass(DestReg);
    unsigned IncomingReg = MF.getSSARegMap()->createVirtualRegister(RC);

    // Insert a register to register copy in the top of the current block (but
    // after any remaining phi nodes) which copies the new incoming register
    // into the phi node destination.
    //
    MachineBasicBlock::iterator AfterPHIsIt = MBB.begin();
    while (AfterPHIsIt != MBB.end() &&
           (*AfterPHIsIt)->getOpcode() == TargetInstrInfo::PHI)
      ++AfterPHIsIt;    // Skip over all of the PHI nodes...
    RegInfo->copyRegToReg(MBB, AfterPHIsIt, DestReg, IncomingReg, RC);
    
    // Update live variable information if there is any...
    if (LV) {
      MachineInstr *PHICopy = *(AfterPHIsIt-1);

      // Add information to LiveVariables to know that the incoming value is
      // dead.  This says that the register is dead, not killed, because we
      // cannot use the live variable information to indicate that the variable
      // is defined in multiple entry blocks.  Instead, we pretend that this
      // instruction defined it and killed it at the same time.
      //
      LV->addVirtualRegisterDead(IncomingReg, PHICopy);

      // Since we are going to be deleting the PHI node, if it is the last use
      // of any registers, or if the value itself is dead, we need to move this
      // information over to the new copy we just inserted...
      //
      std::pair<LiveVariables::killed_iterator, LiveVariables::killed_iterator> 
        RKs = LV->killed_range(MI);
      if (RKs.first != RKs.second) {
        for (LiveVariables::killed_iterator I = RKs.first; I != RKs.second; ++I)
          LV->addVirtualRegisterKilled(I->second, PHICopy);
        LV->removeVirtualRegistersKilled(RKs.first, RKs.second);
      }

      RKs = LV->dead_range(MI);
      if (RKs.first != RKs.second) {
        for (LiveVariables::killed_iterator I = RKs.first; I != RKs.second; ++I)
          LV->addVirtualRegisterDead(I->second, PHICopy);
        LV->removeVirtualRegistersDead(RKs.first, RKs.second);
      }
    }

    // Now loop over all of the incoming arguments, changing them to copy into
    // the IncomingReg register in the corresponding predecessor basic block.
    //
    for (int i = MI->getNumOperands() - 1; i >= 2; i-=2) {
      MachineOperand &opVal = MI->getOperand(i-1);
      
      // Get the MachineBasicBlock equivalent of the BasicBlock that is the
      // source path the PHI.
      MachineBasicBlock &opBlock = *MI->getOperand(i).getMachineBasicBlock();

      // Figure out where to insert the copy, which is at the end of the
      // predecessor basic block, but before any terminator/branch
      // instructions...
      MachineBasicBlock::iterator I = opBlock.end();
      if (I != opBlock.begin()) {  // Handle empty blocks
        --I;
        // must backtrack over ALL the branches in the previous block
        while (MII.isTerminatorInstr((*I)->getOpcode()) &&
               I != opBlock.begin())
          --I;
        
        // move back to the first branch instruction so new instructions
        // are inserted right in front of it and not in front of a non-branch
        if (!MII.isTerminatorInstr((*I)->getOpcode()))
          ++I;
      }
      
      // Check to make sure we haven't already emitted the copy for this block.
      // This can happen because PHI nodes may have multiple entries for the
      // same basic block.  It doesn't matter which entry we use though, because
      // all incoming values are guaranteed to be the same for a particular bb.
      //
      // If we emitted a copy for this basic block already, it will be right
      // where we want to insert one now.  Just check for a definition of the
      // register we are interested in!
      //
      bool HaveNotEmitted = true;
      
      if (I != opBlock.begin()) {
        MachineInstr *PrevInst = *(I-1);
        for (unsigned i = 0, e = PrevInst->getNumOperands(); i != e; ++i) {
          MachineOperand &MO = PrevInst->getOperand(i);
          if (MO.isVirtualRegister() && MO.getReg() == IncomingReg)
            if (MO.opIsDef() || MO.opIsDefAndUse()) {
              HaveNotEmitted = false;
              break;
            }             
        }
      }

      if (HaveNotEmitted) {
        assert(opVal.isVirtualRegister() &&
               "Machine PHI Operands must all be virtual registers!");
        RegInfo->copyRegToReg(opBlock, I, IncomingReg, opVal.getReg(), RC);
      }
    }
    
    // really delete the PHI instruction now!
    delete MI;
  }

  return true;
}
