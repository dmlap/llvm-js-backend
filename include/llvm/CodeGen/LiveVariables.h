//===-- llvm/CodeGen/LiveVariables.h - Live Variable Analysis ---*- C++ -*-===//
// 
// This file implements the LiveVariable analysis pass.  For each machine
// instruction in the function, this pass calculates the set of registers that
// are immediately dead after the instruction (i.e., the instruction calculates
// the value, but it is never used) and the set of registers that are used by
// the instruction, but are never used after the instruction (i.e., they are
// killed).
//
// This class computes live variables using are sparse implementation based on
// the machine code SSA form.  This class computes live variable information for
// each virtual and _register allocatable_ physical register in a function.  It
// uses the dominance properties of SSA form to efficiently compute live
// variables for virtual registers, and assumes that physical registers are only
// live within a single basic block (allowing it to do a single local analysis
// to resolve physical register lifetimes in each basic block).  If a physical
// register is not register allocatable, it is not tracked.  This is useful for
// things like the stack pointer and condition codes.
//   
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_LIVEVARIABLES_H
#define LLVM_CODEGEN_LIVEVARIABLES_H

#include "llvm/CodeGen/MachineFunctionPass.h"
#include <map>

class MRegisterInfo;

class LiveVariables : public MachineFunctionPass {
  struct VarInfo {
    /// DefBlock - The basic block which defines this value...
    MachineBasicBlock *DefBlock;
    MachineInstr      *DefInst;

    /// AliveBlocks - Set of blocks of which this value is alive completely
    /// through.  This is a bit set which uses the basic block number as an
    /// index.
    ///
    std::vector<bool> AliveBlocks;

    /// Kills - List of MachineBasicblock's which contain the last use of this
    /// virtual register (kill it).  This also includes the specific instruction
    /// which kills the value.
    ///
    std::vector<std::pair<MachineBasicBlock*, MachineInstr*> > Kills;

    VarInfo() : DefBlock(0), DefInst(0) {}
  };

  /// VirtRegInfo - This list is a mapping from virtual register number to
  /// variable information.  FirstVirtualRegister is subtracted from the virtual
  /// register number before indexing into this list.
  ///
  std::vector<VarInfo> VirtRegInfo;

  /// RegistersKilled - This multimap keeps track of all of the registers that
  /// are dead immediately after an instruction reads its operands.  If an
  /// instruction does not have an entry in this map, it kills no registers.
  ///
  std::multimap<MachineInstr*, unsigned> RegistersKilled;

  /// RegistersDead - This multimap keeps track of all of the registers that are
  /// dead immediately after an instruction executes, which are not dead after
  /// the operands are evaluated.  In practice, this only contains registers
  /// which are defined by an instruction, but never used.
  ///
  std::multimap<MachineInstr*, unsigned> RegistersDead;

  /// AllocatablePhysicalRegisters - This vector keeps track of which registers
  /// are actually register allocatable by the target machine.  We can not track
  /// liveness for values that are not in this set.
  ///
  std::vector<bool> AllocatablePhysicalRegisters;
private:   // Intermediate data structures

  /// BBMap - Maps LLVM basic blocks to their corresponding machine basic block.
  /// This also provides a numbering of the basic blocks in the function.
  std::map<const BasicBlock*, std::pair<MachineBasicBlock*, unsigned> > BBMap;
  
  const MRegisterInfo *RegInfo;

  MachineInstr **PhysRegInfo;
  bool          *PhysRegUsed;

public:

  virtual bool runOnMachineFunction(MachineFunction &MF);

  /// killed_iterator - Iterate over registers killed by a machine instruction
  ///
  typedef std::multimap<MachineInstr*,
			unsigned>::const_iterator killed_iterator;
  
  /// killed_begin/end - Get access to the range of registers killed by a
  /// machine instruction.
  killed_iterator killed_begin(MachineInstr *MI) const {
    return RegistersKilled.lower_bound(MI);
  }
  killed_iterator killed_end(MachineInstr *MI) const {
    return RegistersKilled.upper_bound(MI);
  }

  killed_iterator dead_begin(MachineInstr *MI) const {
    return RegistersDead.lower_bound(MI);
  }
  killed_iterator dead_end(MachineInstr *MI) const {
    return RegistersDead.upper_bound(MI);
  }

  /// addVirtualRegisterKill - Add information about the fact that the specified
  /// register is dead after being used by the specified instruction.
  ///
  void addVirtualRegisterKill(unsigned IncomingReg, MachineInstr *MI) {
    RegistersDead.insert(std::make_pair(MI, IncomingReg));
  }

  virtual void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesAll();
  }

  virtual void releaseMemory() {
    VirtRegInfo.clear();
    RegistersKilled.clear();
    RegistersDead.clear();
  }
private:
  VarInfo &getVarInfo(unsigned RegIdx) {
    if (RegIdx >= VirtRegInfo.size()) {
      if (RegIdx >= 2*VirtRegInfo.size())
	VirtRegInfo.resize(RegIdx*2);
      else
	VirtRegInfo.resize(2*VirtRegInfo.size());
    }
    return VirtRegInfo[RegIdx];
  }

  void MarkVirtRegAliveInBlock(VarInfo &VRInfo, const BasicBlock *BB);
  void HandleVirtRegUse(VarInfo &VRInfo, MachineBasicBlock *MBB,
                       	MachineInstr *MI);
  void HandlePhysRegUse(unsigned Reg, MachineInstr *MI);
  void HandlePhysRegDef(unsigned Reg, MachineInstr *MI);
};

#endif
