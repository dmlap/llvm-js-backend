//===-- MachineInstr.cpp --------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// Methods common to all machine instructions.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/Target/MRegisterInfo.h"
#include "llvm/Support/LeakDetector.h"
#include <iostream>

using namespace llvm;

// Global variable holding an array of descriptors for machine instructions.
// The actual object needs to be created separately for each target machine.
// This variable is initialized and reset by class TargetInstrInfo.
//
// FIXME: This should be a property of the target so that more than one target
// at a time can be active...
//
namespace llvm {
  extern const TargetInstrDescriptor *TargetInstrDescriptors;
}

/// MachineInstr ctor - This constructor only does a _reserve_ of the operands,
/// not a resize for them.  It is expected that if you use this that you call
/// add* methods below to fill up the operands, instead of the Set methods.
/// Eventually, the "resizing" ctors will be phased out.
///
MachineInstr::MachineInstr(short opcode, unsigned numOperands)
  : Opcode(opcode), NumImplicitOps(0), parent(0) {
  Operands.reserve(numOperands);
  // Make sure that we get added to a machine basicblock
  LeakDetector::addGarbageObject(this);
}

void MachineInstr::addImplicitDefUseOperands(const TargetInstrDescriptor &TID) {
  if (TID.ImplicitDefs)
    for (const unsigned *ImpDefs = TID.ImplicitDefs; *ImpDefs; ++ImpDefs) {
      MachineOperand Op;
      Op.opType = MachineOperand::MO_Register;
      Op.IsDef = true;
      Op.IsImp = true;
      Op.IsKill = false;
      Op.IsDead = false;
      Op.contents.RegNo = *ImpDefs;
      Op.offset = 0;
      Operands.push_back(Op);
    }
  if (TID.ImplicitUses)
    for (const unsigned *ImpUses = TID.ImplicitUses; *ImpUses; ++ImpUses) {
      MachineOperand Op;
      Op.opType = MachineOperand::MO_Register;
      Op.IsDef = false;
      Op.IsImp = true;
      Op.IsKill = false;
      Op.IsDead = false;
      Op.contents.RegNo = *ImpUses;
      Op.offset = 0;
      Operands.push_back(Op);
    }
}

/// MachineInstr ctor - This constructor create a MachineInstr and add the
/// implicit operands. It reserves space for numOperand operands.
MachineInstr::MachineInstr(const TargetInstrInfo &TII, short opcode,
                           unsigned numOperands)
  : Opcode(opcode), NumImplicitOps(0), parent(0) {
  const TargetInstrDescriptor &TID = TII.get(opcode);
  if (TID.ImplicitDefs)
    for (const unsigned *ImpDefs = TID.ImplicitDefs; *ImpDefs; ++ImpDefs)
      NumImplicitOps++;
  if (TID.ImplicitUses)
    for (const unsigned *ImpUses = TID.ImplicitUses; *ImpUses; ++ImpUses)
      NumImplicitOps++;
  Operands.reserve(NumImplicitOps + numOperands);
  addImplicitDefUseOperands(TID);
  // Make sure that we get added to a machine basicblock
  LeakDetector::addGarbageObject(this);
}

/// MachineInstr ctor - Work exactly the same as the ctor above, except that the
/// MachineInstr is created and added to the end of the specified basic block.
///
MachineInstr::MachineInstr(MachineBasicBlock *MBB, short opcode,
                           unsigned numOperands)
  : Opcode(opcode), NumImplicitOps(0), parent(0) {
  assert(MBB && "Cannot use inserting ctor with null basic block!");
  const TargetInstrDescriptor &TID = MBB->getParent()->getTarget().
    getInstrInfo()->get(opcode);
  if (TID.ImplicitDefs)
    for (const unsigned *ImpDefs = TID.ImplicitDefs; *ImpDefs; ++ImpDefs)
      NumImplicitOps++;
  if (TID.ImplicitUses)
    for (const unsigned *ImpUses = TID.ImplicitUses; *ImpUses; ++ImpUses)
      NumImplicitOps++;
  Operands.reserve(NumImplicitOps + numOperands);
  addImplicitDefUseOperands(TID);
  // Make sure that we get added to a machine basicblock
  LeakDetector::addGarbageObject(this);
  MBB->push_back(this);  // Add instruction to end of basic block!
}

/// MachineInstr ctor - Copies MachineInstr arg exactly
///
MachineInstr::MachineInstr(const MachineInstr &MI) {
  Opcode = MI.getOpcode();
  NumImplicitOps = MI.NumImplicitOps;
  Operands.reserve(MI.getNumOperands());

  // Add operands
  for (unsigned i = 0; i != MI.getNumOperands(); ++i)
    Operands.push_back(MI.getOperand(i));

  // Set parent, next, and prev to null
  parent = 0;
  prev = 0;
  next = 0;
}


MachineInstr::~MachineInstr() {
  LeakDetector::removeGarbageObject(this);
}

/// removeFromParent - This method unlinks 'this' from the containing basic
/// block, and returns it, but does not delete it.
MachineInstr *MachineInstr::removeFromParent() {
  assert(getParent() && "Not embedded in a basic block!");
  getParent()->remove(this);
  return this;
}


/// OperandComplete - Return true if it's illegal to add a new operand
///
bool MachineInstr::OperandsComplete() const {
  int NumOperands = TargetInstrDescriptors[Opcode].numOperands;
  if ((TargetInstrDescriptors[Opcode].Flags & M_VARIABLE_OPS) == 0 &&
      getNumOperands()-NumImplicitOps >= (unsigned)NumOperands)
    return true;  // Broken: we have all the operands of this instruction!
  return false;
}

/// isIdenticalTo - Return true if this operand is identical to the specified
/// operand.
bool MachineOperand::isIdenticalTo(const MachineOperand &Other) const {
  if (getType() != Other.getType()) return false;
  
  switch (getType()) {
  default: assert(0 && "Unrecognized operand type");
  case MachineOperand::MO_Register:
    return getReg() == Other.getReg() && isDef() == Other.isDef();
  case MachineOperand::MO_Immediate:
    return getImm() == Other.getImm();
  case MachineOperand::MO_MachineBasicBlock:
    return getMBB() == Other.getMBB();
  case MachineOperand::MO_FrameIndex:
    return getFrameIndex() == Other.getFrameIndex();
  case MachineOperand::MO_ConstantPoolIndex:
    return getConstantPoolIndex() == Other.getConstantPoolIndex() &&
           getOffset() == Other.getOffset();
  case MachineOperand::MO_JumpTableIndex:
    return getJumpTableIndex() == Other.getJumpTableIndex();
  case MachineOperand::MO_GlobalAddress:
    return getGlobal() == Other.getGlobal() && getOffset() == Other.getOffset();
  case MachineOperand::MO_ExternalSymbol:
    return !strcmp(getSymbolName(), Other.getSymbolName()) &&
           getOffset() == Other.getOffset();
  }
}

/// setOpcode - Replace the opcode of the current instruction with a new one.
///
void MachineInstr::setOpcode(unsigned Op) {
  Operands.erase(Operands.begin(), Operands.begin()+NumImplicitOps);
  NumImplicitOps = 0;
  Opcode = Op;
  if (!getParent())
    return;
  const TargetInstrDescriptor &TID = getParent()->getParent()->
    getTarget().getInstrInfo()->get(Op);
  if (TID.ImplicitDefs)
    for (const unsigned *ImpDefs = TID.ImplicitDefs; *ImpDefs; ++ImpDefs) {
      MachineOperand Op;
      Op.opType = MachineOperand::MO_Register;
      Op.IsDef = true;
      Op.IsImp = true;
      Op.IsKill = false;
      Op.IsDead = false;
      Op.contents.RegNo = *ImpDefs;
      Op.offset = 0;
      Operands.insert(Operands.begin()+NumImplicitOps, Op);
      NumImplicitOps++;
    }
  if (TID.ImplicitUses)
    for (const unsigned *ImpUses = TID.ImplicitUses; *ImpUses; ++ImpUses) {
      MachineOperand Op;
      Op.opType = MachineOperand::MO_Register;
      Op.IsDef = false;
      Op.IsImp = true;
      Op.IsKill = false;
      Op.IsDead = false;
      Op.contents.RegNo = *ImpUses;
      Op.offset = 0;
      Operands.insert(Operands.begin()+NumImplicitOps, Op);
      NumImplicitOps++;
    }
}


void MachineInstr::dump() const {
  std::cerr << "  " << *this;
}

static inline void OutputReg(std::ostream &os, unsigned RegNo,
                             const MRegisterInfo *MRI = 0) {
  if (!RegNo || MRegisterInfo::isPhysicalRegister(RegNo)) {
    if (MRI)
      os << "%" << MRI->get(RegNo).Name;
    else
      os << "%mreg(" << RegNo << ")";
  } else
    os << "%reg" << RegNo;
}

static void print(const MachineOperand &MO, std::ostream &OS,
                  const TargetMachine *TM) {
  const MRegisterInfo *MRI = 0;

  if (TM) MRI = TM->getRegisterInfo();

  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    OutputReg(OS, MO.getReg(), MRI);
    break;
  case MachineOperand::MO_Immediate:
    OS << MO.getImmedValue();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    OS << "mbb<"
       << ((Value*)MO.getMachineBasicBlock()->getBasicBlock())->getName()
       << "," << (void*)MO.getMachineBasicBlock() << ">";
    break;
  case MachineOperand::MO_FrameIndex:
    OS << "<fi#" << MO.getFrameIndex() << ">";
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    OS << "<cp#" << MO.getConstantPoolIndex() << ">";
    break;
  case MachineOperand::MO_JumpTableIndex:
    OS << "<jt#" << MO.getJumpTableIndex() << ">";
    break;
  case MachineOperand::MO_GlobalAddress:
    OS << "<ga:" << ((Value*)MO.getGlobal())->getName();
    if (MO.getOffset()) OS << "+" << MO.getOffset();
    OS << ">";
    break;
  case MachineOperand::MO_ExternalSymbol:
    OS << "<es:" << MO.getSymbolName();
    if (MO.getOffset()) OS << "+" << MO.getOffset();
    OS << ">";
    break;
  default:
    assert(0 && "Unrecognized operand type");
  }
}

void MachineInstr::print(std::ostream &OS, const TargetMachine *TM) const {
  unsigned StartOp = 0;

   // Specialize printing if op#0 is definition
  if (getNumOperands() && getOperand(0).isReg() && getOperand(0).isDef()) {
    ::print(getOperand(0), OS, TM);
    OS << " = ";
    ++StartOp;   // Don't print this operand again!
  }

  // Must check if Target machine is not null because machine BB could not
  // be attached to a Machine function yet
  if (TM)
    OS << TM->getInstrInfo()->getName(getOpcode());

  for (unsigned i = StartOp, e = getNumOperands(); i != e; ++i) {
    const MachineOperand& mop = getOperand(i);
    if (i != StartOp)
      OS << ",";
    OS << " ";
    ::print(mop, OS, TM);

    if (mop.isReg()) {
      if (mop.isDef() || mop.isKill() || mop.isDead() || mop.isImplicit()) {
        OS << "<";
        bool NeedComma = false;
        if (mop.isImplicit()) {
          OS << (mop.isDef() ? "imp-def" : "imp-use");
          NeedComma = true;
        } else if (mop.isDef()) {
          OS << "def";
          NeedComma = true;
        }
        if (mop.isKill() || mop.isDead()) {
          if (NeedComma)
            OS << ",";
          if (mop.isKill())
            OS << "kill";
          if (mop.isDead())
            OS << "dead";
        }
        OS << ">";
      }
    }
  }

  OS << "\n";
}

std::ostream &llvm::operator<<(std::ostream &os, const MachineInstr &MI) {
  // If the instruction is embedded into a basic block, we can find the target
  // info for the instruction.
  if (const MachineBasicBlock *MBB = MI.getParent()) {
    const MachineFunction *MF = MBB->getParent();
    if (MF)
      MI.print(os, &MF->getTarget());
    else
      MI.print(os, 0);
    return os;
  }

  // Otherwise, print it out in the "raw" format without symbolic register names
  // and such.
  os << TargetInstrDescriptors[MI.getOpcode()].Name;

  for (unsigned i = 0, N = MI.getNumOperands(); i < N; i++) {
    os << "\t" << MI.getOperand(i);
    if (MI.getOperand(i).isReg() && MI.getOperand(i).isDef())
      os << "<d>";
  }

  return os << "\n";
}

std::ostream &llvm::operator<<(std::ostream &OS, const MachineOperand &MO) {
  switch (MO.getType()) {
  case MachineOperand::MO_Register:
    OutputReg(OS, MO.getReg());
    break;
  case MachineOperand::MO_Immediate:
    OS << (long)MO.getImmedValue();
    break;
  case MachineOperand::MO_MachineBasicBlock:
    OS << "<mbb:"
       << ((Value*)MO.getMachineBasicBlock()->getBasicBlock())->getName()
       << "@" << (void*)MO.getMachineBasicBlock() << ">";
    break;
  case MachineOperand::MO_FrameIndex:
    OS << "<fi#" << MO.getFrameIndex() << ">";
    break;
  case MachineOperand::MO_ConstantPoolIndex:
    OS << "<cp#" << MO.getConstantPoolIndex() << ">";
    break;
  case MachineOperand::MO_JumpTableIndex:
    OS << "<jt#" << MO.getJumpTableIndex() << ">";
    break;
  case MachineOperand::MO_GlobalAddress:
    OS << "<ga:" << ((Value*)MO.getGlobal())->getName() << ">";
    break;
  case MachineOperand::MO_ExternalSymbol:
    OS << "<es:" << MO.getSymbolName() << ">";
    break;
  default:
    assert(0 && "Unrecognized operand type");
    break;
  }

  return OS;
}
