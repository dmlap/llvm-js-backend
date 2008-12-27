//===- SPUInstrInfo.cpp - Cell SPU Instruction Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Cell SPU implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "SPURegisterNames.h"
#include "SPUInstrInfo.h"
#include "SPUInstrBuilder.h"
#include "SPUTargetMachine.h"
#include "SPUGenInstrInfo.inc"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/Streams.h"

using namespace llvm;

namespace {
  //! Predicate for an unconditional branch instruction
  inline bool isUncondBranch(const MachineInstr *I) {
    unsigned opc = I->getOpcode();

    return (opc == SPU::BR
	    || opc == SPU::BRA
	    || opc == SPU::BI);
  }

  inline bool isCondBranch(const MachineInstr *I) {
    unsigned opc = I->getOpcode();

    return (opc == SPU::BRNZr32
            || opc == SPU::BRNZv4i32
	    || opc == SPU::BRZr32
	    || opc == SPU::BRZv4i32
	    || opc == SPU::BRHNZr16
	    || opc == SPU::BRHNZv8i16
	    || opc == SPU::BRHZr16
	    || opc == SPU::BRHZv8i16);
  }
}

SPUInstrInfo::SPUInstrInfo(SPUTargetMachine &tm)
  : TargetInstrInfoImpl(SPUInsts, sizeof(SPUInsts)/sizeof(SPUInsts[0])),
    TM(tm),
    RI(*TM.getSubtargetImpl(), *this)
{
  /* NOP */
}

/// getPointerRegClass - Return the register class to use to hold pointers.
/// This is used for addressing modes.
const TargetRegisterClass *
SPUInstrInfo::getPointerRegClass() const
{
  return &SPU::R32CRegClass;
}

bool
SPUInstrInfo::isMoveInstr(const MachineInstr& MI,
                          unsigned& sourceReg,
                          unsigned& destReg) const {
  // Primarily, ORI and OR are generated by copyRegToReg. But, there are other
  // cases where we can safely say that what's being done is really a move
  // (see how PowerPC does this -- it's the model for this code too.)
  switch (MI.getOpcode()) {
  default:
    break;
  case SPU::ORIv4i32:
  case SPU::ORIr32:
  case SPU::ORHIv8i16:
  case SPU::ORHIr16:
  case SPU::ORHIi8i16:
  case SPU::ORBIv16i8:
  case SPU::ORBIr8:
  case SPU::ORIi16i32:
  case SPU::ORIi8i32:
  case SPU::AHIvec:
  case SPU::AHIr16:
  case SPU::AIvec:
    assert(MI.getNumOperands() == 3 &&
           MI.getOperand(0).isReg() &&
           MI.getOperand(1).isReg() &&
           MI.getOperand(2).isImm() &&
           "invalid SPU ORI/ORHI/ORBI/AHI/AI/SFI/SFHI instruction!");
    if (MI.getOperand(2).getImm() == 0) {
      sourceReg = MI.getOperand(1).getReg();
      destReg = MI.getOperand(0).getReg();
      return true;
    }
    break;
  case SPU::AIr32:
    assert(MI.getNumOperands() == 3 &&
           "wrong number of operands to AIr32");
    if (MI.getOperand(0).isReg() &&
        (MI.getOperand(1).isReg() ||
         MI.getOperand(1).isFI()) &&
        (MI.getOperand(2).isImm() &&
         MI.getOperand(2).getImm() == 0)) {
      sourceReg = MI.getOperand(1).getReg();
      destReg = MI.getOperand(0).getReg();
      return true;
    }
    break;
  case SPU::LRr8:
  case SPU::LRr16:
  case SPU::LRr32:
  case SPU::LRf32:
  case SPU::LRr64:
  case SPU::LRf64:
  case SPU::LRr128:
  case SPU::LRv16i8:
  case SPU::LRv8i16:
  case SPU::LRv4i32:
  case SPU::LRv4f32:
  case SPU::LRv2i64:
  case SPU::LRv2f64:
  case SPU::ORv16i8_i8:
  case SPU::ORv8i16_i16:
  case SPU::ORv4i32_i32:
  case SPU::ORv2i64_i64:
  case SPU::ORv4f32_f32:
  case SPU::ORv2f64_f64:
  case SPU::ORi8_v16i8:
  case SPU::ORi16_v8i16:
  case SPU::ORi32_v4i32:
  case SPU::ORi64_v2i64:
  case SPU::ORf32_v4f32:
  case SPU::ORf64_v2f64: {
    assert(MI.getNumOperands() == 2 &&
           MI.getOperand(0).isReg() &&
           MI.getOperand(1).isReg() &&
           "invalid SPU OR<type>_<vec> instruction!");
    if (MI.getOperand(0).getReg() == MI.getOperand(1).getReg()) {
      sourceReg = MI.getOperand(0).getReg();
      destReg = MI.getOperand(0).getReg();
      return true;
    }
    break;
  }
  case SPU::ORv16i8:
  case SPU::ORv8i16:
  case SPU::ORv4i32:
  case SPU::ORr32:
  case SPU::ORr64:
  case SPU::ORf32:
  case SPU::ORf64:
    assert(MI.getNumOperands() == 3 &&
           MI.getOperand(0).isReg() &&
           MI.getOperand(1).isReg() &&
           MI.getOperand(2).isReg() &&
           "invalid SPU OR(vec|r32|r64|gprc) instruction!");
    if (MI.getOperand(1).getReg() == MI.getOperand(2).getReg()) {
      sourceReg = MI.getOperand(1).getReg();
      destReg = MI.getOperand(0).getReg();
      return true;
    }
    break;
  }

  return false;
}

unsigned
SPUInstrInfo::isLoadFromStackSlot(const MachineInstr *MI,
                                  int &FrameIndex) const {
  switch (MI->getOpcode()) {
  default: break;
  case SPU::LQDv16i8:
  case SPU::LQDv8i16:
  case SPU::LQDv4i32:
  case SPU::LQDv4f32:
  case SPU::LQDv2f64:
  case SPU::LQDr128:
  case SPU::LQDr64:
  case SPU::LQDr32:
  case SPU::LQDr16: {
    const MachineOperand MOp1 = MI->getOperand(1);
    const MachineOperand MOp2 = MI->getOperand(2);
    if (MOp1.isImm()
	&& (MOp2.isFI()
	    || (MOp2.isReg() && MOp2.getReg() == SPU::R1))) {
      if (MOp2.isFI())
	FrameIndex = MOp2.getIndex();
      else
	FrameIndex = MOp1.getImm() / SPUFrameInfo::stackSlotSize();
      return MI->getOperand(0).getReg();
    }
    break;
  }
  case SPU::LQXv4i32:
  case SPU::LQXr128:
  case SPU::LQXr64:
  case SPU::LQXr32:
  case SPU::LQXr16:
    if (MI->getOperand(1).isReg() && MI->getOperand(2).isReg()
	&& (MI->getOperand(2).getReg() == SPU::R1
	    || MI->getOperand(1).getReg() == SPU::R1)) {
      FrameIndex = MI->getOperand(2).getIndex();
      return MI->getOperand(0).getReg();
    }
    break;
  }
  return 0;
}

unsigned
SPUInstrInfo::isStoreToStackSlot(const MachineInstr *MI,
                                 int &FrameIndex) const {
  switch (MI->getOpcode()) {
  default: break;
  case SPU::STQDv16i8:
  case SPU::STQDv8i16:
  case SPU::STQDv4i32:
  case SPU::STQDv4f32:
  case SPU::STQDv2f64:
  case SPU::STQDr128:
  case SPU::STQDr64:
  case SPU::STQDr32:
  case SPU::STQDr16:
  case SPU::STQDr8: {
    const MachineOperand MOp1 = MI->getOperand(1);
    const MachineOperand MOp2 = MI->getOperand(2);
    if (MOp1.isImm() && MOp2.isFI()) {
      FrameIndex = MOp2.getIndex();
      return MI->getOperand(0).getReg();
    }
    break;
  }
#if 0
    case SPU::STQXv16i8:
  case SPU::STQXv8i16:
  case SPU::STQXv4i32:
  case SPU::STQXv4f32:
  case SPU::STQXv2f64:
  case SPU::STQXr128:
  case SPU::STQXr64:
  case SPU::STQXr32:
  case SPU::STQXr16:
  case SPU::STQXr8:
    if (MI->getOperand(1).isReg() && MI->getOperand(2).isReg()
	&& (MI->getOperand(2).getReg() == SPU::R1
	    || MI->getOperand(1).getReg() == SPU::R1)) {
      FrameIndex = MI->getOperand(2).getIndex();
      return MI->getOperand(0).getReg();
    }
    break;
#endif
  }
  return 0;
}

bool SPUInstrInfo::copyRegToReg(MachineBasicBlock &MBB,
                                   MachineBasicBlock::iterator MI,
                                   unsigned DestReg, unsigned SrcReg,
                                   const TargetRegisterClass *DestRC,
                                   const TargetRegisterClass *SrcRC) const
{
  // We support cross register class moves for our aliases, such as R3 in any
  // reg class to any other reg class containing R3.  This is required because
  // we instruction select bitconvert i64 -> f64 as a noop for example, so our
  // types have no specific meaning.
  
  if (DestRC == SPU::R8CRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORBIr8), DestReg).addReg(SrcReg).addImm(0);
  } else if (DestRC == SPU::R16CRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORHIr16), DestReg).addReg(SrcReg).addImm(0);
  } else if (DestRC == SPU::R32CRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORIr32), DestReg).addReg(SrcReg).addImm(0);
  } else if (DestRC == SPU::R32FPRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORf32), DestReg).addReg(SrcReg)
      .addReg(SrcReg);
  } else if (DestRC == SPU::R64CRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORr64), DestReg).addReg(SrcReg)
      .addReg(SrcReg);
  } else if (DestRC == SPU::R64FPRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORf64), DestReg).addReg(SrcReg)
      .addReg(SrcReg);
  } /* else if (DestRC == SPU::GPRCRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORgprc), DestReg).addReg(SrcReg)
      .addReg(SrcReg);
  } */ else if (DestRC == SPU::VECREGRegisterClass) {
    BuildMI(MBB, MI, get(SPU::ORv4i32), DestReg).addReg(SrcReg)
      .addReg(SrcReg);
  } else {
    // Attempt to copy unknown/unsupported register class!
    return false;
  }
  
  return true;
}

void
SPUInstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                     MachineBasicBlock::iterator MI,
                                     unsigned SrcReg, bool isKill, int FrameIdx,
                                     const TargetRegisterClass *RC) const
{
  unsigned opc;
  bool isValidFrameIdx = (FrameIdx < SPUFrameInfo::maxFrameOffset());
  if (RC == SPU::GPRCRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr128 : SPU::STQXr128);
  } else if (RC == SPU::R64CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr64 : SPU::STQXr64);
  } else if (RC == SPU::R64FPRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr64 : SPU::STQXr64);
  } else if (RC == SPU::R32CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr32 : SPU::STQXr32);
  } else if (RC == SPU::R32FPRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr32 : SPU::STQXr32);
  } else if (RC == SPU::R16CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr16 : SPU::STQXr16);
  } else if (RC == SPU::R8CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::STQDr8 : SPU::STQXr8);
  } else if (RC == SPU::VECREGRegisterClass) {
    opc = (isValidFrameIdx) ? SPU::STQDv16i8 : SPU::STQXv16i8;
  } else {
    assert(0 && "Unknown regclass!");
    abort();
  }

  addFrameReference(BuildMI(MBB, MI, get(opc))
                    .addReg(SrcReg, false, false, isKill), FrameIdx);
}

void SPUInstrInfo::storeRegToAddr(MachineFunction &MF, unsigned SrcReg,
                                     bool isKill,
                                     SmallVectorImpl<MachineOperand> &Addr,
                                     const TargetRegisterClass *RC,
                                     SmallVectorImpl<MachineInstr*> &NewMIs) const {
  cerr << "storeRegToAddr() invoked!\n";
  abort();

  if (Addr[0].isFI()) {
    /* do what storeRegToStackSlot does here */
  } else {
    unsigned Opc = 0;
    if (RC == SPU::GPRCRegisterClass) {
      /* Opc = PPC::STW; */
    } else if (RC == SPU::R16CRegisterClass) {
      /* Opc = PPC::STD; */
    } else if (RC == SPU::R32CRegisterClass) {
      /* Opc = PPC::STFD; */
    } else if (RC == SPU::R32FPRegisterClass) {
      /* Opc = PPC::STFD; */
    } else if (RC == SPU::R64FPRegisterClass) {
      /* Opc = PPC::STFS; */
    } else if (RC == SPU::VECREGRegisterClass) {
      /* Opc = PPC::STVX; */
    } else {
      assert(0 && "Unknown regclass!");
      abort();
    }
    MachineInstrBuilder MIB = BuildMI(MF, get(Opc))
      .addReg(SrcReg, false, false, isKill);
    for (unsigned i = 0, e = Addr.size(); i != e; ++i) {
      MachineOperand &MO = Addr[i];
      if (MO.isReg())
        MIB.addReg(MO.getReg());
      else if (MO.isImm())
        MIB.addImm(MO.getImm());
      else
        MIB.addFrameIndex(MO.getIndex());
    }
    NewMIs.push_back(MIB);
  }
}

void
SPUInstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        unsigned DestReg, int FrameIdx,
                                        const TargetRegisterClass *RC) const
{
  unsigned opc;
  bool isValidFrameIdx = (FrameIdx < SPUFrameInfo::maxFrameOffset());
  if (RC == SPU::GPRCRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr128 : SPU::LQXr128);
  } else if (RC == SPU::R64CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr64 : SPU::LQXr64);
  } else if (RC == SPU::R64FPRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr64 : SPU::LQXr64);
  } else if (RC == SPU::R32CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr32 : SPU::LQXr32);
  } else if (RC == SPU::R32FPRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr32 : SPU::LQXr32);
  } else if (RC == SPU::R16CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr16 : SPU::LQXr16);
  } else if (RC == SPU::R8CRegisterClass) {
    opc = (isValidFrameIdx ? SPU::LQDr8 : SPU::LQXr8);
  } else if (RC == SPU::VECREGRegisterClass) {
    opc = (isValidFrameIdx) ? SPU::LQDv16i8 : SPU::LQXv16i8;
  } else {
    assert(0 && "Unknown regclass in loadRegFromStackSlot!");
    abort();
  }

  addFrameReference(BuildMI(MBB, MI, get(opc)).addReg(DestReg), FrameIdx);
}

/*!
  \note We are really pessimistic here about what kind of a load we're doing.
 */
void SPUInstrInfo::loadRegFromAddr(MachineFunction &MF, unsigned DestReg,
                                   SmallVectorImpl<MachineOperand> &Addr,
                                   const TargetRegisterClass *RC,
                                   SmallVectorImpl<MachineInstr*> &NewMIs)
    const {
  cerr << "loadRegToAddr() invoked!\n";
  abort();

  if (Addr[0].isFI()) {
    /* do what loadRegFromStackSlot does here... */
  } else {
    unsigned Opc = 0;
    if (RC == SPU::R8CRegisterClass) {
      /* do brilliance here */
    } else if (RC == SPU::R16CRegisterClass) {
      /* Opc = PPC::LWZ; */
    } else if (RC == SPU::R32CRegisterClass) {
      /* Opc = PPC::LD; */
    } else if (RC == SPU::R32FPRegisterClass) {
      /* Opc = PPC::LFD; */
    } else if (RC == SPU::R64FPRegisterClass) {
      /* Opc = PPC::LFS; */
    } else if (RC == SPU::VECREGRegisterClass) {
      /* Opc = PPC::LVX; */
    } else if (RC == SPU::GPRCRegisterClass) {
      /* Opc = something else! */
    } else {
      assert(0 && "Unknown regclass!");
      abort();
    }
    MachineInstrBuilder MIB = BuildMI(MF, get(Opc), DestReg);
    for (unsigned i = 0, e = Addr.size(); i != e; ++i) {
      MachineOperand &MO = Addr[i];
      if (MO.isReg())
        MIB.addReg(MO.getReg());
      else if (MO.isImm())
        MIB.addImm(MO.getImm());
      else
        MIB.addFrameIndex(MO.getIndex());
    }
    NewMIs.push_back(MIB);
  }
}

/// foldMemoryOperand - SPU, like PPC, can only fold spills into
/// copy instructions, turning them into load/store instructions.
MachineInstr *
SPUInstrInfo::foldMemoryOperandImpl(MachineFunction &MF,
                                    MachineInstr *MI,
                                    const SmallVectorImpl<unsigned> &Ops,
                                    int FrameIndex) const
{
#if SOMEDAY_SCOTT_LOOKS_AT_ME_AGAIN
  if (Ops.size() != 1) return NULL;

  unsigned OpNum = Ops[0];
  unsigned Opc = MI->getOpcode();
  MachineInstr *NewMI = 0;
  
  if ((Opc == SPU::ORr32
       || Opc == SPU::ORv4i32)
       && MI->getOperand(1).getReg() == MI->getOperand(2).getReg()) {
    if (OpNum == 0) {  // move -> store
      unsigned InReg = MI->getOperand(1).getReg();
      bool isKill = MI->getOperand(1).isKill();
      if (FrameIndex < SPUFrameInfo::maxFrameOffset()) {
        NewMI = addFrameReference(BuildMI(MF, TII.get(SPU::STQDr32))
                                  .addReg(InReg, false, false, isKill),
                                  FrameIndex);
      }
    } else {           // move -> load
      unsigned OutReg = MI->getOperand(0).getReg();
      bool isDead = MI->getOperand(0).isDead();
      Opc = (FrameIndex < SPUFrameInfo::maxFrameOffset())
        ? SPU::STQDr32 : SPU::STQXr32;
      NewMI = addFrameReference(BuildMI(MF, TII.get(Opc))
                       .addReg(OutReg, true, false, false, isDead), FrameIndex);
    }
  }

  return NewMI;
#else
  return 0;
#endif
}

//! Branch analysis
/*
  \note This code was kiped from PPC. There may be more branch analysis for
  CellSPU than what's currently done here.
 */
bool
SPUInstrInfo::AnalyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB,
			    MachineBasicBlock *&FBB,
			    SmallVectorImpl<MachineOperand> &Cond) const {
  // If the block has no terminators, it just falls into the block after it.
  MachineBasicBlock::iterator I = MBB.end();
  if (I == MBB.begin() || !isUnpredicatedTerminator(--I))
    return false;

  // Get the last instruction in the block.
  MachineInstr *LastInst = I;
  
  // If there is only one terminator instruction, process it.
  if (I == MBB.begin() || !isUnpredicatedTerminator(--I)) {
    if (isUncondBranch(LastInst)) {
      TBB = LastInst->getOperand(0).getMBB();
      return false;
    } else if (isCondBranch(LastInst)) {
      // Block ends with fall-through condbranch.
      TBB = LastInst->getOperand(1).getMBB();
      Cond.push_back(LastInst->getOperand(0));
      Cond.push_back(LastInst->getOperand(1));
      return false;
    }
    // Otherwise, don't know what this is.
    return true;
  }
  
  // Get the instruction before it if it's a terminator.
  MachineInstr *SecondLastInst = I;

  // If there are three terminators, we don't know what sort of block this is.
  if (SecondLastInst && I != MBB.begin() &&
      isUnpredicatedTerminator(--I))
    return true;
  
  // If the block ends with a conditional and unconditional branch, handle it.
  if (isCondBranch(SecondLastInst) && isUncondBranch(LastInst)) {
    TBB =  SecondLastInst->getOperand(1).getMBB();
    Cond.push_back(SecondLastInst->getOperand(0));
    Cond.push_back(SecondLastInst->getOperand(1));
    FBB = LastInst->getOperand(0).getMBB();
    return false;
  }
  
  // If the block ends with two unconditional branches, handle it.  The second
  // one is not executed, so remove it.
  if (isUncondBranch(SecondLastInst) && isUncondBranch(LastInst)) {
    TBB = SecondLastInst->getOperand(0).getMBB();
    I = LastInst;
    I->eraseFromParent();
    return false;
  }

  // Otherwise, can't handle this.
  return true;
}
    
unsigned
SPUInstrInfo::RemoveBranch(MachineBasicBlock &MBB) const {
  MachineBasicBlock::iterator I = MBB.end();
  if (I == MBB.begin())
    return 0;
  --I;
  if (!isCondBranch(I) && !isUncondBranch(I))
    return 0;

  // Remove the first branch.
  I->eraseFromParent();
  I = MBB.end();
  if (I == MBB.begin())
    return 1;

  --I;
  if (isCondBranch(I))
    return 1;

  // Remove the second branch.
  I->eraseFromParent();
  return 2;
}
    
unsigned
SPUInstrInfo::InsertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB,
			   MachineBasicBlock *FBB,
			   const SmallVectorImpl<MachineOperand> &Cond) const {
  // Shouldn't be a fall through.
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert((Cond.size() == 2 || Cond.size() == 0) && 
         "SPU branch conditions have two components!");
  
  // One-way branch.
  if (FBB == 0) {
    if (Cond.empty())   // Unconditional branch
      BuildMI(&MBB, get(SPU::BR)).addMBB(TBB);
    else {              // Conditional branch
      /* BuildMI(&MBB, get(SPU::BRNZ))
        .addImm(Cond[0].getImm()).addReg(Cond[1].getReg()).addMBB(TBB); */
      cerr << "SPUInstrInfo::InsertBranch conditional branch logic needed\n";
      abort();
    }
    return 1;
  }
  
  // Two-way Conditional Branch.
#if 0
  BuildMI(&MBB, get(SPU::BRNZ))
    .addImm(Cond[0].getImm()).addReg(Cond[1].getReg()).addMBB(TBB);
  BuildMI(&MBB, get(SPU::BR)).addMBB(FBB);
#else
  cerr << "SPUInstrInfo::InsertBranch conditional branch logic needed\n";
  abort();
#endif

  return 2;
}


