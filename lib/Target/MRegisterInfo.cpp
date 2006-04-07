//===- MRegisterInfo.cpp - Target Register Information Implementation -----===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MRegisterInfo interface.
//
//===----------------------------------------------------------------------===//

#include "llvm/Target/MRegisterInfo.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineLocation.h"

using namespace llvm;

MRegisterInfo::MRegisterInfo(const TargetRegisterDesc *D, unsigned NR,
                             regclass_iterator RCB, regclass_iterator RCE,
                             int CFSO, int CFDO)
  : Desc(D), NumRegs(NR), RegClassBegin(RCB), RegClassEnd(RCE) {
  assert(NumRegs < FirstVirtualRegister &&
         "Target has too many physical registers!");

  CallFrameSetupOpcode   = CFSO;
  CallFrameDestroyOpcode = CFDO;
}

MRegisterInfo::~MRegisterInfo() {}

std::vector<bool> MRegisterInfo::getAllocatableSet(MachineFunction &MF) const {
  std::vector<bool> Allocatable(NumRegs);
  for (MRegisterInfo::regclass_iterator I = regclass_begin(),
         E = regclass_end(); I != E; ++I) {
    const TargetRegisterClass *RC = *I;
    for (TargetRegisterClass::iterator I = RC->allocation_order_begin(MF),
           E = RC->allocation_order_end(MF); I != E; ++I)
      Allocatable[*I] = true;
  }
  return Allocatable;
}

/// getStackDirection - This method should return the factor by which stacks
/// grow.  The tyical value is -4 which is the grows negatively in 4 byte
/// increments.
int MRegisterInfo::getStackDirection() const {
  return -sizeof(int32_t);
}

/// getLocation - This method should return the actual location of a frame
/// variable given the frame index.  The location is returned in ML.
/// Subclasses should override this method for special handling of frame
/// variables and then call MRegisterInfo::getLocation for the default action.
void MRegisterInfo::getLocation(MachineFunction &MF, unsigned Index,
                        MachineLocation &ML) const {
  MachineFrameInfo *MFI = MF.getFrameInfo();
  ML.set(getFrameRegister(MF),
         MFI->getObjectOffset(Index) + MFI->getStackSize());
}

/// getInitialFrameState - Returns a list of machine moves that are assumed
/// on entry to a function.
void
MRegisterInfo::getInitialFrameState(std::vector<MachineMove *> &Moves) const {
  // Default is to do nothing.
}

