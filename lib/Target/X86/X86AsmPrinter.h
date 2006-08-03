//===-- X86AsmPrinter.h - Convert X86 LLVM code to Intel assembly ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file was developed by the LLVM research group and is distributed under
// the University of Illinois Open Source License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file the shared super class printer that converts from our internal
// representation of machine-dependent LLVM code to Intel and AT&T format
// assembly language.  This printer is the output mechanism used by `llc'.
//
//===----------------------------------------------------------------------===//

#ifndef X86ASMPRINTER_H
#define X86ASMPRINTER_H

#include "X86.h"
#include "X86TargetMachine.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/DwarfWriter.h"
#include "llvm/CodeGen/MachineDebugInfo.h"
#include "llvm/ADT/Statistic.h"
#include <set>


namespace llvm {

extern Statistic<> EmittedInsts;

/// X86DwarfWriter - Dwarf debug info writer customized for Darwin/Mac OS X
///
struct X86DwarfWriter : public DwarfWriter {
  X86DwarfWriter(std::ostream &o, AsmPrinter *ap) : DwarfWriter(o, ap) {
      needsSet = true;
      DwarfAbbrevSection = ".section __DWARF,__debug_abbrev,regular,debug";
      DwarfInfoSection = ".section __DWARF,__debug_info,regular,debug";
      DwarfLineSection = ".section __DWARF,__debug_line,regular,debug";
      DwarfFrameSection = ".section __DWARF,__debug_frame,regular,debug";
      DwarfPubNamesSection = ".section __DWARF,__debug_pubnames,regular,debug";
      DwarfPubTypesSection = ".section __DWARF,__debug_pubtypes,regular,debug";
      DwarfStrSection = ".section __DWARF,__debug_str,regular,debug";
      DwarfLocSection = ".section __DWARF,__debug_loc,regular,debug";
      DwarfARangesSection = ".section __DWARF,__debug_aranges,regular,debug";
      DwarfRangesSection = ".section __DWARF,__debug_ranges,regular,debug";
      DwarfMacInfoSection = ".section __DWARF,__debug_macinfo,regular,debug";
      TextSection = ".text";
      DataSection = ".data";
  }
  virtual void virtfn();  // out of line virtual fn.
};

struct X86SharedAsmPrinter : public AsmPrinter {
  X86DwarfWriter DW;

  X86SharedAsmPrinter(std::ostream &O, X86TargetMachine &TM)
    : AsmPrinter(O, TM), DW(O, this) {
    Subtarget = &TM.getSubtarget<X86Subtarget>();
  }

  bool doInitialization(Module &M);
  bool doFinalization(Module &M);

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesAll();
    if (Subtarget->isTargetDarwin()) {
      AU.addRequired<MachineDebugInfo>();
    }
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  const char *DefaultTextSection;   // "_text" for MASM, ".text" for others.
  const char *DefaultDataSection;   // "_data" for MASM, ".data" for others.
  const X86Subtarget *Subtarget;

  // Necessary for Darwin to print out the apprioriate types of linker stubs
  std::set<std::string> FnStubs, GVStubs, LinkOnceStubs;

  inline static bool isScale(const MachineOperand &MO) {
    return MO.isImmediate() &&
          (MO.getImmedValue() == 1 || MO.getImmedValue() == 2 ||
          MO.getImmedValue() == 4 || MO.getImmedValue() == 8);
  }

  inline static bool isMem(const MachineInstr *MI, unsigned Op) {
    if (MI->getOperand(Op).isFrameIndex()) return true;
    return Op+4 <= MI->getNumOperands() &&
      MI->getOperand(Op  ).isRegister() && isScale(MI->getOperand(Op+1)) &&
      MI->getOperand(Op+2).isRegister() &&
      (MI->getOperand(Op+3).isImmediate() ||
       MI->getOperand(Op+3).isGlobalAddress() ||
       MI->getOperand(Op+3).isConstantPoolIndex());
  }
};

} // end namespace llvm

#endif
