//===-- X86ATTAsmPrinter.h - Convert X86 LLVM code to AT&T assembly -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// AT&T assembly code printer class.
//
//===----------------------------------------------------------------------===//

#ifndef X86ATTASMPRINTER_H
#define X86ATTASMPRINTER_H

#include "../X86.h"
#include "../X86MachineFunctionInfo.h"
#include "../X86TargetMachine.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/DwarfWriter.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/ValueTypes.h"
#include "llvm/Support/Compiler.h"

namespace llvm {

class MachineJumpTableInfo;
class MCContext;
class MCInst;
class MCStreamer;

class VISIBILITY_HIDDEN X86ATTAsmPrinter : public AsmPrinter {
  const X86Subtarget *Subtarget;
  
  MCContext *Context;
  MCStreamer *Streamer;
 public:
  explicit X86ATTAsmPrinter(raw_ostream &O, X86TargetMachine &TM,
                            const TargetAsmInfo *T, CodeGenOpt::Level OL,
                            bool V)
    : AsmPrinter(O, TM, T, OL, V) {
    Subtarget = &TM.getSubtarget<X86Subtarget>();
    Context = 0;
    Streamer = 0;
  }

  virtual const char *getPassName() const {
    return "X86 AT&T-Style Assembly Printer";
  }

  void getAnalysisUsage(AnalysisUsage &AU) const {
    AU.setPreservesAll();
    if (Subtarget->isTargetDarwin() ||
        Subtarget->isTargetELF() ||
        Subtarget->isTargetCygMing()) {
      AU.addRequired<MachineModuleInfo>();
    }
    AU.addRequired<DwarfWriter>();
    AsmPrinter::getAnalysisUsage(AU);
  }

  bool doInitialization(Module &M);
  bool doFinalization(Module &M);

  /// printInstruction - This method is automatically generated by tablegen
  /// from the instruction set description.  This method returns true if the
  /// machine instruction was sufficiently described to print it, otherwise it
  /// returns false.
  bool printInstruction(const MachineInstr *MI);
  
  
  // New MCInst printing stuff.
  bool printInstruction(const MCInst *MI);

  void printOperand(const MCInst *MI, unsigned OpNo,
                    const char *Modifier = 0);
  void printMemReference(const MCInst *MI, unsigned Op);
  void printLeaMemReference(const MCInst *MI, unsigned Op);
  void printSSECC(const MCInst *MI, unsigned Op);
  void printPICLabel(const MCInst *MI, unsigned Op);
  void print_pcrel_imm(const MCInst *MI, unsigned OpNo);
  
  void printi8mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi16mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi32mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi64mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi128mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf32mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf64mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf80mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf128mem(const MCInst *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printlea32mem(const MCInst *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo);
  }
  void printlea64mem(const MCInst *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo);
  }
  void printlea64_32mem(const MCInst *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo);
  }
  
  

  // These methods are used by the tablegen'erated instruction printer.
  void printOperand(const MachineInstr *MI, unsigned OpNo,
                    const char *Modifier = 0);
  void print_pcrel_imm(const MachineInstr *MI, unsigned OpNo);
  void printi8mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi16mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi32mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi64mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi128mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printi256mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf32mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf64mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf80mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf128mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printf256mem(const MachineInstr *MI, unsigned OpNo) {
    printMemReference(MI, OpNo);
  }
  void printlea32mem(const MachineInstr *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo);
  }
  void printlea64mem(const MachineInstr *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo);
  }
  void printlea64_32mem(const MachineInstr *MI, unsigned OpNo) {
    printLeaMemReference(MI, OpNo, "subreg64");
  }

  bool printAsmMRegister(const MachineOperand &MO, char Mode);
  bool PrintAsmOperand(const MachineInstr *MI, unsigned OpNo,
                       unsigned AsmVariant, const char *ExtraCode);
  bool PrintAsmMemoryOperand(const MachineInstr *MI, unsigned OpNo,
                             unsigned AsmVariant, const char *ExtraCode);

  void printMachineInstruction(const MachineInstr *MI);
  void printSSECC(const MachineInstr *MI, unsigned Op);
  void printMemReference(const MachineInstr *MI, unsigned Op,
                         const char *Modifier=NULL);
  void printLeaMemReference(const MachineInstr *MI, unsigned Op,
                            const char *Modifier=NULL);
  void printPICJumpTableSetLabel(unsigned uid,
                                 const MachineBasicBlock *MBB) const;
  void printPICJumpTableSetLabel(unsigned uid, unsigned uid2,
                                 const MachineBasicBlock *MBB) const {
    AsmPrinter::printPICJumpTableSetLabel(uid, uid2, MBB);
  }
  void printPICJumpTableEntry(const MachineJumpTableInfo *MJTI,
                              const MachineBasicBlock *MBB,
                              unsigned uid) const;

  void printPICLabel(const MachineInstr *MI, unsigned Op);
  void printModuleLevelGV(const GlobalVariable* GVar);

  void PrintPICBaseSymbol() const;
  
  bool runOnMachineFunction(MachineFunction &F);

  void emitFunctionHeader(const MachineFunction &MF);

  // Necessary for Darwin to print out the apprioriate types of linker stubs
  StringSet<> FnStubs, GVStubs, HiddenGVStubs;

  // Necessary for dllexport support
  StringSet<> DLLExportedFns, DLLExportedGVs;

  // We have to propagate some information about MachineFunction to
  // AsmPrinter. It's ok, when we're printing the function, since we have
  // access to MachineFunction and can get the appropriate MachineFunctionInfo.
  // Unfortunately, this is not possible when we're printing reference to
  // Function (e.g. calling it and so on). Even more, there is no way to get the
  // corresponding MachineFunctions: it can even be not created at all. That's
  // why we should use additional structure, when we're collecting all necessary
  // information.
  //
  // This structure is using e.g. for name decoration for stdcall & fastcall'ed
  // function, since we have to use arguments' size for decoration.
  typedef std::map<const Function*, X86MachineFunctionInfo> FMFInfoMap;
  FMFInfoMap FunctionInfoMap;

  void decorateName(std::string& Name, const GlobalValue* GV);
};

} // end namespace llvm

#endif
