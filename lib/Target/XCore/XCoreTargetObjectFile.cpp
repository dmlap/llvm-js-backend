//===-- XCoreTargetObjectFile.cpp - XCore object files --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "XCoreTargetObjectFile.h"
#include "XCoreSubtarget.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;


void XCoreTargetObjectFile::Initialize(MCContext &Ctx, const TargetMachine &TM){
  TargetLoweringObjectFileELF::Initialize(Ctx, TM);

  DataSection = getELFSection(".dp.data", MCSectionELF::SHT_PROGBITS, 
                              MCSectionELF::SHF_ALLOC | MCSectionELF::SHF_WRITE,
                              SectionKind::getDataRel());
  BSSSection = getELFSection(".dp.bss", MCSectionELF::SHT_NOBITS,
                              MCSectionELF::SHF_ALLOC | MCSectionELF::SHF_WRITE,
                              SectionKind::getBSS());
  
  // TLS globals are lowered in the backend to arrays indexed by the current
  // thread id. After lowering they require no special handling by the linker
  // and can be placed in the standard data / bss sections.
  TLSDataSection = DataSection;
  TLSBSSSection = BSSSection;
  
  if (TM.getSubtarget<XCoreSubtarget>().isXS1A())
    // FIXME: Why is this writable ("datarel")???
    ReadOnlySection = 
      getELFSection(".dp.rodata", MCSectionELF::SHT_PROGBITS,
                    MCSectionELF::SHF_ALLOC | MCSectionELF::SHF_WRITE,
                    SectionKind::getDataRel());
  else
    ReadOnlySection = 
      getELFSection(".cp.rodata", MCSectionELF::SHT_PROGBITS, 
                    MCSectionELF::SHF_ALLOC, SectionKind::getReadOnly());
}
