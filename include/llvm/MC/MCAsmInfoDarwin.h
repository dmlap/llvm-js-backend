//===---- MCAsmInfoDarwin.h - Darwin asm properties -------------*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines target asm properties related what form asm statements
// should take in general on Darwin-based targets
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_DARWIN_TARGET_ASM_INFO_H
#define LLVM_DARWIN_TARGET_ASM_INFO_H

#include "llvm/MC/MCAsmInfo.h"

namespace llvm {
  class GlobalValue;
  class GlobalVariable;
  class Type;
  class Mangler;

  struct DarwinMCAsmInfo : public MCAsmInfo {
    explicit DarwinMCAsmInfo();
  };
}


#endif // LLVM_DARWIN_TARGET_ASM_INFO_H
