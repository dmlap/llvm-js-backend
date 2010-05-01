//===-- JsBackendTargetInfo.cpp - JsBackend Target Implementation -------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "JsTargetMachine.h"
#include "llvm/Module.h"
#include "llvm/Target/TargetRegistry.h"
using namespace llvm;

Target llvm::TheJsBackendTarget;

static unsigned JsBackend_TripleMatchQuality(const std::string &TT) {
  // This class always works, but shouldn't be the default in most cases.
  return 1;
}

extern "C" void LLVMInitializeJsBackendTargetInfo() { 
  TargetRegistry::RegisterTarget(TheJsBackendTarget, "js",    
                                  "javascript backend",
                                  &JsBackend_TripleMatchQuality);
}
