//=====-- SparcSubtarget.h - Define Subtarget for the SPARC ----*- C++ -*-====//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file declares the SPARC specific subclass of TargetSubtarget.
//
//===----------------------------------------------------------------------===//

#ifndef SPARC_SUBTARGET_H
#define SPARC_SUBTARGET_H

#include "llvm/Target/TargetSubtarget.h"
#include <string>

namespace llvm {
  class Module;
  
class SparcSubtarget : public TargetSubtarget {
  bool IsV9;
  bool V8DeprecatedInsts;
  bool IsVIS;
public:
  SparcSubtarget(const Module &M, const std::string &FS);

  bool isV9() const { return IsV9; }
  bool isVIS() const { return IsVIS; }
  bool useDeprecatedV8Instructions() const { return V8DeprecatedInsts; }
  
  /// ParseSubtargetFeatures - Parses features string setting specified 
  /// subtarget options.  Definition of function is auto generated by tblgen.
  std::string ParseSubtargetFeatures(const std::string &FS,
                                     const std::string &CPU);

};

} // end namespace llvm

#endif
