//===-- GCMetadataPrinter.cpp - Garbage collection infrastructure ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the abstract base class GCMetadataPrinter.
//
//===----------------------------------------------------------------------===//

#include "llvm/CodeGen/GCMetadataPrinter.h"

using namespace llvm;

// -----------------------------------------------------------------------------

template<> GCMetadataPrinterRegistry::node *GCMetadataPrinterRegistry::Head = 0;
template<> GCMetadataPrinterRegistry::node *GCMetadataPrinterRegistry::Tail = 0;
template<> GCMetadataPrinterRegistry::listener *
GCMetadataPrinterRegistry::ListenerHead = 0;
template<> GCMetadataPrinterRegistry::listener *
GCMetadataPrinterRegistry::ListenerTail = 0;

// -----------------------------------------------------------------------------

GCMetadataPrinter::GCMetadataPrinter() { }

GCMetadataPrinter::~GCMetadataPrinter() { }

void GCMetadataPrinter::beginAssembly(std::ostream &OS, AsmPrinter &AP,
                                      const TargetAsmInfo &TAI) {
  // Default is no action.
}

void GCMetadataPrinter::finishAssembly(std::ostream &OS, AsmPrinter &AP,
                                       const TargetAsmInfo &TAI) {
  // Default is no action.
}
