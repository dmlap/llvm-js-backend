//===- llvm/Transforms/Scalar/InstructionCombining.h -------------*- C++ -*--=//
//
// InstructionCombining - Combine instructions to form fewer, simple
//   instructions.  This pass does not modify the CFG, and has a tendancy to
//   make instructions dead, so a subsequent DCE pass is useful.
//
// This pass combines things like:
//    %Y = add int 1, %X
//    %Z = add int 1, %Y
// into:
//    %Z = add int 2, %X
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TRANSFORMS_SCALAR_INSTRUCTIONCOMBINING_H
#define LLVM_TRANSFORMS_SCALAR_INSTRUCTIONCOMBINING_H

class Pass;
Pass *createInstructionCombiningPass();

#endif
