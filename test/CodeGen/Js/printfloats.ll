; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/printfloats.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @printfloats() {
entry:
; CHECK: = Infinity + 0
  %0 = fadd double 0x7FF0000000000000, 0.0
; CHECK: = NaN + 0
  %1 = fadd double 0x7ff8000000000000, 0.0
  ret void
}
