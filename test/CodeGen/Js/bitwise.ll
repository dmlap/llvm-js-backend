; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/bitwise.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @bitwise() {
entry:
; CHECK: = 1 << 2;
  %0 = shl i32 1, 2
; CHECK: = -1 >>> 32;
  %1 = lshr i32 -1, 32
; CHECK: = -1 >> 2;
  %2 = ashr i32 -1, 2
; CHECK: = 1 & 0;
  %3 = and i32 1, 0
; CHECK: = 0 | 1;
  %4 = or i32 0, 1
; CHECK: = 2 ^ 1;
  %5 = xor i32 2, 1
  ret void
}