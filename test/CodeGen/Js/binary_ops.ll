; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/binary_ops.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @binary_ops() {
entry:
; CHECK: = 1 + 1;
  %0	= add i32 1, 1          ; %0 = 2
; CHECK: = 0.1 + 0.1;
  %1	= fadd double 0.1, 0.1  ; %1 ~ 0.2
; CHECK: = 1 - 1;
  %2	= sub i16  1, 1         ; %2 = 0
; CHECK: = 6 - 5;
  %3	= fsub float 6.0, 5.0   ; %3 ~ 0.1
; CHECK: = 2 * 3;
  %4	= mul i32 2, 3          ; %4 = 6
; CHECK: = 2 * 4;
  %5	= fmul double 2.0, 4.0  ; %5 ~ 8.0
; CHECK: = Math.abs(Math.floor(-13 / 7));
  %6	= udiv i32 -13, 7       ; %6 = 1
; CHECK: = Math.floor(-13 / 7);
  %7	= sdiv i32 -13, 7       ; %7 = -1
; CHECK: = 3 / 3;
  %8	= fdiv float 3.0, 3.0   ; %8 ~ 1.0
; CHECK: = Math.abs(-13 % 7);
  %9	= urem i32 -13, 7       ; %9 = 6
; CHECK: -13 % 7;
  %10	= srem i32 -13, 7       ; %10 = -6
; CHECK: = 7 % 3;
  %11   = frem double 7.0, 3.0  ; %11 ~ 1.0
  ret void
}