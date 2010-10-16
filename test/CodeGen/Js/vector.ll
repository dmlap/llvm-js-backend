; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/vector.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @vector() {
entry:
; CHECK: = ([ 1, 2 ])[1];
  %0 = extractelement <2 x i32> <i32 1, i32 2>, i32 1
; CHECK: = [ false, false, false ];
; CHECK: {{[_$A-z0-9]+}}[2] = (true);
  %1 = insertelement <3 x i1> <i1 0, i1 0, i1 0>, i1 1, i32 2
; CHECK: = [7, 5, 6, 4];
  %2 = shufflevector <2 x i32> <i32 4, i32 5>, <2 x i32> <i32 6, i32 7>, <4 x i32> <i32 3, i32 1, i32 2, i32 0>
  ret void
}