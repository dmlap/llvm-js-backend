; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/memory.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @memory() {
entry:
; CHECK: var
; CHECK: = _p();
  %ptr = alloca i32
; CHECK: {{[_$A-z0-9]+}}.s(3);
  store i32 3, i32* %ptr
; CHECK: {{[_$A-z0-9]+}} = {{[_$A-z0-9]+}}();
  %val = load i32* %ptr
  ret void
}