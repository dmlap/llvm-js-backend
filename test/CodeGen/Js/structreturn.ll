; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/structreturn.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

%struct_t = type {i32, i32, i32 }

define void @usestruct(%struct_t* %struct) {
entry:
  ret void
}

define void @structreturn(%struct_t* sret %result) {
entry:
; CHECK: usestruct(
  call void @usestruct(%struct_t* %result)
; CHECK: return;
  ret void
}
