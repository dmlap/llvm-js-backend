; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/intrinsics.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @intrinsics() {
entry:
; CHECK: = Math.sqrt(4.5);
  %0 = call float @llvm.sqrt.f32(float 4.5)
; CHECK: = Math.sqrt(4.4);
  %1 = call double @llvm.sqrt.f64(double 4.4)
; CHECK: = Math.pow(4.5, 2);
  %2 = call float @llvm.powi.f32(float 4.5, i32 2)
; CHECK: = Math.pow(4.4, 3);
  %3 = call double @llvm.powi.f64(double 4.4, i32 3)
; CHECK-NOT: prefetch
  call void @llvm.prefetch(i8* undef, i32 0, i32 0)
; CHECK: return;
  ret void
}

declare float     @llvm.sqrt.f32(float %Val)
declare double    @llvm.sqrt.f64(double %Val)
declare float     @llvm.powi.f32(float  %Val, i32 %power)
declare double    @llvm.powi.f64(double %Val, i32 %power)
declare void @llvm.prefetch(i8* %addr, i32 %rw, i32 %locality)