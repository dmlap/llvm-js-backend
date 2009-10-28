; RUN: opt < %s -dse -S | FileCheck %s

target datalayout = "e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:128:128"
target triple = "i386-apple-darwin7"

define i8 @test2(i8* %P) nounwind {
; CHECK: @test2
; CHECK-NOT: store i8 1
; CHECK: ret i8 0
entry:
  call void @llvm.lifetime.start(i64 32, i8* %P)
  call void @llvm.lifetime.end(i64 32, i8* %P)
  store i8 1, i8* %P
  ret i8 0
}

declare {}* @llvm.lifetime.start(i64 %S, i8* nocapture %P) readonly
declare void @llvm.lifetime.end(i64 %S, i8* nocapture %P)