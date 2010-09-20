; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/varargs.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @misc(...) {
entry:
  ; Initialize variable argument processing
  %ap = alloca i8*
  %ap2 = bitcast i8** %ap to i8*
; CHECK: ._ = 0;
  call void @llvm.va_start(i8* %ap2)

  ; Read a single integer argument
; CHECK: = arguments[
; CHECK: ._++];
  %tmp = va_arg i8** %ap, i32

  ; Demonstrate usage of llvm.va_copy and llvm.va_end
  %aq = alloca i8*
  %aq2 = bitcast i8** %aq to i8*
; CHECK: ._ =
; CHECK: ._;
  call void @llvm.va_copy(i8* %aq2, i8* %ap2)
  call void @llvm.va_end(i8* %aq2)

  ; Stop processing of arguments.
  call void @llvm.va_end(i8* %ap2)
  ret void
}

declare void @llvm.va_start(i8*)
declare void @llvm.va_copy(i8*, i8*)
declare void @llvm.va_end(i8*)