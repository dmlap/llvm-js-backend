; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/simple.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

; CHECK: [
; ModuleID = '/dev/shm/tmp/src.cpp.o'
target datalayout = "e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32"
target triple = "i386-pc-linux-gnu"

; CHECK: { "ident": "@.str", "intertype": "globalVariable", "lineNum": 5, "type": "[15 x i8]", "value": { "text": ""hello, world!\0A\00" } }
@.str = private constant [15 x i8] c"hello, world!\0A\00" ; [#uses=1]

; [#uses=0]
define i32 @main() {
  %1 = alloca i32, align 4                        ; [#uses=1]
  store i32 0, i32* %1
  %2 = call i32 (i8*, ...)* @printf(i8* getelementptr inbounds ([15 x i8]* @.str, i32 0, i32 0)) ; [#uses=0]
  ret i32 0
}

; [#uses=1]
declare i32 @printf(i8*, ...)
