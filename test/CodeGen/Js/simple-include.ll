; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/simple-include.html
; RUN: llc < %s -march=js -O0 | FileCheck %s

; ModuleID = 'test.c'
target datalayout = "e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:32:32-n8:16:32"
target triple = "i386-pc-linux-gnu"

@hello = global i8* getelementptr inbounds ([7 x i8]* @.str, i32 0, i32 0) ; <i8**> [#uses=1]
@.str = private constant [7 x i8] c"hello\0A\00", align 1 ; <[7 x i8]*> [#uses=1]
; CHECK: var hello = _.hello = 

; CHECK: _.main = function
define i32 @main() nounwind {
entry:
  %retval = alloca i32                            ; <i32*> [#uses=2]
  %0 = alloca i32                                 ; <i32*> [#uses=2]
  %"alloca point" = bitcast i32 0 to i32          ; <i32> [#uses=0]
  %1 = load i8** @hello, align 4                  ; <i8*> [#uses=1]
  %2 = call i32 (i8*, ...)* @printf(i8* noalias %1) nounwind ; <i32> [#uses=0]
  store i32 0, i32* %0, align 4
  %3 = load i32* %0, align 4                      ; <i32> [#uses=1]
  store i32 %3, i32* %retval, align 4
  br label %return

return:                                           ; preds = %entry
  %retval1 = load i32* %retval                    ; <i32> [#uses=1]
  ret i32 %retval1
}

declare i32 @printf(i8* noalias, ...) nounwind
