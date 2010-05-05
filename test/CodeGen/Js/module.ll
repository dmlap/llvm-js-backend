; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -o %t2
; RUN: llc < %s -march=js | FileCheck %s

; CHECK: <!doctype html>
; CHECK-NEXT: <html>
; CHECK-NEXT: <head>
; CHECK-NEXT:   <title><stdin></title>
; CHECK-NEXT: </head>
; CHECK-NEXT: <body>
; CHECK-NEXT:   <script>
; ModuleID = 'hello'
; CHECK: window.hello = {};

; var
@.str = private constant [12 x i8] c"Hello World\00", align 1 ; <[12 x i8]*> [#uses=1]

define i32 @main() nounwind {
entry:
  %0 = tail call i32 (i8*, ...)* @printf(i8* noalias getelementptr inbounds ([12 x i8]* @.str, i64 0, i64 0)) nounwind ; <i32> [#uses=0]
  ret i32 0
}

declare i32 @printf(i8* nocapture, ...) nounwind
; CHECK-NEXT:   </script>
; CHECK-NEXT: <body>
; CHECK-NEXT: </html>