; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/aggregates.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

; CHECK: _p("a");
@STR = private constant [2 x i8] c"a\00"


define void @aggregates() {
entry:
; CHECK: = ("hi"[1]);
  %0 = extractvalue [3 x i8] c"hi\00", 1
; CHECK: = ([0, 1][0]);
  %1 = extractvalue [2 x i32] [i32 0, i32 1], 0
; CHECK: = ([7, false][1]);
  %2 = extractvalue { i32, i1 } { i32 7, i1 0 }, 1
; CHECK: = ([null][0]);
  %3 = extractvalue [1 x i32] [i32 undef], 0
; CHECK: = (STR)();
  %4 = getelementptr [2 x i8]* @STR
  %5 = load [2 x i8]* %4
; CHECK = [0]);
  %6 = extractvalue [2 x i8] %5, 0
  ret void
}
