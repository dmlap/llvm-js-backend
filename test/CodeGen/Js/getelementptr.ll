; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -o %t2
; RUN: llc < %s -march=js | FileCheck %s

%RT = type { i8 , [10 x [20 x i32]], i8  }
%ST = type { i32, double, %RT }

define i32* @getElementPtrTest(%ST* %s) {
entry:
; CHECK: ({{[_$A-z0-9]+}}[1][2][1][5][13])
  %reg = getelementptr %ST* %s, i32 1, i32 2, i32 1, i32 5, i32 13
  ret i32* %reg
}