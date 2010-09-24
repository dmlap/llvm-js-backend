; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o getelementptr.html
; RUN: llc < %s -march=js | FileCheck %s

%RT = type { i8 , [4 x [3 x i32]], i8  }
%ST = type { i32, double, %RT }

@S = global %ST { i32 1, double 1.5, %RT {i8 4, [4 x [3 x i32]][[3 x i32][i32 0, i32 1, i32 2], [3 x i32][i32 3, i32 4, i32 5], [3 x i32][i32 6, i32 7, i32 8], [3 x i32][i32 9, i32 10, i32 11]], i8 2}}

define i32* @getElementPtrTest(%ST* %s) {
entry:
; CHECK: (_a({{[_$A-z0-9]+}}(0)[2][1][3],2));
  %reg = getelementptr %ST* %s, i32 0, i32 2, i32 1, i32 3, i32 2
  ret i32* %reg
}
define i32 @main() nounwind {
entry:
  %0 = getelementptr %ST* @S
  %1 = call i32* @getElementPtrTest(%ST* %0)
  %2 = load i32*  %1
  ret i32 %2
}