; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/misc.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

@X = constant float 1.5
    
define i32 @test(i32) {
entry:
  ret i32 0;
}

define void @misc() {
entry:
; CHECK: (4 == 5)
  %A = icmp eq i32 4, 5
  %AA = add i1 %A, 1
; CHECK: (X !== X)
  %B = icmp ne float* @X, @X
  %BB = add i1 %B, 1
; CHECK: ((4 & 65535) < (5 & 65535))
  %C = icmp ult i16  4, 5
  %CC = add i1 %C, 1
; CHECK: (4 > 5)
  %D = icmp sgt i16  4, 5
  %DD = add i1 %D, 1
; CHECK: ((-4 & 65535) <= (5 & 65535))
  %E = icmp ule i16 -4, 5
  %EE = add i1 %E, 1
; CHECK: (4 >= 5)
  %F = icmp sge i16  4, 5
  %FF = add i1 %F, 1

; CHECK: (4 == 5)
  %G = fcmp oeq float 4.0, 5.0    ; yields: result=false
  %GG = add i1 %G, 1
; CHECK: (4 != 5 && 4 == 4 && 5 == 5)
  %H = fcmp one float 4.0, 5.0    ; yields: result=true
  %HH = add i1 %H, 1
; CHECK: (4 < 5)
  %I = fcmp olt float 4.0, 5.0    ; yields: result=true
  %II = add i1 %I, 1
; CHECK: (1 == 2 || 1 != 1 || 2 != 2)
  %J = fcmp ueq double 1.0, 2.0   ; yields: result=false
  %JJ = add i1 %J, 1
; CHECK: (0)
  %K = fcmp false double 1.0, 1.0
  %KK = add i1 %K, 1
; CHECK: (1 == 1 && 1 == 1)
  %L = fcmp ord double 1.0, 1.0
  %LL = add i1 %L, 1
  br label %bb0

bb0:
; CHECK: = false;
; CHECK: = true;
  %N = phi i1 [0, %entry], [1, %bb0]
  br i1 %N, label %bb1, label %bb0

bb1:
; CHECK: ((true) ? (17) : (42));
  %O = select i1 true, i8 17, i8 42

  %P = add i32 5, 5
; CHECK: test(
  %Q = tail call fastcc i32 @test(i32 %P)
  ret void
}
