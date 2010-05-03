; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -o %t2
; RUN: llc < %s -march=js | FileCheck %s

; CHECK: var factorial;
; CHECK: factorial = function factorial({{.*X}}) {
define i32 @factorial(i32 %X) nounwind readnone {
; CHECK: var {{[_A-z0-9]+, [_A-z0-9]+, [_A-z0-9]+, [_A-z0-9]+, [_A-z0-9]+}};

; CHECK: var _ = '{{.*}}';
; CHECK-NEXT: while(1) {
; CHECK-NEXT:   switch(_) {
; CHECK-NEXT: case '{{.*}}':
entry:
  %0 = icmp eq i32 %X, 0                          ; <i1> [#uses=1]
; CHECK: _ = '{{.*}}';
; CHECK: _ = '{{.*}}';
; CHECK: continue;
  br i1 %0, label %bb2, label %bb1

; CHECK: case '{{.*}}':
bb1:                                              ; preds = %entry
; CHECK: {{[_A-z0-9]+}} = {{[_A-z0-9]+}} + -1;
  %1 = add nsw i32 %X, -1                         ; <i32> [#uses=2]
  %2 = icmp eq i32 %1, 0                          ; <i1> [#uses=1]
; CHECK: _ = '{{.*}}';
; CHECK: _ = '{{.*}}';
; CHECK: continue;
  br i1 %2, label %factorial.exit, label %bb1.i

; CHECK: case '{{.*}}':
bb1.i:                                            ; preds = %bb1
  %3 = add nsw i32 %X, -2                         ; <i32> [#uses=1]
  %4 = tail call i32 @factorial(i32 %3) nounwind  ; <i32> [#uses=1]
  %5 = mul nsw i32 %4, %1                         ; <i32> [#uses=1]
; CHECK: _ = '{{.*}}';
; CHECK: continue;
  br label %factorial.exit

; CHECK: case '{{.*}}':
factorial.exit:                                   ; preds = %bb1.i, %bb1
  %6 = phi i32 [ %5, %bb1.i ], [ 1, %bb1 ]        ; <i32> [#uses=1]
  %7 = mul nsw i32 %6, %X                         ; <i32> [#uses=1]
; CHECK: return
  ret i32 %7

; CHECK: case '{{.*}}':
bb2:                                              ; preds = %entry
; CHECK: return 1;
  ret i32 1
}

; while(1) {
;   switch(_) {
;     case 0: continue;
;     case 1: continue;
;     case 2: return;
;   }
; }