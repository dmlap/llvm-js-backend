; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o %t2.html
; RUN: llc < %s -march=js -O0 | FileCheck %s

define i32 @main(i32 %argc, i8** nocapture %argv) nounwind {
entry:
; CHECK: if (1) {
; CHECK: _ =
; CHECK: continue;
; CHECK: } else {
; CHECK: _ =
; CHECK: continue;
; CHECK: }
  br i1 1, label %bb_1, label %pit

; CHECK-NOT: do {

bb_1:
; CHECK: case '{{[$_A-z0-9]+}}':
; CHECK-NEXT: _ =
; CHECK-NEXT: continue;
  br label %bb_2
bb_2:
; CHECK: switch (2) {
; CHECK: case 0:
; CHECK: case 1:
; CHECK: case 2:
  switch i32 2, label %pit [ i32 0, label %pit
                             i32 1, label %bb_1
			     i32 2, label %bb_3 ]
bb_3:
; CHECK: return 1;
  ret i32 1
pit:
; CHECK: throw
  unreachable
}


define void @retVoid() nounwind {
entry:
; CHECK: return;
  ret void
}

define void @unwind() {
entry:
; CHECK: throw
  unwind
}