; RUN: llc < %s -march=arm                  | FileCheck %s --check-prefix=ARM
; RUN: llc < %s -march=thumb -mattr=+thumb2 | FileCheck %s --check-prefix=T2

define i32 @t1(i32 %c) nounwind readnone {
entry:
; ARM: t1:
; ARM: mov r1, #101
; ARM: orr r1, r1, #1, 24
; ARM: movgt r0, #123

; T2: t1:
; T2: movw r0, #357
; T2: movgt r0, #123

  %0 = icmp sgt i32 %c, 1
  %1 = select i1 %0, i32 123, i32 357
  ret i32 %1
}

define i32 @t2(i32 %c) nounwind readnone {
entry:
; ARM: t2:
; ARM: mov r1, #101
; ARM: orr r1, r1, #1, 24
; ARM: movle r0, #123

; T2: t2:
; T2: mov.w r0, #123
; T2: movwgt r0, #357

  %0 = icmp sgt i32 %c, 1
  %1 = select i1 %0, i32 357, i32 123
  ret i32 %1
}

define i32 @t3(i32 %a) nounwind readnone {
entry:
; ARM: t3:
; ARM: mov r0, #0
; ARM: moveq r0, #1

; T2: t3:
; T2: mov.w r0, #0
; T2: moveq r0, #1
  %0 = icmp eq i32 %a, 160
  %1 = zext i1 %0 to i32
  ret i32 %1
}

define i32 @t4(i32 %a, i32 %b, i32 %x) nounwind {
entry:
; ARM: t4:
; ARM: ldr
; ARM: movlt

; T2: t4:
; T2: mvnlt.w r0, #11141290
  %0 = icmp slt i32 %a, %b
  %1 = select i1 %0, i32 4283826005, i32 %x
  ret i32 %1
}
