; RUN: llvm-as < %s | llc -march=msp430 | FileCheck %s
target datalayout = "e-p:16:8:8-i8:8:8-i8:8:8-i32:8:8"
target triple = "msp430-generic-generic"
@foo = common global i8 0, align 1

define void @mov() nounwind {
; CHECK: mov:
; CHECK: mov.b	#2, &foo
	store i8 2, i8 * @foo
	ret void
}

define void @add() nounwind {
; CHECK: add:
; CHECK: add.b	#2, &foo
	%1 = load i8* @foo
	%2 = add i8 %1, 2
	store i8 %2, i8 * @foo
	ret void
}

define void @and() nounwind {
; CHECK: and:
; CHECK: and.b	#2, &foo
	%1 = load i8* @foo
	%2 = and i8 %1, 2
	store i8 %2, i8 * @foo
	ret void
}

define void @bis() nounwind {
; CHECK: bis:
; CHECK: bis.b	#2, &foo
	%1 = load i8* @foo
	%2 = or i8 %1, 2
	store i8 %2, i8 * @foo
	ret void
}

define void @xor() nounwind {
; CHECK: xor:
; CHECK: xor.b	#2, &foo
	%1 = load i8* @foo
	%2 = xor i8 %1, 2
	store i8 %2, i8 * @foo
	ret void
}
