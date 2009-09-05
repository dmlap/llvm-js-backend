; RUN: opt -S -gvn -enable-load-pre %s | FileCheck %s
;
; Make sure the load in bb3.backedge is removed and moved into bb1 after the 
; call.  This makes the non-call case faster. 
;
; This test is derived from this C++ code (GCC PR 37810):
; void g();
; struct A { 
;   int n; int m;
;   A& operator++(void) { ++n; if (n == m) g(); return *this; }
;   A() : n(0), m(0) { } 
;   friend bool operator!=(A const& a1, A const& a2) { return a1.n != a2.n; }
; };
; void testfunction(A& iter) { A const end; while (iter != end) ++iter; }
;
target datalayout = "e-p:32:32:32-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:32:64-f32:32:32-f64:32:64-v64:64:64-v128:128:128-a0:0:64-f80:128:128"
target triple = "i386-apple-darwin7"
	%struct.A = type { i32, i32 }

define void @_Z12testfunctionR1A(%struct.A* %iter) {
entry:
	%0 = getelementptr %struct.A* %iter, i32 0, i32 0		; <i32*> [#uses=3]
	%1 = load i32* %0, align 4		; <i32> [#uses=2]
	%2 = icmp eq i32 %1, 0		; <i1> [#uses=1]
	br i1 %2, label %return, label %bb.nph

bb.nph:		; preds = %entry
	%3 = getelementptr %struct.A* %iter, i32 0, i32 1		; <i32*> [#uses=1]
	br label %bb

bb:		; preds = %bb3.backedge, %bb.nph
	%.rle = phi i32 [ %1, %bb.nph ], [ %7, %bb3.backedge ]		; <i32> [#uses=1]
	%4 = add i32 %.rle, 1		; <i32> [#uses=2]
	store i32 %4, i32* %0, align 4
	%5 = load i32* %3, align 4		; <i32> [#uses=1]
	%6 = icmp eq i32 %4, %5		; <i1> [#uses=1]
	br i1 %6, label %bb1, label %bb3.backedge

bb1:		; preds = %bb
	tail call void @_Z1gv()
	br label %bb3.backedge

bb3.backedge:		; preds = %bb, %bb1
; CHECK: bb3.backedge:
; CHECK-NEXT: phi
; CHECK-NEXT: icmp
	%7 = load i32* %0, align 4		; <i32> [#uses=2]
	%8 = icmp eq i32 %7, 0		; <i1> [#uses=1]
	br i1 %8, label %return, label %bb

return:		; preds = %bb3.backedge, %entry
	ret void
}

declare void @_Z1gv()
