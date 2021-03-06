; RUN: llc < %s -mtriple=arm-apple-darwin | FileCheck %s

@X = external global [0 x i32]          ; <[0 x i32]*> [#uses=5]

define i32 @t1() {
; CHECK: t1:
; CHECK: ldmia
        %tmp = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 0)            ; <i32> [#uses=1]
        %tmp3 = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 1)           ; <i32> [#uses=1]
        %tmp4 = tail call i32 @f1( i32 %tmp, i32 %tmp3 )                ; <i32> [#uses=1]
        ret i32 %tmp4
}

define i32 @t2() {
; CHECK: t2:
; CHECK: ldmia
        %tmp = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 2)            ; <i32> [#uses=1]
        %tmp3 = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 3)           ; <i32> [#uses=1]
        %tmp5 = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 4)           ; <i32> [#uses=1]
        %tmp6 = tail call i32 @f2( i32 %tmp, i32 %tmp3, i32 %tmp5 )             ; <i32> [#uses=1]
        ret i32 %tmp6
}

define i32 @t3() {
; CHECK: t3:
; CHECK: ldmib
; CHECK: ldmia sp!
        %tmp = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 1)            ; <i32> [#uses=1]
        %tmp3 = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 2)           ; <i32> [#uses=1]
        %tmp5 = load i32* getelementptr ([0 x i32]* @X, i32 0, i32 3)           ; <i32> [#uses=1]
        %tmp6 = call i32 @f2( i32 %tmp, i32 %tmp3, i32 %tmp5 )             ; <i32> [#uses=1]
        ret i32 %tmp6
}

declare i32 @f1(i32, i32)

declare i32 @f2(i32, i32, i32)
