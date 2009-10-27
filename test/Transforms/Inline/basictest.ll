; RUN: opt < %s -inline -scalarrepl -S | FileCheck %s

define i32 @test1f(i32 %i) {
        ret i32 %i
}

define i32 @test1(i32 %W) {
        %X = call i32 @test1f(i32 7)
        %Y = add i32 %X, %W
        ret i32 %Y
; CHECK: @test1(
; CHECK-NEXT: %Y = add i32 7, %W
; CHECK-NEXT: ret i32 %Y
}



; rdar://7339069

%T = type { i32, i32 }

; CHECK-NOT: @test2f
define internal %T* @test2f(i1 %cond, %T* %P) {
  br i1 %cond, label %T, label %F
  
T:
  %A = getelementptr %T* %P, i32 0, i32 0
  store i32 42, i32* %A
  ret %T* %P
  
F:
  ret %T* %P
}

define i32 @test2(i1 %cond) {
  %A = alloca %T
  
  %B = call %T* @test2f(i1 %cond, %T* %A)
  %C = getelementptr %T* %B, i32 0, i32 0
  %D = load i32* %C
  ret i32 %D
  
; CHECK: @test2(
; CHECK-NOT: = alloca
; CHECK: ret i32 42
}
