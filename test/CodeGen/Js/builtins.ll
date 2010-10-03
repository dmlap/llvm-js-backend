; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/builtins.html
; RUN: llc < %s -march=js | FileCheck %s

define i32 @main(i32 %argc, i8** %argv) {
entry:
; CHECK: ();
  %0 = call i8* @llvm.stacksave()
  call void @llvm.stackrestore(i8* %0)
  ret i32 0
}

declare i8* @llvm.stacksave() nounwind

declare void @llvm.stackrestore(i8*)