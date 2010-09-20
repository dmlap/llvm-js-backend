; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/conversion.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

define void @conversion() {
entry:
; CHECK: (257 & 65535);
  %W = trunc i32 257 to i16
; CHECK: (257 & 255);
  %X = trunc i32 257 to i8
; CHECK: (123 & 1);
  %Y = trunc i32 123 to i1
; CHECK: (122 & 1);
  %Z = trunc i32 122 to i1

; CHECK: = (true & 1);
  %A = zext i1 true to i32
; CHECK: = (257 & 65535);
  %B = zext i16 257 to i32
; CHECK: = (-16 & 65535);
  %C = zext i16 -16 to i32
  
; CHECK: = (-1 & 65535);
  %D = sext i8 -1 to i16
; CHECK: = (0 - true);
  %E = sext i1 true to i32

; CHECK: = (123);
  %F = fptrunc double 123.0 to float
; CHECK: = (1e+300);
  %G = fptrunc double 1.0E+300 to float

; CHECK: = 1.5;
  %H = fpext float 1.5 to double

; CHECK: = (123);
  %I = fptoui double 123.0 to i32
; CHECK: = (1e+10 & 1);
  %J = fptoui float 1.0E+10 to i1

; CHECK: = (-123);
  %K = fptosi double -123.0 to i32
; CHECK: = (-1e+10 & 1);
  %L = fptosi float -1.0e+10 to i1

; CHECK: = (257);
  %M = uitofp i32 257 to float
; CHECK: = (-1 & 255);
  %N = uitofp i8 -1 to double

; CHECK: = (257);
  %O = sitofp i32 257 to float
; CHECK: = (-1);
  %P = sitofp i8 -1 to double

; pointer to int conversions??

; CHECK: = -2;
  %Q = bitcast i8 254 to i8
;  %R = bitcast <2 x i16> <i16 1, i16 2> to i32

  ret void
}