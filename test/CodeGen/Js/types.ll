; RUN: llvm-as < %s | llvm-dis > %t1
; RUN: llc < %s -march=js -O0 -o Output/types.js
; RUN: llc < %s -march=js -O0 | FileCheck %s

; CHECK: var pi = _p(7)
@pi = private constant i32 7
; CHECK: , lpf = _p(1.5)
@lpf = linker_private global float 1.5
; CHECK: , id = _p(1.75)
@id = internal constant double 1.75
; CHECK: , ps = _p("ab")
@ps = private constant [3 x i8] c"ab\00"
; CHECK: ;

; CHECK: _.cps = _p([ false, false ]);
@cps = common global < { i1, i1 } > zeroinitializer
; CHECK: _.gv = [
; CHECK: ,
; CHECK: ];
@gv = global < 2 x double >  < double 2.0, double 2.2 >
; CHECK: _.gfp = gf
declare extern_weak i8* @gf(i8*)
@gfp = global i8* (i8*)* @gf
; CHECK: _.aep = null;
@aep = available_externally global i8* null
; CHECK: _.los = [1, 2, 3];
@los = linkonce constant { i32, i32, i32 } { i32 1, i32 2, i32 3 }
; CHECK: _.wg = 3;
@wg = weak global i32 3
; CHECK: _.av = [false, true, false];
@av = appending global [3 x i1] [i1 0, i1 1, i1 0]
; CHECK: _.looi = 2;
@looi = linkonce_odr global i8 2
; CHECK: _.wof =
; CHECK: ;
@wof = weak_odr global float 1.0
; CHECK: _.evd =
; CHECK: ;
@evd = global double 16.1
