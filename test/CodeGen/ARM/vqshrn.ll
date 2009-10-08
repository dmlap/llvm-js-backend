; RUN: llc < %s -march=arm -mattr=+neon | FileCheck %s

define <8 x i8> @vqshrns8(<8 x i16>* %A) nounwind {
;CHECK: vqshrns8:
;CHECK: vqshrn.s16
	%tmp1 = load <8 x i16>* %A
	%tmp2 = call <8 x i8> @llvm.arm.neon.vqshiftns.v8i8(<8 x i16> %tmp1, <8 x i16> < i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8 >)
	ret <8 x i8> %tmp2
}

define <4 x i16> @vqshrns16(<4 x i32>* %A) nounwind {
;CHECK: vqshrns16:
;CHECK: vqshrn.s32
	%tmp1 = load <4 x i32>* %A
	%tmp2 = call <4 x i16> @llvm.arm.neon.vqshiftns.v4i16(<4 x i32> %tmp1, <4 x i32> < i32 -16, i32 -16, i32 -16, i32 -16 >)
	ret <4 x i16> %tmp2
}

define <2 x i32> @vqshrns32(<2 x i64>* %A) nounwind {
;CHECK: vqshrns32:
;CHECK: vqshrn.s64
	%tmp1 = load <2 x i64>* %A
	%tmp2 = call <2 x i32> @llvm.arm.neon.vqshiftns.v2i32(<2 x i64> %tmp1, <2 x i64> < i64 -32, i64 -32 >)
	ret <2 x i32> %tmp2
}

define <8 x i8> @vqshrnu8(<8 x i16>* %A) nounwind {
;CHECK: vqshrnu8:
;CHECK: vqshrn.u16
	%tmp1 = load <8 x i16>* %A
	%tmp2 = call <8 x i8> @llvm.arm.neon.vqshiftnu.v8i8(<8 x i16> %tmp1, <8 x i16> < i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8 >)
	ret <8 x i8> %tmp2
}

define <4 x i16> @vqshrnu16(<4 x i32>* %A) nounwind {
;CHECK: vqshrnu16:
;CHECK: vqshrn.u32
	%tmp1 = load <4 x i32>* %A
	%tmp2 = call <4 x i16> @llvm.arm.neon.vqshiftnu.v4i16(<4 x i32> %tmp1, <4 x i32> < i32 -16, i32 -16, i32 -16, i32 -16 >)
	ret <4 x i16> %tmp2
}

define <2 x i32> @vqshrnu32(<2 x i64>* %A) nounwind {
;CHECK: vqshrnu32:
;CHECK: vqshrn.u64
	%tmp1 = load <2 x i64>* %A
	%tmp2 = call <2 x i32> @llvm.arm.neon.vqshiftnu.v2i32(<2 x i64> %tmp1, <2 x i64> < i64 -32, i64 -32 >)
	ret <2 x i32> %tmp2
}

define <8 x i8> @vqshruns8(<8 x i16>* %A) nounwind {
;CHECK: vqshruns8:
;CHECK: vqshrun.s16
	%tmp1 = load <8 x i16>* %A
	%tmp2 = call <8 x i8> @llvm.arm.neon.vqshiftnsu.v8i8(<8 x i16> %tmp1, <8 x i16> < i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8, i16 -8 >)
	ret <8 x i8> %tmp2
}

define <4 x i16> @vqshruns16(<4 x i32>* %A) nounwind {
;CHECK: vqshruns16:
;CHECK: vqshrun.s32
	%tmp1 = load <4 x i32>* %A
	%tmp2 = call <4 x i16> @llvm.arm.neon.vqshiftnsu.v4i16(<4 x i32> %tmp1, <4 x i32> < i32 -16, i32 -16, i32 -16, i32 -16 >)
	ret <4 x i16> %tmp2
}

define <2 x i32> @vqshruns32(<2 x i64>* %A) nounwind {
;CHECK: vqshruns32:
;CHECK: vqshrun.s64
	%tmp1 = load <2 x i64>* %A
	%tmp2 = call <2 x i32> @llvm.arm.neon.vqshiftnsu.v2i32(<2 x i64> %tmp1, <2 x i64> < i64 -32, i64 -32 >)
	ret <2 x i32> %tmp2
}

declare <8 x i8>  @llvm.arm.neon.vqshiftns.v8i8(<8 x i16>, <8 x i16>) nounwind readnone
declare <4 x i16> @llvm.arm.neon.vqshiftns.v4i16(<4 x i32>, <4 x i32>) nounwind readnone
declare <2 x i32> @llvm.arm.neon.vqshiftns.v2i32(<2 x i64>, <2 x i64>) nounwind readnone

declare <8 x i8>  @llvm.arm.neon.vqshiftnu.v8i8(<8 x i16>, <8 x i16>) nounwind readnone
declare <4 x i16> @llvm.arm.neon.vqshiftnu.v4i16(<4 x i32>, <4 x i32>) nounwind readnone
declare <2 x i32> @llvm.arm.neon.vqshiftnu.v2i32(<2 x i64>, <2 x i64>) nounwind readnone

declare <8 x i8>  @llvm.arm.neon.vqshiftnsu.v8i8(<8 x i16>, <8 x i16>) nounwind readnone
declare <4 x i16> @llvm.arm.neon.vqshiftnsu.v4i16(<4 x i32>, <4 x i32>) nounwind readnone
declare <2 x i32> @llvm.arm.neon.vqshiftnsu.v2i32(<2 x i64>, <2 x i64>) nounwind readnone
