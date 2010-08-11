; RUN: llc < %s -mtriple=x86_64-apple-darwin -march=x86 -mcpu=corei7 -mattr=avx | FileCheck %s

define <2 x i64> @test_x86_aesni_aesdec(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vaesdec
  %res = call <2 x i64> @llvm.x86.aesni.aesdec(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aesdec(<2 x i64>, <2 x i64>) nounwind readnone


define <2 x i64> @test_x86_aesni_aesdeclast(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vaesdeclast
  %res = call <2 x i64> @llvm.x86.aesni.aesdeclast(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aesdeclast(<2 x i64>, <2 x i64>) nounwind readnone


define <2 x i64> @test_x86_aesni_aesenc(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vaesenc
  %res = call <2 x i64> @llvm.x86.aesni.aesenc(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aesenc(<2 x i64>, <2 x i64>) nounwind readnone


define <2 x i64> @test_x86_aesni_aesenclast(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vaesenclast
  %res = call <2 x i64> @llvm.x86.aesni.aesenclast(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aesenclast(<2 x i64>, <2 x i64>) nounwind readnone


define <2 x i64> @test_x86_aesni_aesimc(<2 x i64> %a0) {
  ; CHECK: vaesimc
  %res = call <2 x i64> @llvm.x86.aesni.aesimc(<2 x i64> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aesimc(<2 x i64>) nounwind readnone


define <2 x i64> @test_x86_aesni_aeskeygenassist(<2 x i64> %a0) {
  ; CHECK: vaeskeygenassist
  %res = call <2 x i64> @llvm.x86.aesni.aeskeygenassist(<2 x i64> %a0, i8 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.aesni.aeskeygenassist(<2 x i64>, i8) nounwind readnone


define <2 x double> @test_x86_sse2_add_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vaddsd
  %res = call <2 x double> @llvm.x86.sse2.add.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.add.sd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_cmp_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcmpordpd
  %res = call <2 x double> @llvm.x86.sse2.cmp.pd(<2 x double> %a0, <2 x double> %a1, i8 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cmp.pd(<2 x double>, <2 x double>, i8) nounwind readnone


define <2 x double> @test_x86_sse2_cmp_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcmpordsd
  %res = call <2 x double> @llvm.x86.sse2.cmp.sd(<2 x double> %a0, <2 x double> %a1, i8 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cmp.sd(<2 x double>, <2 x double>, i8) nounwind readnone


define i32 @test_x86_sse2_comieq_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: sete
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comieq.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comieq.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_comige_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: setae
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comige.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comige.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_comigt_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: seta
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comigt.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comigt.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_comile_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: setbe
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comile.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comile.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_comilt_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: setb
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comilt.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comilt.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_comineq_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vcomisd
  ; CHECK: setne
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.comineq.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.comineq.sd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_cvtdq2pd(<4 x i32> %a0) {
  ; CHECK: vcvtdq2pd
  %res = call <2 x double> @llvm.x86.sse2.cvtdq2pd(<4 x i32> %a0) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cvtdq2pd(<4 x i32>) nounwind readnone


define <4 x float> @test_x86_sse2_cvtdq2ps(<4 x i32> %a0) {
  ; CHECK: vcvtdq2ps
  %res = call <4 x float> @llvm.x86.sse2.cvtdq2ps(<4 x i32> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse2.cvtdq2ps(<4 x i32>) nounwind readnone


define <4 x i32> @test_x86_sse2_cvtpd2dq(<2 x double> %a0) {
  ; CHECK: vcvtpd2dq
  %res = call <4 x i32> @llvm.x86.sse2.cvtpd2dq(<2 x double> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.cvtpd2dq(<2 x double>) nounwind readnone


define <4 x float> @test_x86_sse2_cvtpd2ps(<2 x double> %a0) {
  ; CHECK: vcvtpd2ps
  %res = call <4 x float> @llvm.x86.sse2.cvtpd2ps(<2 x double> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse2.cvtpd2ps(<2 x double>) nounwind readnone


define <4 x i32> @test_x86_sse2_cvtps2dq(<4 x float> %a0) {
  ; CHECK: vcvtps2dq
  %res = call <4 x i32> @llvm.x86.sse2.cvtps2dq(<4 x float> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.cvtps2dq(<4 x float>) nounwind readnone


define <2 x double> @test_x86_sse2_cvtps2pd(<4 x float> %a0) {
  ; CHECK: vcvtps2pd
  %res = call <2 x double> @llvm.x86.sse2.cvtps2pd(<4 x float> %a0) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cvtps2pd(<4 x float>) nounwind readnone


define i32 @test_x86_sse2_cvtsd2si(<2 x double> %a0) {
  ; CHECK: vcvtsd2si
  %res = call i32 @llvm.x86.sse2.cvtsd2si(<2 x double> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.cvtsd2si(<2 x double>) nounwind readnone


define <4 x float> @test_x86_sse2_cvtsd2ss(<4 x float> %a0, <2 x double> %a1) {
  ; CHECK: vcvtsd2ss
  %res = call <4 x float> @llvm.x86.sse2.cvtsd2ss(<4 x float> %a0, <2 x double> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse2.cvtsd2ss(<4 x float>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_cvtsi2sd(<2 x double> %a0) {
  ; CHECK: movl
  ; CHECK: vcvtsi2sd
  %res = call <2 x double> @llvm.x86.sse2.cvtsi2sd(<2 x double> %a0, i32 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cvtsi2sd(<2 x double>, i32) nounwind readnone


define <2 x double> @test_x86_sse2_cvtss2sd(<2 x double> %a0, <4 x float> %a1) {
  ; CHECK: vcvtss2sd
  %res = call <2 x double> @llvm.x86.sse2.cvtss2sd(<2 x double> %a0, <4 x float> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.cvtss2sd(<2 x double>, <4 x float>) nounwind readnone


define <4 x i32> @test_x86_sse2_cvttpd2dq(<2 x double> %a0) {
  ; CHECK: vcvttpd2dq
  %res = call <4 x i32> @llvm.x86.sse2.cvttpd2dq(<2 x double> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.cvttpd2dq(<2 x double>) nounwind readnone


define <4 x i32> @test_x86_sse2_cvttps2dq(<4 x float> %a0) {
  ; CHECK: vcvttps2dq
  %res = call <4 x i32> @llvm.x86.sse2.cvttps2dq(<4 x float> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.cvttps2dq(<4 x float>) nounwind readnone


define i32 @test_x86_sse2_cvttsd2si(<2 x double> %a0) {
  ; CHECK: vcvttss2si
  %res = call i32 @llvm.x86.sse2.cvttsd2si(<2 x double> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.cvttsd2si(<2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_div_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vdivsd
  %res = call <2 x double> @llvm.x86.sse2.div.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.div.sd(<2 x double>, <2 x double>) nounwind readnone


define <16 x i8> @test_x86_sse2_loadu_dq(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vmovdqu
  %res = call <16 x i8> @llvm.x86.sse2.loadu.dq(i8* %a0) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.loadu.dq(i8*) nounwind readonly


define <2 x double> @test_x86_sse2_loadu_pd(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vmovupd
  %res = call <2 x double> @llvm.x86.sse2.loadu.pd(i8* %a0) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.loadu.pd(i8*) nounwind readonly


define void @test_x86_sse2_maskmov_dqu(<16 x i8> %a0, <16 x i8> %a1, i8* %a2) {
  ; CHECK: pushl
  ; CHECK: movl
  ; CHECK: vmaskmovdqu
  ; CHECK: popl
  call void @llvm.x86.sse2.maskmov.dqu(<16 x i8> %a0, <16 x i8> %a1, i8* %a2)
  ret void
}
declare void @llvm.x86.sse2.maskmov.dqu(<16 x i8>, <16 x i8>, i8*) nounwind


define <2 x double> @test_x86_sse2_max_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vmaxpd
  %res = call <2 x double> @llvm.x86.sse2.max.pd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.max.pd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_max_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vmaxsd
  %res = call <2 x double> @llvm.x86.sse2.max.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.max.sd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_min_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vminpd
  %res = call <2 x double> @llvm.x86.sse2.min.pd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.min.pd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_min_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vminsd
  %res = call <2 x double> @llvm.x86.sse2.min.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.min.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_movmsk_pd(<2 x double> %a0) {
  ; CHECK: vmovmskpd
  %res = call i32 @llvm.x86.sse2.movmsk.pd(<2 x double> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.movmsk.pd(<2 x double>) nounwind readnone


define void @test_x86_sse2_movnt_dq(i8* %a0, <2 x i64> %a1) {
  ; CHECK: movl
  ; CHECK: vmovntdq
  call void @llvm.x86.sse2.movnt.dq(i8* %a0, <2 x i64> %a1)
  ret void
}
declare void @llvm.x86.sse2.movnt.dq(i8*, <2 x i64>) nounwind


define void @test_x86_sse2_movnt_pd(i8* %a0, <2 x double> %a1) {
  ; CHECK: movl
  ; CHECK: vmovntpd
  call void @llvm.x86.sse2.movnt.pd(i8* %a0, <2 x double> %a1)
  ret void
}
declare void @llvm.x86.sse2.movnt.pd(i8*, <2 x double>) nounwind


define <2 x double> @test_x86_sse2_mul_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vmulsd
  %res = call <2 x double> @llvm.x86.sse2.mul.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.mul.sd(<2 x double>, <2 x double>) nounwind readnone


define <8 x i16> @test_x86_sse2_packssdw_128(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpackssdw
  %res = call <8 x i16> @llvm.x86.sse2.packssdw.128(<4 x i32> %a0, <4 x i32> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.packssdw.128(<4 x i32>, <4 x i32>) nounwind readnone


define <16 x i8> @test_x86_sse2_packsswb_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpacksswb
  %res = call <16 x i8> @llvm.x86.sse2.packsswb.128(<8 x i16> %a0, <8 x i16> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.packsswb.128(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_packuswb_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpackuswb
  %res = call <16 x i8> @llvm.x86.sse2.packuswb.128(<8 x i16> %a0, <8 x i16> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.packuswb.128(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_padds_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpaddsb
  %res = call <16 x i8> @llvm.x86.sse2.padds.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.padds.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_padds_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpaddsw
  %res = call <8 x i16> @llvm.x86.sse2.padds.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.padds.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_paddus_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpaddusb
  %res = call <16 x i8> @llvm.x86.sse2.paddus.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.paddus.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_paddus_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpaddusw
  %res = call <8 x i16> @llvm.x86.sse2.paddus.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.paddus.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_pavg_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpavgb
  %res = call <16 x i8> @llvm.x86.sse2.pavg.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.pavg.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_pavg_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpavgw
  %res = call <8 x i16> @llvm.x86.sse2.pavg.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pavg.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_pcmpeq_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpeqb
  %res = call <16 x i8> @llvm.x86.sse2.pcmpeq.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.pcmpeq.b(<16 x i8>, <16 x i8>) nounwind readnone


define <4 x i32> @test_x86_sse2_pcmpeq_d(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpcmpeqd
  %res = call <4 x i32> @llvm.x86.sse2.pcmpeq.d(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.pcmpeq.d(<4 x i32>, <4 x i32>) nounwind readnone


define <8 x i16> @test_x86_sse2_pcmpeq_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpcmpeqw
  %res = call <8 x i16> @llvm.x86.sse2.pcmpeq.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pcmpeq.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_pcmpgt_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpgtb
  %res = call <16 x i8> @llvm.x86.sse2.pcmpgt.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.pcmpgt.b(<16 x i8>, <16 x i8>) nounwind readnone


define <4 x i32> @test_x86_sse2_pcmpgt_d(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpcmpgtd
  %res = call <4 x i32> @llvm.x86.sse2.pcmpgt.d(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.pcmpgt.d(<4 x i32>, <4 x i32>) nounwind readnone


define <8 x i16> @test_x86_sse2_pcmpgt_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpcmpgtw
  %res = call <8 x i16> @llvm.x86.sse2.pcmpgt.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pcmpgt.w(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse2_pmadd_wd(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmaddwd
  %res = call <4 x i32> @llvm.x86.sse2.pmadd.wd(<8 x i16> %a0, <8 x i16> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.pmadd.wd(<8 x i16>, <8 x i16>) nounwind readnone


define <8 x i16> @test_x86_sse2_pmaxs_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmaxsw
  %res = call <8 x i16> @llvm.x86.sse2.pmaxs.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pmaxs.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_pmaxu_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpmaxub
  %res = call <16 x i8> @llvm.x86.sse2.pmaxu.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.pmaxu.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_pmins_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpminsw
  %res = call <8 x i16> @llvm.x86.sse2.pmins.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pmins.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_pminu_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpminub
  %res = call <16 x i8> @llvm.x86.sse2.pminu.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.pminu.b(<16 x i8>, <16 x i8>) nounwind readnone


define i32 @test_x86_sse2_pmovmskb_128(<16 x i8> %a0) {
  ; CHECK: vpmovmskb
  %res = call i32 @llvm.x86.sse2.pmovmskb.128(<16 x i8> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.pmovmskb.128(<16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_pmulh_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmulhw
  %res = call <8 x i16> @llvm.x86.sse2.pmulh.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pmulh.w(<8 x i16>, <8 x i16>) nounwind readnone


define <8 x i16> @test_x86_sse2_pmulhu_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmulhuw
  %res = call <8 x i16> @llvm.x86.sse2.pmulhu.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pmulhu.w(<8 x i16>, <8 x i16>) nounwind readnone


define <2 x i64> @test_x86_sse2_pmulu_dq(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpmuludq
  %res = call <2 x i64> @llvm.x86.sse2.pmulu.dq(<4 x i32> %a0, <4 x i32> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.pmulu.dq(<4 x i32>, <4 x i32>) nounwind readnone


define <2 x i64> @test_x86_sse2_psad_bw(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpsadbw
  %res = call <2 x i64> @llvm.x86.sse2.psad.bw(<16 x i8> %a0, <16 x i8> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psad.bw(<16 x i8>, <16 x i8>) nounwind readnone


define <4 x i32> @test_x86_sse2_psll_d(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpslld
  %res = call <4 x i32> @llvm.x86.sse2.psll.d(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.psll.d(<4 x i32>, <4 x i32>) nounwind readnone


define <2 x i64> @test_x86_sse2_psll_dq(<2 x i64> %a0) {
  ; CHECK: vpslldq
  %res = call <2 x i64> @llvm.x86.sse2.psll.dq(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psll.dq(<2 x i64>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_psll_dq_bs(<2 x i64> %a0) {
  ; CHECK: vpslldq
  %res = call <2 x i64> @llvm.x86.sse2.psll.dq.bs(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psll.dq.bs(<2 x i64>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_psll_q(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vpsllq
  %res = call <2 x i64> @llvm.x86.sse2.psll.q(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psll.q(<2 x i64>, <2 x i64>) nounwind readnone


define <8 x i16> @test_x86_sse2_psll_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsllw
  %res = call <8 x i16> @llvm.x86.sse2.psll.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psll.w(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse2_pslli_d(<4 x i32> %a0) {
  ; CHECK: vpslld
  %res = call <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32> %a0, i32 7) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.pslli.d(<4 x i32>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_pslli_q(<2 x i64> %a0) {
  ; CHECK: vpsllq
  %res = call <2 x i64> @llvm.x86.sse2.pslli.q(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.pslli.q(<2 x i64>, i32) nounwind readnone


define <8 x i16> @test_x86_sse2_pslli_w(<8 x i16> %a0) {
  ; CHECK: vpsllw
  %res = call <8 x i16> @llvm.x86.sse2.pslli.w(<8 x i16> %a0, i32 7) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.pslli.w(<8 x i16>, i32) nounwind readnone


define <4 x i32> @test_x86_sse2_psra_d(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpsrad
  %res = call <4 x i32> @llvm.x86.sse2.psra.d(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.psra.d(<4 x i32>, <4 x i32>) nounwind readnone


define <8 x i16> @test_x86_sse2_psra_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsraw
  %res = call <8 x i16> @llvm.x86.sse2.psra.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psra.w(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse2_psrai_d(<4 x i32> %a0) {
  ; CHECK: vpsrad
  %res = call <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32> %a0, i32 7) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.psrai.d(<4 x i32>, i32) nounwind readnone


define <8 x i16> @test_x86_sse2_psrai_w(<8 x i16> %a0) {
  ; CHECK: vpsraw
  %res = call <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16> %a0, i32 7) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psrai.w(<8 x i16>, i32) nounwind readnone


define <4 x i32> @test_x86_sse2_psrl_d(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpsrld
  %res = call <4 x i32> @llvm.x86.sse2.psrl.d(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.psrl.d(<4 x i32>, <4 x i32>) nounwind readnone


define <2 x i64> @test_x86_sse2_psrl_dq(<2 x i64> %a0) {
  ; CHECK: vpsrldq
  %res = call <2 x i64> @llvm.x86.sse2.psrl.dq(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psrl.dq(<2 x i64>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_psrl_dq_bs(<2 x i64> %a0) {
  ; CHECK: vpsrldq
  %res = call <2 x i64> @llvm.x86.sse2.psrl.dq.bs(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psrl.dq.bs(<2 x i64>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_psrl_q(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vpsrlq
  %res = call <2 x i64> @llvm.x86.sse2.psrl.q(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psrl.q(<2 x i64>, <2 x i64>) nounwind readnone


define <8 x i16> @test_x86_sse2_psrl_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsrlw
  %res = call <8 x i16> @llvm.x86.sse2.psrl.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psrl.w(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse2_psrli_d(<4 x i32> %a0) {
  ; CHECK: vpsrld
  %res = call <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32> %a0, i32 7) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse2.psrli.d(<4 x i32>, i32) nounwind readnone


define <2 x i64> @test_x86_sse2_psrli_q(<2 x i64> %a0) {
  ; CHECK: vpsrlq
  %res = call <2 x i64> @llvm.x86.sse2.psrli.q(<2 x i64> %a0, i32 7) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse2.psrli.q(<2 x i64>, i32) nounwind readnone


define <8 x i16> @test_x86_sse2_psrli_w(<8 x i16> %a0) {
  ; CHECK: vpsrlw
  %res = call <8 x i16> @llvm.x86.sse2.psrli.w(<8 x i16> %a0, i32 7) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psrli.w(<8 x i16>, i32) nounwind readnone


define <16 x i8> @test_x86_sse2_psubs_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpsubsb
  %res = call <16 x i8> @llvm.x86.sse2.psubs.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.psubs.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_psubs_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsubsw
  %res = call <8 x i16> @llvm.x86.sse2.psubs.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psubs.w(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse2_psubus_b(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpsubusb
  %res = call <16 x i8> @llvm.x86.sse2.psubus.b(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse2.psubus.b(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse2_psubus_w(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsubusw
  %res = call <8 x i16> @llvm.x86.sse2.psubus.w(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse2.psubus.w(<8 x i16>, <8 x i16>) nounwind readnone


define <2 x double> @test_x86_sse2_sqrt_pd(<2 x double> %a0) {
  ; CHECK: vsqrtpd
  %res = call <2 x double> @llvm.x86.sse2.sqrt.pd(<2 x double> %a0) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.sqrt.pd(<2 x double>) nounwind readnone


define <2 x double> @test_x86_sse2_sqrt_sd(<2 x double> %a0) {
  ; CHECK: vsqrtsd
  %res = call <2 x double> @llvm.x86.sse2.sqrt.sd(<2 x double> %a0) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.sqrt.sd(<2 x double>) nounwind readnone


define void @test_x86_sse2_storel_dq(i8* %a0, <4 x i32> %a1) {
  ; CHECK: movl
  ; CHECK: vmovq
  call void @llvm.x86.sse2.storel.dq(i8* %a0, <4 x i32> %a1)
  ret void
}
declare void @llvm.x86.sse2.storel.dq(i8*, <4 x i32>) nounwind


define void @test_x86_sse2_storeu_dq(i8* %a0, <16 x i8> %a1) {
  ; CHECK: movl
  ; CHECK: vmovdqu
  call void @llvm.x86.sse2.storeu.dq(i8* %a0, <16 x i8> %a1)
  ret void
}
declare void @llvm.x86.sse2.storeu.dq(i8*, <16 x i8>) nounwind


define void @test_x86_sse2_storeu_pd(i8* %a0, <2 x double> %a1) {
  ; CHECK: movl
  ; CHECK: vmovupd
  call void @llvm.x86.sse2.storeu.pd(i8* %a0, <2 x double> %a1)
  ret void
}
declare void @llvm.x86.sse2.storeu.pd(i8*, <2 x double>) nounwind


define <2 x double> @test_x86_sse2_sub_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vsubsd
  %res = call <2 x double> @llvm.x86.sse2.sub.sd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse2.sub.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomieq_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: sete
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomieq.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomieq.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomige_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: setae
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomige.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomige.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomigt_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: seta
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomigt.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomigt.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomile_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: setbe
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomile.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomile.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomilt_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: setb
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomilt.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomilt.sd(<2 x double>, <2 x double>) nounwind readnone


define i32 @test_x86_sse2_ucomineq_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vucomisd
  ; CHECK: setne
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse2.ucomineq.sd(<2 x double> %a0, <2 x double> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse2.ucomineq.sd(<2 x double>, <2 x double>) nounwind readnone


define <2 x double> @test_x86_sse3_addsub_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vaddsubpd
  %res = call <2 x double> @llvm.x86.sse3.addsub.pd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse3.addsub.pd(<2 x double>, <2 x double>) nounwind readnone


define <4 x float> @test_x86_sse3_addsub_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vaddsubps
  %res = call <4 x float> @llvm.x86.sse3.addsub.ps(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse3.addsub.ps(<4 x float>, <4 x float>) nounwind readnone


define <2 x double> @test_x86_sse3_hadd_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vhaddpd
  %res = call <2 x double> @llvm.x86.sse3.hadd.pd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse3.hadd.pd(<2 x double>, <2 x double>) nounwind readnone


define <4 x float> @test_x86_sse3_hadd_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vhaddps
  %res = call <4 x float> @llvm.x86.sse3.hadd.ps(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse3.hadd.ps(<4 x float>, <4 x float>) nounwind readnone


define <2 x double> @test_x86_sse3_hsub_pd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vhsubpd
  %res = call <2 x double> @llvm.x86.sse3.hsub.pd(<2 x double> %a0, <2 x double> %a1) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse3.hsub.pd(<2 x double>, <2 x double>) nounwind readnone


define <4 x float> @test_x86_sse3_hsub_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vhsubps
  %res = call <4 x float> @llvm.x86.sse3.hsub.ps(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse3.hsub.ps(<4 x float>, <4 x float>) nounwind readnone


define <16 x i8> @test_x86_sse3_ldu_dq(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vlddqu
  %res = call <16 x i8> @llvm.x86.sse3.ldu.dq(i8* %a0) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse3.ldu.dq(i8*) nounwind readonly


define <2 x double> @test_x86_sse41_blendpd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vblendpd
  %res = call <2 x double> @llvm.x86.sse41.blendpd(<2 x double> %a0, <2 x double> %a1, i32 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse41.blendpd(<2 x double>, <2 x double>, i32) nounwind readnone


define <4 x float> @test_x86_sse41_blendps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vblendps
  %res = call <4 x float> @llvm.x86.sse41.blendps(<4 x float> %a0, <4 x float> %a1, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.blendps(<4 x float>, <4 x float>, i32) nounwind readnone


define <2 x double> @test_x86_sse41_blendvpd(<2 x double> %a0, <2 x double> %a1, <2 x double> %a2) {
  ; CHECK: vblendvpd
  %res = call <2 x double> @llvm.x86.sse41.blendvpd(<2 x double> %a0, <2 x double> %a1, <2 x double> %a2) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse41.blendvpd(<2 x double>, <2 x double>, <2 x double>) nounwind readnone


define <4 x float> @test_x86_sse41_blendvps(<4 x float> %a0, <4 x float> %a1, <4 x float> %a2) {
  ; CHECK: vblendvps
  %res = call <4 x float> @llvm.x86.sse41.blendvps(<4 x float> %a0, <4 x float> %a1, <4 x float> %a2) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.blendvps(<4 x float>, <4 x float>, <4 x float>) nounwind readnone


define <2 x double> @test_x86_sse41_dppd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vdppd
  %res = call <2 x double> @llvm.x86.sse41.dppd(<2 x double> %a0, <2 x double> %a1, i32 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse41.dppd(<2 x double>, <2 x double>, i32) nounwind readnone


define <4 x float> @test_x86_sse41_dpps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vdpps
  %res = call <4 x float> @llvm.x86.sse41.dpps(<4 x float> %a0, <4 x float> %a1, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.dpps(<4 x float>, <4 x float>, i32) nounwind readnone


define <4 x float> @test_x86_sse41_insertps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vinsertps
  %res = call <4 x float> @llvm.x86.sse41.insertps(<4 x float> %a0, <4 x float> %a1, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.insertps(<4 x float>, <4 x float>, i32) nounwind readnone


define <2 x i64> @test_x86_sse41_movntdqa(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vmovntdqa
  %res = call <2 x i64> @llvm.x86.sse41.movntdqa(i8* %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.movntdqa(i8*) nounwind readonly


define <16 x i8> @test_x86_sse41_mpsadbw(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vmpsadbw
  %res = call <16 x i8> @llvm.x86.sse41.mpsadbw(<16 x i8> %a0, <16 x i8> %a1, i32 7) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse41.mpsadbw(<16 x i8>, <16 x i8>, i32) nounwind readnone


define <8 x i16> @test_x86_sse41_packusdw(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpackusdw
  %res = call <8 x i16> @llvm.x86.sse41.packusdw(<4 x i32> %a0, <4 x i32> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.packusdw(<4 x i32>, <4 x i32>) nounwind readnone


define <16 x i8> @test_x86_sse41_pblendvb(<16 x i8> %a0, <16 x i8> %a1, <16 x i8> %a2) {
  ; CHECK: vpblendvb
  %res = call <16 x i8> @llvm.x86.sse41.pblendvb(<16 x i8> %a0, <16 x i8> %a1, <16 x i8> %a2) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse41.pblendvb(<16 x i8>, <16 x i8>, <16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse41_pblendw(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpblendw
  %res = call <8 x i16> @llvm.x86.sse41.pblendw(<8 x i16> %a0, <8 x i16> %a1, i32 7) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.pblendw(<8 x i16>, <8 x i16>, i32) nounwind readnone


define <2 x i64> @test_x86_sse41_pcmpeqq(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vpcmpeqq
  %res = call <2 x i64> @llvm.x86.sse41.pcmpeqq(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pcmpeqq(<2 x i64>, <2 x i64>) nounwind readnone


define <8 x i16> @test_x86_sse41_phminposuw(<8 x i16> %a0) {
  ; CHECK: vphminposuw
  %res = call <8 x i16> @llvm.x86.sse41.phminposuw(<8 x i16> %a0) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.phminposuw(<8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse41_pmaxsb(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpmaxsb
  %res = call <16 x i8> @llvm.x86.sse41.pmaxsb(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse41.pmaxsb(<16 x i8>, <16 x i8>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmaxsd(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpmaxsd
  %res = call <4 x i32> @llvm.x86.sse41.pmaxsd(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmaxsd(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmaxud(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpmaxud
  %res = call <4 x i32> @llvm.x86.sse41.pmaxud(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmaxud(<4 x i32>, <4 x i32>) nounwind readnone


define <8 x i16> @test_x86_sse41_pmaxuw(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmaxuw
  %res = call <8 x i16> @llvm.x86.sse41.pmaxuw(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.pmaxuw(<8 x i16>, <8 x i16>) nounwind readnone


define <16 x i8> @test_x86_sse41_pminsb(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpminsb
  %res = call <16 x i8> @llvm.x86.sse41.pminsb(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse41.pminsb(<16 x i8>, <16 x i8>) nounwind readnone


define <4 x i32> @test_x86_sse41_pminsd(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpminsd
  %res = call <4 x i32> @llvm.x86.sse41.pminsd(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pminsd(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i32> @test_x86_sse41_pminud(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpminud
  %res = call <4 x i32> @llvm.x86.sse41.pminud(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pminud(<4 x i32>, <4 x i32>) nounwind readnone


define <8 x i16> @test_x86_sse41_pminuw(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpminuw
  %res = call <8 x i16> @llvm.x86.sse41.pminuw(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.pminuw(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmovsxbd(<16 x i8> %a0) {
  ; CHECK: vpmovsxbd
  %res = call <4 x i32> @llvm.x86.sse41.pmovsxbd(<16 x i8> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmovsxbd(<16 x i8>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovsxbq(<16 x i8> %a0) {
  ; CHECK: vpmovsxbq
  %res = call <2 x i64> @llvm.x86.sse41.pmovsxbq(<16 x i8> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovsxbq(<16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse41_pmovsxbw(<16 x i8> %a0) {
  ; CHECK: vpmovsxbw
  %res = call <8 x i16> @llvm.x86.sse41.pmovsxbw(<16 x i8> %a0) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.pmovsxbw(<16 x i8>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovsxdq(<4 x i32> %a0) {
  ; CHECK: vpmovsxdq
  %res = call <2 x i64> @llvm.x86.sse41.pmovsxdq(<4 x i32> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovsxdq(<4 x i32>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmovsxwd(<8 x i16> %a0) {
  ; CHECK: vpmovsxwd
  %res = call <4 x i32> @llvm.x86.sse41.pmovsxwd(<8 x i16> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmovsxwd(<8 x i16>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovsxwq(<8 x i16> %a0) {
  ; CHECK: vpmovsxwq
  %res = call <2 x i64> @llvm.x86.sse41.pmovsxwq(<8 x i16> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovsxwq(<8 x i16>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmovzxbd(<16 x i8> %a0) {
  ; CHECK: vpmovzxbd
  %res = call <4 x i32> @llvm.x86.sse41.pmovzxbd(<16 x i8> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmovzxbd(<16 x i8>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovzxbq(<16 x i8> %a0) {
  ; CHECK: vpmovzxbq
  %res = call <2 x i64> @llvm.x86.sse41.pmovzxbq(<16 x i8> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovzxbq(<16 x i8>) nounwind readnone


define <8 x i16> @test_x86_sse41_pmovzxbw(<16 x i8> %a0) {
  ; CHECK: vpmovzxbw
  %res = call <8 x i16> @llvm.x86.sse41.pmovzxbw(<16 x i8> %a0) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.sse41.pmovzxbw(<16 x i8>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovzxdq(<4 x i32> %a0) {
  ; CHECK: vpmovzxdq
  %res = call <2 x i64> @llvm.x86.sse41.pmovzxdq(<4 x i32> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovzxdq(<4 x i32>) nounwind readnone


define <4 x i32> @test_x86_sse41_pmovzxwd(<8 x i16> %a0) {
  ; CHECK: vpmovzxwd
  %res = call <4 x i32> @llvm.x86.sse41.pmovzxwd(<8 x i16> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.sse41.pmovzxwd(<8 x i16>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmovzxwq(<8 x i16> %a0) {
  ; CHECK: vpmovzxwq
  %res = call <2 x i64> @llvm.x86.sse41.pmovzxwq(<8 x i16> %a0) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmovzxwq(<8 x i16>) nounwind readnone


define <2 x i64> @test_x86_sse41_pmuldq(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpmuldq
  %res = call <2 x i64> @llvm.x86.sse41.pmuldq(<4 x i32> %a0, <4 x i32> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse41.pmuldq(<4 x i32>, <4 x i32>) nounwind readnone


define i32 @test_x86_sse41_ptestc(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vptest 
  ; CHECK: setb
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse41.ptestc(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse41.ptestc(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse41_ptestnzc(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vptest 
  ; CHECK: seta
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse41.ptestnzc(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse41.ptestnzc(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse41_ptestz(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vptest 
  ; CHECK: sete
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse41.ptestz(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse41.ptestz(<4 x float>, <4 x float>) nounwind readnone


define <2 x double> @test_x86_sse41_round_pd(<2 x double> %a0) {
  ; CHECK: vroundpd
  %res = call <2 x double> @llvm.x86.sse41.round.pd(<2 x double> %a0, i32 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse41.round.pd(<2 x double>, i32) nounwind readnone


define <4 x float> @test_x86_sse41_round_ps(<4 x float> %a0) {
  ; CHECK: vroundps
  %res = call <4 x float> @llvm.x86.sse41.round.ps(<4 x float> %a0, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.round.ps(<4 x float>, i32) nounwind readnone


define <2 x double> @test_x86_sse41_round_sd(<2 x double> %a0, <2 x double> %a1) {
  ; CHECK: vroundsd
  %res = call <2 x double> @llvm.x86.sse41.round.sd(<2 x double> %a0, <2 x double> %a1, i32 7) ; <<2 x double>> [#uses=1]
  ret <2 x double> %res
}
declare <2 x double> @llvm.x86.sse41.round.sd(<2 x double>, <2 x double>, i32) nounwind readnone


define <4 x float> @test_x86_sse41_round_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vroundss
  %res = call <4 x float> @llvm.x86.sse41.round.ss(<4 x float> %a0, <4 x float> %a1, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse41.round.ss(<4 x float>, <4 x float>, i32) nounwind readnone


define i32 @test_x86_sse42_pcmpestri128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestri128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestri128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpestria128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestria128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestria128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpestric128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestric128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestric128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpestrio128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestrio128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestrio128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpestris128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestris128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestris128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpestriz128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpestriz128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpestriz128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define <16 x i8> @test_x86_sse42_pcmpestrm128(<16 x i8> %a0, <16 x i8> %a2) {
  ; CHECK: movl
  ; CHECK: movl
  ; CHECK: vpcmpestrm
  %res = call <16 x i8> @llvm.x86.sse42.pcmpestrm128(<16 x i8> %a0, i32 7, <16 x i8> %a2, i32 7, i8 7) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse42.pcmpestrm128(<16 x i8>, i32, <16 x i8>, i32, i8) nounwind readnone


define <2 x i64> @test_x86_sse42_pcmpgtq(<2 x i64> %a0, <2 x i64> %a1) {
  ; CHECK: vpcmpgtq
  %res = call <2 x i64> @llvm.x86.sse42.pcmpgtq(<2 x i64> %a0, <2 x i64> %a1) ; <<2 x i64>> [#uses=1]
  ret <2 x i64> %res
}
declare <2 x i64> @llvm.x86.sse42.pcmpgtq(<2 x i64>, <2 x i64>) nounwind readnone


define i32 @test_x86_sse42_pcmpistri128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistri128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistri128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpistria128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistria128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistria128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpistric128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistric128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistric128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpistrio128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistrio128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistrio128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpistris128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistris128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistris128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define i32 @test_x86_sse42_pcmpistriz128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistri
  ; CHECK: movl
  %res = call i32 @llvm.x86.sse42.pcmpistriz128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse42.pcmpistriz128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define <16 x i8> @test_x86_sse42_pcmpistrm128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpcmpistrm
  %res = call <16 x i8> @llvm.x86.sse42.pcmpistrm128(<16 x i8> %a0, <16 x i8> %a1, i8 7) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.sse42.pcmpistrm128(<16 x i8>, <16 x i8>, i8) nounwind readnone


define <4 x float> @test_x86_sse_add_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vaddss
  %res = call <4 x float> @llvm.x86.sse.add.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.add.ss(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_cmp_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcmpordps
  %res = call <4 x float> @llvm.x86.sse.cmp.ps(<4 x float> %a0, <4 x float> %a1, i8 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.cmp.ps(<4 x float>, <4 x float>, i8) nounwind readnone


define <4 x float> @test_x86_sse_cmp_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcmpordss
  %res = call <4 x float> @llvm.x86.sse.cmp.ss(<4 x float> %a0, <4 x float> %a1, i8 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.cmp.ss(<4 x float>, <4 x float>, i8) nounwind readnone


define i32 @test_x86_sse_comieq_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: sete
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comieq.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comieq.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_comige_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: setae
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comige.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comige.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_comigt_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: seta
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comigt.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comigt.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_comile_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: setbe
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comile.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comile.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_comilt_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: setb
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comilt.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comilt.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_comineq_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vcomiss
  ; CHECK: setne
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.comineq.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.comineq.ss(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_cvtsi2ss(<4 x float> %a0) {
  ; CHECK: movl
  ; CHECK: vcvtsi2ss
  %res = call <4 x float> @llvm.x86.sse.cvtsi2ss(<4 x float> %a0, i32 7) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.cvtsi2ss(<4 x float>, i32) nounwind readnone


define i32 @test_x86_sse_cvtss2si(<4 x float> %a0) {
  ; CHECK: vcvtss2si
  %res = call i32 @llvm.x86.sse.cvtss2si(<4 x float> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.cvtss2si(<4 x float>) nounwind readnone


define i32 @test_x86_sse_cvttss2si(<4 x float> %a0) {
  ; CHECK: vcvttss2si
  %res = call i32 @llvm.x86.sse.cvttss2si(<4 x float> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.cvttss2si(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_div_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vdivss
  %res = call <4 x float> @llvm.x86.sse.div.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.div.ss(<4 x float>, <4 x float>) nounwind readnone


define void @test_x86_sse_ldmxcsr(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vldmxcsr
  call void @llvm.x86.sse.ldmxcsr(i8* %a0)
  ret void
}
declare void @llvm.x86.sse.ldmxcsr(i8*) nounwind


define <4 x float> @test_x86_sse_loadu_ps(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vmovups
  %res = call <4 x float> @llvm.x86.sse.loadu.ps(i8* %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.loadu.ps(i8*) nounwind readonly


define <4 x float> @test_x86_sse_max_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vmaxps
  %res = call <4 x float> @llvm.x86.sse.max.ps(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.max.ps(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_max_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vmaxss
  %res = call <4 x float> @llvm.x86.sse.max.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.max.ss(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_min_ps(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vminps
  %res = call <4 x float> @llvm.x86.sse.min.ps(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.min.ps(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_min_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vminss
  %res = call <4 x float> @llvm.x86.sse.min.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.min.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_movmsk_ps(<4 x float> %a0) {
  ; CHECK: vmovmskps
  %res = call i32 @llvm.x86.sse.movmsk.ps(<4 x float> %a0) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.movmsk.ps(<4 x float>) nounwind readnone


define void @test_x86_sse_movnt_ps(i8* %a0, <4 x float> %a1) {
  ; CHECK: movl
  ; CHECK: vmovntps
  call void @llvm.x86.sse.movnt.ps(i8* %a0, <4 x float> %a1)
  ret void
}
declare void @llvm.x86.sse.movnt.ps(i8*, <4 x float>) nounwind


define <4 x float> @test_x86_sse_mul_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vmulss
  %res = call <4 x float> @llvm.x86.sse.mul.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.mul.ss(<4 x float>, <4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_rcp_ps(<4 x float> %a0) {
  ; CHECK: vrcpps
  %res = call <4 x float> @llvm.x86.sse.rcp.ps(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.rcp.ps(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_rcp_ss(<4 x float> %a0) {
  ; CHECK: vrcpss
  %res = call <4 x float> @llvm.x86.sse.rcp.ss(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.rcp.ss(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_rsqrt_ps(<4 x float> %a0) {
  ; CHECK: vrsqrtps
  %res = call <4 x float> @llvm.x86.sse.rsqrt.ps(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.rsqrt.ps(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_rsqrt_ss(<4 x float> %a0) {
  ; CHECK: vrsqrtss
  %res = call <4 x float> @llvm.x86.sse.rsqrt.ss(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.rsqrt.ss(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_sqrt_ps(<4 x float> %a0) {
  ; CHECK: vsqrtps
  %res = call <4 x float> @llvm.x86.sse.sqrt.ps(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.sqrt.ps(<4 x float>) nounwind readnone


define <4 x float> @test_x86_sse_sqrt_ss(<4 x float> %a0) {
  ; CHECK: vsqrtss
  %res = call <4 x float> @llvm.x86.sse.sqrt.ss(<4 x float> %a0) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.sqrt.ss(<4 x float>) nounwind readnone


define void @test_x86_sse_stmxcsr(i8* %a0) {
  ; CHECK: movl
  ; CHECK: vstmxcsr
  call void @llvm.x86.sse.stmxcsr(i8* %a0)
  ret void
}
declare void @llvm.x86.sse.stmxcsr(i8*) nounwind


define void @test_x86_sse_storeu_ps(i8* %a0, <4 x float> %a1) {
  ; CHECK: movl
  ; CHECK: vmovups
  call void @llvm.x86.sse.storeu.ps(i8* %a0, <4 x float> %a1)
  ret void
}
declare void @llvm.x86.sse.storeu.ps(i8*, <4 x float>) nounwind


define <4 x float> @test_x86_sse_sub_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vsubss
  %res = call <4 x float> @llvm.x86.sse.sub.ss(<4 x float> %a0, <4 x float> %a1) ; <<4 x float>> [#uses=1]
  ret <4 x float> %res
}
declare <4 x float> @llvm.x86.sse.sub.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomieq_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: sete
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomieq.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomieq.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomige_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: setae
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomige.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomige.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomigt_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: seta
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomigt.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomigt.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomile_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: setbe
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomile.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomile.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomilt_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: setb
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomilt.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomilt.ss(<4 x float>, <4 x float>) nounwind readnone


define i32 @test_x86_sse_ucomineq_ss(<4 x float> %a0, <4 x float> %a1) {
  ; CHECK: vucomiss
  ; CHECK: setne
  ; CHECK: movzbl
  %res = call i32 @llvm.x86.sse.ucomineq.ss(<4 x float> %a0, <4 x float> %a1) ; <i32> [#uses=1]
  ret i32 %res
}
declare i32 @llvm.x86.sse.ucomineq.ss(<4 x float>, <4 x float>) nounwind readnone


define <8 x i8> @test_x86_ssse3_pabs_b(<8 x i8> %a0) {
  ; CHECK: vpabsb
  %res = call <8 x i8> @llvm.x86.ssse3.pabs.b(<8 x i8> %a0) ; <<8 x i8>> [#uses=1]
  ret <8 x i8> %res
}
declare <8 x i8> @llvm.x86.ssse3.pabs.b(<8 x i8>) nounwind readnone


define <16 x i8> @test_x86_ssse3_pabs_b_128(<16 x i8> %a0) {
  ; CHECK: vpabsb
  %res = call <16 x i8> @llvm.x86.ssse3.pabs.b.128(<16 x i8> %a0) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.ssse3.pabs.b.128(<16 x i8>) nounwind readnone


define <2 x i32> @test_x86_ssse3_pabs_d(<2 x i32> %a0) {
  ; CHECK: vpabsd
  %res = call <2 x i32> @llvm.x86.ssse3.pabs.d(<2 x i32> %a0) ; <<2 x i32>> [#uses=1]
  ret <2 x i32> %res
}
declare <2 x i32> @llvm.x86.ssse3.pabs.d(<2 x i32>) nounwind readnone


define <4 x i32> @test_x86_ssse3_pabs_d_128(<4 x i32> %a0) {
  ; CHECK: vpabsd
  %res = call <4 x i32> @llvm.x86.ssse3.pabs.d.128(<4 x i32> %a0) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.ssse3.pabs.d.128(<4 x i32>) nounwind readnone


define <4 x i16> @test_x86_ssse3_pabs_w(<4 x i16> %a0) {
  ; CHECK: vpabsw
  %res = call <4 x i16> @llvm.x86.ssse3.pabs.w(<4 x i16> %a0) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.pabs.w(<4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_pabs_w_128(<8 x i16> %a0) {
  ; CHECK: vpabsw
  %res = call <8 x i16> @llvm.x86.ssse3.pabs.w.128(<8 x i16> %a0) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.pabs.w.128(<8 x i16>) nounwind readnone


define <2 x i32> @test_x86_ssse3_phadd_d(<2 x i32> %a0, <2 x i32> %a1) {
  ; CHECK: vphaddd
  %res = call <2 x i32> @llvm.x86.ssse3.phadd.d(<2 x i32> %a0, <2 x i32> %a1) ; <<2 x i32>> [#uses=1]
  ret <2 x i32> %res
}
declare <2 x i32> @llvm.x86.ssse3.phadd.d(<2 x i32>, <2 x i32>) nounwind readnone


define <4 x i32> @test_x86_ssse3_phadd_d_128(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vphaddd
  %res = call <4 x i32> @llvm.x86.ssse3.phadd.d.128(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.ssse3.phadd.d.128(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i16> @test_x86_ssse3_phadd_sw(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vphaddsw
  %res = call <4 x i16> @llvm.x86.ssse3.phadd.sw(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.phadd.sw(<4 x i16>, <4 x i16>) nounwind readnone


define <4 x i32> @test_x86_ssse3_phadd_sw_128(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vphaddsw
  %res = call <4 x i32> @llvm.x86.ssse3.phadd.sw.128(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.ssse3.phadd.sw.128(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i16> @test_x86_ssse3_phadd_w(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vphaddw
  %res = call <4 x i16> @llvm.x86.ssse3.phadd.w(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.phadd.w(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_phadd_w_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vphaddw
  %res = call <8 x i16> @llvm.x86.ssse3.phadd.w.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.phadd.w.128(<8 x i16>, <8 x i16>) nounwind readnone


define <2 x i32> @test_x86_ssse3_phsub_d(<2 x i32> %a0, <2 x i32> %a1) {
  ; CHECK: vphsubd
  %res = call <2 x i32> @llvm.x86.ssse3.phsub.d(<2 x i32> %a0, <2 x i32> %a1) ; <<2 x i32>> [#uses=1]
  ret <2 x i32> %res
}
declare <2 x i32> @llvm.x86.ssse3.phsub.d(<2 x i32>, <2 x i32>) nounwind readnone


define <4 x i32> @test_x86_ssse3_phsub_d_128(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vphsubd
  %res = call <4 x i32> @llvm.x86.ssse3.phsub.d.128(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.ssse3.phsub.d.128(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i16> @test_x86_ssse3_phsub_sw(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vphsubsw
  %res = call <4 x i16> @llvm.x86.ssse3.phsub.sw(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.phsub.sw(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_phsub_sw_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vphsubsw
  %res = call <8 x i16> @llvm.x86.ssse3.phsub.sw.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.phsub.sw.128(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i16> @test_x86_ssse3_phsub_w(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vphsubw
  %res = call <4 x i16> @llvm.x86.ssse3.phsub.w(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.phsub.w(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_phsub_w_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vphsubw
  %res = call <8 x i16> @llvm.x86.ssse3.phsub.w.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.phsub.w.128(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i16> @test_x86_ssse3_pmadd_ub_sw(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vpmaddubsw
  %res = call <4 x i16> @llvm.x86.ssse3.pmadd.ub.sw(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.pmadd.ub.sw(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_pmadd_ub_sw_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmaddubsw
  %res = call <8 x i16> @llvm.x86.ssse3.pmadd.ub.sw.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.pmadd.ub.sw.128(<8 x i16>, <8 x i16>) nounwind readnone


define <4 x i16> @test_x86_ssse3_pmul_hr_sw(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vpmulhrsw
  %res = call <4 x i16> @llvm.x86.ssse3.pmul.hr.sw(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.pmul.hr.sw(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_pmul_hr_sw_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpmulhrsw
  %res = call <8 x i16> @llvm.x86.ssse3.pmul.hr.sw.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.pmul.hr.sw.128(<8 x i16>, <8 x i16>) nounwind readnone


define <8 x i8> @test_x86_ssse3_pshuf_b(<8 x i8> %a0, <8 x i8> %a1) {
  ; CHECK: vpshufb
  %res = call <8 x i8> @llvm.x86.ssse3.pshuf.b(<8 x i8> %a0, <8 x i8> %a1) ; <<8 x i8>> [#uses=1]
  ret <8 x i8> %res
}
declare <8 x i8> @llvm.x86.ssse3.pshuf.b(<8 x i8>, <8 x i8>) nounwind readnone


define <16 x i8> @test_x86_ssse3_pshuf_b_128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpshufb
  %res = call <16 x i8> @llvm.x86.ssse3.pshuf.b.128(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.ssse3.pshuf.b.128(<16 x i8>, <16 x i8>) nounwind readnone


define <8 x i8> @test_x86_ssse3_psign_b(<8 x i8> %a0, <8 x i8> %a1) {
  ; CHECK: vpsignb
  %res = call <8 x i8> @llvm.x86.ssse3.psign.b(<8 x i8> %a0, <8 x i8> %a1) ; <<8 x i8>> [#uses=1]
  ret <8 x i8> %res
}
declare <8 x i8> @llvm.x86.ssse3.psign.b(<8 x i8>, <8 x i8>) nounwind readnone


define <16 x i8> @test_x86_ssse3_psign_b_128(<16 x i8> %a0, <16 x i8> %a1) {
  ; CHECK: vpsignb
  %res = call <16 x i8> @llvm.x86.ssse3.psign.b.128(<16 x i8> %a0, <16 x i8> %a1) ; <<16 x i8>> [#uses=1]
  ret <16 x i8> %res
}
declare <16 x i8> @llvm.x86.ssse3.psign.b.128(<16 x i8>, <16 x i8>) nounwind readnone


define <2 x i32> @test_x86_ssse3_psign_d(<2 x i32> %a0, <2 x i32> %a1) {
  ; CHECK: vpsignd
  %res = call <2 x i32> @llvm.x86.ssse3.psign.d(<2 x i32> %a0, <2 x i32> %a1) ; <<2 x i32>> [#uses=1]
  ret <2 x i32> %res
}
declare <2 x i32> @llvm.x86.ssse3.psign.d(<2 x i32>, <2 x i32>) nounwind readnone


define <4 x i32> @test_x86_ssse3_psign_d_128(<4 x i32> %a0, <4 x i32> %a1) {
  ; CHECK: vpsignd
  %res = call <4 x i32> @llvm.x86.ssse3.psign.d.128(<4 x i32> %a0, <4 x i32> %a1) ; <<4 x i32>> [#uses=1]
  ret <4 x i32> %res
}
declare <4 x i32> @llvm.x86.ssse3.psign.d.128(<4 x i32>, <4 x i32>) nounwind readnone


define <4 x i16> @test_x86_ssse3_psign_w(<4 x i16> %a0, <4 x i16> %a1) {
  ; CHECK: vpsignw
  %res = call <4 x i16> @llvm.x86.ssse3.psign.w(<4 x i16> %a0, <4 x i16> %a1) ; <<4 x i16>> [#uses=1]
  ret <4 x i16> %res
}
declare <4 x i16> @llvm.x86.ssse3.psign.w(<4 x i16>, <4 x i16>) nounwind readnone


define <8 x i16> @test_x86_ssse3_psign_w_128(<8 x i16> %a0, <8 x i16> %a1) {
  ; CHECK: vpsignw
  %res = call <8 x i16> @llvm.x86.ssse3.psign.w.128(<8 x i16> %a0, <8 x i16> %a1) ; <<8 x i16>> [#uses=1]
  ret <8 x i16> %res
}
declare <8 x i16> @llvm.x86.ssse3.psign.w.128(<8 x i16>, <8 x i16>) nounwind readnone

