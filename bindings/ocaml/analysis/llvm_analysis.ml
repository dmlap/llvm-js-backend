(*===-- llvm_analysis.ml - LLVM Ocaml Interface -----------------*- C++ -*-===*
 *
 *                     The LLVM Compiler Infrastructure
 *
 * This file was developed by Gordon Henriksen and is distributed under the
 * University of Illinois Open Source License. See LICENSE.TXT for details.
 *
 *===----------------------------------------------------------------------===*)


external verify_module : Llvm.llmodule -> string option = "llvm_verify_module"

external verify_function : Llvm.llvalue -> bool = "llvm_verify_function"

external assert_valid_module : Llvm.llmodule -> unit
                             = "llvm_assert_valid_module"

external assert_valid_function : Llvm.llvalue -> unit
                               = "llvm_assert_valid_function"
