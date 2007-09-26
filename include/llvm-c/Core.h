/*===-- llvm-c/Core.h - Core Library C Interface ------------------*- C -*-===*\
|*                                                                            *|
|*                     The LLVM Compiler Infrastructure                       *|
|*                                                                            *|
|* This file was developed by Gordon Henriksen and is distributed under the   *|
|* University of Illinois Open Source License. See LICENSE.TXT for details.   *|
|*                                                                            *|
|*===----------------------------------------------------------------------===*|
|*                                                                            *|
|* This header declares the C interface to libLLVMCore.a, which implements    *|
|* the LLVM intermediate representation.                                      *|
|*                                                                            *|
|* LLVM uses a polymorphic type hierarchy which C cannot represent, therefore *|
|* parameters must be passed as base types. Despite the declared types, most  *|
|* of the functions provided operate only on branches of the type hierarchy.  *|
|* The declared parameter names are descriptive and specify which type is     *|
|* required. Additionally, each type hierarchy is documented along with the   *|
|* functions that operate upon it. For more detail, refer to LLVM's C++ code. *|
|* If in doubt, refer to Core.cpp, which performs paramter downcasts in the   *|
|* form unwrap<RequiredType>(Param).                                          *|
|*                                                                            *|
|* Many exotic languages can interoperate with C code but have a harder time  *|
|* with C++ due to name mangling. So in addition to C, this interface enables *|
|* tools written in such languages.                                           *|
|*                                                                            *|
\*===----------------------------------------------------------------------===*/

#ifndef LLVM_C_CORE_H
#define LLVM_C_CORE_H

#ifdef __cplusplus
extern "C" {
#endif


/* Opaque types. */
typedef struct LLVMOpaqueModule *LLVMModuleRef;
typedef struct LLVMOpaqueType *LLVMTypeRef;
typedef struct LLVMOpaqueValue *LLVMValueRef;
typedef struct LLVMOpaqueBasicBlock *LLVMBasicBlockRef;
typedef struct LLVMOpaqueBuilder *LLVMBuilderRef;

typedef enum {
  LLVMVoidTypeKind,        /* type with no size */
  LLVMFloatTypeKind,       /* 32 bit floating point type */
  LLVMDoubleTypeKind,      /* 64 bit floating point type */
  LLVMX86_FP80TypeKind,    /* 80 bit floating point type (X87) */
  LLVMFP128TypeKind,       /* 128 bit floating point type (112-bit mantissa) */
  LLVMPPC_FP128TypeKind,   /* 128 bit floating point type (two 64-bits) */
  LLVMLabelTypeKind,       /* Labels */
  LLVMIntegerTypeKind,     /* Arbitrary bit width integers */
  LLVMFunctionTypeKind,    /* Functions */
  LLVMStructTypeKind,      /* Structures */
  LLVMArrayTypeKind,       /* Arrays */
  LLVMPointerTypeKind,     /* Pointers */
  LLVMOpaqueTypeKind,      /* Opaque: type with unknown structure */
  LLVMVectorTypeKind       /* SIMD 'packed' format, or other vector type */
} LLVMTypeKind;

typedef enum {
  LLVMExternalLinkage,    /* Externally visible function */
  LLVMLinkOnceLinkage,    /* Keep one copy of function when linking (inline) */
  LLVMWeakLinkage,        /* Keep one copy of function when linking (weak) */
  LLVMAppendingLinkage,   /* Special purpose, only applies to global arrays */
  LLVMInternalLinkage,    /* Rename collisions when linking (static functions)*/
  LLVMDLLImportLinkage,   /* Function to be imported from DLL */
  LLVMDLLExportLinkage,   /* Function to be accessible from DLL */
  LLVMExternalWeakLinkage,/* ExternalWeak linkage description */
  LLVMGhostLinkage        /* Stand-in functions for streaming fns from bitcode*/
} LLVMLinkage;

typedef enum {
  LLVMDefaultVisibility,  /* The GV is visible */
  LLVMHiddenVisibility,   /* The GV is hidden */
  LLVMProtectedVisibility /* The GV is protected */
} LLVMVisibility;

typedef enum {
  LLVMCCallConv           = 0,
  LLVMFastCallConv        = 8,
  LLVMColdCallConv        = 9,
  LLVMX86StdcallCallConv  = 64,
  LLVMX86FastcallCallConv = 65
} LLVMCallConv;

typedef enum {
  LLVMIntEQ = 32, /* equal */
  LLVMIntNE,      /* not equal */
  LLVMIntUGT,     /* unsigned greater than */
  LLVMIntUGE,     /* unsigned greater or equal */
  LLVMIntULT,     /* unsigned less than */
  LLVMIntULE,     /* unsigned less or equal */
  LLVMIntSGT,     /* signed greater than */
  LLVMIntSGE,     /* signed greater or equal */
  LLVMIntSLT,     /* signed less than */
  LLVMIntSLE      /* signed less or equal */
} LLVMIntPredicate;

typedef enum {
  LLVMRealPredicateFalse, /* Always false (always folded) */
  LLVMRealOEQ,            /* True if ordered and equal */
  LLVMRealOGT,            /* True if ordered and greater than */
  LLVMRealOGE,            /* True if ordered and greater than or equal */
  LLVMRealOLT,            /* True if ordered and less than */
  LLVMRealOLE,            /* True if ordered and less than or equal */
  LLVMRealONE,            /* True if ordered and operands are unequal */
  LLVMRealORD,            /* True if ordered (no nans) */
  LLVMRealUNO,            /* True if unordered: isnan(X) | isnan(Y) */
  LLVMRealUEQ,            /* True if unordered or equal */
  LLVMRealUGT,            /* True if unordered or greater than */
  LLVMRealUGE,            /* True if unordered, greater than, or equal */
  LLVMRealULT,            /* True if unordered or less than */
  LLVMRealULE,            /* True if unordered, less than, or equal */
  LLVMRealUNE,            /* True if unordered or not equal */
  LLVMRealPredicateTrue   /* Always true (always folded) */
} LLVMRealPredicate;


/*===-- Modules -----------------------------------------------------------===*/

/* Create and destroy modules. */ 
LLVMModuleRef LLVMModuleCreateWithName(const char *ModuleID);
void LLVMDisposeModule(LLVMModuleRef M);

/* Same as Module::addTypeName. */
int LLVMAddTypeName(LLVMModuleRef M, const char *Name, LLVMTypeRef Ty);
void LLVMDeleteTypeName(LLVMModuleRef M, const char *Name);


/*===-- Types -------------------------------------------------------------===*/

/* LLVM types conform to the following hierarchy:
 * 
 *   types:
 *     integer type
 *     real type
 *     function type
 *     sequence types:
 *       array type
 *       pointer type
 *       vector type
 *     void type
 *     label type
 *     opaque type
 */

LLVMTypeKind LLVMGetTypeKind(LLVMTypeRef Ty);
void LLVMRefineAbstractType(LLVMTypeRef AbstractType, LLVMTypeRef ConcreteType);

/* Operations on integer types */
LLVMTypeRef LLVMInt1Type();
LLVMTypeRef LLVMInt8Type();
LLVMTypeRef LLVMInt16Type();
LLVMTypeRef LLVMInt32Type();
LLVMTypeRef LLVMInt64Type();
LLVMTypeRef LLVMCreateIntType(unsigned NumBits);
unsigned LLVMGetIntTypeWidth(LLVMTypeRef IntegerTy);

/* Operations on real types */
LLVMTypeRef LLVMFloatType();
LLVMTypeRef LLVMDoubleType();
LLVMTypeRef LLVMX86FP80Type();
LLVMTypeRef LLVMFP128Type();
LLVMTypeRef LLVMPPCFP128Type();

/* Operations on function types */
LLVMTypeRef LLVMCreateFunctionType(LLVMTypeRef ReturnType,
                                   LLVMTypeRef *ParamTypes, unsigned ParamCount,
                                   int IsVarArg);
int LLVMIsFunctionVarArg(LLVMTypeRef FunctionTy);
LLVMTypeRef LLVMGetReturnType(LLVMTypeRef FunctionTy);
unsigned LLVMCountParamTypes(LLVMTypeRef FunctionTy);
void LLVMGetParamTypes(LLVMTypeRef FunctionTy, LLVMTypeRef *Dest);

/* Operations on struct types */
LLVMTypeRef LLVMCreateStructType(LLVMTypeRef *ElementTypes,
                                 unsigned ElementCount, int Packed);
unsigned LLVMCountStructElementTypes(LLVMTypeRef StructTy);
void LLVMGetStructElementTypes(LLVMTypeRef StructTy, LLVMTypeRef *Dest);
int LLVMIsPackedStruct(LLVMTypeRef StructTy);

/* Operations on array, pointer, and vector types (sequence types) */
LLVMTypeRef LLVMCreateArrayType(LLVMTypeRef ElementType, unsigned ElementCount);
LLVMTypeRef LLVMCreatePointerType(LLVMTypeRef ElementType);
LLVMTypeRef LLVMCreateVectorType(LLVMTypeRef ElementType,unsigned ElementCount);

LLVMTypeRef LLVMGetElementType(LLVMTypeRef Ty);
unsigned LLVMGetArrayLength(LLVMTypeRef ArrayTy);
unsigned LLVMGetVectorSize(LLVMTypeRef VectorTy);

/* Operations on other types */
LLVMTypeRef LLVMVoidType();
LLVMTypeRef LLVMLabelType();
LLVMTypeRef LLVMCreateOpaqueType();


/*===-- Values ------------------------------------------------------------===*/

/* The bulk of LLVM's object model consists of values, which comprise a very
 * rich type hierarchy.
 * 
 *   values:
 *     constants:
 *       scalar constants
 *       composite contants
 *       globals:
 *         global variable
 *         function
 *         alias
 *       basic blocks
 */

/* Operations on all values */
LLVMTypeRef LLVMTypeOf(LLVMValueRef Val);
const char *LLVMGetValueName(LLVMValueRef Val);
void LLVMSetValueName(LLVMValueRef Val, const char *Name);

/* Operations on constants of any type */
LLVMValueRef LLVMGetNull(LLVMTypeRef Ty); /* all zeroes */
LLVMValueRef LLVMGetAllOnes(LLVMTypeRef Ty); /* only for int/vector */
LLVMValueRef LLVMGetUndef(LLVMTypeRef Ty);
int LLVMIsConstant(LLVMValueRef Val);
int LLVMIsNull(LLVMValueRef Val);
int LLVMIsUndef(LLVMValueRef Val);

/* Operations on scalar constants */
LLVMValueRef LLVMGetIntConstant(LLVMTypeRef IntTy, unsigned long long N,
                                int SignExtend);
LLVMValueRef LLVMGetRealConstant(LLVMTypeRef RealTy, double N);

/* Operations on composite constants */
LLVMValueRef LLVMGetStringConstant(const char *Str, unsigned Length,
                                   int DontNullTerminate);
LLVMValueRef LLVMGetArrayConstant(LLVMTypeRef ArrayTy,
                                  LLVMValueRef *ConstantVals, unsigned Length);
LLVMValueRef LLVMGetStructConstant(LLVMValueRef *ConstantVals, unsigned Count,
                                   int packed);
LLVMValueRef LLVMGetVectorConstant(LLVMValueRef *ScalarConstantVals,
                                   unsigned Size);

/* Operations on global variables, functions, and aliases (globals) */
int LLVMIsDeclaration(LLVMValueRef Global);
LLVMLinkage LLVMGetLinkage(LLVMValueRef Global);
void LLVMSetLinkage(LLVMValueRef Global, LLVMLinkage Linkage);
const char *LLVMGetSection(LLVMValueRef Global);
void LLVMSetSection(LLVMValueRef Global, const char *Section);
LLVMVisibility LLVMGetVisibility(LLVMValueRef Global);
void LLVMSetVisibility(LLVMValueRef Global, LLVMVisibility Viz);
unsigned LLVMGetAlignment(LLVMValueRef Global);
void LLVMSetAlignment(LLVMValueRef Global, unsigned Bytes);

/* Operations on global variables */
LLVMValueRef LLVMAddGlobal(LLVMModuleRef M, LLVMTypeRef Ty, const char *Name);
void LLVMDeleteGlobal(LLVMValueRef GlobalVar);
int LLVMHasInitializer(LLVMValueRef GlobalVar);
LLVMValueRef LLVMGetInitializer(LLVMValueRef GlobalVar);
void LLVMSetInitializer(LLVMValueRef GlobalVar, LLVMValueRef ConstantVal);
int LLVMIsThreadLocal(LLVMValueRef GlobalVar);
void LLVMSetThreadLocal(LLVMValueRef GlobalVar, int IsThreadLocal);

/* Operations on functions */
LLVMValueRef LLVMAddFunction(LLVMModuleRef M, const char *Name,
                             LLVMTypeRef FunctionTy);
void LLVMDeleteFunction(LLVMValueRef Fn);
unsigned LLVMCountParams(LLVMValueRef Fn);
void LLVMGetParams(LLVMValueRef Fn, LLVMValueRef *Params);
LLVMValueRef LLVMGetParam(LLVMValueRef Fn, unsigned Index);
unsigned LLVMGetIntrinsicID(LLVMValueRef Fn);
unsigned LLVMGetFunctionCallConv(LLVMValueRef Fn);
void LLVMSetFunctionCallConv(LLVMValueRef Fn, unsigned CC);

/* Operations on basic blocks */
LLVMValueRef LLVMBasicBlockAsValue(LLVMBasicBlockRef Bb);
int LLVMValueIsBasicBlock(LLVMValueRef Val);
LLVMBasicBlockRef LLVMValueAsBasicBlock(LLVMValueRef Val);
unsigned LLVMCountBasicBlocks(LLVMValueRef Fn);
void LLVMGetBasicBlocks(LLVMValueRef Fn, LLVMBasicBlockRef *BasicBlocks);
LLVMBasicBlockRef LLVMGetEntryBasicBlock(LLVMValueRef Fn);
LLVMBasicBlockRef LLVMAppendBasicBlock(LLVMValueRef Fn, const char *Name);
LLVMBasicBlockRef LLVMInsertBasicBlock(LLVMBasicBlockRef InsertBeforeBB,
                                       const char *Name);
void LLVMDeleteBasicBlock(LLVMBasicBlockRef BB);


/*===-- Instruction builders ----------------------------------------------===*/

/* An instruction builder represents a point within a basic block, and is the
 * exclusive means of building instructions using the C interface.
 */

LLVMBuilderRef LLVMCreateBuilder();
void LLVMPositionBuilderBefore(LLVMBuilderRef Builder, LLVMValueRef Instr);
void LLVMPositionBuilderAtEnd(LLVMBuilderRef Builder, LLVMBasicBlockRef Block);
void LLVMDisposeBuilder(LLVMBuilderRef Builder);

/* Terminators */
LLVMValueRef LLVMBuildRetVoid(LLVMBuilderRef);
LLVMValueRef LLVMBuildRet(LLVMBuilderRef, LLVMValueRef V);
LLVMValueRef LLVMBuildBr(LLVMBuilderRef, LLVMBasicBlockRef Dest);
LLVMValueRef LLVMBuildCondBr(LLVMBuilderRef, LLVMValueRef If,
                             LLVMBasicBlockRef Then, LLVMBasicBlockRef Else);
LLVMValueRef LLVMBuildSwitch(LLVMBuilderRef, LLVMValueRef V,
                             LLVMBasicBlockRef Else, unsigned NumCases);
LLVMValueRef LLVMBuildInvoke(LLVMBuilderRef, LLVMValueRef Fn,
                             LLVMValueRef *Args, unsigned NumArgs,
                             LLVMBasicBlockRef Then, LLVMBasicBlockRef Catch,
                             const char *Name);
LLVMValueRef LLVMBuildUnwind(LLVMBuilderRef);
LLVMValueRef LLVMBuildUnreachable(LLVMBuilderRef);

/* Arithmetic */
LLVMValueRef LLVMBuildAdd(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildSub(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildMul(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildUDiv(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildSDiv(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildFDiv(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildURem(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildSRem(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildFRem(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildShl(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildLShr(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildAShr(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildAnd(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildOr(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildXor(LLVMBuilderRef, LLVMValueRef LHS, LLVMValueRef RHS,
                          const char *Name);
LLVMValueRef LLVMBuildNeg(LLVMBuilderRef, LLVMValueRef V, const char *Name);
LLVMValueRef LLVMBuildNot(LLVMBuilderRef, LLVMValueRef V, const char *Name);

/* Memory */
LLVMValueRef LLVMBuildMalloc(LLVMBuilderRef, LLVMTypeRef Ty, const char *Name);
LLVMValueRef LLVMBuildArrayMalloc(LLVMBuilderRef, LLVMTypeRef Ty,
                                  LLVMValueRef Val, const char *Name);
LLVMValueRef LLVMBuildAlloca(LLVMBuilderRef, LLVMTypeRef Ty, const char *Name);
LLVMValueRef LLVMBuildArrayAlloca(LLVMBuilderRef, LLVMTypeRef Ty,
                                  LLVMValueRef Val, const char *Name);
LLVMValueRef LLVMBuildFree(LLVMBuilderRef, LLVMValueRef PointerVal);
LLVMValueRef LLVMBuildLoad(LLVMBuilderRef, LLVMValueRef PointerVal,
                           const char *Name);
LLVMValueRef LLVMBuildStore(LLVMBuilderRef, LLVMValueRef Val, LLVMValueRef Ptr);
LLVMValueRef LLVMBuildGEP(LLVMBuilderRef B, LLVMValueRef Pointer,
                          LLVMValueRef *Indices, unsigned NumIndices,
                          const char *Name);

/* Casts */
LLVMValueRef LLVMBuildTrunc(LLVMBuilderRef, LLVMValueRef Val,
                            LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildZExt(LLVMBuilderRef, LLVMValueRef Val,
                           LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildSExt(LLVMBuilderRef, LLVMValueRef Val,
                           LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildFPToUI(LLVMBuilderRef, LLVMValueRef Val,
                             LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildFPToSI(LLVMBuilderRef, LLVMValueRef Val,
                             LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildUIToFP(LLVMBuilderRef, LLVMValueRef Val,
                             LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildSIToFP(LLVMBuilderRef, LLVMValueRef Val,
                             LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildFPTrunc(LLVMBuilderRef, LLVMValueRef Val,
                              LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildFPExt(LLVMBuilderRef, LLVMValueRef Val,
                            LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildPtrToInt(LLVMBuilderRef, LLVMValueRef Val,
                               LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildIntToPtr(LLVMBuilderRef, LLVMValueRef Val,
                               LLVMTypeRef DestTy, const char *Name);
LLVMValueRef LLVMBuildBitCast(LLVMBuilderRef, LLVMValueRef Val,
                              LLVMTypeRef DestTy, const char *Name);

/* Comparisons */
LLVMValueRef LLVMBuildICmp(LLVMBuilderRef, LLVMIntPredicate Op,
                           LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);
LLVMValueRef LLVMBuildFCmp(LLVMBuilderRef, LLVMRealPredicate Op,
                           LLVMValueRef LHS, LLVMValueRef RHS,
                           const char *Name);

/* Miscellaneous instructions */
LLVMValueRef LLVMBuildPhi(LLVMBuilderRef, LLVMTypeRef Ty, const char *Name);
LLVMValueRef LLVMBuildCall(LLVMBuilderRef, LLVMValueRef Fn,
                           LLVMValueRef *Args, unsigned NumArgs,
                           const char *Name);
LLVMValueRef LLVMBuildSelect(LLVMBuilderRef, LLVMValueRef If,
                             LLVMValueRef Then, LLVMValueRef Else,
                             const char *Name);
LLVMValueRef LLVMBuildVAArg(LLVMBuilderRef, LLVMValueRef List, LLVMTypeRef Ty,
                            const char *Name);
LLVMValueRef LLVMBuildExtractElement(LLVMBuilderRef, LLVMValueRef VecVal,
                                     LLVMValueRef Index, const char *Name);
LLVMValueRef LLVMBuildInsertElement(LLVMBuilderRef, LLVMValueRef VecVal,
                                    LLVMValueRef EltVal, LLVMValueRef Index,
                                    const char *Name);
LLVMValueRef LLVMBuildShuffleVector(LLVMBuilderRef, LLVMValueRef V1,
                                    LLVMValueRef V2, LLVMValueRef Mask,
                                    const char *Name);

#ifdef __cplusplus
}
#endif

#endif
