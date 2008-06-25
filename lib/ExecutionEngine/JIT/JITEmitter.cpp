//===-- JITEmitter.cpp - Write machine code to executable memory ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file defines a MachineCodeEmitter object that is used by the JIT to
// write machine code to memory and remember where relocatable values are.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "jit"
#include "JIT.h"
#include "JITDwarfEmitter.h"
#include "llvm/Constant.h"
#include "llvm/Module.h"
#include "llvm/Type.h"
#include "llvm/CodeGen/MachineCodeEmitter.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineConstantPool.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRelocation.h"
#include "llvm/ExecutionEngine/JITMemoryManager.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Target/TargetJITInfo.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MutexGuard.h"
#include "llvm/System/Disassembler.h"
#include "llvm/System/Memory.h"
#include "llvm/Target/TargetInstrInfo.h"
#include "llvm/ADT/Statistic.h"
#include <algorithm>
using namespace llvm;

STATISTIC(NumBytes, "Number of bytes of machine code compiled");
STATISTIC(NumRelos, "Number of relocations applied");
static JIT *TheJIT = 0;


//===----------------------------------------------------------------------===//
// JIT lazy compilation code.
//
namespace {
  class JITResolverState {
  private:
    /// FunctionToStubMap - Keep track of the stub created for a particular
    /// function so that we can reuse them if necessary.
    std::map<Function*, void*> FunctionToStubMap;

    /// StubToFunctionMap - Keep track of the function that each stub
    /// corresponds to.
    std::map<void*, Function*> StubToFunctionMap;

    /// GlobalToLazyPtrMap - Keep track of the lazy pointer created for a
    /// particular GlobalVariable so that we can reuse them if necessary.
    std::map<GlobalValue*, void*> GlobalToLazyPtrMap;

  public:
    std::map<Function*, void*>& getFunctionToStubMap(const MutexGuard& locked) {
      assert(locked.holds(TheJIT->lock));
      return FunctionToStubMap;
    }

    std::map<void*, Function*>& getStubToFunctionMap(const MutexGuard& locked) {
      assert(locked.holds(TheJIT->lock));
      return StubToFunctionMap;
    }

    std::map<GlobalValue*, void*>&
    getGlobalToLazyPtrMap(const MutexGuard& locked) {
      assert(locked.holds(TheJIT->lock));
      return GlobalToLazyPtrMap;
    }
  };

  /// JITResolver - Keep track of, and resolve, call sites for functions that
  /// have not yet been compiled.
  class JITResolver {
    /// LazyResolverFn - The target lazy resolver function that we actually
    /// rewrite instructions to use.
    TargetJITInfo::LazyResolverFn LazyResolverFn;

    JITResolverState state;

    /// ExternalFnToStubMap - This is the equivalent of FunctionToStubMap for
    /// external functions.
    std::map<void*, void*> ExternalFnToStubMap;

    //map addresses to indexes in the GOT
    std::map<void*, unsigned> revGOTMap;
    unsigned nextGOTIndex;

    static JITResolver *TheJITResolver;
  public:
    explicit JITResolver(JIT &jit) : nextGOTIndex(0) {
      TheJIT = &jit;

      LazyResolverFn = jit.getJITInfo().getLazyResolverFunction(JITCompilerFn);
      assert(TheJITResolver == 0 && "Multiple JIT resolvers?");
      TheJITResolver = this;
    }
    
    ~JITResolver() {
      TheJITResolver = 0;
    }

    /// getFunctionStub - This returns a pointer to a function stub, creating
    /// one on demand as needed.
    void *getFunctionStub(Function *F);

    /// getExternalFunctionStub - Return a stub for the function at the
    /// specified address, created lazily on demand.
    void *getExternalFunctionStub(void *FnAddr);

    /// getGlobalValueLazyPtr - Return a lazy pointer containing the specified
    /// GV address.
    void *getGlobalValueLazyPtr(GlobalValue *V, void *GVAddress);

    /// AddCallbackAtLocation - If the target is capable of rewriting an
    /// instruction without the use of a stub, record the location of the use so
    /// we know which function is being used at the location.
    void *AddCallbackAtLocation(Function *F, void *Location) {
      MutexGuard locked(TheJIT->lock);
      /// Get the target-specific JIT resolver function.
      state.getStubToFunctionMap(locked)[Location] = F;
      return (void*)(intptr_t)LazyResolverFn;
    }

    /// getGOTIndexForAddress - Return a new or existing index in the GOT for
    /// an address.  This function only manages slots, it does not manage the
    /// contents of the slots or the memory associated with the GOT.
    unsigned getGOTIndexForAddr(void *addr);

    /// JITCompilerFn - This function is called to resolve a stub to a compiled
    /// address.  If the LLVM Function corresponding to the stub has not yet
    /// been compiled, this function compiles it first.
    static void *JITCompilerFn(void *Stub);
  };
}

JITResolver *JITResolver::TheJITResolver = 0;

/// getFunctionStub - This returns a pointer to a function stub, creating
/// one on demand as needed.
void *JITResolver::getFunctionStub(Function *F) {
  MutexGuard locked(TheJIT->lock);

  // If we already have a stub for this function, recycle it.
  void *&Stub = state.getFunctionToStubMap(locked)[F];
  if (Stub) return Stub;

  // Call the lazy resolver function unless we already KNOW it is an external
  // function, in which case we just skip the lazy resolution step.
  void *Actual = (void*)(intptr_t)LazyResolverFn;
  if (F->isDeclaration() && !F->hasNotBeenReadFromBitcode())
    Actual = TheJIT->getPointerToFunction(F);

  // Otherwise, codegen a new stub.  For now, the stub will call the lazy
  // resolver function.
  Stub = TheJIT->getJITInfo().emitFunctionStub(F, Actual,
                                               *TheJIT->getCodeEmitter());

  if (Actual != (void*)(intptr_t)LazyResolverFn) {
    // If we are getting the stub for an external function, we really want the
    // address of the stub in the GlobalAddressMap for the JIT, not the address
    // of the external function.
    TheJIT->updateGlobalMapping(F, Stub);
  }

  DOUT << "JIT: Stub emitted at [" << Stub << "] for function '"
       << F->getName() << "'\n";

  // Finally, keep track of the stub-to-Function mapping so that the
  // JITCompilerFn knows which function to compile!
  state.getStubToFunctionMap(locked)[Stub] = F;
  return Stub;
}

/// getGlobalValueLazyPtr - Return a lazy pointer containing the specified
/// GV address.
void *JITResolver::getGlobalValueLazyPtr(GlobalValue *GV, void *GVAddress) {
  MutexGuard locked(TheJIT->lock);

  // If we already have a stub for this global variable, recycle it.
  void *&LazyPtr = state.getGlobalToLazyPtrMap(locked)[GV];
  if (LazyPtr) return LazyPtr;

  // Otherwise, codegen a new lazy pointer.
  LazyPtr = TheJIT->getJITInfo().emitGlobalValueLazyPtr(GV, GVAddress,
                                                    *TheJIT->getCodeEmitter());

  DOUT << "JIT: Stub emitted at [" << LazyPtr << "] for GV '"
       << GV->getName() << "'\n";

  return LazyPtr;
}

/// getExternalFunctionStub - Return a stub for the function at the
/// specified address, created lazily on demand.
void *JITResolver::getExternalFunctionStub(void *FnAddr) {
  // If we already have a stub for this function, recycle it.
  void *&Stub = ExternalFnToStubMap[FnAddr];
  if (Stub) return Stub;

  Stub = TheJIT->getJITInfo().emitFunctionStub(0, FnAddr,
                                               *TheJIT->getCodeEmitter());

  DOUT << "JIT: Stub emitted at [" << Stub
       << "] for external function at '" << FnAddr << "'\n";
  return Stub;
}

unsigned JITResolver::getGOTIndexForAddr(void* addr) {
  unsigned idx = revGOTMap[addr];
  if (!idx) {
    idx = ++nextGOTIndex;
    revGOTMap[addr] = idx;
    DOUT << "Adding GOT entry " << idx << " for addr " << addr << "\n";
  }
  return idx;
}

/// JITCompilerFn - This function is called when a lazy compilation stub has
/// been entered.  It looks up which function this stub corresponds to, compiles
/// it if necessary, then returns the resultant function pointer.
void *JITResolver::JITCompilerFn(void *Stub) {
  JITResolver &JR = *TheJITResolver;

  MutexGuard locked(TheJIT->lock);

  // The address given to us for the stub may not be exactly right, it might be
  // a little bit after the stub.  As such, use upper_bound to find it.
  std::map<void*, Function*>::iterator I =
    JR.state.getStubToFunctionMap(locked).upper_bound(Stub);
  assert(I != JR.state.getStubToFunctionMap(locked).begin() &&
         "This is not a known stub!");
  Function *F = (--I)->second;

  // If we have already code generated the function, just return the address.
  void *Result = TheJIT->getPointerToGlobalIfAvailable(F);
  
  if (!Result) {
    // Otherwise we don't have it, do lazy compilation now.
    
    // If lazy compilation is disabled, emit a useful error message and abort.
    if (TheJIT->isLazyCompilationDisabled()) {
      cerr << "LLVM JIT requested to do lazy compilation of function '"
      << F->getName() << "' when lazy compiles are disabled!\n";
      abort();
    }
  
    // We might like to remove the stub from the StubToFunction map.
    // We can't do that! Multiple threads could be stuck, waiting to acquire the
    // lock above. As soon as the 1st function finishes compiling the function,
    // the next one will be released, and needs to be able to find the function
    // it needs to call.
    //JR.state.getStubToFunctionMap(locked).erase(I);

    DOUT << "JIT: Lazily resolving function '" << F->getName()
         << "' In stub ptr = " << Stub << " actual ptr = "
         << I->first << "\n";

    Result = TheJIT->getPointerToFunction(F);
  }

  // We don't need to reuse this stub in the future, as F is now compiled.
  JR.state.getFunctionToStubMap(locked).erase(F);

  // FIXME: We could rewrite all references to this stub if we knew them.

  // What we will do is set the compiled function address to map to the
  // same GOT entry as the stub so that later clients may update the GOT
  // if they see it still using the stub address.
  // Note: this is done so the Resolver doesn't have to manage GOT memory
  // Do this without allocating map space if the target isn't using a GOT
  if(JR.revGOTMap.find(Stub) != JR.revGOTMap.end())
    JR.revGOTMap[Result] = JR.revGOTMap[Stub];

  return Result;
}

//===----------------------------------------------------------------------===//
// Function Index Support

// On MacOS we generate an index of currently JIT'd functions so that
// performance tools can determine a symbol name and accurate code range for a
// PC value.  Because performance tools are generally asynchronous, the code
// below is written with the hope that it could be interrupted at any time and
// have useful answers.  However, we don't go crazy with atomic operations, we
// just do a "reasonable effort".
#ifdef __APPLE__ 
#define ENABLE_JIT_SYMBOL_TABLE 0
#endif

/// JitSymbolEntry - Each function that is JIT compiled results in one of these
/// being added to an array of symbols.  This indicates the name of the function
/// as well as the address range it occupies.  This allows the client to map
/// from a PC value to the name of the function.
struct JitSymbolEntry {
  const char *FnName;   // FnName - a strdup'd string.
  void *FnStart;
  intptr_t FnSize;
};


struct JitSymbolTable {
  /// NextPtr - This forms a linked list of JitSymbolTable entries.  This
  /// pointer is not used right now, but might be used in the future.  Consider
  /// it reserved for future use.
  JitSymbolTable *NextPtr;
  
  /// Symbols - This is an array of JitSymbolEntry entries.  Only the first
  /// 'NumSymbols' symbols are valid.
  JitSymbolEntry *Symbols;
  
  /// NumSymbols - This indicates the number entries in the Symbols array that
  /// are valid.
  unsigned NumSymbols;
  
  /// NumAllocated - This indicates the amount of space we have in the Symbols
  /// array.  This is a private field that should not be read by external tools.
  unsigned NumAllocated;
};

#if ENABLE_JIT_SYMBOL_TABLE 
JitSymbolTable *__jitSymbolTable;
#endif

static void AddFunctionToSymbolTable(const char *FnName, 
                                     void *FnStart, intptr_t FnSize) {
  assert(FnName != 0 && FnStart != 0 && "Bad symbol to add");
  JitSymbolTable **SymTabPtrPtr = 0;
#if !ENABLE_JIT_SYMBOL_TABLE
  return;
#else
  SymTabPtrPtr = &__jitSymbolTable;
#endif
  
  // If this is the first entry in the symbol table, add the JitSymbolTable
  // index.
  if (*SymTabPtrPtr == 0) {
    JitSymbolTable *New = new JitSymbolTable();
    New->NextPtr = 0;
    New->Symbols = 0;
    New->NumSymbols = 0;
    New->NumAllocated = 0;
    *SymTabPtrPtr = New;
  }
  
  JitSymbolTable *SymTabPtr = *SymTabPtrPtr;
  
  // If we have space in the table, reallocate the table.
  if (SymTabPtr->NumSymbols >= SymTabPtr->NumAllocated) {
    // If we don't have space, reallocate the table.
    unsigned NewSize = std::max(64U, SymTabPtr->NumAllocated*2);
    JitSymbolEntry *NewSymbols = new JitSymbolEntry[NewSize];
    JitSymbolEntry *OldSymbols = SymTabPtr->Symbols;
    
    // Copy the old entries over.
    memcpy(NewSymbols, OldSymbols,
           SymTabPtr->NumSymbols*sizeof(OldSymbols[0]));
    
    // Swap the new symbols in, delete the old ones.
    SymTabPtr->Symbols = NewSymbols;
    SymTabPtr->NumAllocated = NewSize;
    delete [] OldSymbols;
  }
  
  // Otherwise, we have enough space, just tack it onto the end of the array.
  JitSymbolEntry &Entry = SymTabPtr->Symbols[SymTabPtr->NumSymbols];
  Entry.FnName = strdup(FnName);
  Entry.FnStart = FnStart;
  Entry.FnSize = FnSize;
  ++SymTabPtr->NumSymbols;
}

static void RemoveFunctionFromSymbolTable(void *FnStart) {
  assert(FnStart && "Invalid function pointer");
  JitSymbolTable **SymTabPtrPtr = 0;
#if !ENABLE_JIT_SYMBOL_TABLE
  return;
#else
  SymTabPtrPtr = &__jitSymbolTable;
#endif
  
  JitSymbolTable *SymTabPtr = *SymTabPtrPtr;
  JitSymbolEntry *Symbols = SymTabPtr->Symbols;
  
  // Scan the table to find its index.  The table is not sorted, so do a linear
  // scan.
  unsigned Index;
  for (Index = 0; Symbols[Index].FnStart != FnStart; ++Index)
    assert(Index != SymTabPtr->NumSymbols && "Didn't find function!");
  
  // Once we have an index, we know to nuke this entry, overwrite it with the
  // entry at the end of the array, making the last entry redundant.
  const char *OldName = Symbols[Index].FnName;
  Symbols[Index] = Symbols[SymTabPtr->NumSymbols-1];
  free((void*)OldName);
  
  // Drop the number of symbols in the table.
  --SymTabPtr->NumSymbols;

  // Finally, if we deleted the final symbol, deallocate the table itself.
  if (SymTabPtr->NumSymbols != 0) 
    return;
  
  *SymTabPtrPtr = 0;
  delete [] Symbols;
  delete SymTabPtr;
}

//===----------------------------------------------------------------------===//
// JITEmitter code.
//
namespace {
  /// JITEmitter - The JIT implementation of the MachineCodeEmitter, which is
  /// used to output functions to memory for execution.
  class JITEmitter : public MachineCodeEmitter {
    JITMemoryManager *MemMgr;

    // When outputting a function stub in the context of some other function, we
    // save BufferBegin/BufferEnd/CurBufferPtr here.
    unsigned char *SavedBufferBegin, *SavedBufferEnd, *SavedCurBufferPtr;

    /// Relocations - These are the relocations that the function needs, as
    /// emitted.
    std::vector<MachineRelocation> Relocations;
    
    /// MBBLocations - This vector is a mapping from MBB ID's to their address.
    /// It is filled in by the StartMachineBasicBlock callback and queried by
    /// the getMachineBasicBlockAddress callback.
    std::vector<intptr_t> MBBLocations;

    /// ConstantPool - The constant pool for the current function.
    ///
    MachineConstantPool *ConstantPool;

    /// ConstantPoolBase - A pointer to the first entry in the constant pool.
    ///
    void *ConstantPoolBase;

    /// JumpTable - The jump tables for the current function.
    ///
    MachineJumpTableInfo *JumpTable;
    
    /// JumpTableBase - A pointer to the first entry in the jump table.
    ///
    void *JumpTableBase;

    /// Resolver - This contains info about the currently resolved functions.
    JITResolver Resolver;
    
    /// DE - The dwarf emitter for the jit.
    JITDwarfEmitter *DE;

    /// LabelLocations - This vector is a mapping from Label ID's to their 
    /// address.
    std::vector<intptr_t> LabelLocations;

    /// MMI - Machine module info for exception informations
    MachineModuleInfo* MMI;

  public:
    JITEmitter(JIT &jit, JITMemoryManager *JMM) : Resolver(jit) {
      MemMgr = JMM ? JMM : JITMemoryManager::CreateDefaultMemManager();
      if (jit.getJITInfo().needsGOT()) {
        MemMgr->AllocateGOT();
        DOUT << "JIT is managing a GOT\n";
      }

      if (ExceptionHandling) DE = new JITDwarfEmitter(jit);
    }
    ~JITEmitter() { 
      delete MemMgr;
      if (ExceptionHandling) delete DE;
    }
    
    JITResolver &getJITResolver() { return Resolver; }

    virtual void startFunction(MachineFunction &F);
    virtual bool finishFunction(MachineFunction &F);
    
    void emitConstantPool(MachineConstantPool *MCP);
    void initJumpTableInfo(MachineJumpTableInfo *MJTI);
    void emitJumpTableInfo(MachineJumpTableInfo *MJTI);
    
    virtual void startFunctionStub(const GlobalValue* F, unsigned StubSize,
                                   unsigned Alignment = 1);
    virtual void* finishFunctionStub(const GlobalValue *F);

    virtual void addRelocation(const MachineRelocation &MR) {
      Relocations.push_back(MR);
    }
    
    virtual void StartMachineBasicBlock(MachineBasicBlock *MBB) {
      if (MBBLocations.size() <= (unsigned)MBB->getNumber())
        MBBLocations.resize((MBB->getNumber()+1)*2);
      MBBLocations[MBB->getNumber()] = getCurrentPCValue();
    }

    virtual intptr_t getConstantPoolEntryAddress(unsigned Entry) const;
    virtual intptr_t getJumpTableEntryAddress(unsigned Entry) const;

    virtual intptr_t getMachineBasicBlockAddress(MachineBasicBlock *MBB) const {
      assert(MBBLocations.size() > (unsigned)MBB->getNumber() && 
             MBBLocations[MBB->getNumber()] && "MBB not emitted!");
      return MBBLocations[MBB->getNumber()];
    }

    /// deallocateMemForFunction - Deallocate all memory for the specified
    /// function body.
    void deallocateMemForFunction(Function *F) {
      MemMgr->deallocateMemForFunction(F);
    }
    
    virtual void emitLabel(uint64_t LabelID) {
      if (LabelLocations.size() <= LabelID)
        LabelLocations.resize((LabelID+1)*2);
      LabelLocations[LabelID] = getCurrentPCValue();
    }

    virtual intptr_t getLabelAddress(uint64_t LabelID) const {
      assert(LabelLocations.size() > (unsigned)LabelID && 
             LabelLocations[LabelID] && "Label not emitted!");
      return LabelLocations[LabelID];
    }
 
    virtual void setModuleInfo(MachineModuleInfo* Info) {
      MMI = Info;
      if (ExceptionHandling) DE->setModuleInfo(Info);
    }

  private:
    void *getPointerToGlobal(GlobalValue *GV, void *Reference, bool NoNeedStub);
    void *getPointerToGVLazyPtr(GlobalValue *V, void *Reference,
                                bool NoNeedStub);
  };
}

void *JITEmitter::getPointerToGlobal(GlobalValue *V, void *Reference,
                                     bool DoesntNeedStub) {
  if (GlobalVariable *GV = dyn_cast<GlobalVariable>(V)) {
    /// FIXME: If we straightened things out, this could actually emit the
    /// global immediately instead of queuing it for codegen later!
    return TheJIT->getOrEmitGlobalVariable(GV);
  }
  if (GlobalAlias *GA = dyn_cast<GlobalAlias>(V))
    return TheJIT->getPointerToGlobal(GA->resolveAliasedGlobal());

  // If we have already compiled the function, return a pointer to its body.
  Function *F = cast<Function>(V);
  void *ResultPtr = TheJIT->getPointerToGlobalIfAvailable(F);
  if (ResultPtr) return ResultPtr;

  if (F->isDeclaration() && !F->hasNotBeenReadFromBitcode()) {
    // If this is an external function pointer, we can force the JIT to
    // 'compile' it, which really just adds it to the map.
    if (DoesntNeedStub)
      return TheJIT->getPointerToFunction(F);

    return Resolver.getFunctionStub(F);
  }

  // Okay, the function has not been compiled yet, if the target callback
  // mechanism is capable of rewriting the instruction directly, prefer to do
  // that instead of emitting a stub.
  if (DoesntNeedStub)
    return Resolver.AddCallbackAtLocation(F, Reference);

  // Otherwise, we have to emit a lazy resolving stub.
  return Resolver.getFunctionStub(F);
}

void *JITEmitter::getPointerToGVLazyPtr(GlobalValue *V, void *Reference,
                                        bool DoesntNeedStub) {
  // Make sure GV is emitted first.
  // FIXME: For now, if the GV is an external function we force the JIT to
  // compile it so the lazy pointer will contain the fully resolved address.
  void *GVAddress = getPointerToGlobal(V, Reference, true);
  return Resolver.getGlobalValueLazyPtr(V, GVAddress);
}

static unsigned GetConstantPoolSizeInBytes(MachineConstantPool *MCP) {
  const std::vector<MachineConstantPoolEntry> &Constants = MCP->getConstants();
  if (Constants.empty()) return 0;

  MachineConstantPoolEntry CPE = Constants.back();
  unsigned Size = CPE.Offset;
  const Type *Ty = CPE.isMachineConstantPoolEntry()
    ? CPE.Val.MachineCPVal->getType() : CPE.Val.ConstVal->getType();
  Size += TheJIT->getTargetData()->getABITypeSize(Ty);
  return Size;
}

static unsigned GetJumpTableSizeInBytes(MachineJumpTableInfo *MJTI) {
  const std::vector<MachineJumpTableEntry> &JT = MJTI->getJumpTables();
  if (JT.empty()) return 0;
  
  unsigned NumEntries = 0;
  for (unsigned i = 0, e = JT.size(); i != e; ++i)
    NumEntries += JT[i].MBBs.size();

  unsigned EntrySize = MJTI->getEntrySize();

  return NumEntries * EntrySize;
}

static uintptr_t RoundUpToAlign(uintptr_t Size, unsigned Alignment) {
  if (Alignment == 0) Alignment = 1;
  // Since we do not know where the buffer will be allocated, be pessimistic. 
  return Size + Alignment;
}

void JITEmitter::startFunction(MachineFunction &F) {
  uintptr_t ActualSize = 0;
  if (MemMgr->NeedsExactSize()) {
    const TargetInstrInfo* TII = F.getTarget().getInstrInfo();
    MachineJumpTableInfo *MJTI = F.getJumpTableInfo();
    MachineConstantPool *MCP = F.getConstantPool();
    
    // Ensure the constant pool/jump table info is at least 4-byte aligned.
    ActualSize = RoundUpToAlign(ActualSize, 16);
    
    // Add the alignment of the constant pool
    ActualSize = RoundUpToAlign(ActualSize, 
                                1 << MCP->getConstantPoolAlignment());

    // Add the constant pool size
    ActualSize += GetConstantPoolSizeInBytes(MCP);

    // Add the aligment of the jump table info
    ActualSize = RoundUpToAlign(ActualSize, MJTI->getAlignment());

    // Add the jump table size
    ActualSize += GetJumpTableSizeInBytes(MJTI);
    
    // Add the alignment for the function
    ActualSize = RoundUpToAlign(ActualSize,
                                std::max(F.getFunction()->getAlignment(), 8U));

    // Add the function size
    ActualSize += TII->GetFunctionSizeInBytes(F);
  }

  BufferBegin = CurBufferPtr = MemMgr->startFunctionBody(F.getFunction(),
                                                         ActualSize);
  BufferEnd = BufferBegin+ActualSize;
  
  // Ensure the constant pool/jump table info is at least 4-byte aligned.
  emitAlignment(16);

  emitConstantPool(F.getConstantPool());
  initJumpTableInfo(F.getJumpTableInfo());

  // About to start emitting the machine code for the function.
  emitAlignment(std::max(F.getFunction()->getAlignment(), 8U));
  TheJIT->updateGlobalMapping(F.getFunction(), CurBufferPtr);

  MBBLocations.clear();
}

bool JITEmitter::finishFunction(MachineFunction &F) {
  if (CurBufferPtr == BufferEnd) {
    // FIXME: Allocate more space, then try again.
    cerr << "JIT: Ran out of space for generated machine code!\n";
    abort();
  }
  
  emitJumpTableInfo(F.getJumpTableInfo());
  
  // FnStart is the start of the text, not the start of the constant pool and
  // other per-function data.
  unsigned char *FnStart =
    (unsigned char *)TheJIT->getPointerToGlobalIfAvailable(F.getFunction());
  unsigned char *FnEnd   = CurBufferPtr;
  
  MemMgr->endFunctionBody(F.getFunction(), BufferBegin, FnEnd);
  NumBytes += FnEnd-FnStart;

  if (!Relocations.empty()) {
    NumRelos += Relocations.size();

    // Resolve the relocations to concrete pointers.
    for (unsigned i = 0, e = Relocations.size(); i != e; ++i) {
      MachineRelocation &MR = Relocations[i];
      void *ResultPtr;
      if (MR.isString()) {
        ResultPtr = TheJIT->getPointerToNamedFunction(MR.getString());

        // If the target REALLY wants a stub for this function, emit it now.
        if (!MR.doesntNeedStub())
          ResultPtr = Resolver.getExternalFunctionStub(ResultPtr);
      } else if (MR.isGlobalValue()) {
        ResultPtr = getPointerToGlobal(MR.getGlobalValue(),
                                       BufferBegin+MR.getMachineCodeOffset(),
                                       MR.doesntNeedStub());
      } else if (MR.isGlobalValueLazyPtr()) {
        ResultPtr = getPointerToGVLazyPtr(MR.getGlobalValue(),
                                          BufferBegin+MR.getMachineCodeOffset(),
                                          MR.doesntNeedStub());
      } else if (MR.isBasicBlock()) {
        ResultPtr = (void*)getMachineBasicBlockAddress(MR.getBasicBlock());
      } else if (MR.isConstantPoolIndex()) {
        ResultPtr=(void*)getConstantPoolEntryAddress(MR.getConstantPoolIndex());
      } else {
        assert(MR.isJumpTableIndex());
        ResultPtr=(void*)getJumpTableEntryAddress(MR.getJumpTableIndex());
      }

      MR.setResultPointer(ResultPtr);

      // if we are managing the GOT and the relocation wants an index,
      // give it one
      if (MR.isGOTRelative() && MemMgr->isManagingGOT()) {
        unsigned idx = Resolver.getGOTIndexForAddr(ResultPtr);
        MR.setGOTIndex(idx);
        if (((void**)MemMgr->getGOTBase())[idx] != ResultPtr) {
          DOUT << "GOT was out of date for " << ResultPtr
               << " pointing at " << ((void**)MemMgr->getGOTBase())[idx]
               << "\n";
          ((void**)MemMgr->getGOTBase())[idx] = ResultPtr;
        }
      }
    }

    TheJIT->getJITInfo().relocate(BufferBegin, &Relocations[0],
                                  Relocations.size(), MemMgr->getGOTBase());
  }

  // Update the GOT entry for F to point to the new code.
  if (MemMgr->isManagingGOT()) {
    unsigned idx = Resolver.getGOTIndexForAddr((void*)BufferBegin);
    if (((void**)MemMgr->getGOTBase())[idx] != (void*)BufferBegin) {
      DOUT << "GOT was out of date for " << (void*)BufferBegin
           << " pointing at " << ((void**)MemMgr->getGOTBase())[idx] << "\n";
      ((void**)MemMgr->getGOTBase())[idx] = (void*)BufferBegin;
    }
  }

  // Invalidate the icache if necessary.
  sys::Memory::InvalidateInstructionCache(FnStart, FnEnd-FnStart);
  
  // Add it to the JIT symbol table if the host wants it.
  AddFunctionToSymbolTable(F.getFunction()->getNameStart(),
                           FnStart, FnEnd-FnStart);

  DOUT << "JIT: Finished CodeGen of [" << (void*)FnStart
       << "] Function: " << F.getFunction()->getName()
       << ": " << (FnEnd-FnStart) << " bytes of text, "
       << Relocations.size() << " relocations\n";
  Relocations.clear();

#ifndef NDEBUG
  if (sys::hasDisassembler())
    DOUT << "Disassembled code:\n"
         << sys::disassembleBuffer(FnStart, FnEnd-FnStart, (uintptr_t)FnStart);
#endif
  if (ExceptionHandling) {
    uintptr_t ActualSize = 0;
    SavedBufferBegin = BufferBegin;
    SavedBufferEnd = BufferEnd;
    SavedCurBufferPtr = CurBufferPtr;
    
    if (MemMgr->NeedsExactSize()) {
      ActualSize = DE->GetDwarfTableSizeInBytes(F, *this, FnStart, FnEnd);
    }

    BufferBegin = CurBufferPtr = MemMgr->startExceptionTable(F.getFunction(),
                                                             ActualSize);
    BufferEnd = BufferBegin+ActualSize;
    unsigned char* FrameRegister = DE->EmitDwarfTable(F, *this, FnStart, FnEnd);
    MemMgr->endExceptionTable(F.getFunction(), BufferBegin, CurBufferPtr,
                              FrameRegister);
    BufferBegin = SavedBufferBegin;
    BufferEnd = SavedBufferEnd;
    CurBufferPtr = SavedCurBufferPtr;

    TheJIT->RegisterTable(FrameRegister);
  }
  MMI->EndFunction();
 
  return false;
}

void JITEmitter::emitConstantPool(MachineConstantPool *MCP) {
  const std::vector<MachineConstantPoolEntry> &Constants = MCP->getConstants();
  if (Constants.empty()) return;

  MachineConstantPoolEntry CPE = Constants.back();
  unsigned Size = CPE.Offset;
  const Type *Ty = CPE.isMachineConstantPoolEntry()
    ? CPE.Val.MachineCPVal->getType() : CPE.Val.ConstVal->getType();
  Size += TheJIT->getTargetData()->getABITypeSize(Ty);

  unsigned Align = 1 << MCP->getConstantPoolAlignment();
  ConstantPoolBase = allocateSpace(Size, Align);
  ConstantPool = MCP;

  if (ConstantPoolBase == 0) return;  // Buffer overflow.

  DOUT << "JIT: Emitted constant pool at [" << ConstantPoolBase
       << "] (size: " << Size << ", alignment: " << Align << ")\n";

  // Initialize the memory for all of the constant pool entries.
  for (unsigned i = 0, e = Constants.size(); i != e; ++i) {
    void *CAddr = (char*)ConstantPoolBase+Constants[i].Offset;
    if (Constants[i].isMachineConstantPoolEntry()) {
      // FIXME: add support to lower machine constant pool values into bytes!
      cerr << "Initialize memory with machine specific constant pool entry"
           << " has not been implemented!\n";
      abort();
    }
    TheJIT->InitializeMemory(Constants[i].Val.ConstVal, CAddr);
    DOUT << "JIT:   CP" << i << " at [" << CAddr << "]\n";
  }
}

void JITEmitter::initJumpTableInfo(MachineJumpTableInfo *MJTI) {
  const std::vector<MachineJumpTableEntry> &JT = MJTI->getJumpTables();
  if (JT.empty()) return;
  
  unsigned NumEntries = 0;
  for (unsigned i = 0, e = JT.size(); i != e; ++i)
    NumEntries += JT[i].MBBs.size();

  unsigned EntrySize = MJTI->getEntrySize();

  // Just allocate space for all the jump tables now.  We will fix up the actual
  // MBB entries in the tables after we emit the code for each block, since then
  // we will know the final locations of the MBBs in memory.
  JumpTable = MJTI;
  JumpTableBase = allocateSpace(NumEntries * EntrySize, MJTI->getAlignment());
}

void JITEmitter::emitJumpTableInfo(MachineJumpTableInfo *MJTI) {
  const std::vector<MachineJumpTableEntry> &JT = MJTI->getJumpTables();
  if (JT.empty() || JumpTableBase == 0) return;
  
  if (TargetMachine::getRelocationModel() == Reloc::PIC_) {
    assert(MJTI->getEntrySize() == 4 && "Cross JIT'ing?");
    // For each jump table, place the offset from the beginning of the table
    // to the target address.
    int *SlotPtr = (int*)JumpTableBase;

    for (unsigned i = 0, e = JT.size(); i != e; ++i) {
      const std::vector<MachineBasicBlock*> &MBBs = JT[i].MBBs;
      // Store the offset of the basic block for this jump table slot in the
      // memory we allocated for the jump table in 'initJumpTableInfo'
      intptr_t Base = (intptr_t)SlotPtr;
      for (unsigned mi = 0, me = MBBs.size(); mi != me; ++mi) {
        intptr_t MBBAddr = getMachineBasicBlockAddress(MBBs[mi]);
        *SlotPtr++ = TheJIT->getJITInfo().getPICJumpTableEntry(MBBAddr, Base);
      }
    }
  } else {
    assert(MJTI->getEntrySize() == sizeof(void*) && "Cross JIT'ing?");
    
    // For each jump table, map each target in the jump table to the address of 
    // an emitted MachineBasicBlock.
    intptr_t *SlotPtr = (intptr_t*)JumpTableBase;

    for (unsigned i = 0, e = JT.size(); i != e; ++i) {
      const std::vector<MachineBasicBlock*> &MBBs = JT[i].MBBs;
      // Store the address of the basic block for this jump table slot in the
      // memory we allocated for the jump table in 'initJumpTableInfo'
      for (unsigned mi = 0, me = MBBs.size(); mi != me; ++mi)
        *SlotPtr++ = getMachineBasicBlockAddress(MBBs[mi]);
    }
  }
}

void JITEmitter::startFunctionStub(const GlobalValue* F, unsigned StubSize,
                                   unsigned Alignment) {
  SavedBufferBegin = BufferBegin;
  SavedBufferEnd = BufferEnd;
  SavedCurBufferPtr = CurBufferPtr;
  
  BufferBegin = CurBufferPtr = MemMgr->allocateStub(F, StubSize, Alignment);
  BufferEnd = BufferBegin+StubSize+1;
}

void *JITEmitter::finishFunctionStub(const GlobalValue* F) {
  NumBytes += getCurrentPCOffset();
  std::swap(SavedBufferBegin, BufferBegin);
  BufferEnd = SavedBufferEnd;
  CurBufferPtr = SavedCurBufferPtr;
  return SavedBufferBegin;
}

// getConstantPoolEntryAddress - Return the address of the 'ConstantNum' entry
// in the constant pool that was last emitted with the 'emitConstantPool'
// method.
//
intptr_t JITEmitter::getConstantPoolEntryAddress(unsigned ConstantNum) const {
  assert(ConstantNum < ConstantPool->getConstants().size() &&
         "Invalid ConstantPoolIndex!");
  return (intptr_t)ConstantPoolBase +
         ConstantPool->getConstants()[ConstantNum].Offset;
}

// getJumpTableEntryAddress - Return the address of the JumpTable with index
// 'Index' in the jumpp table that was last initialized with 'initJumpTableInfo'
//
intptr_t JITEmitter::getJumpTableEntryAddress(unsigned Index) const {
  const std::vector<MachineJumpTableEntry> &JT = JumpTable->getJumpTables();
  assert(Index < JT.size() && "Invalid jump table index!");
  
  unsigned Offset = 0;
  unsigned EntrySize = JumpTable->getEntrySize();
  
  for (unsigned i = 0; i < Index; ++i)
    Offset += JT[i].MBBs.size();
  
   Offset *= EntrySize;
  
  return (intptr_t)((char *)JumpTableBase + Offset);
}

//===----------------------------------------------------------------------===//
//  Public interface to this file
//===----------------------------------------------------------------------===//

MachineCodeEmitter *JIT::createEmitter(JIT &jit, JITMemoryManager *JMM) {
  return new JITEmitter(jit, JMM);
}

// getPointerToNamedFunction - This function is used as a global wrapper to
// JIT::getPointerToNamedFunction for the purpose of resolving symbols when
// bugpoint is debugging the JIT. In that scenario, we are loading an .so and
// need to resolve function(s) that are being mis-codegenerated, so we need to
// resolve their addresses at runtime, and this is the way to do it.
extern "C" {
  void *getPointerToNamedFunction(const char *Name) {
    if (Function *F = TheJIT->FindFunctionNamed(Name))
      return TheJIT->getPointerToFunction(F);
    return TheJIT->getPointerToNamedFunction(Name);
  }
}

// getPointerToFunctionOrStub - If the specified function has been
// code-gen'd, return a pointer to the function.  If not, compile it, or use
// a stub to implement lazy compilation if available.
//
void *JIT::getPointerToFunctionOrStub(Function *F) {
  // If we have already code generated the function, just return the address.
  if (void *Addr = getPointerToGlobalIfAvailable(F))
    return Addr;
  
  // Get a stub if the target supports it.
  assert(dynamic_cast<JITEmitter*>(MCE) && "Unexpected MCE?");
  JITEmitter *JE = static_cast<JITEmitter*>(getCodeEmitter());
  return JE->getJITResolver().getFunctionStub(F);
}

/// freeMachineCodeForFunction - release machine code memory for given Function.
///
void JIT::freeMachineCodeForFunction(Function *F) {
  
  // Delete translation for this from the ExecutionEngine, so it will get
  // retranslated next time it is used.
  void *OldPtr = updateGlobalMapping(F, 0);

  if (OldPtr)
    RemoveFunctionFromSymbolTable(OldPtr);

  // Free the actual memory for the function body and related stuff.
  assert(dynamic_cast<JITEmitter*>(MCE) && "Unexpected MCE?");
  static_cast<JITEmitter*>(MCE)->deallocateMemForFunction(F);
}

