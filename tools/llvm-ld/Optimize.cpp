//===- Optimize.cpp - Optimize a complete program -------------------------===//
// 
//                     The LLVM Compiler Infrastructure
//
// This file was developed by Reid Spencer and is distributed under the 
// University of Illinois Open Source License. See LICENSE.TXT for details.
// 
//===----------------------------------------------------------------------===//
//
// This file implements all optimization of the linked module for llvm-ld.
//
//===----------------------------------------------------------------------===//

#include "llvm/Module.h"
#include "llvm/PassManager.h"
#include "llvm/Analysis/LoadValueNumbering.h"
#include "llvm/Analysis/Passes.h"
#include "llvm/Analysis/Verifier.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/System/DynamicLibrary.h"
#include "llvm/Target/TargetData.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Transforms/Scalar.h"
using namespace llvm;

// Optimization Options

enum OptimizationLevels {
  OPT_FAST_COMPILE         = 1,
  OPT_SIMPLE               = 2,
  OPT_AGGRESSIVE           = 3,
  OPT_LINK_TIME            = 4,
  OPT_AGGRESSIVE_LINK_TIME = 5
};

static cl::opt<OptimizationLevels> OptLevel(
  cl::desc("Choose level of optimization to apply:"),
  cl::init(OPT_FAST_COMPILE), cl::values(
    clEnumValN(OPT_FAST_COMPILE,"O0",
      "An alias for the -O1 option."),
    clEnumValN(OPT_FAST_COMPILE,"O1",
      "Optimize for linking speed, not execution speed."),
    clEnumValN(OPT_SIMPLE,"O2",
      "Perform only required/minimal optimizations"),
    clEnumValN(OPT_AGGRESSIVE,"O3",
      "An alias for the -O2 option."),
    clEnumValN(OPT_LINK_TIME,"O4",
      "Perform standard link time optimizations"),
    clEnumValN(OPT_AGGRESSIVE_LINK_TIME,"O5",
      "Perform aggressive link time optimizations"),
    clEnumValEnd
  )
);

static cl::opt<bool> DisableInline("disable-inlining", 
  cl::desc("Do not run the inliner pass"));

static cl::opt<bool>
DisableOptimizations("disable-opt",
  cl::desc("Do not run any optimization passes"));

static cl::opt<bool> DisableInternalize("disable-internalize",
  cl::desc("Do not mark all symbols as internal"));

static cl::opt<bool> Verify("verify", 
  cl::desc("Verify intermediate results of all passes"));

static cl::opt<bool> Strip("s", 
  cl::desc("Strip symbol info from executable"));

static cl::alias ExportDynamic("export-dynamic", 
  cl::aliasopt(DisableInternalize),
  cl::desc("Alias for -disable-internalize"));

static cl::list<std::string> LoadableModules("load",
  cl::value_desc("path to loadable optimization module"),
  cl::desc("Load an optimization module and run it"));

// A utility function that adds a pass to the pass manager but will also add
// a verifier pass after if we're supposed to verify.
static inline void addPass(PassManager &PM, Pass *P) {
  // Add the pass to the pass manager...
  PM.add(P);
  
  // If we are verifying all of the intermediate steps, add the verifier...
  if (Verify) 
    PM.add(createVerifierPass());
}

namespace llvm {

/// Optimize - Perform link time optimizations. This will run the scalar 
/// optimizations, any loaded plugin-optimization modules, and then the 
/// inter-procedural optimizations if applicable.
void Optimize(Module* M) {

  // Instantiate the pass manager to organize the passes.
  PassManager Passes;

  // If we're verifying, start off with a verification pass.
  if (Verify) 
    Passes.add(createVerifierPass());

  // Add an appropriate TargetData instance for this module...
  addPass(Passes, new TargetData("gccld", M));

  // Often if the programmer does not specify proper prototypes for the
  // functions they are calling, they end up calling a vararg version of the
  // function that does not get a body filled in (the real function has typed
  // arguments).  This pass merges the two functions.
  addPass(Passes, createFunctionResolvingPass());

  if (!DisableOptimizations) {
    if (!DisableInternalize) {
      // Now that composite has been compiled, scan through the module, looking
      // for a main function.  If main is defined, mark all other functions
      // internal.
      addPass(Passes, createInternalizePass());
    }

    // Now that we internalized some globals, see if we can hack on them!
    addPass(Passes, createGlobalOptimizerPass());

    // Linking modules together can lead to duplicated global constants, only
    // keep one copy of each constant...
    addPass(Passes, createConstantMergePass());

    // If the -s command line option was specified, strip the symbols out of the
    // resulting program to make it smaller.  -s is a GLD option that we are
    // supporting.
    if (Strip)
      addPass(Passes, createStripSymbolsPass());

    // Propagate constants at call sites into the functions they call.
    addPass(Passes, createIPConstantPropagationPass());

    // Remove unused arguments from functions...
    addPass(Passes, createDeadArgEliminationPass());

    if (!DisableInline)
      addPass(Passes, createFunctionInliningPass()); // Inline small functions

    addPass(Passes, createPruneEHPass());            // Remove dead EH info
    addPass(Passes, createGlobalDCEPass());          // Remove dead functions

    // If we didn't decide to inline a function, check to see if we can
    // transform it to pass arguments by value instead of by reference.
    addPass(Passes, createArgumentPromotionPass());

    // The IPO passes may leave cruft around.  Clean up after them.
    addPass(Passes, createInstructionCombiningPass());

    addPass(Passes, createScalarReplAggregatesPass()); // Break up allocas

    // Run a few AA driven optimizations here and now, to cleanup the code.
    addPass(Passes, createGlobalsModRefPass());      // IP alias analysis

    addPass(Passes, createLICMPass());               // Hoist loop invariants
    addPass(Passes, createLoadValueNumberingPass()); // GVN for load instrs
    addPass(Passes, createGCSEPass());               // Remove common subexprs
    addPass(Passes, createDeadStoreEliminationPass()); // Nuke dead stores

    // Cleanup and simplify the code after the scalar optimizations.
    addPass(Passes, createInstructionCombiningPass());

    // Now that we have optimized the program, discard unreachable functions...
    addPass(Passes, createGlobalDCEPass());
  }

  std::vector<std::string> plugins = LoadableModules;
  for (std::vector<std::string>::iterator I = plugins.begin(), 
      E = plugins.end(); I != E; ++I) {
    sys::DynamicLibrary dll(I->c_str());
    typedef void (*OptimizeFunc)(PassManager&,int);
    OptimizeFunc OF = OptimizeFunc(
        dll.GetAddressOfSymbol("RunOptimizations"));
    if (OF == 0) {
      throw std::string("Optimization Module '") + *I + 
        "' is missing the RunOptimizations symbol";
    }
    (*OF)(Passes,OptLevel);
  }

  // Make sure everything is still good.
  Passes.add(createVerifierPass());

  // Run our queue of passes all at once now, efficiently.
  Passes.run(*M);
}

}
