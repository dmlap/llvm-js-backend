//===-- SparcRegClassInfo.h - Register class def'ns for Sparc ----*- C++ -*--=//
//
//  This file defines the register classes used by the Sparc target description.
//
//===----------------------------------------------------------------------===//

#ifndef SPARC_REG_CLASS_INFO_H
#define SPARC_REG_CLASS_INFO_H

#include "llvm/Target/MachineRegInfo.h"
#include "llvm/CodeGen/IGNode.h"

//-----------------------------------------------------------------------------
// Integer Register Class
//-----------------------------------------------------------------------------


struct SparcIntRegClass : public MachineRegClassInfo {
  SparcIntRegClass(unsigned ID) 
    : MachineRegClassInfo(ID, NumOfAvailRegs, NumOfAllRegs) {  }

  void colorIGNode(IGNode *Node, std::vector<bool> &IsColorUsedArr) const;

  inline bool isRegVolatile(int Reg) const {
    return (Reg < (int)StartOfNonVolatileRegs); 
  }

  enum {   // colors possible for a LR (in preferred order)
     // --- following colors are volatile across function calls
     // %g0 can't be used for coloring - always 0
     o0, o1, o2, o3, o4, o5, o7,  // %o0-%o5, 

     // %o6 is sp, 
     // all %0's can get modified by a call

     // --- following colors are NON-volatile across function calls
     l0, l1, l2, l3, l4, l5, l6, l7,    //  %l0-%l7
     i0, i1, i2, i3, i4, i5,         // %i0-%i5: i's need not be preserved 
      
     // %i6 is the fp - so not allocated
     // %i7 is the ret address by convention - can be used for others

     // max # of colors reg coloring  can allocate (NumOfAvailRegs)

     // --- following colors are not available for allocation within this phase
     // --- but can appear for pre-colored ranges 

     i6, i7, g0,  g1, g2, g3, g4, g5, g6, g7, o6,

     NumOfAllRegs,  // Must be first AFTER registers...
     
     //*** NOTE: If we decide to use some %g regs, they are volatile
     // (see sparc64ABI)
     // Move the %g regs from the end of the enumeration to just above the
     // enumeration of %o0 (change StartOfAllRegs below)
     // change isRegVloatile method below
     // Also change IntRegNames above.

     // max # of colors reg coloring  can allocate
     NumOfAvailRegs = i6,

     StartOfNonVolatileRegs = l0,
     StartOfAllRegs = o0,
  };

  static const char * const getRegName(unsigned reg);
};




//-----------------------------------------------------------------------------
// Float Register Class
//-----------------------------------------------------------------------------

class SparcFloatRegClass : public MachineRegClassInfo {
  int findFloatColor(const LiveRange *LR, unsigned Start,
		     unsigned End, std::vector<bool> &IsColorUsedArr) const;
public:
  SparcFloatRegClass(unsigned ID) 
    : MachineRegClassInfo(ID, NumOfAvailRegs, NumOfAllRegs) {}

  void colorIGNode(IGNode *Node, std::vector<bool> &IsColorUsedArr) const;

  // according to  Sparc 64 ABI, all %fp regs are volatile
  inline bool isRegVolatile(int Reg) const { return true; }

  enum {
    f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, 
    f10, f11, f12, f13, f14, f15, f16, f17, f18, f19,
    f20, f21, f22, f23, f24, f25, f26, f27, f28, f29,
    f30, f31, f32, f33, f34, f35, f36, f37, f38, f39,
    f40, f41, f42, f43, f44, f45, f46, f47, f48, f49,
    f50, f51, f52, f53, f54, f55, f56, f57, f58, f59,
    f60, f61, f62, f63,

    // there are 64 regs alltogether but only 32 regs can be allocated at
    // a time.
    //
    NumOfAvailRegs = 32,
    NumOfAllRegs = 64,

    StartOfNonVolatileRegs = f32,
    StartOfAllRegs = f0,
  };

  static const char * const getRegName(unsigned reg);
};




//-----------------------------------------------------------------------------
// Int CC Register Class
// Only one integer cc register is available. However, this register is
// referred to as %xcc when instructions like subcc are executed but 
// referred to as %ccr (i.e., %xcc + %icc") when this register is moved
// into an integer register using RD or WR instrcutions. So, two ids are
// allocated for two names.
//-----------------------------------------------------------------------------

struct SparcIntCCRegClass : public MachineRegClassInfo {
  SparcIntCCRegClass(unsigned ID) 
    : MachineRegClassInfo(ID, 1, 2) {  }
  
  void colorIGNode(IGNode *Node, std::vector<bool> &IsColorUsedArr) const {
    if (IsColorUsedArr[0])
      Node->getParentLR()->markForSpill();
    else
      Node->setColor(0);    // only one int cc reg is available
  }
  
  // according to  Sparc 64 ABI,  %ccr is volatile
  //
  inline bool isRegVolatile(int Reg) const { return true; }

  enum {
    xcc, ccr   // only one is available - see the note above
  };

  static const char * const getRegName(unsigned reg);
};




//-----------------------------------------------------------------------------
// Float CC Register Class
// Only 4 Float CC registers are available
//-----------------------------------------------------------------------------

struct SparcFloatCCRegClass : public MachineRegClassInfo {
  SparcFloatCCRegClass(unsigned ID) 
    : MachineRegClassInfo(ID, 4, 4) {  }

  void colorIGNode(IGNode *Node, std::vector<bool> &IsColorUsedArr) const {
    for(unsigned c = 0; c != 4; ++c)
      if (!IsColorUsedArr[c]) { // find unused color
        Node->setColor(c);   
        return;
      }

    Node->getParentLR()->markForSpill();
  }
  
  // according to  Sparc 64 ABI, all %fp CC regs are volatile
  //
  inline bool isRegVolatile(int Reg) const { return true; }

  enum {
    fcc0, fcc1, fcc2, fcc3
  };

  static const char * const getRegName(unsigned reg);
};

#endif
