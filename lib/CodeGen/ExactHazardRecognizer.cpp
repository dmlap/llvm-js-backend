//===----- ExactHazardRecognizer.cpp - hazard recognizer -------- ---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This implements a hazard recognizer using the instructions itineraries
// defined for the current target.
//
//===----------------------------------------------------------------------===//

#define DEBUG_TYPE "post-RA-sched"
#include "ExactHazardRecognizer.h"
#include "llvm/CodeGen/ScheduleHazardRecognizer.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetInstrItineraries.h"

using namespace llvm;

ExactHazardRecognizer::
ExactHazardRecognizer(const InstrItineraryData &LItinData) :
  ScheduleHazardRecognizer(), ItinData(LItinData) 
{
  // Determine the maximum depth of any itinerary. This determines the
  // depth of the scoreboard. We always make the scoreboard at least 1
  // cycle deep to avoid dealing with the boundary condition.
  unsigned ScoreboardDepth = 1;
  if (!ItinData.isEmpty()) {
    for (unsigned idx = 0; ; ++idx) {
      if (ItinData.isEndMarker(idx))
        break;

      const InstrStage *IS = ItinData.beginStage(idx);
      const InstrStage *E = ItinData.endStage(idx);
      unsigned ItinDepth = 0;
      for (; IS != E; ++IS)
        ItinDepth += IS->getCycles();

      ScoreboardDepth = std::max(ScoreboardDepth, ItinDepth);
    }
  }

  Scoreboard.reset(ScoreboardDepth);

  DEBUG(dbgs() << "Using exact hazard recognizer: ScoreboardDepth = " 
               << ScoreboardDepth << '\n');
}

void ExactHazardRecognizer::Reset() {
  Scoreboard.reset();
}

void ExactHazardRecognizer::ScoreBoard::dump() const {
  dbgs() << "Scoreboard:\n";

  unsigned last = Depth - 1;
  while ((last > 0) && ((*this)[last] == 0))
    last--;

  for (unsigned i = 0; i <= last; i++) {
    unsigned FUs = (*this)[i];
    dbgs() << "\t";
    for (int j = 31; j >= 0; j--)
      dbgs() << ((FUs & (1 << j)) ? '1' : '0');
    dbgs() << '\n';
  }
}

ExactHazardRecognizer::HazardType ExactHazardRecognizer::getHazardType(SUnit *SU) {
  if (ItinData.isEmpty())
    return NoHazard;

  unsigned cycle = 0;

  // Use the itinerary for the underlying instruction to check for
  // free FU's in the scoreboard at the appropriate future cycles.
  unsigned idx = SU->getInstr()->getDesc().getSchedClass();
  for (const InstrStage *IS = ItinData.beginStage(idx),
         *E = ItinData.endStage(idx); IS != E; ++IS) {
    // We must find one of the stage's units free for every cycle the
    // stage is occupied. FIXME it would be more accurate to find the
    // same unit free in all the cycles.
    for (unsigned int i = 0; i < IS->getCycles(); ++i) {
      assert(((cycle + i) < Scoreboard.getDepth()) &&
             "Scoreboard depth exceeded!");

      unsigned freeUnits = IS->getUnits() & ~Scoreboard[cycle + i];
      if (!freeUnits) {
        DEBUG(dbgs() << "*** Hazard in cycle " << (cycle + i) << ", ");
        DEBUG(dbgs() << "SU(" << SU->NodeNum << "): ");
        DEBUG(SU->getInstr()->dump());
        return Hazard;
      }
    }

    // Advance the cycle to the next stage.
    cycle += IS->getNextCycles();
  }

  return NoHazard;
}

void ExactHazardRecognizer::EmitInstruction(SUnit *SU) {
  if (ItinData.isEmpty())
    return;

  unsigned cycle = 0;

  // Use the itinerary for the underlying instruction to reserve FU's
  // in the scoreboard at the appropriate future cycles.
  unsigned idx = SU->getInstr()->getDesc().getSchedClass();
  for (const InstrStage *IS = ItinData.beginStage(idx), 
         *E = ItinData.endStage(idx); IS != E; ++IS) {
    // We must reserve one of the stage's units for every cycle the
    // stage is occupied. FIXME it would be more accurate to reserve
    // the same unit free in all the cycles.
    for (unsigned int i = 0; i < IS->getCycles(); ++i) {
      assert(((cycle + i) < Scoreboard.getDepth()) &&
             "Scoreboard depth exceeded!");

      unsigned freeUnits = IS->getUnits() & ~Scoreboard[cycle + i];

      // reduce to a single unit
      unsigned freeUnit = 0;
      do {
        freeUnit = freeUnits;
        freeUnits = freeUnit & (freeUnit - 1);
      } while (freeUnits);

      assert(freeUnit && "No function unit available!");
      Scoreboard[cycle + i] |= freeUnit;
    }

    // Advance the cycle to the next stage.
    cycle += IS->getNextCycles();
  }

  DEBUG(Scoreboard.dump());
}

void ExactHazardRecognizer::AdvanceCycle() {
  Scoreboard[0] = 0;
  Scoreboard.advance();
}
