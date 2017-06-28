/********************************************************
 Stanford Driving Software
 Copyright (c) 2011 Stanford University
 All rights reserved.

 Redistribution and use in source and binary forms, with
 or without modification, are permitted provided that the
 following conditions are met:

 * Redistributions of source code must retain the above
 copyright notice, this list of conditions and the
 following disclaimer.
 * Redistributions in binary form must reproduce the above
 copyright notice, this list of conditions and the
 following disclaimer in the documentation and/or other
 materials provided with the distribution.
 * The names of the contributors may not be used to endorse
 or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 DAMAGE.
 ********************************************************/

#include <aw_StPause.hpp>
#include <aw_StDrive.hpp>
#include <aw_StStop.hpp>

namespace vlr {
/*---------------------------------------------------------------------------
 * StStop
 *---------------------------------------------------------------------------*/
StStop::StStop(my_context ctx) :
  my_base(ctx), StBase<StStop> (std::string("StStop")) {
}

StStop::~StStop() {
}

sc::result StStop::react(const EvProcess&) {
  return forward_event();
}

sc::result StStop::react(const EvDrive&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  return transit<StDrive> ();
}

/*---------------------------------------------------------------------------
 * StStopping
 *---------------------------------------------------------------------------*/
StStopping::StStopping(my_context ctx) :
  my_base(ctx), StBase<StStopping>(std::string("StStopping")) {
}

StStopping::~StStopping() {
}

sc::result StStopping::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  // calculate distance to stop point
  double goal_dist = context<ChsmPlanner> ().topology_->distToMissionEnd();

  // generate curve points
  context<ChsmPlanner> ().generateCurvePoints(context<ChsmPlanner> ().params().max_speed_goal);

  // transitions
  if (goal_dist < STOP_DIST_THRESHOLD && context<ChsmPlanner> ().currentPose().v() < STOP_SPEED_THRESHOLD) {
    return transit<StPause> ();
  }

  return forward_event();
}

} // namespace vlr
