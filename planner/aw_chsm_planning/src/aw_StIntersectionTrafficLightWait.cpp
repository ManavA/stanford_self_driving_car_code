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
#include <aw_StReplan.hpp>
#include <aw_StIntersectionTrafficLightWait.hpp>

namespace vlr {

StIntersectionTrafficLightWait::StIntersectionTrafficLightWait(my_context ctx) :
  my_base(ctx), StBase<StIntersectionTrafficLightWait>(std::string("StIntersectionTrafficLightWait")) {
  IntersectionManager* isec_man = context<StIntersection> ().isec_man;

  assert(isec_man);
  isec_man->stoppedOnStopline();
  context<StIntersection> ().recover_mode = StIntersection::RECOVER_TO_EXIT;
}

StIntersectionTrafficLightWait::~StIntersectionTrafficLightWait() {
}

sc::result StIntersectionTrafficLightWait::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // get global data
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = context<ChsmPlanner>().topology_;
  IntersectionManager* isec_man = context<StIntersection>().isec_man;
  assert(isec_man);

  // Transition: Recovery Mode
  if (isec_man->hasPrioMovement()) {
    context<StIntersection> ().clearRecoveryIndicator();
  }

  // measure progress in the parent state
  if (planner.params().enable_recovery && (context<StIntersection> ().checkRecovery())) {
    return transit<StIntersectionRecover> ();
  }

  // Transition: Replanning (because route is blocked)
  if (topology->isRouteBlocked()) return transit<StReplan> ();

  // generate stop trajectory
  planner.generateStopTrajectory();

  // set turn signal
  planner.turn_signal_.signal = context<StIntersection> ().turnDirection;

////     Transition: drive in intersection
////
////     We will go if it's our turn and rely on vehicle prediction
////     to not run into anybody
////     if (isec_man->hasRightOfWay() && !context<StIntersection>().isec_man->hasToStop()) {
  if (isec_man->hasRightOfWay() && !context<StIntersection>().isec_man->hasToStop() && !isec_man->isVehicleOnIntersectionInFront()) {
    return transit<StIntersectionDriveInside> ();
  }


  return forward_event();
}

} // namespace vlr
