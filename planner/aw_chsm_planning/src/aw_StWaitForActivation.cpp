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


#include <aw_StWaitForActivation.hpp>
#include <aw_StActive.hpp>
#include <aw_StPause.hpp>
#include <aw_StReplan.hpp>

namespace drc = driving_common;

namespace vlr {

StWaitForActivation::StWaitForActivation(my_context ctx) :
  my_base(ctx), StBase<StWaitForActivation>(std::string("StWaitForActivation")), moved(false) {

  wait_until_ = drc::Time::current() + MIN_WAIT_ACTIVATION;
  ChsmPlanner& planner = context<ChsmPlanner> ();
  if (planner.distance(planner.robot_pose_in_pause_) > TRIGGER_DIST_CLEAR_HISTORY) {
    planner.addMessage("car moved while in pause -> clearing history");
    context<ChsmPlanner> ().clear_deep_history<StActive, 0> ();
    moved = true;
  }
  else {
    planner.addMessage("resuming with history");
  }
  context<ChsmPlanner> ().velocity_desired_ = 0;
}

StWaitForActivation::~StWaitForActivation() {
}

sc::result StWaitForActivation::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;

  // set velocity to 0 -> park mode
  context<ChsmPlanner> ().generateStopTrajectory();

//  context<ChsmPlanner> ().vehiclecmd.beeper_on = 1;
//  context<ChsmPlanner> ().vehiclecmd.hazard_lights_on = 1;

  if (isExpired(wait_until_)) {

    // Transition: Replanning (because ego vehicle is off track)
    if (topology->isOffTrack() || moved) return transit<StReplan> ();

    // Transition: restore history
    return transit<StActiveHistory> ();

  }
  else {
    return forward_event();
  }
}
sc::result StWaitForActivation::react(const sc::exception_thrown&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();
  try {
    throw ;
  }
  /* we can catch special exceptions here and handle them
   catch ( const std::runtime_error & )
   {
   // only std::runtime_errors will lead to a transition
   // to Defective ...
   return transit< StError >();
   }*/
  catch ( ... ) {
    return forward_event();
  }
}

} // namespace vlr
