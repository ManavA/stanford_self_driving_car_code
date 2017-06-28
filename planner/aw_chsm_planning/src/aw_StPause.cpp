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
#include <aw_StPause.hpp>
#include <aw_StActive.hpp>

namespace drc = driving_common;

namespace vlr {

StPause::StPause(my_context ctx) :
  my_base(ctx), StBase<StPause> (std::string("StPause")) {
  ChsmPlanner& planner = context<ChsmPlanner> ();
  planner.robot_pose_in_pause_ = planner.currentPose(); // TODO: causes mutex lock
  //save_curvepoints = planner.curvepoints;
  planner.inPause = true;

}

StPause::~StPause() {
  context<ChsmPlanner> ().inPause = false;
  //context<ChsmPlanner>().curvepoints = save_curvepoints;
}

sc::result StPause::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  //  ChsmPlanner& planner = context<ChsmPlanner> ();
  //  planner.generateStopTrajectory(true); ?!?
  //  planner.vehiclecmd.beeper_on = 0;
  //  planner.vehiclecmd.hazard_lights_on = 0;
  return forward_event();
  //return transit<StIntersectionApproach>();
}
sc::result StPause::react(const sc::exception_thrown&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  try {
    throw;
  }
  /* we can catch special exceptions here and handle them
   catch ( const std::runtime_error & )
   {
   // only std::runtime_errors will lead to a transition
   // to Defective ...
   return transit< StError >();
   }*/
  catch (...) {
    return forward_event();
  }
}

StPauseShortTerm::StPauseShortTerm(my_context ctx) :
  my_base(ctx), StBase<StPauseShortTerm> (std::string("StPauseShortTerm")) {
  switchTime += drc::Time::current() + SWITCH_TO_LONGTERM_PAUSE;
}

StPauseShortTerm::~StPauseShortTerm() {
}

sc::result StPauseShortTerm::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  if (isExpired(switchTime)) {
    return forward_event();
  }
  else {
    return forward_event();
  }
}

StPauseLongTerm::StPauseLongTerm(my_context ctx) :
  my_base(ctx), StBase<StPauseLongTerm>(std::string("StPauseLongTerm")) {
  // TODO: activate parking brake
}

StPauseLongTerm::~StPauseLongTerm() {
  // TODO: release parking brake
}

sc::result StPauseLongTerm::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  return forward_event();
}
} // namespace vlr
