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


#include <aw_RndfVertex.h>

#include <aw_StActive.hpp>
#include <aw_StPause.hpp>
#include <aw_StDriveKTurn.hpp>
#include <aw_StError.hpp>

namespace vlr  {

StActive::StActive(my_context ctx) : my_base(ctx), StBase<StActive>(std::string("StActive"))
{
	setRecoveryTime(GLOBAL_RECOVERY_TIMEOUT);
	clearRecoveryIndicator();
}

StActive::~StActive()
{
}

sc::result StActive::react(const EvProcess&)
{
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }
	return forward_event();
}

sc::result StActive::react(const EvAfterProcess&)
{
  // TODO: check when this is called...
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }

//	context<ChsmPlanner>().vehiclecmd.beeper_on = 1;
//	context<ChsmPlanner>().vehiclecmd.hazard_lights_on = 1;
  if(context<ChsmPlanner>().emergencyStopInitiated()) {
    return transit<StPause>();
  }
	if (detectedErrornousTransitions()) return transit<StError>();

	if (checkRecovery()) {
		return transit<StGlobalRecover>();
	}

	return forward_event();
}

sc::result StActive::react(const sc::exception_thrown&)
{
	detectedErrornousTransitions();
    try
    {
      throw;
    }
    /* we can catch special exceptions here and handle them
    catch ( const std::runtime_error & )
    {
      // only std::runtime_errors will lead to a transition
      // to Defective ...
      return transit< StError >();
    ChsmPlanner& planner = context<ChsmPlanner>();
    if(planner.emergencyStopInitiated()) {
      return transit<StPause>();
    }
    }*/
    catch ( ... )
    {
      return forward_event();
    }
}


StGlobalRecover::StGlobalRecover(my_context ctx) : my_base(ctx), StBase<StGlobalRecover>(std::string("StGlobalRecover")),
  done(false), checkpoint(0)
{
  ChsmPlanner& planner = context<ChsmPlanner>();

  Topology* topology = planner.topology_;

  topology->nextCheckPointIt();

  done = topology->next_check_point_it == topology->checkpoints.end();

  if (!done) {
  	checkpoint = *topology->next_check_point_it;

    double x,y,psi;

    x = checkpoint->x();
    y = checkpoint->y();

    if (checkpoint->numInEdges()) {
    	RndfVertex* v = (*(checkpoint->inEdges().begin()))->fromVertex();
    	psi = atan2(v->y() - y, v->x() - x);
    } else if (checkpoint->numOutEdges()) {
    	RndfVertex* v = (*(checkpoint->outEdges().begin()))->toVertex();
    	psi = atan2(y - v->y(), x - v->x());
    } else {
    	psi = 0;
    }

//   TODO: reactivate gpp / emergency planner
  }
}

StGlobalRecover::~StGlobalRecover() {
}

sc::result StGlobalRecover::react(const EvAfterProcess&)
{
	detectedErrornousTransitions();
  ChsmPlanner& planner = context<ChsmPlanner>();
  Topology* topology = planner.topology_;

  // check if vehicle is at exit
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(topology->current_edge_it_on_complete_mission_graph);

    //   TODO: reactivate gpp / emergency planner
  return forward_event();
}

} // namespace vlr
