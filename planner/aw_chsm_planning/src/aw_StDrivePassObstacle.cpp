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


#include <aw_StDrivePassObstacle.hpp>
#include <aw_StPause.hpp>
#include <aw_StError.hpp>

namespace drc = driving_common;

namespace vlr {

/*---------------------------------------------------------------------------
 * StDrivePassObstacle
 *---------------------------------------------------------------------------*/
StDrivePassObstacle::StDrivePassObstacle(my_context ctx) : my_base(ctx), StBase<StDrivePassObstacle>(std::string("StDrivePassObstacle"))
 ,lcm(0), pom(0)
{
	lcm = new LaneChangeManager(context<ChsmPlanner>().topology_, context<ChsmPlanner>().topology_->vehicle_manager);
	pom = new PassObstacleManager(context<ChsmPlanner>().topology_);
}

StDrivePassObstacle::~StDrivePassObstacle() {
	delete lcm;
	delete pom;
}

sc::result StDrivePassObstacle::react(const EvProcess&) {
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDrivePassObstacleStop
 *---------------------------------------------------------------------------*/
StDrivePassObstacleStop::StDrivePassObstacleStop(my_context ctx) : my_base(ctx), StBase<StDrivePassObstacleStop>(std::string("StDrivePassObstacleStop"))
{
}

StDrivePassObstacleStop::~StDrivePassObstacleStop() {
}

sc::result StDrivePassObstacleStop::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	Vehicle* obstacle = planner.topology_->get_next_vehicle();
	if (!parent.pom->isInited() && obstacle) {
		parent.pom->setObstacle(obstacle);
	}
	parent.pom->update();

	double st_vh_dist, mv_vh_dist;
	planner.getVehicleDistances(st_vh_dist, mv_vh_dist);

	if (mv_vh_dist < st_vh_dist) { // TODO: maybe delay this transition so that mis-readings don't result in immediate transitions
		return transit<StDrive>();
	}

	if (st_vh_dist < STD_VEHICLE_LENGTH && planner.currentPose().v() < STOP_SPEED_THRESHOLD) {
		return transit<StDrivePassObstacleWait>();
	}

	// is done in St...Wait - so car has to stop before doing recover
	//if (!parent.pom->isPassPossible()) {
	//	return transit<StDriveRecover>();
	//}

	planner.generateCurvePoints(planner.params().max_speed_pass_obstacle); // curvepoints stop in front of obstacle
	return forward_event();
}


/*---------------------------------------------------------------------------
 * StDrivePassObstacleWait
 *---------------------------------------------------------------------------*/
StDrivePassObstacleWait::StDrivePassObstacleWait(my_context ctx) : my_base(ctx), StBase<StDrivePassObstacleWait>(std::string("StDrivePassObstacleWait"))
{
	waitTimeout = drc::Time::current();
	waitTimeout += MIN_WAIT_OBSTACLE;

	congestionTimeout = drc::Time::current();
	congestionTimeout += CONGESTION_TIMEOUT;
}

StDrivePassObstacleWait::~StDrivePassObstacleWait() {
}

sc::result StDrivePassObstacleWait::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	if (!parent.pom->isInited()) {
		Vehicle* obstacle = planner.topology_->get_next_vehicle();
		if (obstacle) {
			parent.pom->setObstacle(obstacle);
		}
	}
	if (!parent.pom->isInited()) {
		planner.addMessage("PassObstacleManager has no valid obstacle to pass");
		return transit<StError>();
	}
	parent.pom->update();

	double st_vh_dist, mv_vh_dist;
	planner.getVehicleDistances(st_vh_dist, mv_vh_dist);

	// Transition: obstacle moves again
	if (mv_vh_dist < st_vh_dist) { // TODO: maybe delay this transition so that mis-readings don't result in immediate transitions
		return transit<StDrive>();
	}

	if (parent.pom->mayPass(planner.currentPose(), planner.currentPose().v(), 2.0) && isExpired(waitTimeout)) {
		return transit<StDrivePassObstaclePass>();
	}

	// Transition: recover from failure
	//if (isExpired(congestionTimeout) || !parent.pom->isPassPossible()) {
	//	return transit<StDriveRecover>();
	//}

	//planner.generateCurvePoints(0.0, std::numeric_limits<double>::max()); // TODO: what's the best way to stop the car?
	planner.velocity_desired_ = 0.5; // TODO: ?!?
	return forward_event();
}

/*---------------------------------------------------------------------------
 * StDrivePassObstaclePass
 *---------------------------------------------------------------------------*/
StDrivePassObstaclePass::StDrivePassObstaclePass(my_context ctx) : my_base(ctx), StBase<StDrivePassObstaclePass>(std::string("StDrivePassObstaclePass"))
{

}

StDrivePassObstaclePass::~StDrivePassObstaclePass() {
}

sc::result StDrivePassObstaclePass::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

	ChsmPlanner& planner = context<ChsmPlanner>();
	StDrivePassObstacle& parent = context<StDrivePassObstacle>();

	if (!parent.pom->isInited()) {
		Vehicle* obstacle = planner.topology_->get_next_vehicle();
		if (obstacle) {
			parent.pom->setObstacle(obstacle);
		}
	}
	if (!parent.pom->isInited()) {
		planner.addMessage("PassObstacleManager has no valid obstacle to pass");
		return transit<StError>();
	}
	parent.pom->update();

	if (!parent.pom->mayPass(planner.currentPose(), planner.currentPose().v(), 2.0)) {
//		if (!parent.pom->mayPass(planner.currentPose().v(), 2.0)) {
		std::cerr << "While passing an obstacle, PassObstacleManager doesn't allow the pass anymore. ignore this and go on..." << std::endl;
	}

	// TODO: implement and check
  double front_sample_length = std::max(planner.params().center_line_min_lookahead_dist, planner.traj_eval_->params().checked_horizon * planner.params().max_speed_pass_obstacle);
  parent.pom->generateTrajectory(planner.currentPose(), planner.center_line_, front_sample_length, planner.params().center_line_back_sample_length);

  // calculate desired velocity based on speed limits and curvature
  // TODO: make this work again..
//  planner.velocity_desired_ = planner.calculateVelocity(planner.params().max_speed_pass_obstacle, front_sample_length);  // TODO: correct front sample length
//	planner.velocity_following_ = parent.pom->getFollowingSpeed();

	return forward_event();
}

/*---------------------------------------------------------------------------
 * StDrivePassObstacleChangeLanes
 *---------------------------------------------------------------------------*/
StDrivePassObstacleChangeLanes::StDrivePassObstacleChangeLanes(my_context ctx) : my_base(ctx), StBase<StDrivePassObstacleChangeLanes>(std::string("StDrivePassObstacleChangeLanes"))
{
}

StDrivePassObstacleChangeLanes::~StDrivePassObstacleChangeLanes() {
}

sc::result StDrivePassObstacleChangeLanes::react(const EvProcess&) {
	if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

//	ChsmPlanner& planner = context<ChsmPlanner>();
//	StDrivePassObstacle& parent = context<StDrivePassObstacle>();
	return forward_event();
	// TODO: implement
}

} // namespace vlr
