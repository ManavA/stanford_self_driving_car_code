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


#include <aw_StDriveKTurn.hpp>
#include <aw_StPause.hpp>

namespace drc = driving_common;

namespace vlr {

#define C2_KTURN_SPEED      2.0 /*[m/s]*/
#define C2_KTURN_CHANGELANE 1
#define C2_KTURN_SETBACK    2
#define C2_KTURN_FINISH     3

int c2_circlePoints(double xc, double yc, double r, double theta_start, double theta_end, double* pointsX, double* pointsY, double* pointsTheta,
    int numberOfPoints);
//int c2_kturn(double delta, double r_turn_min, CurvePoints* curvepointsObj,int submaneuver);


/*---------------------------------------------------------------------------
 * StKTurn
 *---------------------------------------------------------------------------*/
StDriveKTurn::StDriveKTurn(my_context ctx) :
  my_base(ctx), StBase<StDriveKTurn>("StDriveKTurn") {
#ifdef A_STAR_KTURN
  map_timer = drc::Time::current();
  map_timer += MIN_WAIT_FOR_OBSTACLEMAP;
#else
  // get local copy of parameter set so that all three phases react with the same parameter
  radius = context<ChsmPlanner>().params().kturn_radius;
  delta = context<ChsmPlanner>().params().kturn_delta;
  switch_speed = context<ChsmPlanner>().params().kturn_switch_speed;
  switch_distance = context<ChsmPlanner>().params().kturn_switch_distance;
#endif
}

StDriveKTurn::~StDriveKTurn() {
#ifdef A_STAR_KTURN
  //   TODO: reactivate gpp
#endif
}

sc::result StDriveKTurn::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // do _not_ bubble up the event, we want to go through all our phases
  return discard_event();
}

/*---------------------------------------------------------------------------
 * StDriveKTurnApproach
 *---------------------------------------------------------------------------*/
StDriveKTurnApproach::StDriveKTurnApproach(my_context ctx) :
  my_base(ctx), StBase<StDriveKTurnApproach>(std::string("StDriveKTurnApproach")) {
#ifdef A_STAR_KTURN
  //   TODO: reactivate gpp
#endif
}

StDriveKTurnApproach::~StDriveKTurnApproach() {
}

sc::result StDriveKTurnApproach::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  // calculate distance to stop point
  ChsmPlanner& planner = context<ChsmPlanner> ();
  Topology* topology = planner.topology_;
  double kturn_dist = topology->distToNextKTurn();

  // switch to phase 1 when vehicle is close to kturn point
#ifdef A_STAR_KTURN
  if (kturn_dist < STOP_DIST_THRESHOLD) {
    if (isExpired(context<StDriveKTurn> ().map_timer)) { // TODO: mergecheck or at least "is there something" check

      return transit<StDriveKTurnAStar> ();
    }
    else {
      context<ChsmPlanner> ().generateStopTrajectory();
    }
  }
#else
  if (kturn_dist < STOP_DIST_THRESHOLD) {
    return transit<StDriveKTurnPhase1>();
  }
#endif
  // generate curvepoints
  context<ChsmPlanner> ().generateCurvePoints(planner.params().max_speed_kturn);

  return forward_event();
}

#ifdef A_STAR_KTURN
/*---------------------------------------------------------------------------
 * StDriveKTurnAStar
 *---------------------------------------------------------------------------*/
StDriveKTurnAStar::StDriveKTurnAStar(my_context ctx) :
  my_base(ctx), StBase<StDriveKTurnAStar>("StDriveKTurnAStar") {
  return;

  double x, y, psi;
  ChsmPlanner& planner = context<ChsmPlanner> ();
  RoutePlanner::Route::RouteEdgeList::iterator& opposite_lane_edge = context<StDriveKTurn> ().opposite_lane_edge;

  // get kturn edge
  RoutePlanner::Route::RouteEdgeList::iterator anno_edge_it(planner.topology_->current_edge_it_on_complete_mission_graph);
  std::cout << "Looking for kturn maneuvers: " << std::endl;
  while (anno_edge_it != planner.topology_->route.route.end()) {
    if ((*anno_edge_it)->hasAnnotation(UC_MANEUVER_U_TURN)) {
      ++anno_edge_it;
      if (anno_edge_it != planner.topology_->route.route.end()) {
        ++anno_edge_it;
        if (anno_edge_it != planner.topology_->route.route.end()) opposite_lane_edge = anno_edge_it;
      }
      break;
    }
    ++anno_edge_it;
  }

  // set end position
  RoutePlanner::AnnotatedRouteEdge* edge = *opposite_lane_edge;
  assert(edge);
  x = edge->edge()->fromVertex()->x();
  y = edge->edge()->fromVertex()->y();
  psi = atan2(edge->edge()->toVertex()->y() - edge->edge()->fromVertex()->y(), edge->edge()->toVertex()->x() - edge->edge()->fromVertex()->x());

  //   TODO: reactivate gpp

  std::cout << "GPP interface not reimplemented\n"; // TODO
}

StDriveKTurnAStar::~StDriveKTurnAStar() {
  //   TODO: reactivate gpp
}

sc::result StDriveKTurnAStar::react(const EvProcess&) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover> ();

  ChsmPlanner& planner = context<ChsmPlanner> ();

  planner.turn_signal_.signal = driving_common::TurnSignal::LEFT;

  RoutePlanner::Route::RouteEdgeList::iterator current_edge_it(planner.topology_->current_edge_it_on_complete_mission_graph);

  // check if vehicle is parked
  //  if(drive_state == UC_NAVI_DRIVE_STOP_DEST) {
  planner.turn_signal_.signal = driving_common::TurnSignal::NONE;
  return transit<StActive> ();
  //  }

  //   TODO: reactivate gpp
  return forward_event();
}
#else
/*---------------------------------------------------------------------------
 * StDriveKTurnPhaseBaseImpl
 *---------------------------------------------------------------------------*/

void StDriveKTurnPhaseBaseImpl::transform_curvepoints_and_define_goal(dgc_pose_t& goal, CurvePoints* curvepoints, const dgc_pose_t& start)
{
  double z=0;
  kogmo_transform_t t;
  kogmo_transform_identity(t);
  kogmo_transform_rotate_z(t, start.yaw);
  kogmo_transform_translate(t, start.x, start.y, 0);
  for(int i=0; i < curvepoints->numberValidPoints; i++) {
    kogmo_transform_point(&curvepoints->curvepoints[i].x,
        &curvepoints->curvepoints[i].y,
        &z
        ,t);
    curvepoints->curvepoints[i].theta =
    kogmo_normalize_theta(curvepoints->curvepoints[i].theta + start.yaw);

  }

  // define subgoal
  int lastIndex = curvepoints->numberValidPoints-1;
  goal.x = curvepoints->curvepoints[lastIndex].x;
  goal.y = curvepoints->curvepoints[lastIndex].y;
  goal.yaw = curvepoints->curvepoints[lastIndex].theta;
}

bool StDriveKTurnPhaseBaseImpl::goal_reached(const dgc_pose_t& goal, double switch_distance, double switch_speed, const dgc_pose_t& robot_pose, double speed)
{
  return hypot(robot_pose.x - goal.x, robot_pose.y - goal.y) < switch_distance && speed < switch_speed;
}

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase1
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase1::StDriveKTurnPhase1(my_context ctx)
: my_base(ctx), StBase("StDriveKTurnPhase1")
{
  // save start pose
  context<StDriveKTurn>().start = context<ChsmPlanner>().robot_pose;
  double delta = context<StDriveKTurn>().delta;
  dgc_pose_t pose = context<ChsmPlanner>().robot_pose;
  dgc_pose_t start = context<StDriveKTurn>().start;
  CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
  double dx = delta * cos(pose.yaw + PI_2);
  double dy = delta * sin(pose.yaw + PI_2);
  dgc_pose_t overall_goal = start;
  overall_goal.x+=dx;
  overall_goal.y+=dy;
  overall_goal.yaw=kogmo_normalize_theta(goal.yaw+TWOPI);

  c2_kturn(delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_CHANGELANE);
  transform_curvepoints_and_define_goal(curvepoints, start);

  // need local copies of this because StDriveKTurnPhaseBase cannot access context<>() method
  switch_distance = context<StDriveKTurn>().switch_distance;
  switch_speed = context<StDriveKTurn>().switch_speed;

}

StDriveKTurnPhase1::~StDriveKTurnPhase1()
{
}

sc::result StDriveKTurnPhase1::react(const EvProcess& event) {

  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // blinker setzen: links
  context<ChsmPlanner>().turn_signal_.signal = driving_common::TurnSignal::LEFT;

  if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().v())) {
    return transit<StDriveKTurnPhase2>();
  }
  else {
    return forward_event();
  }

}

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase2
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase2::StDriveKTurnPhase2(my_context ctx)
: my_base(ctx), StBase("StDriveKTurnPhase2")
{
  CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
  c2_kturn(context<StDriveKTurn>().delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_SETBACK);
  transform_curvepoints_and_define_goal(curvepoints, context<StDriveKTurn>().start);

  switch_distance = context<StDriveKTurn>().switch_distance;
  switch_speed = context<StDriveKTurn>().switch_speed;
}

StDriveKTurnPhase2::~StDriveKTurnPhase2()
{
}

sc::result StDriveKTurnPhase2::react(const EvProcess& event) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // set turn signal: right
  context<ChsmPlanner>().turn_signal_.signal = driving_common::TurnSignal::RIGHT;

  if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().currentPose().v())) {
    return transit<StDriveKTurnPhase3>();
  }
  else {
    return forward_event();
  }
}

/*---------------------------------------------------------------------------
 * StDriveKTurnPhase3
 *---------------------------------------------------------------------------*/
StDriveKTurnPhase3::StDriveKTurnPhase3(my_context ctx)
: my_base(ctx), StBase("StDriveKTurnPhase3")
{
  CurvePoints* curvepoints = &context<ChsmPlanner>().curvepoints;
  c2_kturn(context<StDriveKTurn>().delta, context<StDriveKTurn>().radius, curvepoints, C2_KTURN_FINISH);
  transform_curvepoints_and_define_goal(curvepoints, context<StDriveKTurn>().start);

  switch_distance = context<StDriveKTurn>().switch_distance;
  switch_speed = context<StDriveKTurn>().switch_speed;
}

StDriveKTurnPhase3::~StDriveKTurnPhase3()
{
}

sc::result StDriveKTurnPhase3::react(const EvProcess& event) {
  if (detectedErrornousTransitions()) return transit<StGlobalRecover>();

  // set turn signal: left
  context<ChsmPlanner>().turn_signal_.signal = driving_common::TurnSignal::LEFT;

  if(goal_reached(context<ChsmPlanner>().robot_pose, context<ChsmPlanner>().currentPose().v())) {
    return transit<StActive>(); // let StActive decide what to do next
  }
  else {
    return forward_event();
  }
}

int c2_circlePoints(double xc, double yc, double r, double theta_start, double theta_end, double* pointsX, double* pointsY, double* pointsTheta, int numberOfPoints)
{
  double dTheta = ( theta_end - theta_start ) / ( numberOfPoints - 1.0 );
  int i;
  double phi;
  double theta;
  int direction = ((theta_start < theta_end) ? 1 : -1);

  for(i=0;i<numberOfPoints;i++)
  {

    phi = theta_start + i*dTheta;
    pointsX[i] = xc + r*cos(phi);
    pointsY[i] = yc + r*sin(phi);
    theta = phi + PI/2.0*direction;
    pointsTheta[i] = theta + floor(0.5-theta/(2*PI))*(2*PI);
  }
  return 1;
}

int c2_kturn(double delta, double r_turn_min, CurvePoints* curvepointsObj,int submaneuver)
{
  double xc1, xc2, xc3;
  double yc1, yc2, yc3;
  double theta_start1, theta_start2, theta_start3;
  double theta_end1, theta_end2, theta_end3;
  double term = sqrt( 4.0* ( r_turn_min )*( r_turn_min ) - ( 2*r_turn_min - delta )*( 2*r_turn_min - delta )/4.0 );
  double kappa = 0;
  double x[C2_CURVEPOINTS_POINTSMAX];
  double y[C2_CURVEPOINTS_POINTSMAX];
  double theta[C2_CURVEPOINTS_POINTSMAX];
  int i;
  int sgnSpeed=0;

  if ( r_turn_min <= 0) return 0;

  switch ( submaneuver )
  {
    case C2_KTURN_CHANGELANE:
    xc1 = 0;
    yc1 = r_turn_min;
    theta_start1 = -PI/2.0;
    theta_end1 = atan2(-(r_turn_min-delta/2.0), term);
    //fprintf(stderr,"theta_start1 %f\n",theta_start1);
    //fprintf(stderr,"theta_end1 %f\n",theta_end1);
    if(!c2_circlePoints(xc1, yc1, r_turn_min, theta_start1, theta_end1, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    kappa = (1.0/r_turn_min);
    sgnSpeed = 1;
    break;

    case C2_KTURN_SETBACK:
    xc2 = term;
    yc2 = delta/2.0;
    theta_start2 = atan2( r_turn_min-delta/2.0, -term );
    theta_end2 = theta_start2 + 2*atan( (r_turn_min-delta/2.0) /term );
    //fprintf(stderr,"theta_start2 %f\n",theta_start2);
    //fprintf(stderr,"theta_end2 %f\n",theta_end2);
    if(!c2_circlePoints(xc2, yc2, r_turn_min, theta_start2, theta_end2, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    for(i=0;i<C2_CURVEPOINTS_POINTSMAX;++i) theta[i]+=PI;
    kappa = -(1.0/r_turn_min);
    sgnSpeed = -1;
    break;

    case C2_KTURN_FINISH:
    xc3 = 0;
    yc3 = -(r_turn_min-delta);
    theta_start3 = atan2( r_turn_min-delta/2.0, term);
    theta_end3 = PI/2.0;
    if(!c2_circlePoints(xc3, yc3, r_turn_min, theta_start3, theta_end3, x, y, theta, C2_CURVEPOINTS_POINTSMAX)) return 0;
    kappa = (1.0/r_turn_min);
    sgnSpeed = 1;
    break;
  }
  for(i=0;i<C2_CURVEPOINTS_POINTSMAX;i++)
  {
    curvepointsObj->curvepoints[i].x = x[i];
    curvepointsObj->curvepoints[i].y = y[i];
    curvepointsObj->curvepoints[i].theta = theta[i];
    curvepointsObj->curvepoints[i].kappa = kappa;
  }

  curvepointsObj->state = C2_CURVEPOINTS_STATE_STOP_DISTANCE;
  curvepointsObj->velocity_desired = sgnSpeed*C2_KTURN_SPEED;
  curvepointsObj->numberValidPoints = C2_CURVEPOINTS_POINTSMAX;
  return 1;
}

#endif // A_STAR_KTURN

} // namespace vlr
