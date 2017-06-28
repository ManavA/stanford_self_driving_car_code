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


#ifndef AW_VEHICLE_MANAGER_HPP
#define AW_VEHICLE_MANAGER_HPP

#include <cmath>
#include <global.h>
#include <perception/PerceptionObstacles.h>
#include <aw_RndfGraph.h>
#include <aw_Vehicle.h>
//#include <aw_MergeFeasabilityCheck.hpp>

namespace vlr {

class Topology;

//--------------------------------------------------------
//             VehicleManager
//--------------------------------------------------------

class VehicleManager {
public:
  VehicleManager(Topology& topology);

  bool updateVehicles(const std::vector<perception::DynamicObstacle>& dyn_obstacles, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res);
  const std::map<int, Vehicle>& vehicles() const {return vehicle_map;}
  const std::map<int, Vehicle*>& blockades() const {return blockage_map;}
  const std::map<int, Vehicle*>& movingVehicles() const {return moving_map;}
//  std::map<int, Vehicle>& vehicles() {return vehicle_map;}

protected:
  int add_vehicle(const perception::DynamicObstacle& vehicle, double offset_x, double offset_y, double t0, double checked_horizon, double time_sample_res);
  void delete_vehicle(const int id);
  int update_vehicle(const perception::DynamicObstacle& vehicle, double offset_x, double offset_y);

  class VehicleState {
    public:
    double x,y,speed;
  	double time;
  	bool valid;
  	VehicleState() : x(0), y(0), time(), valid(false) { time = vlr::Time::current(); }
  	VehicleState(double x, double y) : x(x), y(y), time(), valid(true) { time = vlr::Time::current(); }
  	bool isMoving(const Vehicle& veh) {
  		// TODO: tune this constants
  		const double DELTA_D = 0.75;
  		const double DELTA_T = 5;
  		const double SPEED_THRESHOLD = 1.0;

  		bool isAtIntersection = false;
  		// in intersection
  		if (veh.edge() && veh.edge()->intersection()) {
  			isAtIntersection = true;
  		}
  		// at stopline
  		if (veh.edge() && veh.edge()->toVertex() && veh.edge()->toVertex()->isStopVertex() && veh.distToEnd() - veh.length()/2.0 < 4.0) {
  			isAtIntersection = true;
  		}
  		if (!valid || isAtIntersection
  				|| std::abs(veh.xMatchedFrom() - x) > DELTA_D
  				|| std::abs(veh.yMatchedFrom() - y) > DELTA_D
  				|| std::abs(veh.speed()) > SPEED_THRESHOLD) {
	  		x = veh.xMatchedFrom();
	  		y = veh.yMatchedFrom();
	  		speed = veh.speed();
	  		valid = true;
	  		time = vlr::Time::current();
	  		return true;
  		} else {
  			return vlr::Time::current() < time + DELTA_T;
  		}
  	};
  };
  std::map<int, VehicleState> state_map;

  void update_state(Vehicle& veh);
public:
  // mapping from internal id to vehicles
  std::map<int, Vehicle> vehicle_map; // this map holds all vehicle objects
  std::map<int, Vehicle*> moving_map; // this map holds all moving vehicles
  std::map<int, Vehicle*> blockage_map; // this map holds all "vehicles" classified as blockages

  Topology& topology;
  Vehicle robot;

  void setBlocked(int veh_id);

  void updateBlockages();

  bool isMoving(int veh_id);

  bool intersectsWithAnotherVehicle(const Vehicle& veh) const;
};


void block_adjacent_lanechange_edges(RoutePlanner::RndfEdge* edge);

//--------------------------------------------------------
//             Operators
//--------------------------------------------------------

std::ostream& operator << (std::ostream& ostrm, const Vehicle& obj);

} // namespace vlr

#endif
