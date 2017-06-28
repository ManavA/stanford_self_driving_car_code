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


#ifndef AW_INTERSECTIONMANAGER_H
#define AW_INTERSECTIONMANAGER_H

#include <string>
#include <limits.h>
#include <set>
#include <map>

#include <aw_RndfGraph.h>
#include <aw_Vehicle.h>
#include <aw_VehicleManager.hpp>
#include <aw_MergeFeasabilityCheck.hpp>
#include <aw_RndfGraphSearch.h>
#include <driving_common/TrafficLightState.h>

namespace vlr {

class Topology;

class IntersectionManager {
public:
  IntersectionManager(Topology& topology_, VehicleManager& vehicle_manager, double max_merge_speed,
      std::map<std::string, driving_common::TrafficLightState>& tl_states, pthread_mutex_t& intersection_predictor_mutex);
  ~IntersectionManager();

  /*
   * Checks for precedence on intersections with stop lines
   * Precondition: vehicle stopped on stopline
   */
  void stoppedOnStopline();

  //! returns true if vehicle is on an priority lane
  bool isOnPrio();

  //! generally returns true if the vehicle has the right of way at an intersection
  bool hasRightOfWay();

  //! returns true if vehicle has to stop at intersection
  bool hasToStop();

  bool isVehicleOnIntersectionInFront();

  bool isInfrontMergePoint();

  //! distance and vehicle pointer to next vehicle - uses multi-matching
  //std::pair<double, Vehicle*> distToVehicleOnWayThrouIntersection();

  const VehicleMap vehiclesWithRow() const {
    return vehicles_with_row;
  }

  const VehicleMap vehiclesWithPriority() const {
    return vehicles_with_priority;
  }

  RndfIntersection* intersection() {
    return intersection_;
  }

  std::pair<bool, double> isPrioOppositeMergeAllowed(double velocity_desired);

  // is there activity on any prio lane? (do no normal recover)
  bool hasPrioMovement();

  // vehicles need to make 0.75m progress in 20 seconds
  static const double last_pose_pose_threshold; // [m]
  static const double last_pose_time_threshold; // [s]
  static const double last_pose_time_diffusion; // [s] random +-

protected:
  bool isMergeAllowed();

  void do_merge_check(MergeFeasabilityCheck::Entity& ego, GraphPlace& ego_place, bool on_prio, size_t& merge_allowed, const Vehicle& veh, const RndfEdge* edge);
  bool _hasPrioMovement(VehicleMap& map);

  std::pair<RndfEdge*, double> getMergingPointDistance();

  // get rid of non-moving obstacles
  void purgeFakeVehicles(VehicleMap& map, size_t& counter);

  // clears the timestamps used for purgeFakeVehicles
  void clearTimestamps(VehicleMap& map);

  Topology* topology_;
  RndfGraph* graph;
  VehicleManager* vehicle_manager_;
  RndfIntersection* intersection_;
  MergeFeasabilityCheck* mfc;
  double max_merge_speed_;

  VehicleMap vehicles_with_row; /*!< row = right of way */
  VehicleMap vehicles_on_stopline;
  VehicleMap vehicles_with_priority;
  VehicleMap vehicles_on_opposite_prio;
  bool right_to_drive;
  VehiclePoseMap last_pose;
  VehiclePoseMap last_prio_pose;
  VehicleIdSet ignored_vehicles;

  //! indicates the direction of the turn (-1 left, 0 straight, 1 right)
  int turn_dir;
  std::map<std::string, driving_common::TrafficLightState>& traffic_light_states_;
};

} // namespace vlr

#endif // INTERSECTIONMANAGER_H_
