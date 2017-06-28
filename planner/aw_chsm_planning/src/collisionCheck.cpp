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


#include <float.h>
#include <iostream>
#include <cmath>
#include <vector>

//#include <IL/il.h>
#include <aw_Vehicle.h>

#include <trajectoryEvaluator.h>

namespace vlr {

void TrajectoryEvaluator::setStaticObstacleMap(uint8_t* map, double width, double height, double res, double cx, double cy, double timestamp) {
  obstacle_map_ = map;
  obstacle_map_width_ = width;
  obstacle_map_height_ = height;
  obstacle_map_res_ = res;
  obstacle_map_cx_ = cx;
  obstacle_map_cy_ = cy;
  obstacle_map_timestamp_ = timestamp;
}

bool TrajectoryEvaluator::checkCollisionVehicles(MovingBox& our_car_mb, const MovingBox& other_car_mb) {

	// First check for collision of circumcircles
	Vehicle::createCircumCircle(our_car_mb);

	if ( !checkCollisionCircles(our_car_mb.circum_circle_, other_car_mb.circum_circle_) ) {
		//std::cout << " no collision guaranteed! \n";
		return false; // no collision
	}

    // .. taking a closer look
  Vehicle::createCircles(our_car_mb);

  for (uint32_t i = 0; i < our_car_mb.num_circles_; i++) {
    for (uint32_t j = 0; j < other_car_mb.num_circles_; j++) {
      if(checkCollisionCircles(our_car_mb.circles_[i], other_car_mb.circles_[j])) {
        //std::cout << "circle collision" << std::endl;
        return true; // collision
      }
      else {
        //std::cout << "no circle collision" << std::endl;
      }
    }
  }

	return false; // no collision
}

// width/length_safety    .. includes desired distance to other obstacles
// width/length_no_safety   .. describes car with a fairly small safety margin in order be robust to noise of obstacle position
// pull_away_time     .. time to pull away from a car that got too close

bool TrajectoryEvaluator::checkCollisionOfTrajectoriesDynamic(const std::vector<driving_common::TrajectoryPoint2D>& trajectory,
					const std::map<int, Vehicle>& obstacle_predictions, double& collision_time ) {

  const double& width_safety = params().safety_width;
  const double& width_no_safety = params().no_safety_width;
  const double& length_safety = params().safety_length;
  const double& length_no_safety = params().no_safety_length;
  const double& pull_away_time = params().pull_away_time;
  const double& offset = params().passat_offset;

  const double& t_0 = trajectory[0].t;
	// Generate vector of iterators to obstacles
	std::vector<std::vector<MovingBox>::const_iterator> obstacle_traj_it_vector;
//	obstacle_traj_it_vector.resize(obstacle_predictions.size());

	std::map<int, Vehicle>::const_iterator vit=obstacle_predictions.begin(), vit_end=obstacle_predictions.end();
	for (; vit != vit_end; vit++) {
		const Vehicle& veh = (*vit).second;
//		double ndx = cos(trajectory[0].theta+0.5*M_PI);
//		double ndy = sin(trajectory[0].theta+0.5*M_PI);
//    double dx = veh.xMatchedFrom() - trajectory[0].x;
//    double dy = veh.yMatchedFrom() - trajectory[0].y;
      double dx = veh.xMatchedFrom() - trajectory[0].x;
      double dy = veh.yMatchedFrom() - trajectory[0].y;
      if(dx !=0 || dy !=0) {
      double car2car_angle=atan2(dy, dx);
      double da = CGAL_Geometry::deltaAngle(car2car_angle, trajectory[0].theta);
		  if(da<-0.5*M_PI || da>0.5*M_PI) {
//		    printf("Ignoring car behind us...\n");
		    continue;
		  }
		}
		else {  // there's a car right where we are standing...
		  return true;
		}
//      obstacle_traj_it_vector[j] = veh.predictedTrajectory().begin();
      obstacle_traj_it_vector.push_back(veh.predictedTrajectory().begin());
	}

	// Check for each point in time...
	std::vector<driving_common::TrajectoryPoint2D>::const_iterator it_prop_traj;
	for (it_prop_traj  = trajectory.begin(); it_prop_traj != trajectory.end(); it_prop_traj++) {
		// ... each obstacle
		// TODO: check for length of obstacle_traj vector
		std::vector<std::vector<MovingBox>::const_iterator>::iterator it_obstacle_traj_it_vector;
		for ( 	it_obstacle_traj_it_vector  = obstacle_traj_it_vector.begin();
				it_obstacle_traj_it_vector != obstacle_traj_it_vector.end();
				it_obstacle_traj_it_vector++ ) {

			MovingBox our_car_mb;
			our_car_mb.t		= it_prop_traj->t;
			our_car_mb.x 	= it_prop_traj->x;
			our_car_mb.y 	= it_prop_traj->y;
			our_car_mb.psi 	= it_prop_traj->theta;

			const MovingBox& other_car_prediction_mb = *(*it_obstacle_traj_it_vector);
			(*it_obstacle_traj_it_vector)++ ; // instead of for(*)


			// increase continuously size of car from min to max within deltaT
			//in order to be robust against noise

			const double& width_max  = width_safety;
			const double& width_min  = width_no_safety;
			const double& length_max = length_safety;
			const double& length_min = length_no_safety;
			const double pull_away_time_inv = 1.0/pull_away_time;

			const double t_minus_t0 = our_car_mb.t-t_0;
			double width_of_t;
			double length_of_t;
			if ( t_minus_t0 < pull_away_time ) {
				width_of_t  = width_min  + t_minus_t0 * (width_max  - width_min)  * pull_away_time_inv;
				length_of_t = length_min + t_minus_t0 * (length_max - length_min) * pull_away_time_inv;
			}
			else { // saturate on max values
				width_of_t 	= width_max;
			    length_of_t = length_max;
			}

      our_car_mb.length = length_of_t;
      our_car_mb.width = width_of_t;
      our_car_mb.ref_offset = offset; // TODO: Make this also time dependent?!?

      if ( checkCollisionVehicles(our_car_mb, other_car_prediction_mb) ) {
				collision_time = it_prop_traj->t;
				return true;
			}
		}
		// for next loop (*)
		// our vehicle
//		for (uint32_t j = 0; j < obstacle_traj_it_vector.size(); j++) {
//			obstacle_traj_it_vector[j]++; // obstacles
//		}
	}
	collision_time = DBL_MAX;
	return false;
}

TrjOccupancyState_t TrajectoryEvaluator::checkCollisionStatic(const std::vector<driving_common::TrajectoryPoint2D>& trajectory) {

  static int start=0; start++;
  const double& car_width = params().safety_width;
  const double& car_length = params().safety_length;
  const double& dist_rear_ref = params().passat_offset;

  MovingBox our_car_mb;
  our_car_mb.length = car_length;
  our_car_mb.width = car_width;
  our_car_mb.ref_offset = dist_rear_ref;

  TrjOccupancyState_t trj_state=TRJ_FREE;
  std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit = trajectory.begin();
  std::vector<driving_common::TrajectoryPoint2D>::const_iterator tit_end = trajectory.end();
  for (; tit != tit_end; tit++) {
    our_car_mb.t    = (*tit).t;
    our_car_mb.x    = (*tit).x;
    our_car_mb.y    = (*tit).y;
    our_car_mb.psi  = (*tit).theta;
    Vehicle::createCircles(our_car_mb);

    for (uint32_t i = 0; i < our_car_mb.num_circles_; i++) {
      int32_t xi = obstacle_map_width_ / 2 - (int32_t) ((our_car_mb.circles_[i].x - obstacle_map_cx_) / obstacle_map_res_ + .5);
      int32_t yi = obstacle_map_height_ / 2 - (int32_t) ((our_car_mb.circles_[i].y - obstacle_map_cy_) / obstacle_map_res_ + .5);
      //     printf("for %f, %f @ %f, %f - %i, %i\n", car_circles[i].x, car_circles[i].y, obstacle_map_cx_, obstacle_map_cy_, xi, yi);
      if (xi >= 0 && xi < obstacle_map_width_ && yi >= 0 && yi < obstacle_map_height_) {
        //        obstacle_map_[yi * obstacle_map_width_ + xi]=255;
        uint8_t val = obstacle_map_[yi * obstacle_map_width_ + xi];

        if (road_map_ && !road_map_[yi * obstacle_map_width_ + xi]) {
          //          printf("val: %i\n", road_map_[yi * obstacle_map_width_ + xi]);
//          return TRJ_BLOCKED; // hit lane boundary
        }
        if (val == OSM_BLOCKED) {
          return TRJ_BLOCKED; // collision
        }
        else if (val == OSM_MAYBE_BLOCKED) {
          trj_state = TRJ_MAYBE_BLOCKED;
        }
      }
    }
  }

//    // DEBUG
//  tit = trajectory.begin();
//  tit_end = trajectory.end();
//  for (; tit != tit_end; tit++) {
//    Vehicle::createCircles(our_car_mb);
//
////    printf("check: ");
//    for (uint32_t i = 0; i < our_car_mb.num_circles_; i++) {
//      int32_t xi = obstacle_map_width_/2 - (int32_t)((our_car_mb.circles_[i].x - obstacle_map_cx_) / obstacle_map_res_+.5);
//      int32_t yi = obstacle_map_height_/2 - (int32_t)((our_car_mb.circles_[i].y - obstacle_map_cy_) / obstacle_map_res_+.5);
////      printf("(%i, %i), ", xi, yi);
////      printf("for %f, %f @ %f, %f - %i, %i\n", car_circles[i].x, car_circles[i].y, obstacle_map_cx_, obstacle_map_cy_, xi, yi);
//      if (xi >= 0 && xi < obstacle_map_width_ && yi >= 0 && yi < obstacle_map_height_) {
//        obstacle_map_[yi * obstacle_map_width_ + xi]=250;
//      }
//    }
////  printf("\n");
//  }
//
/*
if(road_map_) {
  static uint32_t img_id=0;
  if(img_id==0) {
    ilInit();
    ilGenImages(1, &img_id);
    ilBindImage(img_id);
  }
  char buf[100];
  static uint32_t frame_num=0;
  sprintf(buf, "smap_with_ccheck%04d.png", frame_num);
  printf("%s\n", buf);
  ilTexImage(obstacle_map_width_, obstacle_map_height_, 1, 1, IL_LUMINANCE, IL_UNSIGNED_BYTE, road_map_);
  ilSave(IL_PNG, buf);
  frame_num++;
}*/
  return trj_state; // no collision but maybe maybe...
}

} // namespace vlr
