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


#include <grid.h>
#include <perception.h>
#include <tracker.h>
#include <utils.h>
#include <lltransform.h>
#include <opencv2/core/core.hpp>
#include <highgui.h>

using namespace std;
using namespace std::tr1;
using namespace vlr;
using namespace dgc;

namespace drc = driving_common;

namespace perception {

//void use_only_obstacles_with_neighbors() {
//  static bool init = true;
//  static int offset[8];
//  char * cptr, *nptr[8];
//  static char * start, *end;
//  int i, j, ctr = 0;
//
//  if (init) {
//    offset[0] = -grid->bytes_per_cell;
//    offset[1] = grid->bytes_per_cell;
//    offset[2] = -grid->cols * grid->bytes_per_cell;
//    offset[3] = grid->cols * grid->bytes_per_cell;
//    offset[4] = offset[2] + offset[0];
//    offset[5] = offset[2] + offset[1];
//    offset[6] = offset[3] + offset[0];
//    offset[7] = offset[3] + offset[1];
//    start = (char *) grid->cell;
//    end = (char *) (start + grid->cols * grid->rows * grid->bytes_per_cell);
//    init = false;
//  }
//
//  for (i = 0; i < obstacles_s->num; i++) {
//    cptr = (char *) (obstacles_s->cell[i]);
//    for (j = 0; j < 8; j++) {
//      nptr[j] = (char *) (cptr + offset[j]);
//    }
//    for (j = 0; j < 8; j++) {
//      if ((char *) nptr[j] >= start && (char *) nptr[j] < end && ((PerceptionCell*) nptr[j])->obstacle) {
//        obstacles_s->cell[ctr++] = obstacles_s->cell[i];
//        break;
//      }
//    }
//  }
//  for (i = 0; i < obstacles_s->num; i++) {
//    obstacles_s->cell[i]->obstacle = FALSE;
//  }
//
//  obstacles_s->num = ctr;
//}

void Perception::prepareObstacles(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_t* points, vector<boost::shared_ptr<TrackedObstacle> > obstacles, uint16_t counter) {
  float ll_x, ll_y;

  msg_.dynamic_obstacle.clear();
  msg_.static_point.clear();
  msg_.timestamp = points->timestamp;
//  msg_.counter = counter;
  int32_t mc = grid.cols_ / 2;
  int32_t mr = grid.rows_ / 2;
  ll_x = (grid.map_c0_) * grid.resolution_;
  ll_y = (grid.map_r0_) * grid.resolution_;
  int32_t max_num = points->num;


  perception::StaticObstaclePoint p;
  for(int32_t r=0; r<grid.rows_; r++) {
    for(int32_t c=0; c<grid.cols_; c++) {
      PerceptionCell* cell = grid.getRCLocalUnsafe(r, c);
      if(!cell) {
        throw VLRException("No cell :-(");
      }

      //static obstacles that were not seen in the last 100 frames get removed
      if(cell->last_obstacle < max(1, counter - 500)) {continue;}

      //dynamic obstacles that were not seen in the last 2 frames get removed

      if(cell->last_dynamic <= counter && cell->last_dynamic > 0 && cell->last_dynamic != cell->last_obstacle)
      {
        continue;
      }

      // TODO: do before tracking
      if(cell->hits <= 2)
      {
//        if(cell->last_obstacle != counter)
  //        cell->hits = 0;

        continue;
      }




      int16_t px, py;
      grid.cellToRCLocal(cell, &py, &px);
      p.x = ll_x + ((px + 0.5) * grid.resolution_);
      p.y = ll_y + ((py + 0.5) * grid.resolution_);
//      int r, c;
//      grid_xy_to_rc(grid, p.x, p.y, &r, &c);
  //    buf[r*grid.cols+c] = 255;//cell->
      //    msg_.point[msg_.num_points].type = cell->region;
      if (cell->last_dynamic == counter) {
        continue;
      }
      else {
        p.type = perception::StaticObstaclePoint::HIGH; // TODO: replace with a better description
      }

      p.z_min = cell->min;
      p.z_max = cell->max;

      msg_.static_point.push_back(p);
    }
  }

////  static uint8_t* buf = new uint8_t[grid.cols*grid.rows];
////  memset(buf, 0, grid.cols*grid.rows);
//  perception::StaticObstaclePoint p;
//  for (int i = 0; i < max_num; i++) {
//    cell_to_coord(grid, points->cell[i], &px, &py);
//    p.x = ll_x + ((px + 0.5) * grid.resolution_);
//    p.y = ll_y + ((py + 0.5) * grid.resolution_);
//    int r, c;
//    grid_xy_to_rc(grid, p.x, p.y, &r, &c);
////    buf[r*grid.cols+c] = 255;//points->cell[i]->
//    //    msg_.point[msg_.num_points].type = points->cell[i]->region;
//    if (points->cell[i]->last_dynamic == counter) {
//      p.type = perception::StaticObstaclePoint::DYNAMIC;  // TODO: remove
//    }
//    else {
//      p.type = perception::StaticObstaclePoint::HIGH; // TODO: replace with a better description
//    }
//
//    p.z_min = points->cell[i]->min;
//    p.z_max = points->cell[i]->max;
//
//    msg_.static_point.push_back(p);
//  }

// static int num=0;
//
//  char imagename[100];
//  sprintf(imagename, "velomap_%02d.png", num);
//  num++;
//  printf("NEW FILE NAME: %s\n", imagename);
//  cv::Mat tmp_mat(grid.cols, grid.rows, CV_8UC1, buf);
//  cv::imwrite(imagename, tmp_mat);
//  printf("Prepared %u points for publishing\n", (uint32_t) msg_.static_point.size());
  int num_obstacles = obstacles.size();
  int num_peds = 0;
  int num_bicycles = 0;
  int num_cars = 0;
  int num_unknown = 0;
  perception::DynamicObstacle obj;
  for (int i = 0; i < num_obstacles; i++) {
    obj.id = obstacles[i]->id;
    switch (obstacles[i]->type) {
      case OBSTACLE_BICYCLIST:
        num_bicycles++;
        break;
      case OBSTACLE_CAR:
        num_cars++;
        break;
      case OBSTACLE_PEDESTRIAN:
        num_peds++;
        break;
      default:
        num_unknown++;
        break;
    }

    obj.type = obstacles[i]->type;
      // TODO: remove. Makes new tracks pedestrians for debugging
    if(obstacles[i]->getNumObservations() < 2) {obj.type = OBSTACLE_PEDESTRIAN;}
    obj.type_this_frame = obstacles[i]->type_this_frame_;
    obj.classified_this_frame = obstacles[i]->classified_this_frame_;

    obj.x = obstacles[i]->pose.x;
    obj.y = obstacles[i]->pose.y;

    obj.direction = obstacles[i]->pose.yaw;
    obj.velocity = obstacles[i]->getVelocity();
    //msg_.dynamic_obstacle[i].velocity = 5;

    obj.length = obstacles[i]->length;
    obj.width = obstacles[i]->width;

    //msg_.dynamic_obstacle[i].confidence = max(0.0, min(1.0, obstacles[i]->filter->getPositionUncertainty() * 200));
    obj.confidence = (unsigned char) min(254.9, obstacles[i]->getNumObservations() * 1.0);
    obj.x_var = 0;//obstacles[i]->getXVar();
    obj.y_var = 0;//obstacles[i]->getYVar();
    obj.xy_cov = 0;//obstacles[i]->getXYCov();
    msg_.dynamic_obstacle.push_back(obj);
  }

//  printf("Cars:        %d\n", num_cars);
//  printf("Bicycles:    %d\n", num_bicycles);
//  printf("Pedestrians: %d\n", num_peds);
//  printf("Other:       %d\n", num_unknown);

  /*if (pose->speed < dgc_mph2ms(5.0)) {
   dgc_transform_t t;
   dgc_transform_t* radar;
   for (int r = 0; r < NUM_LRR2_RADARS; r++) {
   switch (r) {
   case 0: radar = &radar_offset[0]; break;
   case 1: radar = &radar_offset[1]; break;
   case 2: radar = &radar_offset[3]; break;
   case 3: radar = &radar_offset[4]; break;
   }

   dgc_transform_copy(t, *radar);
   dgc_transform_rotate_x( t, pose->roll );
   dgc_transform_rotate_y( t, pose->pitch );
   dgc_transform_rotate_z( t, pose->yaw );
   dgc_transform_translate( t, pose->smooth_x, pose->smooth_y, pose->smooth_z );
   double rx = 0; double ry = 0; double rz = 0;
   dgc_transform_point(&rx, &ry, &rz, t);
   for (i = 0; i < radar_lrr2[r].num_targets; i++) {
   RadarTarget* target = &radar_lrr2[r].target[i];
   if (fabs(target->relative_velocity) < dgc_mph2ms(5.0))
   continue;

   msg_.dynamic_obstacle[num_obstacles].id = target->id;
   msg_.dynamic_obstacle[num_obstacles].obstacleType = 0;

   double x = target->distance + 1.0;
   double y = target->lateral_offset;
   double z = 0.0;
   dgc_transform_point(&x, &y, &z, t);

   msg_.dynamic_obstacle[num_obstacles].x = x;
   msg_.dynamic_obstacle[num_obstacles].y = y;

   msg_.dynamic_obstacle[num_obstacles].velocity = target->relative_velocity;
   msg_.dynamic_obstacle[num_obstacles].direction = (target->relative_velocity > 0) ? atan2(y-ry, x-rx) : atan2(y-ry, x-rx) + 2*M_PI;

   msg_.dynamic_obstacle[num_obstacles].length = 1.0;
   msg_.dynamic_obstacle[num_obstacles].width = 1.0;

   x = 0.0;
   y = target->lateral_offset_var;
   z = 0.0;
   dgc_transform_point(&x, &y, &z, t);
   msg_.dynamic_obstacle[num_obstacles].confidence = 30.0;
   msg_.dynamic_obstacle[num_obstacles].x_var = x;
   msg_.dynamic_obstacle[num_obstacles].y_var = y;
   msg_.dynamic_obstacle[num_obstacles].xy_cov = 0;
   num_obstacles++;
   }
   }
   }*/

  //  printf("%d obstacles with radar\n", num_obstacles);

}
void Perception::integrateSensors(const std::vector<velodyne::Block>& scans) {
  static double time0;
  static double delta_s;

  time0 = scans[0].timestamp;//drc::Time::current();
  delta_s = time0 - last_integration_time_;

  drc::GlobalPose current_pose;
	bool got_pose = false;
	ros::Rate r(200);
	while (!got_pose) {
		try {
			current_pose = pose(time0);
		} catch (vlr::Ex<>& e) {
			std::cout << e.what() << std::endl;
			r.sleep();
			continue;
		}
	got_pose = true;
	}
	grid_stat.center.x = current_pose.x();
	grid_stat.center.y = current_pose.y();

  grid_->recenter(grid_stat.center.x, grid_stat.center.y);

  /*---------------------------------------*/
  pthread_mutex_lock(&integration_mutex);
  velo_client_->integrate(scans, counter_);
  pthread_mutex_unlock(&integration_mutex);
  /*---------------------------------------*/

  counter_++;

  if (settings_.extract_dynamic) {
    tracker_->trackFrame(scans[0].timestamp); // TODO: not the best time representation
  }

  pthread_mutex_lock(&publish_mutex);
  prepareObstacles(*grid_, obstacles_s, obstacles_tracked, counter_);
  pthread_mutex_unlock(&publish_mutex);

  display_time("Integrate", delta_s);
  last_integration_time_ = time0;
}

void Perception::integrateSensors(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
     static double time0;
     static double delta_s;

     time0 = cloud.header.stamp.toSec();
     delta_s = time0 - last_integration_time_;

     drc::GlobalPose current_pose;
     bool got_pose = false;
     ros::Rate r(200);
     while (!got_pose) {
       try {
         current_pose = pose(time0);
       } catch (vlr::Ex<>& e) {
         std::cout << e.what() << std::endl;
         r.sleep();
         continue;
       }
     got_pose = true;
     }
     grid_stat.center.x = current_pose.x();
     grid_stat.center.y = current_pose.y();

     grid_->recenter(grid_stat.center.x, grid_stat.center.y);

     /*---------------------------------------*/
     pthread_mutex_lock(&integration_mutex);
     point_cloud_client_->integrate(cloud, counter_);
     pthread_mutex_unlock(&integration_mutex);
     /*---------------------------------------*/

     counter_++;

     if (settings_.extract_dynamic) {
       tracker_->trackFrame(cloud.header.stamp.toSec());
     }

     pthread_mutex_lock(&publish_mutex);
     prepareObstacles(*grid_, obstacles_s, obstacles_tracked, counter_);
     pthread_mutex_unlock(&publish_mutex);

     display_time("Integrate", delta_s);
     last_integration_time_ = time0;
   }

} // namespace perception
