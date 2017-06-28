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


#include <cstdio>
#include <iostream>
#include <global.h>
#include <crosswalks.h>

static const double SIM_PEDESTRIAN_WIDTH = .5;
static const double SIM_PEDESTRIAN_LENGTH = .3;

using namespace std;
using namespace vlr::rndf;

namespace vlr {
extern void makeStaticPoints(std::vector<perception::StaticObstaclePoint>::iterator pit, int n, double x1, double y1, double x2, double y2, int id);

CrosswalkSimulator::CrosswalkSimulator(vlr::rndf::RoadNetwork& rn, ros::NodeHandle& nh) :
  nh_(nh), lastTime_(0.) {

  readParameters();
  srand(1);

  SimCrosswalkPedestrian temp;
  temp.active = false;
  temp.current_x = 0.;

  double time = driving_common::Time::current();
  double r;
  double dx, dy;

  const TCrosswalkMap& crosswalks = rn.crosswalks();
  TCrosswalkMap::const_iterator cwit = crosswalks.begin(), cwit_end = crosswalks.end();
  for (; cwit != cwit_end; cwit++) {
    std::string name = (*cwit).first;
    vlr::rndf::Crosswalk* cw = (*cwit).second;

    temp.max_x = fabs(hypot(cw->utmY2() - cw->utmY1(), cw->utmX2() - cw->utmX1())) / 2. + 1;
    temp.theta = atan2(cw->utmY2() - cw->utmY1(), cw->utmX2() - cw->utmX1());
    temp.sintheta = sin(temp.theta);
    temp.costheta = cos(temp.theta);

    r = -cw->width() / 2.0 + .5 * SIM_PEDESTRIAN_WIDTH;
    int max_k = cw->width() / SIM_PEDESTRIAN_WIDTH - 1;
    if (max_k < 1) max_k = 1;
    for (int k = 0; k < max_k; ++k) {
      dx = r * cos(temp.theta + M_PI_2);
      dy = r * sin(temp.theta + M_PI_2);

      temp.utm_x = (cw->utmX2() + cw->utmX1()) / 2. + dx;
      temp.utm_y = (cw->utmY2() + cw->utmY1()) / 2. + dy;
      temp.next_time = time + ((double) (rand() % 10000)) / 10000. * params_.max_wait_time;
      sprintf(temp.id, "%s.%d", cw->name().c_str(), k + 1);
      pedestrians_.push_back(temp);

      r += cw->width() / (double) max_k;
    }
  }

  obstacles_.static_point.resize(pedestrians_.size() * 16);
  obstacles_.dynamic_obstacle.resize(pedestrians_.size());
}

CrosswalkSimulator::~CrosswalkSimulator() {
}

void CrosswalkSimulator::update(double time) {
  if (!params_.generate_pedestrians) {
    return;
  }

  static const int pedestrian_obstacle_id = 1024;
//  int32_t num_obstacles = obstacles_.dynamic_obstacle.size();
//  int32_t mark = 0;
  int32_t num_obstacles = 0;

  int id = 1; //moving
  //int id = 0; //static
  std::vector<SimCrosswalkPedestrian>::iterator it;
  std::vector<perception::StaticObstaclePoint>::iterator pit=obstacles_.static_point.begin();
  for (it = pedestrians_.begin(); it != pedestrians_.end(); ++it) {
    if (it->active) {

      if (it->current_x > it->max_x || it->current_x < -it->max_x) {
        it->active = false;
        it->next_time = time + ((double) (rand() % 10000)) / 10000. * params_.max_wait_time;
//        std::cout << "Pedestrian " << it->id << " has finished crossing." << std::endl;
      }
      else {
        it->current_x += it->speed * (time - lastTime_);

        double& ctheta = it->costheta;
        double& stheta = it->sintheta;
        double utm_x = it->utm_x + it->current_x * ctheta;
        double utm_y = it->utm_y + it->current_x * stheta;

        obstacles_.dynamic_obstacle[num_obstacles].confidence = 255;
        obstacles_.dynamic_obstacle[num_obstacles].id = num_obstacles + pedestrian_obstacle_id;
        obstacles_.dynamic_obstacle[num_obstacles].x = utm_x;
        obstacles_.dynamic_obstacle[num_obstacles].y = utm_y;
        obstacles_.dynamic_obstacle[num_obstacles].direction = it->theta;
        obstacles_.dynamic_obstacle[num_obstacles].velocity = it->speed;
        obstacles_.dynamic_obstacle[num_obstacles].length = SIM_PEDESTRIAN_LENGTH;
        obstacles_.dynamic_obstacle[num_obstacles].width = SIM_PEDESTRIAN_WIDTH;
        obstacles_.dynamic_obstacle[num_obstacles].type = OBSTACLE_PEDESTRIAN;
        obstacles_.dynamic_obstacle[num_obstacles].xy_cov = 0.0;
        num_obstacles++;


        makeStaticPoints(pit, 4, utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5 * (SIM_PEDESTRIAN_LENGTH * stheta
            + SIM_PEDESTRIAN_WIDTH * ctheta), utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta), id);
        pit += 4;
        makeStaticPoints(pit, 4, utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5 * (SIM_PEDESTRIAN_LENGTH * stheta
            - SIM_PEDESTRIAN_WIDTH * ctheta), utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta), id);
        pit+=4;
        makeStaticPoints(pit, 4, utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta + SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta - SIM_PEDESTRIAN_WIDTH * ctheta), utm_x + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta + SIM_PEDESTRIAN_WIDTH * ctheta), id);
        pit+=4;
        makeStaticPoints(pit, 4, utm_x + 0.5 * (-SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5
            * (-SIM_PEDESTRIAN_LENGTH * stheta + SIM_PEDESTRIAN_WIDTH * ctheta),
            utm_x + 0.5 * (SIM_PEDESTRIAN_LENGTH * ctheta - SIM_PEDESTRIAN_WIDTH * stheta), utm_y + 0.5 * (SIM_PEDESTRIAN_LENGTH * stheta
                + SIM_PEDESTRIAN_WIDTH * ctheta), id);
       pit+=4;
      }
    }
    if (!it->active) if (time > it->next_time) {
//      std::cout << "Pedestrian " << it->id << " starts crossing now." << std::endl;
      it->active = true;
      if (rand() % 2) {
        it->current_x = -it->max_x;
        it->speed = ((double) (rand() % 1000)) / 1000. * (params_.max_speed - params_.min_speed) + params_.min_speed;
      }
      else {
        it->current_x = it->max_x;
        it->speed = -((double) (rand() % 1000)) / 1000. * (params_.max_speed - params_.min_speed) - params_.min_speed;
      }
    }
  }

  lastTime_ = time;
}

void CrosswalkSimulator::readParameters() {
  nh_.getParam("sim/crosswalk_generate_pedestrians", params_.generate_pedestrians);
  nh_.getParam("sim/crosswalk_max_wait_time", params_.max_wait_time);
  nh_.getParam("sim/crosswalk_min_pedestrian_speed", params_.min_speed);
  nh_.getParam("sim/crosswalk_max_pedestrian_speed", params_.max_speed);
}

} // namespace vlr
