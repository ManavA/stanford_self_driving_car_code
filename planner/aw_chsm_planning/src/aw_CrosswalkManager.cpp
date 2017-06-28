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


#include <sys/types.h>
#include <algorithm>

#include <aw_Topology.hpp>

#include "aw_ChsmPlanner.hpp"
#include "aw_CrosswalkManager.hpp"

using namespace std;

namespace drc = driving_common;

namespace vlr {

#undef TRACE
//#define TRACE(str) cout << "[CrosswalkManager] " << str << endl;
#define TRACE(str)

CrosswalkManager::CrosswalkManager(Topology* top) : top_(top), graph_(NULL), last_occupied_(0) {
	if(!top_) {throw VLRException("zero pointer to topology");}
	graph_ = top_->complete_graph;
  if(!graph_) {throw VLRException("zero pointer to complete graph");}

  std::vector<std::string> cw_names;
  top->distToNextCrosswalk(&cw_names, NULL);
  if(cw_names.empty()) {throw VLRException("could not determine associated crosswalk names");}

  std::vector<std::string>::const_iterator cwnit=cw_names.begin(), cwnit_end=cw_names.end();
  for(; cwnit != cwnit_end; cwnit++) {
    rndf::Crosswalk* cw = const_cast<rndf::RoadNetwork*>(&top->roadNetwork())->crosswalk(*cwnit);
    if(cw) {crosswalks_.push_back(cw);}
  }
  if(crosswalks_.empty()) {throw VLRException("could not determine associated crosswalks");}
}

CrosswalkManager::~CrosswalkManager() {
}

bool CrosswalkManager::isOccupied(std::vector<ObstaclePrediction>& pedestrians) {

  std::vector<rndf::Crosswalk*>::const_iterator cwit=crosswalks_.begin(), cwit_end=crosswalks_.end();
  for(; cwit != cwit_end; cwit++) {
    if(pedestriansOnCrosswalk(*cwit, pedestrians)) {
      printf("CROSSWALK OCCUPIED!\n");
      last_occupied_ = drc::Time::current();
      return true;
    }
  }


  if ((drc::Time::current() - last_occupied_) > min_free_time_) {
    printf("CROSSWALK FREE!\n");
//    printf("CROSSWALK FREE FOR %f sec!\n", min_free_time_);
    return false;
  }

  return true;
}

bool CrosswalkManager::pedestriansOnCrosswalk(rndf::Crosswalk* crosswalk, std::vector<ObstaclePrediction>& pedestrians) {
  std::vector<ObstaclePrediction>::const_iterator pit = pedestrians.begin(), pit_end = pedestrians.end();
  for (; pit != pit_end; pit++) {
//    printf("testing pedestrian\n");
    std::vector<MovingBox>::const_iterator predit = (*pit).predicted_traj_.begin(), predit_end = (*pit).predicted_traj_.end();

    for (; predit != predit_end; predit++) {

      double px = (*predit).x;// + localize_pose_x_offset;
      double py = (*predit).y;// + localize_pose_y_offset;
      double r;

      if (crosswalk->width() == 0) {r = 2.5;} else {r = crosswalk->width() / 2.0;}

      double theta = atan2(crosswalk->utmY2() - crosswalk->utmY1(), crosswalk->utmX2() - crosswalk->utmX1());
      double dx = r * cos(theta + M_PI_2);
      double dy = r * sin(theta + M_PI_2);

      double lx = crosswalk->utmX2() + dx - (crosswalk->utmX1() + dx);
      double ly = crosswalk->utmY2() + dy - (crosswalk->utmY1() + dy);

      double lpx = px - (crosswalk->utmX1() + dx);
      double lpy = py - (crosswalk->utmY1() + dy);

//      printf("%f, %f, %f, %f\n", px, py, crosswalk->utmX1(), crosswalk->utmY1());
      //    printf("%f, %f, %f, %f, %f, %f, %f\n", r, px, py, lx, ly, lpx, lpy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utmX1() - dx - (crosswalk->utmX1() + dx);
      ly = crosswalk->utmY1() - dy - (crosswalk->utmY1() + dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utmX2() + dx - (crosswalk->utmX2() - dx);
      ly = crosswalk->utmY2() + dy - (crosswalk->utmY2() - dy);

      lpx = px - (crosswalk->utmX2() - dx);
      lpy = py - (crosswalk->utmY2() - dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

      lx = crosswalk->utmX1() - dx - (crosswalk->utmX2() - dx);
      ly = crosswalk->utmY1() - dy - (crosswalk->utmY2() - dy);

      if (lx * lpx + ly * lpy < 0) {
        continue;
      }

//      printf("ped inside\n");

      return true;
    }
  }

//  printf("no ped inside\n");
  return false;
}

double CrosswalkManager::min_free_time_ = CROSSWALK_MIN_FREE_TIME;

} // namespace vlr
