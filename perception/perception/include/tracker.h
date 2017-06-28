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


#ifndef PERCEPTION_TRACKER_H_
#define PERCEPTION_TRACKER_H_

#include <global.h>
#include <aw_roadNetworkSearch.h>
#include <obstacle.h>
#include <kalman_multitracker.h>
#include <linear_kalman_filter/linear_kalman_filter.h>
#include <classifier_stack.h>

namespace perception {

class Tracker {
public:
  Tracker(const std::string& rndf_filename, const tracker_settings_t& tracker_settings, const tracker_kf_settings_t& kf_settings, Perception& perception);
  virtual ~Tracker();

  void trackFrame(double timestamp);
  void trackObstacles(std::vector<boost::shared_ptr<Obstacle> >& detected_obstacles, std::vector<boost::shared_ptr<TrackedObstacle> >& tracked_obstacles, double timestamp);

private:
  void classifierThread();
  void classifyObstacles(std::vector< boost::shared_ptr<Obstacle> >& obstacles, double timestamp);
  void filterObstacleHeight(std::vector<boost::shared_ptr<Obstacle> >& obstacles, double min_height);
  void filterObstaclesTracked(std::vector< boost::shared_ptr<TrackedObstacle> >& obstacles);
  void filterObstaclesRNDF(std::vector< boost::shared_ptr<Obstacle> >& obstacles, double max_distance);
  void filterObstaclesCoarse(std::vector<boost::shared_ptr<Obstacle> >& obstacles);
  void filterObstaclesFine(std::vector<boost::shared_ptr<Obstacle> >& obstacles);
  const std::string& obstacleTypeToString(dgc_obstacle_type type);

private:
  KalmanMultiTracker* tracker_;
  tracker_settings_t tracker_settings_;
  tracker_kf_settings_t kf_settings_;
  ClassifierStack classifier_stack_;
  Perception& perception_;
  vlr::rndf::RoadNetwork rn_;
  vlr::rndf::RoadNetworkSearch* rn_search_;

private:
 static const vector<string> obstacle_type_str_;
};

} // namespace perception

#endif // PERCEPTION_OBSTACLE_H_
