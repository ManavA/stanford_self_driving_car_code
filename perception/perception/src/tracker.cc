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


#include <assert.h>
#include <lltransform.h>
#include <aw_roadNetworkSearch.h>

#include <perception.h>
#include <tracker.h>
#include <utils.h>
#include <segment.h>
#include <laser_segment.h>
#include <box.h>

#include <iostream>
#include <pthread.h>
#include <queue>

using namespace std;
using namespace std::tr1;
using namespace dgc;

using namespace vlr::rndf;

namespace drc = driving_common;

USING_PART_OF_NAMESPACE_EIGEN;

namespace perception {

//TODO: turn into param?
#define MIN_CONFIDENCE_CLUSTER      1.0

Tracker::Tracker(const std::string& rndf_filename, const tracker_settings_t& tracker_settings, const tracker_kf_settings_t& kf_settings, Perception& perception) :
  tracker_settings_(tracker_settings), kf_settings_(kf_settings), perception_(perception) {
  tracker_ = new KalmanMultiTracker(kf_settings.correspondence_threshold, kf_settings.pruning_threshold, kf_settings.measurement_variance,
      kf_settings.position_variance, kf_settings.velocity_variance, kf_settings.initial_position_variance, kf_settings.initial_velocity_variance, perception_);

  rn_.loadRNDF(rndf_filename);
  rn_search_ = new RoadNetworkSearch(&rn_);
}

Tracker::~Tracker() {
  delete tracker_;
  delete rn_search_;
}

const string& Tracker::obstacleTypeToString(dgc_obstacle_type type) {
  if (type <= OBSTACLE_BICYCLIST) {
    return obstacle_type_str_[(int)type];
  }
  else return obstacle_type_str_[0];
}

void Tracker::classifierThread() {
  // TODO: You may want to match this with the number of threads you're using...
  static const unsigned int batch_size = 8;

  vector<boost::shared_ptr<Obstacle> > filtered;
  filtered.reserve(batch_size);
  while (ros::ok()) {
    //  while (!classifier_stack_.inbox_empty()) {
    pthread_mutex_lock(&perception_.shutdownMutex());
    if (perception_.shutdown() == 2) {
      printf("Classifier shutting down\n");
      perception_.setShutdown(1);
      pthread_mutex_unlock(&perception_.shutdownMutex());
      break;
    }
    pthread_mutex_unlock(&perception_.shutdownMutex());
    if (classifier_stack_.inbox_empty()) {
      usleep(1000);
    }
    else {
      filtered.clear();

      // process batch_size at a time
      while ((!classifier_stack_.inbox_empty()) && (filtered.size() < batch_size)) {
        filtered.push_back(classifier_stack_.pop_inbox());
      }

      //      printf("classifying %d obstacles\n", filtered.size());

      // -- Convert dgc clouds into Eigen clouds.
      vector<boost::shared_ptr<MatrixXf> > clouds(filtered.size());
      vector<boost::shared_ptr<VectorXf> > intensities(filtered.size());
      for (size_t i = 0; i < filtered.size(); ++i) {
        dgcToEigen(filtered[i]->getPoints(), &clouds[i], &intensities[i]);
      }

      // -- Classify them.
      vector<VectorXf> responses = perception_.classifierPipeline()->classify(clouds, intensities);
      // -- Set the label for each.
      assert(responses.size() == filtered.size());
      for (size_t i = 0; i < filtered.size(); ++i) {
        boost::shared_ptr<Obstacle> observation = filtered[i];
        observation->response_ = responses[i];
        int idx;
        float max = observation->response_.maxCoeff(&idx);
        if (max <= 0) observation->type = OBSTACLE_UNKNOWN;
        else {
          printf("found %s\n", perception_.multiBooster()->class_map_.toName(idx).c_str());
          if (perception_.multiBooster()->class_map_.toName(idx).compare("car") == 0) observation->type = OBSTACLE_CAR;
          else if (perception_.multiBooster()->class_map_.toName(idx).compare("pedestrian") == 0) observation->type = OBSTACLE_PEDESTRIAN;
          else if (perception_.multiBooster()->class_map_.toName(idx).compare("bicyclist") == 0) observation->type = OBSTACLE_BICYCLIST;
else          assert(0);
        }

        classifier_stack_.push_outbox(observation);
      }
    }
  }
}

bool compareObstaclesRndfDist (boost::shared_ptr<Obstacle> a, boost::shared_ptr<Obstacle> b) {
  return (a->getRndfDist() < b->getRndfDist());
}

void Tracker::classifyObstacles(vector< boost::shared_ptr<Obstacle> >& obstacles, double timestamp) {
  if(getenv("HALT_EARLY")) {
    static int cts = 0;
    cts++;
    if(cts == 5)
    exit(0);
  }

  if(!perception_.multiBooster() || obstacles.empty()) {return;}

  static boost::thread thread(boost::bind(&Tracker::classifierThread, this));

  // sort by distance to rndf
  sort(obstacles.begin(), obstacles.end(), compareObstaclesRndfDist);

  // Add new obstacles to classify()
  int cleared = classifier_stack_.clear_inbox();
  printf("unable to classify %d obstacles\n", cleared);

  int num_to_classify = 0;
  for(size_t i = 0; i < obstacles.size(); ++i) {

    if(obstacles[i]->getPoints().size() < 10) {
      // @TODO move this somewhere more appropriate.  
      obstacles[i]->response_ = 0.5 * perception_.multiBooster()->prior_;
    }
    else {
      // @TODO move this somewhere more appropriate.  
      classifier_stack_.push_inbox(obstacles[i]);
      num_to_classify++;
    }
  }

  perception_.segmentedObstacles().clear();
  // Handle classified obstacles
//  double starttime = drc::Time::current();
  while ((int)perception_.segmentedObstacles().size() < num_to_classify) {
    //if(starttime + 0.1 < drc::Time::current())
    //  break;
    while(classifier_stack_.outbox_empty()) {
      pthread_mutex_lock(&perception_.shutdownMutex());
      if(perception_.shutdown() > 0) {
        pthread_mutex_unlock(&perception_.shutdownMutex());
        thread.join();
        return;
      }
      pthread_mutex_unlock(&perception_.shutdownMutex());
      usleep(100);
    }
    boost::shared_ptr<Obstacle> observation = classifier_stack_.pop_outbox();
    perception_.segmentedObstacles().push_back(observation);
    //tracker_->classificationUpdate(observation, observation->time_);
  }
  //  classifierThread();
}

//   vector<Object*> objs;
//   if(perception_.multiBooster()) {
//     // -- Collect features, put into objects.
//     vector< vector<point3d_t> > clusters(obstacles.size());
//     size_t cluster_id = 0;
//     for (vector< boost::shared_ptr<Obstacle> >::iterator it = obstacles.begin(), end = obstacles.end(); it != end; it++, ++cluster_id) {
//       boost::shared_ptr<Obstacle> obstacle = *it;
//       std::vector<point3d_t>& points = obstacle->getPoints();
//       clusters[cluster_id] = points;
//     }
//     if(!clusters.empty()) {
// //      cout << "z: " << get_z() << endl;
//       collectDatasetForSpin(clusters, get_z(), &objs); //TODO: replace 0 with robot z height.
//     }
//   }

//   size_t cluster_id = 0;
//   //  int count = 0;
//   for (vector< boost::shared_ptr<Obstacle> >::iterator it = obstacles.begin(), end = obstacles.end(); it != end; it++, ++cluster_id) {
//     boost::shared_ptr<Obstacle> obstacle = *it;
//     obstacle->type = OBSTACLE_UNKNOWN;

//     if(perception_.multiBooster() && !objs.empty() && objs[cluster_id]) {
//       obstacle->response_ = perception_.multiBooster()->treeClassify(*objs[cluster_id]);
//       int idx;
//       float max = obstacle->response_.maxCoeff(&idx);
//       if(max <= 0)
//         obstacle->type = OBSTACLE_UNKNOWN;
//       else {
//         if(perception_.multiBooster()->class_map_.toName(idx).compare("car") == 0)
//           obstacle->type = OBSTACLE_CAR;
//         else if(perception_.multiBooster()->class_map_.toName(idx).compare("pedestrian") == 0)
//           obstacle->type = OBSTACLE_PEDESTRIAN;
//         else if(perception_.multiBooster()->class_map_.toName(idx).compare("bicyclist") == 0)
//           obstacle->type = OBSTACLE_BICYCLIST;
//         else
//           assert(0);
//       }
//     }
//   }

//   for(size_t i=0; i<objs.size(); ++i) {
//     delete objs[i];
//   }


//inline double wrap_angle(double y) {
//  if (y > M_PI)
//    y = M_2_PI - y;
//  return y;
//}
//
//inline double dist_angles(double y1, double y2) {
//  double dist = y1 - y2;
//
//  while (dist < 0)
//    dist += M_2_PI;
//  while (dist > M_PI)
//    dist -= M_2_PI;
//
//  return fabs(dist);
//}
//
//// coarse yaw is coarse, but is in the right quadrant
//// refined yaw is fine, but ambigous relative to quadrant
//double disambiguate_yaw(double refined_yaw, double coarse_yaw) {
//  double yaw = refined_yaw - M_PI;
//  double dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  yaw += M_PI_2;
//  dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  yaw += M_PI_2;
//  dist = dist_angles(yaw, coarse_yaw);
//  if (dist < M_PI_2)
//    return yaw;
//
//  return yaw + M_PI_2;
//}

void Tracker::trackObstacles(vector< boost::shared_ptr<Obstacle> >& detected_obstacles, vector< boost::shared_ptr<TrackedObstacle> >& tracked_obstacles, double timestamp) {
  /*vector<MultiTrackerMeasurement> observations;
   vector< boost::shared_ptr<Obstacle> >::iterator it;
   for(it = detected_obstacles.begin(); it != detected_obstacles.end(); it++) {
   observations.push_back(MultiTrackerMeasurement());
   observations.back().centroid_(0) = (*it)->pose.x;
   observations.back().centroid_(1) = (*it)->pose.y;
   // @TODO: give each obstacle it's own timestamp
   observations.back().timestamp = timestamp;
   }*/
  tracker_->step(detected_obstacles, timestamp);

  //  tracker_->trackGetDynamicObstacles(obstacles_predicted, MIN_CONFIDENCE_CLUSTER);
  //  std::sort(obstacles_predicted.begin(), obstacles_predicted.end(), compareObstaclesConfidence);
  //  if(params.predictive_clustering) {
  //    cluster_obstacles_bb(&frame, &predicted_obstacle_list_, &segmented_obstacle_list_, local_x_vel, local_y_vel, 1);
  //  }
  tracked_obstacles.clear();
  list<boost::shared_ptr<TrackedObstacle> >::iterator it;
  for(it = tracker_->tracks_.begin(); it != tracker_->tracks_.end(); it++) {
    tracked_obstacles.push_back(*it);
  }
}

void Tracker::filterObstacleHeight(vector< boost::shared_ptr<Obstacle> >& obstacles, double min_height) {
  //  static double min_width = 0.5;
  //  static double max_width = 25.0;
  //
  //  static double min_length = 0.5;
  //  static double max_length = 25.0;

  vector< boost::shared_ptr<Obstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    float height = (*it)->maxHeight();
    if (height < min_height) {
      it = obstacles.erase(it);
      continue;
    }

    ++it;
  }
}

void Tracker::filterObstaclesTracked(vector< boost::shared_ptr<TrackedObstacle> >& obstacles) {
  static double max_area = 50;
  static double min_area = 1.0;
  vector< boost::shared_ptr<TrackedObstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    boost::shared_ptr<TrackedObstacle> track = *it;

    if (track->type != OBSTACLE_UNKNOWN) {
      ++it;
      continue;
    }

    double area = track->length * track->width;
    if ((area > max_area)) {
      it = obstacles.erase(it);
      continue;
    }
    if ((area < min_area)) {
      it = obstacles.erase(it);
      continue;
    }

    // TODO: filter small obstacles whose movement does not match velocity over time

    ++it;
  }
  //  printf("Erased %d obstacles from tracked list\n", erased);
}


void Tracker::filterObstaclesRNDF(vector< boost::shared_ptr<Obstacle> >& obstacles, double max_distance) {
  if(!tracker_settings_.filter_rndf) {return;}

  double utm_x, utm_y;
  double distance;

  Lane* l;

  drc::GlobalPose pose = perception_.pose(drc::Time::current());
  double local_x_off = pose.offsetX();
  double local_y_off = pose.offsetY();

  vector< boost::shared_ptr<Obstacle> >::iterator it;
  for(it = obstacles.begin(); it != obstacles.end(); )
  {
    boost::shared_ptr<Obstacle> obstacle = *it;
    if (obstacle->type != OBSTACLE_UNKNOWN) {
      ++it;
      continue;
    }
    if (obstacle->getRndfDist() < 0) {
      double x,y;
      if (obstacle->getCenterOfPoints(&x, &y)) {
        // go to utm coordinates (from local coordinates)
        utm_x = x + local_x_off;
        utm_y = y + local_y_off;

        rn_search_->closest_lane(utm_x, utm_y, l, distance);
        obstacle->setLane(l);
        obstacle->setRndfDist(distance);
      }
      else {
        it = obstacles.erase(it);
        continue;
      }
    }

    // coarse filtering
    if(obstacle->getRndfDist() > max_distance) {
      it = obstacles.erase(it);
      continue;
    }

    ++it;
  }
  //  printf("%u obstacles remaining after RNDF filtering.\n", obstacles.size());
}

void Tracker::filterObstaclesFine(vector< boost::shared_ptr<Obstacle> >& obstacles) {
  filterObstaclesRNDF(obstacles, tracker_settings_.filter_rndf_max_distance);
  printf("(3)Obstacles: %d\n", (int)obstacles.size());
}

void Tracker::filterObstaclesCoarse(vector< boost::shared_ptr<Obstacle> >& obstacles) {
  printf("(0)Obstacles: %d\n", (int)obstacles.size());
  filterObstacleHeight(obstacles, perception_.segmentationSettings().min_height);
  printf("(1)Obstacles: %d\n", (int)obstacles.size());
  filterObstaclesRNDF(obstacles, tracker_settings_.filter_rndf_max_pedestrian_distance);
  printf("(2)Obstacles: %d\n", (int)obstacles.size());
  //perception_merge_near_obstacles(obstacles);
  //printf("(3)Obstacles: %d\n", obstacles->obstacles.size());
}

void Tracker::trackFrame(double timestamp) {
#ifndef USE_GRID_SEGMENTER
  static LaserSegmenter segmenter(rings, perception_);
#else
  static GridSegmenter segmenter(perception_.segmentationSettings(), perception_);
#endif

  static double last_timestamp = 0;

  if (timestamp == last_timestamp) // just for simulation purposes (perception_viz)
  return;
  last_timestamp = timestamp;

  double time = drc::Time::current();
  perception_.staticObstacles()->timestamp = timestamp;

#ifndef USE_GRID_SEGMENTER
  segmenter.segmentVelo(lscan, &perception_.segmentedObstacles());
#else
  segmenter.segmentGrid(perception_.staticObstacles(), &perception_.segmentedObstacles(), perception_.trackedObstacles(), timestamp);
#endif
  display_time("Segmenting", time); time = drc::Time::current();
  filterObstaclesCoarse(perception_.segmentedObstacles());

  display_time("Filtering", time); time = drc::Time::current();
  classifyObstacles(perception_.segmentedObstacles(), timestamp);

  display_time("Classification", time); time = drc::Time::current();
  filterObstaclesFine(perception_.segmentedObstacles());

  display_time("Filtering", time); time = drc::Time::current();
  printf("(0)Tracks: %d\n", (int)perception_.trackedObstacles().size());
  trackObstacles(perception_.segmentedObstacles(), perception_.trackedObstacles(), timestamp);

  printf("(1)Tracks: %d\n", (int)perception_.trackedObstacles().size());
  display_time("Tracking", time);
  filterObstaclesTracked(perception_.trackedObstacles());
  printf("(2)Tracks: %d\n", (int)perception_.trackedObstacles().size());

  vector< boost::shared_ptr<TrackedObstacle> >::iterator it;
  for(it = perception_.trackedObstacles().begin(); it != perception_.trackedObstacles().end(); it++) {
    boost::shared_ptr<TrackedObstacle> track = (*it);
    track->markDynamic(perception_.grid(), perception_.staticObstacles(), perception_.counter());
  }
  // -- Report stats.
  map<int, int> num_identified;
  for(int i=0; i<3; ++i)
  num_identified[i] = 0;
  //  cout << perception_.multiBooster()->class_map_.serialize() << endl;
  for(size_t i=0; i<perception_.trackedObstacles().size(); ++i) {
    //    cout << perception_.trackedObstacles()[i]->type << " with id " << perception_.trackedObstacles()[i]->id << ": " << perception_.trackedObstacles()[i]->getLogOdds().transpose() << endl;
    num_identified[perception_.trackedObstacles()[i]->type]++;
  }
  //  cout << num_identified[OBSTACLE_UNKNOWN] << " background." << endl;
  //  cout << num_identified[OBSTACLE_CAR] << " car." << endl;
  //  cout << num_identified[OBSTACLE_PEDESTRIAN] << " ped." << endl;
  //  cout << num_identified[OBSTACLE_BICYCLIST] << " bike." << endl;

  //  classifyObstacles(perception_.trackedObstacles());
  //  perception_add_radar(perception_.trackedObstacles(), timestamp);

  //  if (settings_.gls_output) {
  //    GLS& gls = perception_.gls();
  //    setup GLS header //
  //    gls.coordinates = GLSOverlay::SMOOTH_COORDINATES;
  //    gls.origin_x = 0;
  //    gls.origin_y = 0;
  //    gls.origin_z = perception_.pose(time).z();
  //    gls.color3f(1.0, 0.0, 0.0);
  //    gls.lineWidth(2.0);
  //    for (int i=0; i < perception_.trackedObstacles().size(); i++) {
  //      gls.pushMatrix();
  //      boost::shared_ptr<TrackedObstacle> obstacle = perception_.trackedObstacles()[i];
  //      gls.translatef(obstacle->pose.x, obstacle->pose.y, 0);
  //      gls.rotatef(dgc_r2d(obstacle->pose.yaw), 0, 0, 1);
  //
  //      float l = obstacle->length;
  //      float w = obstacle->width;
  //      gls.begin(GLSOverlay::LINE_LOOP);
  //      gls.vertex3f(l / 2, w / 2, 0);
  //      gls.vertex3f(l / 2, -w / 2, 0);
  //      gls.vertex3f(-l / 2, -w / 2, 0);
  //      gls.vertex3f(-l / 2, w / 2, 0);
  //      gls.vertex3f(l / 2, 0, 0);
  //      gls.vertex3f(l / 2 + obstacle->getVelocity(), 0, 0);
  //      gls.vertex3f(l / 2, 0, 0);
  //      gls.vertex3f(-l / 2, -w / 2, 0);
  //      gls.vertex3f(-l / 2, w / 2, 0);
  //      gls.end();
  //
  //      gls.popMatrix();
  //    }
  //  }

}

const vector<string> Tracker::obstacle_type_str_ = {"unknown", "car", "pedestrian", "bicyclist"};

} // namespace perception

