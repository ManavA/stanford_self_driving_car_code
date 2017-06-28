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

#include <tracked_obstacle.h>
#include <utils.h>
#include <box.h>

#include <assert.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>


using namespace std;
using namespace Eigen;

namespace perception {

/*bool compareTrackedObstaclesConfidence(const boost::shared_ptr<TrackedObstacle> o1, const boost::shared_ptr<TrackedObstacle> o2)  {
  return (o1->getConfidence() > o2->getConfidence());
}*/

//tracker_kf_settings_p TrackedObstacle::parameters_;

void TrackedObstacle::incorporateClassification(int t, const VectorXf& response) {

  if(!perception_.multiBooster())
    return;

  // 2.0 is really math
  log_odds_ += 2.0 * response - prior_log_odds_;

    // -- Bound the confidence.
  float cap = 10;
  for(int i=0; i<log_odds_.rows(); ++i) {
    if(log_odds_(i) > cap)
      log_odds_(i) = cap;
    else if(log_odds_(i) < -cap)
      log_odds_(i) = -cap;
  }    
  
  // -- Update type.
  int idx;
  float max = log_odds_.maxCoeff(&idx);
  if(max <= 0)
    //type = OBSTACLE_UNKNOWN;
    type = OBSTACLE_CAR;
  else {
    if(perception_.multiBooster()->class_map_.toName(idx).compare("car") == 0)
      type = OBSTACLE_CAR;
    else if(perception_.multiBooster()->class_map_.toName(idx).compare("pedestrian") == 0)
      type = OBSTACLE_PEDESTRIAN;
    else if(perception_.multiBooster()->class_map_.toName(idx).compare("bicyclist") == 0)
      type = OBSTACLE_BICYCLIST;
    else
      assert(0);
  }

  // -- Update type_this_frame_.
  max = response.maxCoeff(&idx);
  if(max <= 0)
    //type_this_frame_ = OBSTACLE_UNKNOWN;
    type_this_frame_ = OBSTACLE_CAR;
  else {
    if(perception_.multiBooster()->class_map_.toName(idx).compare("car") == 0)
      type_this_frame_ = OBSTACLE_CAR;
    else if(perception_.multiBooster()->class_map_.toName(idx).compare("pedestrian") == 0)
      type_this_frame_ = OBSTACLE_PEDESTRIAN;
    else if(perception_.multiBooster()->class_map_.toName(idx).compare("bicyclist") == 0)
      type_this_frame_ = OBSTACLE_BICYCLIST;
    else
      assert(0);
  }
}


void TrackedObstacle::estimateModel()
{
  if (type == OBSTACLE_PEDESTRIAN) {
    pose.yaw = atan2(getYVel(),getXVel());
    getCenterOfPoints(&pose.x, &pose.y);
    width = 1.0;
    length = 1.0;
  } else if (getVelocity() > 0.5) { // more than about 2 mph
    // TODO: use yaw estimate either as:
    //         1) a "measurement" into the system Kalman filter
    //         2) an instantaneous estimate of yaw, using velocity to disambiguate 90 degree rotations
    //         3) a (non-linear) filtered estimate to throw out outliers
    double velocity_yaw = atan2(getYVel(),getXVel());
    //        double geometric_yaw = align_obstacle(obstacle.get());
    //        obstacle->pose.yaw = disambiguate_yaw(geometric_yaw, velocity_yaw);
    pose.yaw = velocity_yaw;
    if(getPoints().size() > 5)
      // @TODO: Fix parameter settings
      bounding_box(this, pose.yaw, 0.5, 0.5);//parameters_->min_car_width, parameters_->min_car_length);
  } else {
    // TODO: use velocity to disambiguate orientation
    if(getPoints().size() > 5) {
      //          double velocity_yaw = atan2(obstacle->getYVel(),obstacle->getXVel());
      double geometric_yaw = align_obstacle(this);
      //          obstacle->pose.yaw = disambiguate_yaw(geometric_yaw, velocity_yaw);
      pose.yaw = geometric_yaw;
//      printf("min_car_width: %f\n", parameters_->min_car_width);
      bounding_box(this, pose.yaw, 0.5, 0.5);//parameters_->min_car_width, parameters_->min_car_length);
    }
  }
}

  
/*
 * Continue a track with new observation
 */
/*
TrackedObstacle::TrackedObstacle (const TrackedObstacle& track, boost::shared_ptr<Obstacle> observation, double timestamp) :
  Obstacle(track),
  filter_(track.filter_),
  lastObservation_(track.lastObservation_),
  pedestrian_label_count(track.pedestrian_label_count),
  observed_(1),
  occluded_(0),
  num_observations_(track.num_observations_),
  confidence_(track.confidence_),
  max_speed_(track.max_speed_),
  timestamp_first_(track.timestamp_first_),
  timestamp_prediction_(track.timestamp_prediction_),
  timestamp_observation_(track.timestamp_observation_),
  log_odds_(track.log_odds_)
{
  assert(num_observations_ > 0);
  if(perception_.multiBooster()) {
    prior_log_odds_ = perception_.multiBooster()->prior_;
  }
  else {
    prior_log_odds_ = VectorXf::Zero(getClassNames().size());
  }
  assert(observation.get() != NULL);

  setConfidence(getConfidence() + parameters_->confidence_increment_obs);

  if (num_observations_ == 1) {
    //    double dt = observation->timestamp - timestamp_observation_;
    double dt = timestamp - timestamp_observation_;
    double last_x, last_y;
    double current_x, current_y;

    lastObservation_->getCenterOfPoints(&last_x, &last_y);
    observation->getCenterOfPoints(&current_x, &current_y);

    x_velocity_ = (current_x - last_x) / dt;
    y_velocity_ = (current_y - last_y) / dt;
    pose.x = current_x;
    pose.y = current_y;


    gsl_vector* state = filter_.getMean();
    vector_set(state, 0, current_x);
    vector_set(state, 1, current_y);
    vector_set(state, 2, x_velocity_);
    vector_set(state, 3, y_velocity_);

    // TODO: update confidence based on range to robot
//    ApplanixPose* pose = applanix_current_pose();
//    double r = range(current_x - pose->smooth_x, current_y - pose->smooth_y);
//    double confidence = logistic(r, parameters_->velodyne_max_range, parameters_->confidence_initial_max, parameters_->confidence_initial_min);

//    double confidence = 0.0;
//    setConfidence(confidence);
  } else {
    gsl_vector* innovation_vector = calculateInnovation(observation);
    filter_.observationUpdate(innovation_vector);
    vector_free(innovation_vector);
    getCenterOfPoints(&pose.x, &pose.y);

    pose.x = vector_get(filter_.getMean(), 0);
    pose.y = vector_get(filter_.getMean(), 1);
    x_velocity_ = vector_get(filter_.getMean(), 2);
    y_velocity_ = vector_get(filter_.getMean(), 3);

  }

  lastObservation_ = observation;
//  timestamp_observation_ = observation->timestamp;
  timestamp_observation_ = timestamp;
  num_observations_++;
}
*/

/*
 * Initialize a new track with first observation
 */
/*TrackedObstacle::TrackedObstacle(int id, const KalmanFilter& filter, boost::shared_ptr<Obstacle> observation, double timestamp) :
  Obstacle(id),
  filter_(filter),
  lastObservation_(observation),
  pedestrian_label_count(0),
  observed_(1),
  occluded_(0),
  num_observations_(1),
  confidence_(0.0),
  max_speed_(0),
  timestamp_first_(timestamp),
  timestamp_prediction_(timestamp),
  timestamp_observation_(timestamp),
  x_velocity_(0.0),
  y_velocity_(0.0)
{
  assert(num_observations_ > 0);
  if(perception_.multiBooster()) {
    prior_log_odds_ = perception_.multiBooster()->prior_;
    log_odds_ = perception_.multiBooster()->prior_;
  }
  else {
    prior_log_odds_ = VectorXf::Zero(getClassNames().size());
    log_odds_ = VectorXf::Zero(getClassNames().size());
  }

  assert(observation.get() != NULL);
}*/


TrackedObstacle::TrackedObstacle(int id, boost::shared_ptr<Obstacle> observation, double timestamp, Perception& perception) :
  Obstacle(id, perception),
  pedestrian_label_count(0),
  num_observations_(0),
  timestamp_first_(timestamp),
  timestamp_prediction_(timestamp),
  lastObservation_(observation),
  timestamp_observation_(timestamp)
{
  if(perception_.multiBooster()) {
    prior_log_odds_ = perception_.multiBooster()->prior_;
    log_odds_ = perception_.multiBooster()->prior_;
  }
  else {
    prior_log_odds_ = VectorXf::Zero(getClassNames().size());
    log_odds_ = VectorXf::Zero(getClassNames().size());
  }
}

/*
 * Continue track with no observation, dings confidence accordingly
 */
/*TrackedObstacle::TrackedObstacle (const TrackedObstacle& track, double timestamp) :
  Obstacle(track),
  filter_(track.filter_),
  lastObservation_(track.lastObservation_),
  pedestrian_label_count(track.pedestrian_label_count),
  observed_(0),
  occluded_(track.occluded_),
  num_observations_(track.num_observations_),
  confidence_(track.confidence_),
  max_speed_(track.max_speed_),
  timestamp_first_(track.timestamp_first_),
  timestamp_prediction_(track.timestamp_prediction_),
  timestamp_observation_(track.timestamp_observation_),
  x_velocity_(track.x_velocity_),
  y_velocity_(track.y_velocity_),
  log_odds_(track.log_odds_)
{
  assert(num_observations_ > 0);
  classified_this_frame_ = false;
  if(perception_.multiBooster()) {
    prior_log_odds_ = perception_.multiBooster()->prior_;
  }
  else {
    prior_log_odds_ = VectorXf::Zero(getClassNames().size());
  }

  
  observed_ = 0;
  
  // TODO: test for occlusion
  setConfidence(confidence_ + parameters_->confidence_increment_unobs);
}*/

/*
 * Straight copy constructor
 */
TrackedObstacle::TrackedObstacle (const TrackedObstacle& o) :
  Obstacle(o),
  num_observations_(o.num_observations_),
  timestamp_first_(o.timestamp_first_),
  timestamp_prediction_(o.timestamp_prediction_),
  log_odds_(o.log_odds_),
  lastObservation_(o.lastObservation_),
  timestamp_observation_(o.timestamp_observation_)
{
  assert(num_observations_ > 0);
  if(perception_.multiBooster()) {
    prior_log_odds_ = perception_.multiBooster()->prior_;
  }
  else {
    prior_log_odds_ = VectorXf::Zero(getClassNames().size());
  }

}

TrackedObstacle::~TrackedObstacle() {

}

void TrackedObstacle::update(double timestamp)
{
  pose.x = filter->mu_[0];
  pose.y = filter->mu_[1];
  x_velocity_ = filter->mu_[2];
  y_velocity_ = filter->mu_[3];
  estimateModel();
  pose.x = filter->mu_[0];
  pose.y = filter->mu_[1];
}

void TrackedObstacle::update(boost::shared_ptr<Obstacle> obstacle, double timestamp)
{
  lastObservation_ = obstacle;
  incorporateClassification(obstacle->type, obstacle->response_);
  points_ = obstacle->getPoints();
  timestamp_observation_ = obstacle->time_; 
  num_observations_++;
  pose.x = filter->mu_[0];
  pose.y = filter->mu_[1];
  x_velocity_ = filter->mu_[2];
  y_velocity_ = filter->mu_[3];
  estimateModel();
  //pose.x = filter->mu_[0];
  //pose.y = filter->mu_[1];
}


double TrackedObstacle::getXVel() const {
  return x_velocity_;
}

double TrackedObstacle::getYVel() const {
  return y_velocity_;
  //return vector_get(filter_.getMean(), 3);
}

double TrackedObstacle::getVelocity() const {
  return range(getXVel(), getYVel());
}
/*
double TrackedObstacle::getXVar() const {
  return matrix_get(filter_.getCovariance(), 0, 0);
}

double TrackedObstacle::getYVar() const {
  return matrix_get(filter_.getCovariance(), 1, 1);
}

double TrackedObstacle::getXYCov() const {
  return matrix_get(filter_.getCovariance(), 1, 0);
}

void TrackedObstacle::transitionUpdate(double timestamp) {
  double dt = timestamp - timestamp_prediction_;

  timestamp_prediction_ = timestamp;
  setConfidence(getConfidence() + dt * parameters_->confidence_decay);
  filter_.transitionUpdate();

  pose.x = vector_get(filter_.getMean(), 0);
  pose.y = vector_get(filter_.getMean(), 1);
  x_velocity_ = vector_get(filter_.getMean(), 2);
  y_velocity_ = vector_get(filter_.getMean(), 3);
}

// TODO: add ICP as option for innovation
gsl_vector* TrackedObstacle::calculateInnovation(boost::shared_ptr<Obstacle> observation) {
//  double x, y;
//  lastObservation_->getCenterOfPoints(&x, &y);
//
//  gsl_vector* last_state = vector_init(2);
//  vector_set(last_state, 0, x);
//  vector_set(last_state, 1, y);
//
//  gsl_vector* state = filter_.predictTransition(last_state);
  gsl_vector* pobs = filter_.predictObservation(filter_.getMean());

  double x, y;
  observation->getCenterOfPoints(&x, &y);
  gsl_vector* innovation_vector = vector_init(pobs->size);
  vector_set_zero(innovation_vector);
  vector_set(innovation_vector, 0, x);
  vector_set(innovation_vector, 1, y);

  vector_subtract_ip(innovation_vector, pobs);


  if (vector_get(pobs, 0) == 0.0) {
    x = x;
  }
//  printf("predicted: %f, %f\n", vector_get(pobs, 0), vector_get(pobs, 1));
//  printf("observed: %f, %f\n", x, y);
//  printf("innovation: %f, %f\n", vector_get(innovation_vector, 0), vector_get(innovation_vector, 1));

  vector_free(pobs);

  return innovation_vector;
}

double computeICPDistance(const sensor_msgs::PointCloud& pc1, const sensor_msgs::PointCloud& pc2) { 
  // Put the points into a kdtree for quick lookups.
  cloud_kdtree::KdTree* kdt = new cloud_kdtree::KdTreeANN(pc1);

  // Accumulate cost as mean distance to closest point in last frame.
  double cost = 0;
  for(size_t i=0; i<pc2.get_points_size(); ++i) {
    vector<int> indices;
    vector<float> distances;
    kdt->nearestKSearch(pc2.points[i], 1, indices, distances);
    //      cout << "Point " << i << ": Adding distance of " << distances[0] << endl;
    cost += distances[0];
  }
  cost /= (double)pc2.get_points_size();
  delete kdt;
  return cost;
}
  
// Returns false if the two obstacles don't look the same.
bool icpFilter(Obstacle& obs1, Obstacle& obs2) {
  std::vector<point3d_t>& pts1 = obs1.getPoints();
  std::vector<point3d_t>& pts2 = obs2.getPoints();
  
  // -- Get the centroids.
  double x1, y1, z1, x2, y2, z2;
  obs1.getCenterOfPoints(&x1, &y1, &z1);
  obs2.getCenterOfPoints(&x2, &y2, &z2);
  
  // -- Convert into ROS pointclouds so we can use their kdtree.
  sensor_msgs::PointCloud cloud1;
  cloud1.set_points_size(pts1.size());
  cloud1.set_channels_size(0);
  for(size_t i=0; i<pts1.size(); ++i) {
    cloud1.points[i].x = pts1[i].x - x1;
    cloud1.points[i].y = pts1[i].y - y1;
    cloud1.points[i].z = pts1[i].z - z1;
  }
  sensor_msgs::PointCloud cloud2;
  cloud2.set_points_size(pts2.size());
  cloud2.set_channels_size(0);
  for(size_t i=0; i<pts2.size(); ++i) {
    cloud2.points[i].x = pts2[i].x - x2;
    cloud2.points[i].y = pts2[i].y - y2;
    cloud2.points[i].z = pts2[i].z - z2;
  }

  // -- Compute symmetric ICP distance.
  double cost = 0;
  cost += computeICPDistance(cloud2, cloud1);
  cost += computeICPDistance(cloud1, cloud2);
  cost /= 2.0;

  //float cost_thresh = 0.0075;
  float cost_thresh = 0.075;
  if(cost > cost_thresh)
    return false;

  return true;
}
*/ 
/*
 * This is only intended as a rough heuristic to eliminate obviously wrong correspondances
 * with as little computation as possible.
 */
/*
double TrackedObstacle::scoreObservation(Obstacle* observation) {
  // TODO: integrate max_dist with parameters
  static double max_dist = 3.0;

  double score = 0.0;

  if (num_observations_ == 1) {
    double x, y, x2, y2;
    observation->getCenterOfPoints(&x, &y);
    lastObservation_->getCenterOfPoints(&x2, &y2);
    double dist = range(x2-x, y2-y);

    if (dist > max_dist) {
      return 0.0;
    }

    int dPoints = abs(observation->getSize() - lastObservation_->getSize());

    // TODO: parameterize and tune
    score = 1.0 / (0.1 * dPoints + dist);
  } else {
    // TODO: use velocity estimates to threshold distance, or just transform center points
    //       using full filter state

    double x, y, x2, y2;
    observation->getCenterOfPoints(&x, &y);
    lastObservation_->getCenterOfPoints(&x2, &y2);
    double dist = range(x2-x, y2-y);

    // TODO: a score function that's a function of dist, num points, variance of points, etc...
    if (dist > max_dist) {
      return 0.0;
    }

    // Option to collect stable tracks only.  Experimental and meant for offline use.
    if(getenv("ICP_FILTER") && !icpFilter(*observation, *lastObservation_))
      return 0.0;
    
    int dPoints = abs(observation->getSize() - lastObservation_->getSize());

    // TODO: parameterize and tune
    score = 1.0 / (0.1 * dPoints + dist);
  }

  return score;
}
*/
void TrackedObstacle::markDynamic(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter) {
  lastObservation_->pose = pose;
  lastObservation_->width = width;
  lastObservation_->length = length;
    // TODO: velocity = 0: do not clear static map in front of dynamic obstacle (prediction for next 0.5s)
    // until orientation and velocity are estimated more robustly
  lastObservation_->markDynamic(grid, obstacles_s, counter, 0);
  // lastObservation_->markDynamic(grid, obstacles_s, counter, this->getVelocity());
}

void TrackedObstacle::populatePoints() {
  this->points_ = lastObservation_->getPoints();
}

float TrackedObstacle::maxHeight() {
  return 0.0; // TODO:??
}

} // namespace perception

