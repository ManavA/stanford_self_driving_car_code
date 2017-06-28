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


#ifndef PERCEPTION_OBSTACLE_H_
#define PERCEPTION_OBSTACLE_H_

#include <global.h>
#include <grid.h>
#include <vector>
#include <aw_roadNetwork.h>

#include <perception_types.h>
#include <multibooster_support2.h>

namespace perception {

class Perception;

class Obstacle {
private:
  double x_center_;
  double y_center_;
  double z_center_;

protected:
  std::vector<point3d_t> points_;
  vlr::rndf::Lane* lane_;
  double rndfDist_;
  bool matched_;
  Perception& perception_;

  virtual void populatePoints();
public:
  int id;
//  double confidence;
  dgc::dgc_pose_t pose;
  double length;
  double width;
  dgc_obstacle_type type;
  dgc_obstacle_type type_this_frame_;
  bool classified_this_frame_;
  double time_;  // TODO: take better care of observation timestamps
//  double timestamp_;            // TODO: need to set this

  Eigen::VectorXf response_;
  
  Obstacle(int id, Perception& perception);
  Obstacle(const Obstacle& o);
  virtual ~Obstacle();
  std::vector<point3d_t>& getPoints();
  void merge(const Obstacle& o);
  bool getCenterOfPoints(double *x, double *y);
  bool getCenterOfPoints(double *x, double *y, double *z);
  vlr::rndf::Lane *getLane() const { return lane_; }
  void setLane(vlr::rndf::Lane *lane) { this->lane_ = lane; }
  double getRndfDist() const {return rndfDist_; }
  void setRndfDist(double dist) {this->rndfDist_ = dist; }

  virtual int  getSize() = 0;
  virtual void markDynamic(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter, double velocity);

  virtual float maxHeight() = 0;

  bool getMatched() const { return matched_; }
  void setMatched(bool matched_) { this->matched_ = matched_; }
};

class GridObstacle : public Obstacle {
private:
  const Grid<PerceptionCell>& grid_;
  std::vector<PerceptionCell*> cells_;
  
protected:
  virtual void populatePoints();
  void pointsInCell(const PerceptionCell& cell, std::vector<point3d_t>& points);

public:

  GridObstacle(int id, const Grid<PerceptionCell>& grid, Perception& perception);
  GridObstacle (const GridObstacle& o);
  virtual ~GridObstacle();
  
  void addCell(PerceptionCell& cell);
  void clear();
  std::vector<PerceptionCell*>& getCells();
  virtual int  getSize();
  void merge(const GridObstacle& o);
  virtual float maxHeight();
//  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
};

class LaserObstacle : public Obstacle {
protected:
  virtual void populatePoints();
  double min_z_, max_z_;

public:
  LaserObstacle(int id, Perception& perception);
  LaserObstacle (const LaserObstacle& o);
  virtual ~LaserObstacle();

  void addPoint(laser_point_p point);
  void reserve(int count);
  void clear();
  virtual int  getSize();
  void merge(const LaserObstacle& o);

  virtual float maxHeight();

//  virtual void markDynamic(dgc_grid_p grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter);
};

class Observation {

};

class LaserObservation : public Observation {

};

class RadarObservation : public Observation {
public:
  RadarObservation();
  RadarObservation (const RadarObservation& o);
  virtual ~RadarObservation();

  int id;

  double x;
  double y;
  double yaw;
  double velocity;
  double x_vel;
  double y_vel;
};

typedef std::vector<Obstacle*> TObstacleVec;

class Obstacles {
public:
  Obstacles() {};
  ~Obstacles() {};
  dgc::dgc_pose_t robot_pose;  // pose of the robot to calculate global obstacle pose from local map
  double timestamp;
  std::vector<Obstacle*> obstacles;
};

} // namespace perception

#endif // PERCEPTION_OBSTACLE_H_
