/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Radu Bogdan Rusu and Alex Sorokin*/
#include <vector>


// ROS core

// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>

// tf
#include <tf/transform_listener.h>

// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

namespace point_cloud_mapping
{
class NormalEstimationInProc
{
protected:
  ros::NodeHandle node_;
  
public:

  // ROS messages
  sensor_msgs::PointCloud cloud_, cloud_down_, cloud_normals_;
  
  tf::TransformListener tf_;
  
  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  std::vector<std::vector<int> > points_indices_;
  
  // Parameters
  bool compute_moments_;
  double radius_;
  int k_;
  // additional downsampling parameters
  int downsample_;
  geometry_msgs::Point leaf_width_;
  double cut_distance_;
  
  std::vector<cloud_geometry::Leaf> leaves_;
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  NormalEstimationInProc();
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  virtual ~NormalEstimationInProc ();
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void updateParametersFromServer ();
  
  /** \brief Get the view point from where the scans were taken in the incoming PointCloud message frame
   * \param cloud_frame the point cloud message TF frame
   * \param viewpoint_cloud the resultant view point in the incoming cloud frame
   * \param tf a pointer to a TransformListener object
   */
  void getCloudViewPoint (const std::string &cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud, tf::TransformListener &tf);
  
  
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Processing
  void process_cloud (const sensor_msgs::PointCloud& cloud_, sensor_msgs::PointCloud& cloud_normals_);
  
};


}
