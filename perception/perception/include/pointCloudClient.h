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


#ifndef VLR_POINTCLOUDCLIENT_H_
#define VLR_POINTCLOUDCLIENT_H_

#include <string>
#include <transform.h>
#include <velodyne_rings.h>
#include <perception_types.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#define    BINS_PER_REV             720

namespace perception {

class Perception;

class PointCloudClient {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  PointCloudClient(Perception& perception);
  virtual ~PointCloudClient();

  const velodyne::Config& config() {return config_;}
  bool blocksAvailable() {return spin_ready_;}
  void read(pcl::PointCloud<pcl::PointXYZI>& packet);

  void integrate(const pcl::PointCloud<pcl::PointXYZI>& packet, unsigned short counter);
  void syncWithTerrain(const PerceptionCell& t, PerceptionCell& c);

 private:
  template<class T> void getParam(std::string key, T& var);
  void getParamTransform(std::string key, dgc::dgc_transform_t& tr);

  void update(const pcl::PointCloud<pcl::PointXYZI>& cloud);
  void spinThread();

  void process(const pcl::PointCloud<pcl::PointXYZI>& cloud);
  void setCellMin(PerceptionCell& cell, float z, unsigned short counter);
  void setCellMax(PerceptionCell& cell, float z, unsigned short counter);
  void labelObstacleCellsNear(int64_t counter, const std::vector<velodyne::Block>& blocks);
  void labelObstacleCells(const pcl::PointCloud<pcl::PointXYZI>& cloud, unsigned short counter);
  bool isObstacle(const pcl::PointXYZI& pt, double ground_threshold);
  void fillScan(const velodyne::Block& block, int encoder, int point_num, laser_scan_t& scan);

private:


  typedef struct {
    int idx; /* velodyne index beam */
    int pb; /* partner beam for the comparison */
    float v_angle; /* vertical angle of the laser beam */
    float h_angle; /* horizontal angle of the laser beam */
    int h_offset; /* horizontal offset of the beam in velodyne ticks */
    float fac; /* approximation factor of function with pb */
  } velodyne_ring_settings_t;

private:
  Perception& perception_;
  ros::NodeHandle nh_;
  tf::TransformListener tf_listener_;
  ros::Subscriber packet_sub_;
  velodyne::Config config_;
  dgc::dgc_transform_t velodyne_offset_;

  double velodyne_ts_;
  double last_velodyne_ts_;

  pcl::PointCloud<pcl::PointXYZI> packet_;

  bool processed_;

  boost::thread* spin_thread_;
  boost::mutex mutex_;
  boost::condition_variable cond_spin_ready_;
  boost::condition_variable cond_spin_read_;
  bool spin_ready_;
  bool spin_read_;
};

} // namespace perception

#endif
