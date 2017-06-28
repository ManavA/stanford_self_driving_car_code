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


#include <pluginlib/class_list_macros.h>
#include <Project3D.h>
#include <velodyne/Packet.h>
#include <velodyne/Projected.h>
#include <boost/thread.hpp>
#include <velodyne.h>
#include <global.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace std;

namespace velodyne
{

  PLUGINLIB_DECLARE_CLASS(velodyne, Project3D, velodyne::Project3D, nodelet::Nodelet);

  Project3D::Project3D()
  {

  }

  Project3D::~Project3D()
  {
  }

  void Project3D::onInit()
  {
    string cal_file, int_file;
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    private_nh.getParam("/driving/velodyne/cal_file", cal_file);
    private_nh.getParam("/driving/velodyne/int_file", int_file);
    readCalibration(cal_file, int_file);
    projected_pub_ = private_nh.advertise<velodyne::Projected>
        ("/driving/velodyne/projected", 4096);
    points_pub_ = private_nh.advertise<pcl::PointCloud<pcl::PointXYZI> >
        ("/driving/velodyne/rawpoints", 4096);
    packet_sub_ = private_nh.subscribe("/driving/velodyne/packet", 4096, 
        &Project3D::projectPacket, this);
    NODELET_INFO("Ready to project points");
  }

  void Project3D::readCalibration(string cal_file, string int_file)
  {
    if(!config_.readCalibration(cal_file))
    {
      NODELET_ERROR_STREAM("Could not open calibration file " << 
          cal_file << string("."));
      throw VLRException("Could not open calibration file " +
          cal_file + string("."));
    }

    if(!config_.readIntensity(int_file))
    {
      NODELET_ERROR_STREAM("Could not open intensity file " << 
          int_file << string("."));
      throw VLRException("Could not open intensity file " +
          int_file + string("."));
    }
  }

  void Project3D::projectPacket(const velodyne::PacketPtr& packet)
  {
    double x, y, z;
    double distance, distance1, cosVertAngle, sinVertAngle, cosRotAngle, sinRotAngle, hOffsetCorr, vOffsetCorr;
    double xyDistance; //, distanceCorr, shortOffset, longOffset;

    velodyne::Projected projected;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (int b = 0; b < velodyne::Packet::NUM_BLOCKS; b++)
    {
      for (int l = 0; l < velodyne::Block::NUM_BEAMS; l++)
      {
        // laser is global laser id, 0-64
        // l is block laser id, 0-32
        int laser = l + velodyne::Block::NUM_BEAMS * packet->block[b].block;
        if ((packet->block[b].laser[l].distance == 0 || 
              packet->block[b].laser[l].distance * VELODYNE_TICKS_TO_METER > 110 || 
              isnan(config_.sin_rot_angle[packet->block[b].encoder][laser]))) 
        {
          projected.block[b].point[l].x = 0;
          projected.block[b].point[l].y = 0;
          projected.block[b].point[l].z = 0;
          projected.block[b].laser[l].distance = 0;
        }
        else 
        {
          distance1 = packet->block[b].laser[l].distance * VELODYNE_TICKS_TO_METER;
          distance = config_.range_offsetX[laser] * distance1 + config_.range_offset[laser];

          cosVertAngle = config_.cos_vert_angle[laser];
          sinVertAngle = config_.sin_vert_angle[laser];
          cosRotAngle = config_.cos_rot_angle[packet->block[b].encoder][laser];
          sinRotAngle = config_.sin_rot_angle[packet->block[b].encoder][laser];
          hOffsetCorr = config_.h_offset[laser];
          vOffsetCorr = config_.v_offset[laser];

          xyDistance = distance * cosVertAngle;

          x = xyDistance * cosRotAngle - hOffsetCorr * sinRotAngle;
          y = xyDistance * sinRotAngle + hOffsetCorr * cosRotAngle;
          z = (xyDistance / cosVertAngle) * sinVertAngle + vOffsetCorr;

          projected.block[b].point[l].x = x;
          projected.block[b].point[l].y = y;
          projected.block[b].point[l].z = z;
          projected.block[b].laser[l].distance = (uint16_t) (xyDistance * 100);
          projected.block[b].laser[l].intensity = config_.intensity_map[config_.inv_beam_order[laser]][packet->block[b].laser[l].intensity];
          pcl::PointXYZI point;
          point.x = x;
          point.y = y;
          point.z = z;
          point.intensity = projected.block[b].laser[l].intensity;
          cloud.push_back(point);
        }
      }
      projected.block[b].block = packet->block[b].block;
      projected.block[b].encoder = packet->block[b].encoder;
    }
    projected.spin_count = packet->spin_count;
    projected.header.stamp = packet->header.stamp;
    projected.header.frame_id = "Velodyne";
    cloud.header.frame_id = "Velodyne";
    cloud.header.stamp = ros::Time::now();
    points_pub_.publish(cloud); 
    projected_pub_.publish(projected);
  }
} // namespace velodyne


