/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
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
 *
 * $Id: cloud_downsampler.cpp 27990 2009-12-19 06:48:50Z hsu $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b cloud_downsampler uniformly downsamples a 3D point cloud.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/point.h>

#include <sys/time.h>

using namespace std;

class CloudDownsampler
{
  protected:
    ros::NodeHandle nh_;

  public:

    // ROS messages
    sensor_msgs::PointCloud cloud_down_, cloud_in_;

    vector<cloud_geometry::Leaf> leaves_;

    ros::Subscriber cloud_sub_;
    ros::Publisher cloud_down_pub_;

    // Parameters
    geometry_msgs::Point leaf_width_;
    double cut_distance_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    CloudDownsampler ()
    {
      nh_.param ("~leaf_width_x", leaf_width_.x, 0.025);      // 2.5cm radius by default
      nh_.param ("~leaf_width_y", leaf_width_.y, 0.025);      // 2.5cm radius by default
      nh_.param ("~leaf_width_z", leaf_width_.z, 0.025);      // 2.5cm radius by default

      ROS_INFO ("Using a default leaf of size: %g,%g,%g.", leaf_width_.x, leaf_width_.y, leaf_width_.z);

      nh_.param ("~cut_distance", cut_distance_, 10.0);       // 10m by default

      string cloud_topic ("tilt_laser_cloud");

      cloud_sub_ = nh_.subscribe (cloud_topic, 1, &CloudDownsampler::cloud_cb, this);
      cloud_down_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("cloud_downsampled", 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~CloudDownsampler () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
    cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      ROS_INFO ("Received %d data points.", (int)cloud->points.size ());
      if (cloud->points.size () == 0)
        return;
      sensor_msgs::PointCloudConstPtr cloud_in = cloud;

      ros::Time ts = ros::Time::now ();

      int d_idx = cloud_geometry::getChannelIndex (cloud_in, "distances");
      cloud_geometry::downsamplePointCloud (cloud_in, cloud_down_, leaf_width_, leaves_, d_idx, cut_distance_);

      ROS_INFO ("Cloud downsampled in %g seconds. Number of points left: %d.", (ros::Time::now () - ts).toSec (), (int)cloud_down_.points.size ());

      cloud_down_pub_.publish (cloud_down_);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "cloud_downsampler_node");

  CloudDownsampler p;
  ros::spin ();

  return (0);
}
/* ]--- */
