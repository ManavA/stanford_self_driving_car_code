/*
 * Copyright (c) 2009, Willow Garage, Inc.
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
 * $Id: pointcloud_to_pcd.cpp 27908 2009-12-18 06:34:19Z sachinc $
 *
 */

/**
@mainpage

\author Radu Bogdan Rusu

@b pointcloud_pcd is a simple node that gets a complete point cloud and saves it to disk into a PCD (Point Cloud Data) file format.

 **/

// ROS core
#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <fstream>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/point.h>

using namespace std;

class PointCloudToPCD
{
  protected:
    ros::NodeHandle nh_;

  public:
    // Save data to disk ?
    char fn_[256];
    bool dump_to_disk_, save_viewpoint_;

    tf::TransformListener tf_;
    string cloud_topic_;

    ros::Subscriber cloud_sub_;

    ////////////////////////////////////////////////////////////////////////////////
    PointCloudToPCD () : dump_to_disk_ (false), save_viewpoint_ (true)
    {
      cloud_topic_ = "tilt_laser_cloud";
      cloud_sub_ = nh_.subscribe (cloud_topic_, 1, &PointCloudToPCD::cloud_cb, this);
      ROS_INFO ("Listening for incoming data on topic %s", nh_.resolveName (cloud_topic_).c_str ());
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Callback
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      if (cloud->points.size () == 0)
        return;
      geometry_msgs::PointStamped pin, pout;
      pin.header.frame_id = "laser_tilt_mount_link";
      pin.point.x = pin.point.y = pin.point.z = 0.0;

      ROS_INFO ("Received %d data points in frame %s with %d channels (%s). Viewpoint is <%.3f, %.3f, %.3f>", (int)cloud->points.size (), cloud->header.frame_id.c_str (), (int)cloud->channels.size (), cloud_geometry::getAvailableChannels (cloud).c_str (), pout.point.x, pout.point.y, pout.point.z);

      save_viewpoint_ = true;
      try
      {
        tf_.transformPoint (cloud->header.frame_id, pin, pout);
      }
      catch (...)
      {
        ROS_ERROR ("TF: Exception caught while trying to transform a point from frame %s into %s!", cloud->header.frame_id.c_str (), pin.header.frame_id.c_str ());
        save_viewpoint_ = false;
        pout.point.x = pout.point.y = pout.point.z = 0;
      }

      double c_time = cloud->header.stamp.sec * 1e3 + cloud->header.stamp.nsec;
      sprintf (fn_, "%.0f.pcd", c_time);
      if (dump_to_disk_)
      {
        ROS_INFO ("Data saved to %s (%f).", fn_, c_time);

        sensor_msgs::PointCloud cloud_out;
        cloud_out.header   = cloud->header;
        cloud_out.points   = cloud->points;
        cloud_out.channels = cloud->channels;
        // Add information about the viewpoint - rudimentary stuff
        if (cloud_geometry::getChannelIndex (cloud, "vx") == -1)
        {
          cloud_out.channels.resize (cloud->channels.size () + 3);
          cloud_out.channels[cloud_out.channels.size () - 3].name = "vx";
          cloud_out.channels[cloud_out.channels.size () - 2].name = "vy";
          cloud_out.channels[cloud_out.channels.size () - 1].name = "vz";
          cloud_out.channels[cloud_out.channels.size () - 3].values.resize (cloud->points.size ());
          cloud_out.channels[cloud_out.channels.size () - 2].values.resize (cloud->points.size ());
          cloud_out.channels[cloud_out.channels.size () - 1].values.resize (cloud->points.size ());
          for (unsigned int i = 0; i < cloud->points.size (); i++)
          {
            cloud_out.channels[cloud_out.channels.size () - 3].values[i] = pout.point.x;
            cloud_out.channels[cloud_out.channels.size () - 2].values[i] = pout.point.y;
            cloud_out.channels[cloud_out.channels.size () - 1].values[i] = pout.point.z;
          }
        }
        cloud_io::savePCDFileASCII (fn_, cloud_out, 5);
      }
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "bag_pcd");

  PointCloudToPCD b;
  b.dump_to_disk_ = true;
  ros::spin ();

  return (0);
}
/* ]--- */
