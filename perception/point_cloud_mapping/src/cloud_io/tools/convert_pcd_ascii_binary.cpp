/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: convert_pcd_ascii_binary.cpp 30246 2010-01-29 19:19:53Z rusu $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b convert_pcd_ascii_binary converts PCD (Point Cloud Data) files from ascii to binary and viceversa (in place).

 **/

#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

using namespace std;
using namespace cloud_io;

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_ERROR ("Syntax is: %s <file_in.pcd> <file_out.pcd>", argv[0]);
    return (-1);
  }

  sensor_msgs::PointCloud msg_cloud;
  int res = loadPCDFile (argv[1], msg_cloud);
  if (res == -1)
  {
    ROS_ERROR ("Error loading %s!", argv[1]);
    return (-1);
  }
  
  if (res == 0)
    ROS_INFO ("Loaded (%d points, ASCII format) with: %s", (int)msg_cloud.points.size (), getAvailableDimensions (msg_cloud).c_str ());
  else
    ROS_INFO ("Loaded (%d points, binary format) with: %s", (int)msg_cloud.points.size (), getAvailableDimensions (msg_cloud).c_str ());
  
  res = !res;
  if (res)
    ROS_INFO ("Saving (in place) to binary format.");
  else
    ROS_INFO ("Saving (in place) to ASCII format.");

  savePCDFile (argv[2], msg_cloud, res);
}
/* ]--- */
