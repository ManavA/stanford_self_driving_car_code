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
 * $Id: cloudmsg_to_screen.cpp 21391 2009-08-10 11:16:47Z ehberger $
 *
 */

/** \author Radu Bogdan Rusu
  *
  * Extremely silly PCD to C++ ROS sensor_msgs PointCloud converter.
  * Useful for tools which don't want to link against \a cloud_io (I guess).
  *
  */
#include <point_cloud_mapping/cloud_io.h>

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 2)
  {
    fprintf (stderr, "Error. No command line argument(s) specified. Syntax is: %s <input.pcd>\n", argv[0]);
    return (-1);
  }

  sensor_msgs::PointCloud points;
  int res = cloud_io::loadPCDFile (argv[1], points);

  if (res == -1)
  {
    fprintf (stderr, "Error loading %s.\n", argv[1]);
    return (-1);
  }

  fprintf (stdout, "sensor_msgs::PointCloud points;\n");
  fprintf (stdout, "points.points.resize (%i);\n", (int)points.points.size ());

  if (points.channels.size () > 0)
  {
    fprintf (stdout, "points.channels.resize (%i);\n", (int)points.channels.size ());
    for (unsigned int d = 0; d < points.channels.size (); d++)
    {
      fprintf (stdout, "points.channels[%i].name = %s;\n", d, points.channels[d].name.c_str ());
      fprintf (stdout, "points.channels[%i].values.resize (%i);", d, (int)points.channels.size ());
    }
  }

  fprintf (stdout, "\n");
  for (unsigned int i = 0; i < points.points.size (); i++)
    fprintf (stdout, "points.points[%5i].x = %f; points.points[%5i].y = %f; points.points[%5i].z = %f;\n", i, points.points[i].x, i, points.points[i].y, i, points.points[i].z);

  fprintf (stdout, "\n");
  for (unsigned int d = 0; d < points.channels.size (); d++)
    for (unsigned int i = 0; i < points.points.size (); i++)
      fprintf (stdout, "points.channels[%i].values[%5i] = %f;\n", d, i, points.channels[d].values[i]);

  return (0);
}
/* ]--- */
