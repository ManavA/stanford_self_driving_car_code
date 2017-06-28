/*
 * Copyright (c) 2008-2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: test_geometry.cpp 12438 2009-03-12 08:59:30Z veedee $
 *
 */

/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/geometry/statistics.h>

using namespace cloud_geometry::statistics;

TEST (Geom, Statistics)
{
  sensor_msgs::PointCloud points;
  points.points.resize (18);

  points.points[0].x = 3.587751;  points.points[0].y = -4.190982;  points.points[0].z = 0;
  points.points[1].x = 3.808883;  points.points[1].y = -4.412265;  points.points[1].z = 0;
  points.points[2].x = 3.587525;  points.points[2].y = -5.809143;  points.points[2].z = 0;
  points.points[3].x = 2.999913;  points.points[3].y = -5.999980;  points.points[3].z = 0;
  points.points[4].x = 2.412224;  points.points[4].y = -5.809090;  points.points[4].z = 0;
  points.points[5].x = 2.191080;  points.points[5].y = -5.587682;  points.points[5].z = 0;
  points.points[6].x = 2.048941;  points.points[6].y = -5.309003;  points.points[6].z = 0;
  points.points[7].x = 2.000397;  points.points[7].y = -4.999944;  points.points[7].z = 0;
  points.points[8].x = 2.999953;  points.points[8].y = -6.000056;  points.points[8].z = 0;
  points.points[9].x = 2.691127;  points.points[9].y = -5.951136;  points.points[9].z = 0;
  points.points[10].x = 2.190892; points.points[10].y = -5.587838; points.points[10].z = 0;
  points.points[11].x = 2.048874; points.points[11].y = -5.309052; points.points[11].z = 0;
  points.points[12].x = 1.999990; points.points[12].y = -5.000147; points.points[12].z = 0;
  points.points[13].x = 2.049026; points.points[13].y = -4.690918; points.points[13].z = 0;
  points.points[14].x = 2.190956; points.points[14].y = -4.412162; points.points[14].z = 0;
  points.points[15].x = 2.412231; points.points[15].y = -4.190918; points.points[15].z = 0;
  points.points[16].x = 2.691027; points.points[16].y = -4.049060; points.points[16].z = 0;
  points.points[17].x = 2;        points.points[17].y = -3;        points.points[17].z = 0;
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
