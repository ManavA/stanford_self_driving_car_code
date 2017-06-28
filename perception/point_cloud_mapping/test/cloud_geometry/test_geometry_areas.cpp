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
 * $Id: test_geometry_angles.cpp 13046 2009-03-27 19:37:45Z veedee $
 *
 */

/** \author Alexander Sorokin */

#include <gtest/gtest.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <point_cloud_mapping/geometry/areas.h>

using namespace cloud_geometry::areas;

TEST (Geometry, PolygonAreas)
{
  geometry_msgs::Point32 p1, p2, p3, p4;
  geometry_msgs::Polygon poly;

  std::vector<double> normal;
  double area;
  
  p1.x = 1;
  p1.y = 0;
  p1.z = 0;
  
  p2.x = 0;
  p2.y = 1;
  p2.z = 0;

  p3.x = 1;
  p3.y = 1;
  p3.z = 0;

  p4.x = 1;
  p4.y = 0.5;
  p4.z = 0;

  poly.set_points_size(4);
  poly.points[0]=p1;
  poly.points[1]=p2;
  poly.points[2]=p3;
  poly.points[3]=p4;


  bool got_normal=compute2DPolygonNormal(poly, normal);
  EXPECT_EQ(got_normal,true);
  EXPECT_NEAR (normal[0], 0, 1e-10);
  EXPECT_NEAR (normal[1], 0, 1e-10);
  EXPECT_NEAR (abs(normal[2]), 1, 1e-10);

  area=compute2DPolygonalArea (poly);
  EXPECT_NEAR (area, 0.5, 1e-10);


}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
