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
 * $Id: test_geometry_angles.cpp 20633 2009-08-04 07:19:09Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <geometry_msgs/Point32.h>

#include <point_cloud_mapping/geometry/angles.h>

using namespace cloud_geometry::angles;

TEST (Geometry, Angles3D)
{
  geometry_msgs::Point32 p1, p2;
  double angle = 0.0;
  
  p1.x = 1;
  p1.y = 0;
  p1.z = 0;
  
  p2.x = -1;
  p2.y = 0;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 180.0, 1e-10);

  p2.x = 0;
  p2.y = -1;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 90.0, 1e-10);

  p2.x = 0;
  p2.y = 1;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 90.0, 1e-10);

  p2.x = .5;
  p2.y = -.5;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 45.0, 1e-10);
  
  p2.x = .5;
  p2.y = .5;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 45.0, 1e-10);

  p2.x = -.5;
  p2.y = .5;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 135.0, 1e-10);

  p2.x = -.5;
  p2.y = -.5;
  p2.z = 0;
  angle = getAngle3D (p1, p2) * 180.0 / M_PI;
  EXPECT_NEAR (angle, 135.0, 1e-10);
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
