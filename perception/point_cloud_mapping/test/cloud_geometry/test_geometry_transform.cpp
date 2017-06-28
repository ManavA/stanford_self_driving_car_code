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

/** \author Marius Muja */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/geometry/transforms.h>

using namespace cloud_geometry::transforms;

TEST (Geom, TransformsCoplanar)
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


  Eigen::Matrix4d transform;

  transform.setZero();
  double alpha = M_PI/6;
  transform(0,0) = cos(alpha);
  transform(0,1) = sin(alpha);
  transform(1,0) = -sin(alpha);
  transform(1,1) = cos(alpha);
  transform(2,2) = 1;

  transform(0,3) = 2;
  transform(1,3) = 3;
  transform(2,3) = 4;
  transform(3,3) = 1;

  sensor_msgs::PointCloud transformed_points;
  cloud_geometry::transforms::transformPoints(points.points, transformed_points.points, transform);

  Eigen::Matrix4d computed_transform;

  cloud_geometry::transforms::getPointsRigidTransformation(points,transformed_points, computed_transform);

  bool ok = true;

  for (int i= 0;i<computed_transform.size();++i) {
	  ok &= (computed_transform(i)-transform(i)<1e-5);
  }

  EXPECT_TRUE(ok);

}

TEST (Geom, Transforms)
{
  sensor_msgs::PointCloud points;
  points.points.resize (18);

  points.points[0].x = 3.587751;  points.points[0].y = -4.190982;  points.points[0].z = 1;
  points.points[1].x = 3.808883;  points.points[1].y = -4.412265;  points.points[1].z = 2;
  points.points[2].x = 3.587525;  points.points[2].y = -5.809143;  points.points[2].z = 1;
  points.points[3].x = 2.999913;  points.points[3].y = -5.999980;  points.points[3].z = 3;
  points.points[4].x = 2.412224;  points.points[4].y = -5.809090;  points.points[4].z = 5;
  points.points[5].x = 2.191080;  points.points[5].y = -5.587682;  points.points[5].z = 2;
  points.points[6].x = 2.048941;  points.points[6].y = -5.309003;  points.points[6].z = 3;
  points.points[7].x = 2.000397;  points.points[7].y = -4.999944;  points.points[7].z = 1;
  points.points[8].x = 2.999953;  points.points[8].y = -6.000056;  points.points[8].z = 2;
  points.points[9].x = 2.691127;  points.points[9].y = -5.951136;  points.points[9].z = 5;
  points.points[10].x = 2.190892; points.points[10].y = -5.587838; points.points[10].z = 2;
  points.points[11].x = 2.048874; points.points[11].y = -5.309052; points.points[11].z = 1;
  points.points[12].x = 1.999990; points.points[12].y = -5.000147; points.points[12].z = 3;
  points.points[13].x = 2.049026; points.points[13].y = -4.690918; points.points[13].z = 2;
  points.points[14].x = 2.190956; points.points[14].y = -4.412162; points.points[14].z = 1;
  points.points[15].x = 2.412231; points.points[15].y = -4.190918; points.points[15].z = 2;
  points.points[16].x = 2.691027; points.points[16].y = -4.049060; points.points[16].z = 0;
  points.points[17].x = 2;        points.points[17].y = -3;        points.points[17].z = 0;


  Eigen::Matrix4d transform;

  transform.setZero();
  double alpha = M_PI/6;
  transform(0,0) = cos(alpha);
  transform(0,1) = sin(alpha);
  transform(1,0) = -sin(alpha);
  transform(1,1) = cos(alpha);
  transform(2,2) = 1;

  transform(0,3) = 2;
  transform(1,3) = 3;
  transform(2,3) = 4;
  transform(3,3) = 1;

  sensor_msgs::PointCloud transformed_points;
  cloud_geometry::transforms::transformPoints(points.points, transformed_points.points, transform);

  Eigen::Matrix4d computed_transform;

  cloud_geometry::transforms::getPointsRigidTransformation(points,transformed_points, computed_transform);

  bool ok = true;

  for (int i= 0;i<computed_transform.size();++i) {
	  ok &= (computed_transform(i)-transform(i)<1e-5);
  }

  EXPECT_TRUE(ok);
}


TEST (Geom, TransformIndices)
{
  sensor_msgs::PointCloud points;
  points.points.resize (18);

  points.points[0].x = 3.587751;  points.points[0].y = -4.190982;  points.points[0].z = 1;
  points.points[1].x = 3.808883;  points.points[1].y = -4.412265;  points.points[1].z = 2;
  points.points[2].x = 3.587525;  points.points[2].y = -5.809143;  points.points[2].z = 1;
  points.points[3].x = 2.999913;  points.points[3].y = -5.999980;  points.points[3].z = 3;
  points.points[4].x = 2.412224;  points.points[4].y = -5.809090;  points.points[4].z = 5;
  points.points[5].x = 2.191080;  points.points[5].y = -5.587682;  points.points[5].z = 2;
  points.points[6].x = 2.048941;  points.points[6].y = -5.309003;  points.points[6].z = 3;
  points.points[7].x = 2.000397;  points.points[7].y = -4.999944;  points.points[7].z = 1;
  points.points[8].x = 2.999953;  points.points[8].y = -6.000056;  points.points[8].z = 2;
  points.points[9].x = 2.691127;  points.points[9].y = -5.951136;  points.points[9].z = 5;
  points.points[10].x = 2.190892; points.points[10].y = -5.587838; points.points[10].z = 2;
  points.points[11].x = 2.048874; points.points[11].y = -5.309052; points.points[11].z = 1;
  points.points[12].x = 1.999990; points.points[12].y = -5.000147; points.points[12].z = 3;
  points.points[13].x = 2.049026; points.points[13].y = -4.690918; points.points[13].z = 2;
  points.points[14].x = 2.190956; points.points[14].y = -4.412162; points.points[14].z = 1;
  points.points[15].x = 2.412231; points.points[15].y = -4.190918; points.points[15].z = 2;
  points.points[16].x = 2.691027; points.points[16].y = -4.049060; points.points[16].z = 0;
  points.points[17].x = 2;        points.points[17].y = -3;        points.points[17].z = 0;


  std::vector<int> indices;

  indices.push_back(0);
  indices.push_back(2);
  indices.push_back(5);
  indices.push_back(9);
  indices.push_back(11);
  indices.push_back(12);
  indices.push_back(17);

  Eigen::Matrix4d transform;

  transform.setZero();
  double alpha = M_PI/6;
  transform(0,0) = cos(alpha);
  transform(0,1) = sin(alpha);
  transform(1,0) = -sin(alpha);
  transform(1,1) = cos(alpha);
  transform(2,2) = 1;

  transform(0,3) = 2;
  transform(1,3) = 3;
  transform(2,3) = 4;
  transform(3,3) = 1;

  alpha = M_PI/3;
  Eigen::Matrix4d transform2;
  transform2.setZero();
  transform2(0,0) = cos(alpha);
  transform2(0,2) = sin(alpha);
  transform2(2,0) = -sin(alpha);
  transform2(2,2) = cos(alpha);
  transform2(1,1) = 1;

  transform2(0,3) = 2;
  transform2(1,3) = 1;
  transform2(2,3) = 3;
  transform2(3,3) = 1;

  transform = transform * transform2;

  sensor_msgs::PointCloud transformed_points;
  cloud_geometry::transforms::transformPoints(points.points,  transformed_points.points, transform);

  Eigen::Matrix4d computed_transform;

  cloud_geometry::transforms::getPointsRigidTransformation(points, indices, transformed_points, indices, computed_transform);
  bool ok = true;

  for (int i= 0;i<computed_transform.size();++i) {
	  ok &= (computed_transform(i)-transform(i)<1e-5);
  }

  EXPECT_TRUE(ok);
}


/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
