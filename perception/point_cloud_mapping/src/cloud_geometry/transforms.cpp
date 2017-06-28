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
 * $Id: transforms.cpp 23275 2009-08-28 20:27:09Z mariusmuja $
 *
 */

/** \author Radu Bogdan Rusu */

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <Eigen/SVD>


namespace cloud_geometry
{
  namespace transforms
  {

  /**
   * See header file
   */
  bool getPointsRigidTransformation(const sensor_msgs::PointCloud& pc_a_, const sensor_msgs::PointCloud& pc_b_,
		  Eigen::Matrix4d &transformation)
  {

	  assert (pc_a_.get_points_size()==pc_b_.get_points_size());

	  sensor_msgs::PointCloud pc_a = pc_a_;
	  sensor_msgs::PointCloud pc_b = pc_b_;

	  // translate both point cloud so that their centroid will be in origin
	  geometry_msgs::Point32 centroid_a;
	  geometry_msgs::Point32 centroid_b;

	  nearest::computeCentroid(pc_a, centroid_a);
	  nearest::computeCentroid(pc_b, centroid_b);

	  for (size_t i=0;i<pc_a.get_points_size();++i) {
		  pc_a.points[i].x -= centroid_a.x;
		  pc_a.points[i].y -= centroid_a.y;
		  pc_a.points[i].z -= centroid_a.z;
	  }

	  for (size_t i=0;i<pc_b.get_points_size();++i) {
		  pc_b.points[i].x -= centroid_b.x;
		  pc_b.points[i].y -= centroid_b.y;
		  pc_b.points[i].z -= centroid_b.z;
	  }

	  // solve for rotation
	  Eigen::Matrix3d correlation;
	  correlation.setZero();

	  for (size_t i=0;i<pc_a.get_points_size();++i) {
		  correlation(0,0) += pc_a.points[i].x * pc_b.points[i].x;
		  correlation(0,1) += pc_a.points[i].x * pc_b.points[i].y;
		  correlation(0,2) += pc_a.points[i].x * pc_b.points[i].z;

		  correlation(1,0) += pc_a.points[i].y * pc_b.points[i].x;
		  correlation(1,1) += pc_a.points[i].y * pc_b.points[i].y;
		  correlation(1,2) += pc_a.points[i].y * pc_b.points[i].z;

		  correlation(2,0) += pc_a.points[i].z * pc_b.points[i].x;
		  correlation(2,1) += pc_a.points[i].z * pc_b.points[i].y;
		  correlation(2,2) += pc_a.points[i].z * pc_b.points[i].z;
	  }

	  Eigen::JacobiSVD<Eigen::Matrix3d> svd(correlation);

	  Eigen::Matrix3d Ut = svd.matrixU().transpose();
	  Eigen::Matrix3d V = svd.matrixV();
	  Eigen::Matrix3d X = V*Ut;

	  double det = X.determinant();
	  if (det<0) {
		  V.col(2) = -V.col(2);
		  X = V*Ut;
	  }

	  transformation.setZero();
	  transformation.topLeftCorner<3,3>() = X;
	  transformation(3,3) = 1;

	  sensor_msgs::PointCloud pc_rotated_a;
	  transformPoints(pc_a_.points, pc_rotated_a.points, transformation);

	  geometry_msgs::Point32 centroid_rotated_a;
	  nearest::computeCentroid(pc_rotated_a, centroid_rotated_a);

	  transformation(0,3) = centroid_b.x - centroid_rotated_a.x;
	  transformation(1,3) = centroid_b.y - centroid_rotated_a.y;
	  transformation(2,3) = centroid_b.z - centroid_rotated_a.z;

	  return true;

  }


  /**
   * See header file
   */
	bool getPointsRigidTransformation(const sensor_msgs::PointCloud& pc_a_, const std::vector<int>& indices_a,
										  const sensor_msgs::PointCloud& pc_b_, const std::vector<int>& indices_b,
										  Eigen::Matrix4d &transformation)
	{

	  assert (indices_a.size()==indices_b.size());

	  sensor_msgs::PointCloud pc_a;
	  sensor_msgs::PointCloud pc_b;

	  getPointCloud(pc_a_, indices_a, pc_a);
	  getPointCloud(pc_b_, indices_b, pc_b);

	  // translate both point cloud so that their centroid will be in origin
	  geometry_msgs::Point32 centroid_a;
	  geometry_msgs::Point32 centroid_b;

	  nearest::computeCentroid(pc_a, centroid_a);
	  nearest::computeCentroid(pc_b, centroid_b);

	  for (size_t i=0;i<pc_a.get_points_size();++i) {
		  pc_a.points[i].x -= centroid_a.x;
		  pc_a.points[i].y -= centroid_a.y;
		  pc_a.points[i].z -= centroid_a.z;
	  }

	  for (size_t i=0;i<pc_b.get_points_size();++i) {
		  pc_b.points[i].x -= centroid_b.x;
		  pc_b.points[i].y -= centroid_b.y;
		  pc_b.points[i].z -= centroid_b.z;
	  }

	  // solve for rotation
	  Eigen::Matrix3d correlation;
	  correlation.setZero();

	  for (size_t i=0;i<pc_a.get_points_size();++i) {
		  correlation(0,0) += pc_a.points[i].x * pc_b.points[i].x;
		  correlation(0,1) += pc_a.points[i].x * pc_b.points[i].y;
		  correlation(0,2) += pc_a.points[i].x * pc_b.points[i].z;

		  correlation(1,0) += pc_a.points[i].y * pc_b.points[i].x;
		  correlation(1,1) += pc_a.points[i].y * pc_b.points[i].y;
		  correlation(1,2) += pc_a.points[i].y * pc_b.points[i].z;

		  correlation(2,0) += pc_a.points[i].z * pc_b.points[i].x;
		  correlation(2,1) += pc_a.points[i].z * pc_b.points[i].y;
		  correlation(2,2) += pc_a.points[i].z * pc_b.points[i].z;
	  }

	  Eigen::JacobiSVD<Eigen::Matrix3d> svd(correlation);

	  Eigen::Matrix3d Ut = svd.matrixU().transpose();
	  Eigen::Matrix3d V = svd.matrixV();
	  Eigen::Matrix3d X = V*Ut;

	  double det = X.determinant();
	  if (det<0) {
		  V.col(2) = -V.col(2);
		  X = V*Ut;
	  }

	  transformation.setZero();
	  transformation.topLeftCorner<3,3>() = X;
	  transformation(3,3) = 1;

	  sensor_msgs::PointCloud pc_rotated_a;
	  getPointCloud(pc_a_, indices_a, pc_a);
	  transformPoints(pc_a.points, pc_rotated_a.points, transformation);

	  geometry_msgs::Point32 centroid_rotated_a;
	  nearest::computeCentroid(pc_rotated_a, centroid_rotated_a);

	  transformation(0,3) = centroid_b.x - centroid_rotated_a.x;
	  transformation(1,3) = centroid_b.y - centroid_rotated_a.y;
	  transformation(2,3) = centroid_b.z - centroid_rotated_a.z;


//	  transformation.setIdentity();
//	  transformation(0,3) = centroid_b.x - centroid_a.x;
//	  transformation(1,3) = centroid_b.y - centroid_a.y;
//	  transformation(2,3) = centroid_b.z - centroid_a.z;

	  return true;

  }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 4x4 rigid transformation matrix (with translation)
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      * \param tx the desired translation on x-axis
      * \param ty the desired translation on y-axis
      * \param tz the desired translation on z-axis
      * \param transformation the resultant transformation matrix
      */
    void
      getPlaneToPlaneTransformation (const std::vector<double> &plane_a, const std::vector<double> &plane_b,
                                     float tx, float ty, float tz, Eigen::Matrix4d &transformation)
    {
      double angle = cloud_geometry::angles::getAngleBetweenPlanes (plane_a, plane_b);
      // Compute the rotation axis R = Nplane x (0, 0, 1)
      geometry_msgs::Point32 r_axis;
      r_axis.x = plane_a[1]*plane_b[2] - plane_a[2]*plane_b[1];
      r_axis.y = plane_a[2]*plane_b[0] - plane_a[0]*plane_b[2];
      r_axis.z = plane_a[0]*plane_b[1] - plane_a[1]*plane_b[0];

      if (r_axis.z < 0)
        angle = -angle;

      // Build a normalized quaternion
      double s = sin (0.5 * angle) / sqrt (r_axis.x * r_axis.x + r_axis.y * r_axis.y + r_axis.z * r_axis.z);
      double x = r_axis.x * s;
      double y = r_axis.y * s;
      double z = r_axis.z * s;
      double w = cos (0.5 * angle);

      // Convert the quaternion to a 3x3 matrix
      double ww = w * w; double xx = x * x; double yy = y * y; double zz = z * z;
      double wx = w * x; double wy = w * y; double wz = w * z;
      double xy = x * y; double xz = x * z; double yz = y * z;

      transformation (0, 0) = xx - yy - zz + ww; transformation (0, 1) = 2*(xy - wz);       transformation (0, 2) = 2*(xz + wy);       transformation (0, 3) = tx;
      transformation (1, 0) = 2*(xy + wz);       transformation (1, 1) = -xx + yy -zz + ww; transformation (1, 2) = 2*(yz - wx);       transformation (1, 3) = ty;
      transformation (2, 0) = 2*(xz - wy);       transformation (2, 1) = 2*(yz + wx);       transformation (2, 2) = -xx -yy + zz + ww; transformation (2, 3) = tz;
      transformation (3, 0) = 0;                 transformation (3, 1) = 0;                 transformation (3, 2) = 0;                 transformation (3, 3) = 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 4x4 rigid transformation matrix (with translation)
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      * \param tx the desired translation on x-axis
      * \param ty the desired translation on y-axis
      * \param tz the desired translation on z-axis
      * \param transformation the resultant transformation matrix
      */
    void
      getPlaneToPlaneTransformation (const std::vector<double> &plane_a, const geometry_msgs::Point32 &plane_b,
                                     float tx, float ty, float tz, Eigen::Matrix4d &transformation)
    {
      double angle = cloud_geometry::angles::getAngleBetweenPlanes (plane_a, plane_b);
      // Compute the rotation axis R = Nplane x (0, 0, 1)
      geometry_msgs::Point32 r_axis;
      r_axis.x = plane_a[1]*plane_b.z - plane_a[2]*plane_b.y;
      r_axis.y = plane_a[2]*plane_b.x - plane_a[0]*plane_b.z;
      r_axis.z = plane_a[0]*plane_b.y - plane_a[1]*plane_b.x;

      if (r_axis.z < 0)
        angle = -angle;

      // Build a normalized quaternion
      double s = sin (0.5 * angle) / sqrt (r_axis.x * r_axis.x + r_axis.y * r_axis.y + r_axis.z * r_axis.z);
      double x = r_axis.x * s;
      double y = r_axis.y * s;
      double z = r_axis.z * s;
      double w = cos (0.5 * angle);

      // Convert the quaternion to a 3x3 matrix
      double ww = w * w; double xx = x * x; double yy = y * y; double zz = z * z;
      double wx = w * x; double wy = w * y; double wz = w * z;
      double xy = x * y; double xz = x * z; double yz = y * z;

      transformation (0, 0) = xx - yy - zz + ww; transformation (0, 1) = 2*(xy - wz);       transformation (0, 2) = 2*(xz + wy);       transformation (0, 3) = tx;
      transformation (1, 0) = 2*(xy + wz);       transformation (1, 1) = -xx + yy -zz + ww; transformation (1, 2) = 2*(yz - wx);       transformation (1, 3) = ty;
      transformation (2, 0) = 2*(xz - wy);       transformation (2, 1) = 2*(yz + wx);       transformation (2, 2) = -xx -yy + zz + ww; transformation (2, 3) = tz;
      transformation (3, 0) = 0;                 transformation (3, 1) = 0;                 transformation (3, 2) = 0;                 transformation (3, 3) = 1;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Convert an axis-angle representation to a 3x3 rotation matrix
      * \note The formula is given by: A = I * cos (th) + ( 1 - cos (th) ) * axis * axis' - E * sin (th), where
      * E = [0 -axis.z axis.y; axis.z 0 -axis.x; -axis.y axis.x 0]
      * \param axis the axis
      * \param angle the angle
      * \param rotation the resultant rotation
      */
    void
      convertAxisAngleToRotationMatrix (const geometry_msgs::Point32 &axis, double angle, Eigen::Matrix3d &rotation)
    {
      double cos_a = cos (angle);
      double sin_a = sin (angle);
      double cos_a_m = 1.0 - cos_a;

      double a_xy = axis.x * axis.y * cos_a_m;
      double a_xz = axis.x * axis.z * cos_a_m;
      double a_yz = axis.y * axis.z * cos_a_m;

      double s_x = sin_a * axis.x;
      double s_y = sin_a * axis.y;
      double s_z = sin_a * axis.z;

      rotation (0, 0) = cos_a + axis.x * axis.x * cos_a_m;
      rotation (0, 1) = a_xy - s_z;
      rotation (0, 2) = a_xz + s_y;
      rotation (1, 0) = a_xy + s_z;
      rotation (1, 1) = cos_a + axis.y * axis.y * cos_a_m;
      rotation (1, 2) = a_yz - s_x;
      rotation (2, 0) = a_xz - s_y;
      rotation (2, 1) = a_yz + s_x;
      rotation (2, 2) = cos_a + axis.z * axis.z * cos_a_m;
    }

  }
}
