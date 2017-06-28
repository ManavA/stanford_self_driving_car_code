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
 * $Id: angles.h 21604 2009-08-12 01:01:32Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_ANGLES_H_
#define _CLOUD_GEOMETRY_ANGLES_H_

// ROS includes
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Polygon.h>

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>

#include <Eigen/Core>

namespace cloud_geometry
{

  namespace angles
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the angle between two planes
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      */
    inline double
      getAngleBetweenPlanes (const std::vector<double> &plane_a, const std::vector<double> &plane_b)
    {
      return (acos (plane_a[0] * plane_b[0] + plane_a[1] * plane_b[1] + plane_a[2] * plane_b[2]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the angle between two planes
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      */
    inline double
      getAngleBetweenPlanes (const geometry_msgs::Point32 &plane_a, const geometry_msgs::Point32 &plane_b)
    {
      return (acos (plane_a.x * plane_b.x + plane_a.y * plane_b.y + plane_a.z * plane_b.z));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the angle between two planes
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      */
    inline double
      getAngleBetweenPlanes (const geometry_msgs::Point32 &plane_a, const std::vector<double> &plane_b)
    {
      return (acos (plane_a.x * plane_b[0] + plane_a.y * plane_b[1] + plane_a.z * plane_b[2]));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the angle between two planes
      * \param plane_a the normalized coefficients of the first plane
      * \param plane_b the normalized coefficients of the second plane
      */
    inline double
      getAngleBetweenPlanes (const std::vector<double> &plane_a, const geometry_msgs::Point32 &plane_b)
    {
      return (acos (plane_a[0] * plane_b.x + plane_a[1] * plane_b.y + plane_a[2] * plane_b.z));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the angle in the [ 0, 2*PI ) interval of a point (direction) with a reference (0, 0) in 2D.
      * \param point a 2D point
      */
    inline double
      getAngle2D (const double point[2])
    {
      double rad;
      if (point[0] == 0)
        rad = (point[1] < 0) ? -M_PI / 2.0 : M_PI / 2.0;
      else
      {
        rad = atan (point[1] / point[0]);
        if (point[0] < 0)
          rad += M_PI;
      }
      if (rad < 0)
        rad += 2 * M_PI;

      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the angle in the [ 0, 2*PI ) interval of a point (direction) with a reference (0, 0) in 2D.
      * \param x the X coordinate of the 2D point
      * \param y the Y coordinate of the 2D point
      */
    inline double
      getAngle2D (double x, double y)
    {
      double rad;
      if (x == 0)
        rad = (y < 0) ? -M_PI / 2.0 : M_PI / 2.0;
      else
      {
        rad = atan (y / x);
        if (x < 0)
          rad += M_PI;
      }
      if (rad < 0)
        rad += 2 * M_PI;

      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
      * \param v1 the first 3D vector
      * \param v2 the second 3D vector
      */
    inline double
      getAngle3D (const geometry_msgs::Point32 &v1, const geometry_msgs::Point32 &v2)
    {
      // Compute the vectors norms
      double norm_v1 = (v1.x * v1.x) + (v1.y * v1.y) + (v1.z * v1.z);
      double norm_v2 = (v2.x * v2.x) + (v2.y * v2.y) + (v2.z * v2.z);

      // Compute the actual angle
      double rad = acos ( cloud_geometry::dot (v1, v2) / sqrt (norm_v1 * norm_v2) );

      // Check against NaN
      if (std::isnan (rad))
        ROS_ERROR ("[cloud_geometry::angles::getAngle3D] got a NaN angle!");
      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
      * \param v1 the first 3D vector
      * \param v2 the second 3D vector
      */
    inline double
      getAngle3D (const geometry_msgs::Point &v1, const geometry_msgs::Point &v2)
    {
      // Compute the vectors norms
      double norm_v1 = (v1.x * v1.x) + (v1.y * v1.y) + (v1.z * v1.z);
      double norm_v2 = (v2.x * v2.x) + (v2.y * v2.y) + (v2.z * v2.z);

      // Compute the actual angle
      double rad = acos ( cloud_geometry::dot (v1, v2) / sqrt (norm_v1 * norm_v2) );

      // Check against NaN
      if (std::isnan (rad))
        ROS_ERROR ("[cloud_geometry::angles::getAngle3D] got a NaN angle!");
      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
      * \param v1 the first 3D vector
      * \param v2 the second 3D vector
      */
    inline double
      getAngle3D (const geometry_msgs::Point32 &v1, const geometry_msgs::Point &v2)
    {
      // Compute the vectors norms
      double norm_v1 = (v1.x * v1.x) + (v1.y * v1.y) + (v1.z * v1.z);
      double norm_v2 = (v2.x * v2.x) + (v2.y * v2.y) + (v2.z * v2.z);

      // Compute the actual angle
      double rad = acos ( cloud_geometry::dot (v1, v2) / sqrt (norm_v1 * norm_v2) );

      // Check against NaN
      if (std::isnan (rad))
        ROS_ERROR ("[cloud_geometry::angles::getAngle3D] got a NaN angle!");
      return (rad);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the smallest angle between two vectors in the [ 0, PI ) interval in 3D.
      * \param v1 the first 3D vector
      * \param v2 the second 3D vector
      */
    inline double
      getAngle3D (const geometry_msgs::Point &v1, const geometry_msgs::Point32 &v2)
    {
      // Compute the vectors norms
      double norm_v1 = (v1.x * v1.x) + (v1.y * v1.y) + (v1.z * v1.z);
      double norm_v2 = (v2.x * v2.x) + (v2.y * v2.y) + (v2.z * v2.z);

      // Compute the actual angle
      double rad = acos ( cloud_geometry::dot (v1, v2) / sqrt (norm_v1 * norm_v2) );

      // Check against NaN
      if (std::isnan (rad))
        ROS_ERROR ("[cloud_geometry::angles::getAngle3D] got a NaN angle!");
      return (rad);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (Eigen::Vector4d &normal, const geometry_msgs::Point32 &point, const geometry_msgs::PointStamped &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.point.x - point.x;
      vp_m[1] = viewpoint.point.y - point.y;
      vp_m[2] = viewpoint.point.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal (0) + vp_m[1] * normal (1) + vp_m[2] * normal (2));

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal (d) *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal (3) = -1 * (normal (0) * point.x + normal (1) * point.y + normal (2) * point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (Eigen::Vector4d &normal, const geometry_msgs::Point32 &point, const geometry_msgs::Point &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.x - point.x;
      vp_m[1] = viewpoint.y - point.y;
      vp_m[2] = viewpoint.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal (0) + vp_m[1] * normal (1) + vp_m[2] * normal (2));

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal (d) *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal (3) = -1 * (normal (0) * point.x + normal (1) * point.y + normal (2) * point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (Eigen::Vector4d &normal, const geometry_msgs::Point32 &point, const geometry_msgs::Point32 &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.x - point.x;
      vp_m[1] = viewpoint.y - point.y;
      vp_m[2] = viewpoint.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal (0) + vp_m[1] * normal (1) + vp_m[2] * normal (2));

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal (d) *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal (3) = -1 * (normal (0) * point.x + normal (1) * point.y + normal (2) * point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (std::vector<double> &normal, const geometry_msgs::Point32 &point, const geometry_msgs::PointStamped &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.point.x - point.x;
      vp_m[1] = viewpoint.point.y - point.y;
      vp_m[2] = viewpoint.point.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal[0] + vp_m[1] * normal[1] + vp_m[2] * normal[2]);

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal[d] *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal[3] = -1 * (normal[0] * point.x + normal[1] * point.y + normal[2] * point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Flip (in place) the estimated normal of a point towards a given viewpoint
      * \param normal the plane normal to be flipped
      * \param point a given point
      * \param viewpoint the viewpoint
      */
    inline void
      flipNormalTowardsViewpoint (std::vector<double> &normal, const geometry_msgs::Point32 &point, const geometry_msgs::Point32 &viewpoint)
    {
      // See if we need to flip any plane normals
      float vp_m[3];
      vp_m[0] = viewpoint.x - point.x;
      vp_m[1] = viewpoint.y - point.y;
      vp_m[2] = viewpoint.z - point.z;

      // Dot product between the (viewpoint - point) and the plane normal
      double cos_theta = (vp_m[0] * normal[0] + vp_m[1] * normal[1] + vp_m[2] * normal[2]);

      // Flip the plane normal
      if (cos_theta < 0)
      {
        for (int d = 0; d < 3; d++)
          normal[d] *= -1;
        // Hessian form (D = nc . p_plane (centroid here) + p)
        normal[3] = -1 * (normal[0] * point.x + normal[1] * point.y + normal[2] * point.z);
      }
    }

  }
}

#endif
