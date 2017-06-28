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
 * $Id: distances.h 21045 2009-08-07 20:52:44Z stuglaser $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_DISTANCES_H_
#define _CLOUD_GEOMETRY_DISTANCES_H_

// ROS includes
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Core>

namespace cloud_geometry
{

  namespace distances
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 2D point to another 2D point in the XY plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointXYDistanceSqr (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 2D point to another 2D point in the XY plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointXYDistance (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( sqrt (pointToPointXYDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 2D point to another 2D point in the XZ plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointXZDistanceSqr (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 2D point to another 2D point in the XZ plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointXZDistance (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( sqrt (pointToPointXZDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 2D point to another 2D point in the YZ plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointYZDistanceSqr (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 2D point to another 2D point in the YZ plane
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointYZDistance (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( sqrt (pointToPointYZDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistanceSqr (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistance (const geometry_msgs::Point32 &p1, const geometry_msgs::Point32 &p2)
    {
      return (sqrt (pointToPointDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistanceSqr (const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistance (const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
    {
      return (sqrt (pointToPointDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistanceSqr (const geometry_msgs::Point32 &p1, const geometry_msgs::Point &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistance (const geometry_msgs::Point32 &p1, const geometry_msgs::Point &p2)
    {
      return (sqrt (pointToPointDistanceSqr (p1, p2) ));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistanceSqr (const geometry_msgs::Point &p1, const geometry_msgs::Point32 &p2)
    {
      return ( (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a 3D point to another 3D point
      * \param p1 the first point
      * \param p2 the second point
      */
    inline double
      pointToPointDistance (const geometry_msgs::Point &p1, const geometry_msgs::Point32 &p2)
    {
      return (sqrt (pointToPointDistanceSqr (p1, p2) ));
    }

    double pointToLineDistance (const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &dir);
    double pointToLineDistance (const geometry_msgs::Point32 &p, const std::vector<double> &line_coefficients);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
      * \param p a point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline double
      pointToPlaneDistanceSigned (const geometry_msgs::Point32 &p, const std::vector<double> &plane_coefficients)
    {
      return (plane_coefficients[0]*p.x + plane_coefficients[1]*p.y + plane_coefficients[2]*p.z + plane_coefficients[3]);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
      * \param p a point
      * \param a the normalized <i>a</i> coefficient of a plane
      * \param b the normalized <i>b</i> coefficient of a plane
      * \param c the normalized <i>c</i> coefficient of a plane
      * \param d the normalized <i>d</i> coefficient of a plane
      */
    inline double
      pointToPlaneDistanceSigned (const geometry_msgs::Point32 &p, double a, double b, double c, double d)
    {
      return (a * p.x + b * p.y + c * p.z + d);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (signed) defined by ax+by+cz+d=0
      * \param p a point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline double
      pointToPlaneDistanceSigned (const geometry_msgs::Point32 &p, const Eigen::Vector4d &plane_coefficients)
    {
      return ( plane_coefficients (0) * p.x + plane_coefficients (1) * p.y + plane_coefficients (2) * p.z + plane_coefficients (3) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (unsigned) defined by ax+by+cz+d=0
      * \param p a point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline double
      pointToPlaneDistance (const geometry_msgs::Point32 &p, const std::vector<double> &plane_coefficients)
    {
      return (fabs (pointToPlaneDistanceSigned (p, plane_coefficients)) );
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (unsigned) defined by ax+by+cz+d=0
      * \param p a point
      * \param a the normalized <i>a</i> coefficient of a plane
      * \param b the normalized <i>b</i> coefficient of a plane
      * \param c the normalized <i>c</i> coefficient of a plane
      * \param d the normalized <i>d</i> coefficient of a plane
      */
    inline double
      pointToPlaneDistance (const geometry_msgs::Point32 &p, double a, double b, double c, double d)
    {
      return (fabs (pointToPlaneDistance (p, a, b, c, d)) );
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a plane (unsigned) defined by ax+by+cz+d=0
      * \param p a point
      * \param plane_coefficients the normalized coefficients (a, b, c, d) of a plane
      */
    inline double
      pointToPlaneDistance (const geometry_msgs::Point32 &p, const Eigen::Vector4d &plane_coefficients)
    {
      return ( fabs (pointToPlaneDistance (p, plane_coefficients)) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a sphere
      * \param p a point
      * \param sphere_coefficients the coefficients (x, y, z, R) of a sphere
      */
    inline double
      pointToSphereDistance (const geometry_msgs::Point32 &p, const std::vector<double> &sphere_coefficients)
    {
      return (sqrt (
                    (p.x - sphere_coefficients[0]) * (p.x - sphere_coefficients[0]) +
                    (p.y - sphere_coefficients[1]) * (p.y - sphere_coefficients[1]) +
                    (p.z - sphere_coefficients[2]) * (p.z - sphere_coefficients[2])
                   ) - sphere_coefficients[3]);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a sphere
      * \param p a point
      * \param x the x center coefficient of a sphere
      * \param y the y center coefficient of a sphere
      * \param z the z center coefficient of a sphere
      * \param R the radius coefficient of a sphere
      */
    inline double
      pointToSphereDistance (const geometry_msgs::Point32 &p, double x, double y, double z, double r)
    {
      return (sqrt ( (p.x - x) * (p.x - x) + (p.y - y) * (p.y - y) + (p.z - z) * (p.z - z) ) - r);
    }


    void lineToLineSegment (const std::vector<double> &line_a, const std::vector<double> &line_b, std::vector<double> &segment);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Creates a parallel 2D line at a given distance in an X-Y plane (Z is ignored)
      * \param line_a the 3D coefficients of the input line (point, direction)
      * \param line_b the 3D coefficients of the resultant parallel line (point, direction), with point.z copied and
      *        direction.z = 0
      * \param distance the desired distance between the lines
      */
    inline void
      createParallel2DLine (const std::vector<double> &line_a, std::vector<double> &line_b, double distance)
    {
      line_b.resize (6);
      // The direction of the resultant line is equal to the first
      line_b[3] = line_a[3];
      line_b[4] = line_a[4];
      line_b[5] = line_a[5];

      // Create a point on an orthogonal line, at the given distance
      double angle = atan2 (line_a[3], -line_a[4]);
      if (angle < 0)
        angle += 2 * M_PI;

      line_b[0] = line_a[0] + distance * cos (angle);
      line_b[1] = line_a[1] + distance * sin (angle);
      line_b[2] = line_a[2];
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the squared distance from a point to a 3D Polygon
      * \param p a point
      * \param poly the polygon
      */
    inline double
      pointToPolygonDistanceSqr (const geometry_msgs::Point32 &p, const geometry_msgs::Polygon &poly)
    {
      double min_distance = FLT_MAX;
      geometry_msgs::Point32 dir, p_t;
      // Treat the polygon as a set of 3D lines, and compute the distance from the point to each line
      unsigned int i = 0;
      for (i = 0; i < poly.points.size () - 1; i++)
      {
        // Compute the direction
        dir.x = poly.points[i + 1].x - poly.points[i].x;
        dir.y = poly.points[i + 1].y - poly.points[i].y;
        dir.z = poly.points[i + 1].z - poly.points[i].z;

        // Calculate the distance from the point to the line
        // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
        p_t.x = poly.points[i + 1].x - p.x;
        p_t.y = poly.points[i + 1].y - p.y;
        p_t.z = poly.points[i + 1].z - p.z;

        geometry_msgs::Point32 c = cross (p_t, dir);
        double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
        if (sqr_distance < min_distance)
          min_distance = sqr_distance;
      }

      // ---[ Check the [i-1 . 0] line too
      // Compute the direction
      dir.x = poly.points[i].x - poly.points[0].x;
      dir.y = poly.points[i].y - poly.points[0].y;
      dir.z = poly.points[i].z - poly.points[0].z;

      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      p_t.x = poly.points[i].x - p.x;
      p_t.y = poly.points[i].y - p.y;
      p_t.z = poly.points[i].z - p.z;

      geometry_msgs::Point32 c = cross (p_t, dir);
      double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
      if (sqr_distance < min_distance)
        min_distance = sqr_distance;

      return (min_distance);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a 3D Polygon
      * \param p a point
      * \param poly the polygon
      */
    inline double
      pointToPolygonDistance (const geometry_msgs::Point32 &p, const geometry_msgs::Polygon &poly)
    {
      return (sqrt (pointToPolygonDistanceSqr (p, poly)));
    }
  }
}

#endif
