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
 * $Id: distances.cpp 20633 2009-08-04 07:19:09Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>

namespace cloud_geometry
{

  namespace distances
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a line (represented by a point and a direction)
      * \param p a point
      * \param q the point on the line
      * \param dir the direction of the line
      */
    double
      pointToLineDistance (const geometry_msgs::Point32 &p, const geometry_msgs::Point32 &q, const geometry_msgs::Point32 &dir)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      geometry_msgs::Point32 r, p_t;
      r.x = q.x + dir.x;
      r.y = q.y + dir.y;
      r.z = q.z + dir.z;
      p_t.x = r.x - p.x;
      p_t.y = r.y - p.y;
      p_t.z = r.z - p.z;

      geometry_msgs::Point32 c = cross (p_t, dir);
      double sqr_distance = (c.x * c.x + c.y * c.y + c.z * c.z) / (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
      return (sqrt (sqr_distance));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the distance from a point to a line (represented by a point and a direction)
      * \param p a point
      * \param line_coefficients the line coefficients (point.x point.y point.z direction.x direction.y direction.z)
      */
    double
      pointToLineDistance (const geometry_msgs::Point32 &p, const std::vector<double> &line_coefficients)
    {
      // Calculate the distance from the point to the line
      // D = ||(P2-P1) x (P1-P0)|| / ||P2-P1|| = norm (cross (p2-p1, p2-p0)) / norm(p2-p1)
      std::vector<double> dir (3), r (3), p_t (3), c;
      dir[0] = line_coefficients[3];
      dir[1] = line_coefficients[4];
      dir[2] = line_coefficients[5];

      r[0] = line_coefficients[0] + dir[0];
      r[1] = line_coefficients[1] + dir[1];
      r[2] = line_coefficients[2] + dir[2];
      p_t[0] = r[0] - p.x;
      p_t[1] = r[1] - p.y;
      p_t[2] = r[2] - p.z;

      cross (p_t, dir, c);
      double sqr_distance = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) / (dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
      return (sqrt (sqr_distance));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the shortest 3D segment between two 3D lines
      * \param line_a the coefficients of the first line (point, direction)
      * \param line_b the coefficients of the second line (point, direction)
      * \param segment the resulting two 3D points that mark the beginning and the end of the segment
      */
    void
      lineToLineSegment (const std::vector<double> &line_a, const std::vector<double> &line_b, std::vector<double> &segment)
    {
      segment.resize (6);

      geometry_msgs::Point32 p2, q2;
      p2.x = line_a.at (0) + line_a.at (3);    // point + direction = 2nd point
      p2.y = line_a.at (1) + line_a.at (4);
      p2.z = line_a.at (2) + line_a.at (5);
      q2.x = line_b.at (0) + line_b.at (3);    // point + direction = 2nd point
      q2.y = line_b.at (1) + line_b.at (4);
      q2.z = line_b.at (2) + line_b.at (5);

      geometry_msgs::Point32 u, v, w;

      // a = x2 - x1 = line_a[1] - line_a[0]
      u.x = p2.x - line_a.at (0);
      u.y = p2.y - line_a.at (1);
      u.z = p2.z - line_a.at (2);
      // b = x4 - x3 = line_b[1] - line_b[0]
      v.x = q2.x - line_b.at (0);
      v.y = q2.y - line_b.at (1);
      v.z = q2.z - line_b.at (2);
      // c = x2 - x3 = line_a[1] - line_b[0]
      w.x = p2.x - line_b.at (0);
      w.y = p2.y - line_b.at (1);
      w.z = p2.z - line_b.at (2);

      double a = dot (u, u);
      double b = dot (u, v);
      double c = dot (v, v);
      double d = dot (u, w);
      double e = dot (v, w);
      double denominator = a*c - b*b;
      double sc, tc;
      // Compute the line parameters of the two closest points
      if (denominator < 1e-5)          // The lines are almost parallel
      {
        sc = 0.0;
        tc = (b > c ? d / b : e / c);  // Use the largest denominator
      }
      else
      {
        sc = (b*e - c*d) / denominator;
        tc = (a*e - b*d) / denominator;
      }
      // Get the closest points
      segment[0] = p2.x + (sc * u.x);
      segment[1] = p2.y + (sc * u.y);
      segment[2] = p2.z + (sc * u.z);

      segment[3] = line_b.at (0) + (tc * v.x);
      segment[4] = line_b.at (1) + (tc * v.y);
      segment[5] = line_b.at (2) + (tc * v.z);
    }

  }
}
