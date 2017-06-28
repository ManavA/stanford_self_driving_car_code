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
 * $Id: intersections.cpp 21045 2009-08-07 20:52:44Z stuglaser $
 *
 */

/** \author Radu Bogdan Rusu */

#include "point_cloud_mapping/geometry/point.h"
#include "point_cloud_mapping/geometry/intersections.h"
#include "point_cloud_mapping/geometry/distances.h"

#include <cfloat>

namespace cloud_geometry
{

  namespace intersections
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the intersection of two planes in 3D space as a 3D line
      * \note if the planes are parallel, the method retunrns false, and \a line is zero-ed
      * \param plane_a the coefficients of the first plane (a,b,c,d)
      * \param plane_b the coefficients of the second plane (a,b,c,d)
      * \param line holder for the computed line coefficients (point, direction)
      */
    bool
      planeWithPlaneIntersection (const std::vector<double> &plane_a, const std::vector<double> &plane_b,
                                  std::vector<double> &line)
    {
      line.resize (6);
      // Compute dir = n1 x n2
      line[3] = plane_a.at (1) * plane_b.at (2) - plane_a.at (2) * plane_b.at (1);
      line[4] = plane_a.at (2) * plane_b.at (0) - plane_a.at (0) * plane_b.at (2);
      line[5] = plane_a.at (0) * plane_b.at (1) - plane_a.at (1) * plane_b.at (0);

      // Find a point on both planes
      if (fabs (line[3]) > FLT_MIN)         // intersect line with the X = 0 plane
      {
        line[0] = 0.0;
        line[1] = (plane_a.at (2) * plane_b.at (3) - plane_a.at (3) * plane_b.at (2)) / line[3];
        line[2] = (plane_a.at (3) * plane_b.at (1) - plane_a.at (1) * plane_b.at (3)) / line[3];
      }
      else if (fabs (line[4]) > FLT_MIN)    // intersect line with the Y = 0 plane
      {
        line[0] = (plane_a.at (3) * plane_b.at (2) - plane_a.at (2) * plane_b.at (3)) / line[4];
        line[1] = 0.0;
        line[2] = (plane_a.at (0) * plane_b.at (3) - plane_a.at (3) * plane_b.at (0)) / line[4];
      }
      else if (fabs (line[5]) > FLT_MIN)    // intersect line with the Z = 0 plane
      {
        line[0] = (plane_a.at (1) * plane_b.at (3) - plane_a.at (3) * plane_b.at (1)) / line[5];
        line[1] = (plane_a.at (3) * plane_b.at (0) - plane_a.at (0) * plane_b.at (3)) / line[5];
        line[2] = 0.0;
      }
      else
      {
        line.resize (0);
        return (false);
      }
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the intersection of a plane and a line in 3D space as a point
      * \note if the line is parallel to the plane, \a point is returned empty
      * \param plane the coefficients of the plane (a,b,c,d)
      * \param line the coefficients of the line (point, direction)
      * \param point holder for the computed 3D point
      */
    bool
      lineWithPlaneIntersection (const std::vector<double> &plane, const std::vector<double> &line, geometry_msgs::Point32 &point)
    {
      // line Direction * plane Normal
      double dn = line.at (3) * plane.at (0) + line.at (4) * plane.at (1) + line.at (5) * plane.at (2);
      // Check for parallelism
      if (fabs (dn) < 1e-7)
        return (false);

      // Get a point on the plane
      // w = P0 - V0
      double w[3];
      w[0] = line.at (0) + (plane.at (3) * plane.at (0));
      w[1] = line.at (1) + (plane.at (3) * plane.at (1));
      w[2] = line.at (2) + (plane.at (3) * plane.at (2));

      // point = P1 - (P0 - P1) . (d + n . P1) / [(P0-P1) . n];
      double u = (plane.at (0) * w[0] + plane.at (1) * w[1] + plane.at (2) * w[2]) / dn;
      point.x = line.at (0) - u * line.at (3);
      point.y = line.at (1) - u * line.at (4);
      point.z = line.at (2) - u * line.at (5);
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the intersection of a two 3D lines in space as a 3D point
      * \param line_a the coefficients of the first line (point, direction)
      * \param line_b the coefficients of the second line (point, direction)
      * \param point holder for the computed 3D point
      * \param sqr_eps maximum allowable squared distance to the true solution
      */
    bool
      lineWithLineIntersection (const std::vector<double> &line_a, const std::vector<double> &line_b, geometry_msgs::Point32 &point,
                                double sqr_eps)
    {
      std::vector<double> segment;
      cloud_geometry::distances::lineToLineSegment (line_a, line_b, segment);

      double sqr_dist = (segment.at (0) - segment.at (3)) * (segment.at (0) - segment.at (3)) +
                        (segment.at (1) - segment.at (4)) * (segment.at (1) - segment.at (4)) +
                        (segment.at (2) - segment.at (5)) * (segment.at (2) - segment.at (5));
      if (sqr_dist < sqr_eps)
      {
        point.x = segment.at (0);
        point.y = segment.at (1);
        point.z = segment.at (2);
        return (true);
      }
      return (false);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain the intersection of a plane with an axis-aligned cube as a (sorted) 2D polygon
      * \param plane the coefficients of the plane (a,b,c,d)
      * \param cube the 6 bounds of the cube
      * \param polygon the resulting polygon
      */
    bool
      planeWithCubeIntersection (const std::vector<double> &plane, const std::vector<double> &cube, geometry_msgs::Polygon &polygon)
    {
      double width[3];
      for (int d = 0; d < 3; d++)
        width[d] = cube.at (d + 3) - cube.at (d);

      double x[3];
      geometry_msgs::Point32 mean;
      mean.x = mean.y = mean.z = 0;

      // Keep one dimension constant
      for (int k = 0; k < 3; k++)
      {
        if (fabs (plane.at (k)) > 0)
        {
          // Get the other two dimensions
          int i = (k + 1) % 3;
          int j = (k + 2) % 3;

          // Iterate over another dimension
          for (double w_i = 0; w_i <= width[i]; w_i += width[i])
          {
            x[i] = cube.at (i) + w_i;
            // Iterate over another dimension
            for (double w_j = 0; w_j <= width[j]; w_j += width[j])
            {
              x[j] = cube.at (j) + w_j;

              // Evaluate the plane equation for this point (plane[k]*x[k] + plane[i]*x[i] + plane[j]*x[j] + plane[3] = 0)
              x[k] = -(plane.at (3) + x[i] * plane.at (i) + x[j] * plane.at (j) ) / plane.at (k);

              // Check if in cell
              if (x[k] >= cube.at (k) && x[k] <= cube.at (k + 3) )
              {
                geometry_msgs::Point32 ci;
                ci.x = x[0]; ci.y = x[1]; ci.z = x[2];
                polygon.points.push_back (ci);

                mean.x += x[0]; mean.y += x[1]; mean.z += x[2];
              } //
            } // for w_j
          } // for w_i
        }
      } // for k

      int npts = polygon.points.size ();
      // Compute the mean
      mean.x /= npts; mean.y /= npts; mean.z /= npts;

      // Sort the points so we create a convex polygon
      for (int i = 0; i < npts; i++)
      {
        double a = atan2 (polygon.points[i].y - mean.y, polygon.points[i].x - mean.x);
        if (a < 0) a += 2*M_PI;
        for (int j = i+1; j < npts; j++)
        {
          double b = atan2 (polygon.points[j].y - mean.y, polygon.points[j].x - mean.x);
          if (b < 0) b += 2*M_PI;
          if (b < a)
          {
            double temp;

            temp = polygon.points[j].x;
            polygon.points[j].x = polygon.points[i].x;
            polygon.points[i].x = temp;

            temp = polygon.points[j].y;
            polygon.points[j].y = polygon.points[i].y;
            polygon.points[i].y = temp;

            temp = polygon.points[j].z;
            polygon.points[j].z = polygon.points[i].z;
            polygon.points[i].z = temp;

            a = b;
          }
        } // for j
      } // for i
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Bounding box intersection modified from Graphics Gems Vol I. The method returns a non-zero value if the
      * bounding box is hit.
      * \param line the coefficients of the line (point, direction)
      * \param cube the 6 bounds of the cube
      */
    bool
      lineToBoxIntersection (const std::vector<double> &line, const std::vector<double> &cube)
    {
      const int _right = 0, _left = 1, _middle = 2;
      bool inside = true;   // start by assuming that the line origin is inside the box
      char    quadrant[3];
      int     which_plane = 0;
      double  max_t[3], candidate_plane[3];

      //  First find closest planes
      for (int d = 0; d < 3; d++)
      {
        if (line.at (d) < cube.at (2*d))
        {
          quadrant[d] = _left;
          candidate_plane[d] = cube.at (2*d);
          inside = false;
        }
        else if (line.at (d) > cube.at (2*d+1))
        {
          quadrant[d] = _right;
          candidate_plane[d] = cube.at (2*d+1);
          inside = false;
        }
        else
        {
          quadrant[d] = _middle;
        }
      }

      //  Check whether origin of ray is inside bbox
      if (inside)
        return (true);

      //  Calculate parametric distances to plane
      for (int d = 0; d < 3; d++)
      {
        if (quadrant[d] != _middle && line.at (d + 3) != 0.0)
          max_t[d] = (candidate_plane[d] - line.at (d)) / line.at (d + 3);
        else
          max_t[d] = -1.0;
      }

      //  Find the largest parametric value of intersection
      for (int d = 0; d < 3; d++)
        if (max_t[which_plane] < max_t[d])
          which_plane = d;

      //  Check for valid intersection along line
      if (max_t[which_plane] > 1.0 || max_t[which_plane] < 0.0)
        return (false);

      //  Intersection point along line is okay.  Check bbox.
      for (int d = 0; d < 3; d++)
      {
        if (which_plane != d)
        {
          double coord = line.at (d) + max_t[which_plane] * line.at (d + 3);
          if ( coord < cube.at (2*d) || coord > cube.at (2*d+1) )
            return (false);
        }
      }
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Check the intersection between a line segment and a circle
      * \param line the coefficients of the line (point, direction)
      * \param circle the coefficients of the circle (center, radius)
      */
    bool
      lineToCircleIntersection (const std::vector<double> &line, const std::vector<double> &circle)
    {
      double u = ((circle.at (0) - line[0]) * (line[3] - line[0]) +
                  (circle.at (1) - line[1]) * (line[4] - line[1]) +
                  (circle[2] - line[2]) * (line[5] - line[2]))
                  /
                ( (line[3] - line[0]) * (line[3] - line[0]) +
                  (line[4] - line[1]) * (line[4] - line[1]) +
                  (line[5] - line[2]) * (line[5] - line[2])
                );

      // Compute the intersection point
      geometry_msgs::Point32 pi;
      pi.x = line[0] + (line[3] - line[0]) * u;
      pi.y = line[0] + (line[4] - line[1]) * u;
      pi.z = line[0] + (line[5] - line[2]) * u;

      // Check the distance between the center of the circle and the intersection point
      if (circle[3] * circle[3] < ( (circle[0] - pi.x) * (circle[0] - pi.x) +
                                    (circle[1] - pi.y) * (circle[1] - pi.y) +
                                    (circle[2] - pi.z) * (circle[2] - pi.z) ))
        return false;

      if ((u >= 0) && (u <= 1))
        return true;
      else
        return false;
    }

  }
}
