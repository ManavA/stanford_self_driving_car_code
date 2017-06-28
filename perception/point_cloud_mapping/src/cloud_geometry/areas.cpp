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
 * $Id: areas.cpp 21675 2009-08-12 19:40:12Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/intersections.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cfloat>
#include <algorithm>

namespace cloud_geometry
{

  namespace areas
  {
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param points the point cloud (planar)
      * \param normal the plane normal
      */
    double
      compute2DPolygonalArea (const sensor_msgs::PointCloud &points, const std::vector<double> &normal)
    {
      int k0, k1, k2;

      // Find axis with largest normal component and project onto perpendicular plane
      k0 = (fabs (normal.at (0) ) > fabs (normal.at (1))) ? 0  : 1;
      k0 = (fabs (normal.at (k0)) > fabs (normal.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // cos(theta), where theta is the angle between the polygon and the projected plane
      double ct = fabs ( normal.at (k0) );

      double area = 0;
      float p_i[3], p_j[3];

      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        p_i[0] = points.points[i].x; p_i[1] = points.points[i].y; p_i[2] = points.points[i].z;
        int j = (i + 1) % points.points.size ();
        p_j[0] = points.points[j].x; p_j[1] = points.points[j].y; p_j[2] = points.points[j].z;

        area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
      }
      area = fabs (area) / (2 * ct);

      return (area);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch - using a given normal
      * \param polygon the planar polygon
      * \param normal the plane normal
      */
    double
      compute2DPolygonalArea (const geometry_msgs::Polygon &polygon, const std::vector<double> &normal)
    {
      int k0, k1, k2;

      // Find axis with largest normal component and project onto perpendicular plane
      k0 = (fabs (normal.at (0) ) > fabs (normal.at (1))) ? 0  : 1;
      k0 = (fabs (normal.at (k0)) > fabs (normal.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // cos(theta), where theta is the angle between the polygon and the projected plane
      double ct = fabs ( normal.at (k0) );

      double area = 0;
      float p_i[3], p_j[3];

      for (unsigned int i = 0; i < polygon.points.size (); i++)
      {
        p_i[0] = polygon.points[i].x; p_i[1] = polygon.points[i].y; p_i[2] = polygon.points[i].z;
        int j = (i + 1) % polygon.points.size ();
        p_j[0] = polygon.points[j].x; p_j[1] = polygon.points[j].y; p_j[2] = polygon.points[j].z;

        area += p_i[k1] * p_j[k2] - p_i[k2] * p_j[k1];
      }
      area = fabs (area) / (2 * ct);

      return (area);
    }


    bool compute2DPolygonNormal(const geometry_msgs::Polygon &poly, std::vector<double> &normal)
    {
      int k0,k1,k2;
      int sz=poly.points.size();
      if(sz<3)
	return false;
      
      k0=0;
      k1=sz/3;
      if(k1<=k0) k1=k0+1;
      k2=(2*sz)/3;
      if(k2<=k1) k2=k1+1;
      
      if(k2>=sz) return false;
      
      Eigen::Vector3d p1, p2, p3;
      
      p1 (0) = poly.points[k0].x-poly.points[k2].x;
      p1 (1) = poly.points[k0].y-poly.points[k2].y;
      p1 (2) = poly.points[k0].z-poly.points[k2].z;
      
      p2 (0) = poly.points[k1].x-poly.points[k2].x;
      p2 (1) = poly.points[k1].y-poly.points[k2].y;
      p2 (2) = poly.points[k1].z-poly.points[k2].z;
      
      p3 = p1.cross (p2);
      p3.normalize();

      normal.resize(3);
      normal[0]=p3(0);
      normal[1]=p3(1);
      normal[2]=p3(2);
      return true;
    }
  
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the area of a 2D planar polygon patch. The normal is computed automatically.
      * \param polygon the planar polygon
      * \param normal the plane normal
      */
    double
      compute2DPolygonalArea (const geometry_msgs::Polygon &polygon)
    {
      std::vector<double> normal;
      if(!compute2DPolygonNormal(polygon,normal))
	return 0; //the polygon is degenerate, so its area is 0;
      
      return compute2DPolygonalArea (polygon,normal);
      
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a 2D convex hull in 3D space using Andrew's monotone chain algorithm
      * \param points the point cloud
      * \param indices the point indices to use from the cloud (they must form a planar model)
      * \param coeff the *normalized* planar model coefficients
      * \param hull the resultant convex hull model as a \a Polygon3D
      */
    void
      convexHull2D (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, const std::vector<double> &coeff,
                    geometry_msgs::Polygon &hull)
    {
      // Copy the point data to a local Eigen::Matrix. This is slow and should be replaced by extending geometry_msgs::Point32
      // to allow []/() accessors.
      std::vector<Eigen::Vector3f> epoints (indices.size ());
      for (unsigned int cp = 0; cp < indices.size (); cp++)
      {
        epoints[cp](0) = points.points[indices.at (cp)].x;
        epoints[cp](1) = points.points[indices.at (cp)].y;
        epoints[cp](2) = points.points[indices.at (cp)].z;
      }

      // Determine the best plane to project points onto
      int k0, k1, k2;
      k0 = (fabs (coeff.at (0) ) > fabs (coeff.at (1))) ? 0  : 1;
      k0 = (fabs (coeff.at (k0)) > fabs (coeff.at (2))) ? k0 : 2;
      k1 = (k0 + 1) % 3;
      k2 = (k0 + 2) % 3;

      // Compute a 2D centroid for two dimensions
      Eigen::Vector2d centroid (0, 0);
      for (unsigned int cp = 0; cp < epoints.size (); cp++)
      {
        centroid (0) += epoints[cp](k1);
        centroid (1) += epoints[cp](k2);
      }
      centroid (0) /= epoints.size ();
      centroid (1) /= epoints.size ();

      // Push projected centered 2d points
      std::vector<geometry_msgs::Point32> epoints_demean (epoints.size ());
      for (unsigned int cp = 0; cp < indices.size (); cp++)
      {
        epoints_demean[cp].x = epoints[cp](k1) - centroid (0);
        epoints_demean[cp].y = epoints[cp](k2) - centroid (1);
      }

      std::sort (epoints_demean.begin (), epoints_demean.end (), comparePoint2D);

      geometry_msgs::Polygon hull_2d;
      convexHull2D (epoints_demean, hull_2d);

      int nr_points_hull = hull_2d.points.size ();
      if (nr_points_hull >= 3)
      {
        // Determine the convex hull direction
        Eigen::Vector3d p1, p2, p3;

        p1 (k0) = 0;
        p1 (k1) = -hull_2d.points[0].x + hull_2d.points[1].x;
        p1 (k2) = -hull_2d.points[0].y + hull_2d.points[1].y;

        p2 (k0) = 0;
        p2 (k1) = -hull_2d.points[0].x + hull_2d.points[2].x;
        p2 (k2) = -hull_2d.points[0].y + hull_2d.points[2].y;

        p3 = p1.cross (p2);

        bool direction = (p3 (k0) * coeff.at (k0) > 0);

        // Create the Polygon3D object
        hull.points.resize (nr_points_hull);

        // Copy hull points in clockwise or anti-clockwise format
        for (int cp = 0; cp < nr_points_hull; cp++)
        {
          int d = direction ? cp : (nr_points_hull - cp - 1);
          Eigen::Vector3f pt;
          pt (k1) = hull_2d.points[cp].x + centroid (0);
          pt (k2) = hull_2d.points[cp].y + centroid (1);
          pt (k0) = -(coeff.at (3) + pt (k1) * coeff.at (k1) + pt (k2) * coeff.at (k2)) / coeff.at (k0);

          // Copy the point data to Polygon3D format
          hull.points[d].x = pt (0);
          hull.points[d].y = pt (1);
          hull.points[d].z = pt (2);
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute a 2D convex hull using Andrew's monotone chain algorithm
      * \note (code snippet inspired from http://www.softsurfer.com/Archive/algorithm_0109/algorithm_0109.htm)
      *        Copyright 2001, softSurfer (www.softsurfer.com)
      * \param points the 2D projected point cloud representing a planar model
      * \param hull the resultant 2D convex hull model as a \a Polygon
      */
    void
      convexHull2D (const std::vector<geometry_msgs::Point32> &points, geometry_msgs::Polygon &hull)
    {
      int nr_points = points.size ();
      hull.points.resize (nr_points + 1);

      // Indices for bottom and top of the stack
      int bot = 0, top = -1;
      int i;

      for (i = 1; i < nr_points; i++)
        // points[0].x represents the smallest X coordinate
        if (points[i].x != points[0].x)
          break;

      // Get the indices of points with min|max y-coord
      int minmax = i - 1;

      // Degenerate case: all x-coords == xmin
      if ( minmax == (nr_points - 1) )
      {
        ++top;
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
        hull.points[top].z = points[0].z;
        // A nontrivial segment
        if (points[minmax].y != points[0].y)
        {
          ++top;
          hull.points[top].x = points[minmax].x;
          hull.points[top].y = points[minmax].y;
          hull.points[top].z = points[minmax].z;
        }
        ++top;
        // Add the polygon's endpoint
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
        hull.points[top].z = points[0].z;
        hull.points.resize (top + 1);
        return;
      }

      int maxmin;
      for (i = nr_points - 2; i >= 0; i--)
        if (points[i].x != points[nr_points - 1].x)
          break;
      maxmin = i + 1;

      // Compute the lower hull
      ++top;
      // Add the polygon's endpoint
      hull.points[top].x = points[0].x;
      hull.points[top].y = points[0].y;
      hull.points[top].z = points[0].z;

      i = minmax;
      while (++i <= maxmin)
      {
        // The lower line joins P[minmin] with P[maxmin]
        if ((i < maxmin) && (
            (points[maxmin].x - points[0].x) * (points[i].y      - points[0].y) -
            (points[i].x      - points[0].x) * (points[maxmin].y - points[0].y) >= 0))
          continue;          // ignore P[i] above or on the lower line

        // If there are at least 2 points on the stack
        while (top > 0)
        {
          // Test if P[i] is left of the line at the stack top
          if ((hull.points[top].x - hull.points[top-1].x) * (points[i].y        - hull.points[top-1].y) -
              (points[i].x        - hull.points[top-1].x) * (hull.points[top].y - hull.points[top-1].y) > 0)
            break;         // P[i] is a new hull vertex
          else
            top--;         // pop top point off stack
        }
        ++top;
        hull.points[top].x = points[i].x;
        hull.points[top].y = points[i].y;
        hull.points[top].z = points[i].z;
      }

      // Next, compute the upper hull above the bottom hull
      if ((nr_points - 1) != maxmin)      // if distinct xmax points
      {
        ++top;
        // Add the point with max X and max Y coordinates to the hull
        hull.points[top].x = points[nr_points - 1].x;
        hull.points[top].y = points[nr_points - 1].y;
        hull.points[top].z = points[nr_points - 1].z;
      }
      // The bottom point of the upper hull stack
      bot = top;

      i = maxmin;
      while (--i >= minmax)
      {
        // The upper line joins P[nr_points - 1] with P[minmax]
        if ((i > minmax) && (
            (points[minmax].x - points[nr_points - 1].x) * (points[i].y      - points[nr_points - 1].y) -
            (points[i].x      - points[nr_points - 1].x) * (points[minmax].y - points[nr_points - 1].y) >= 0))
          continue;          // ignore P[i] below or on the upper line

        // If there are at least 2 points on the stack
        while (top > bot)
        {
          // Test if P[i] is left of the line at the stack top
          if ((hull.points[top].x - hull.points[top-1].x) * (points[i].y        - hull.points[top-1].y) -
              (points[i].x        - hull.points[top-1].x) * (hull.points[top].y - hull.points[top-1].y) > 0)
            break;         // P[i] is a new hull vertex
          else
            top--;         // pop top point off stack
        }
        ++top;

        hull.points[top].x = points[i].x;
        hull.points[top].y = points[i].y;
        hull.points[top].z = points[i].z;
      }

      if (minmax != 0)
      {
        ++top;
        // Add the polygon's endpoint
        hull.points[top].x = points[0].x;
        hull.points[top].y = points[0].y;
        hull.points[top].z = points[0].z;
      }
      hull.points.resize (top + 1);
      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Check if a 2d point (X and Y coordinates considered only!) is inside or outisde a given polygon
      * \note (This is highly optimized code taken from http://www.visibone.com/inpoly/)
      *       Copyright (c) 1995-1996 Galacticomm, Inc.  Freeware source code.
      * \param point a 3D point projected onto the same plane as the polygon
      * \param polygon a polygon
      */
    bool
      isPointIn2DPolygon (const geometry_msgs::Point32 &point, const geometry_msgs::Polygon &polygon)
    {
      bool in_poly = false;
      double x1, x2, y1, y2;

      int nr_poly_points = polygon.points.size ();
      double xold = polygon.points[nr_poly_points - 1].x;
      double yold = polygon.points[nr_poly_points - 1].y;
      for (int i = 0; i < nr_poly_points; i++)
      {
        double xnew = polygon.points[i].x;
        double ynew = polygon.points[i].y;
        if (xnew > xold)
        {
          x1 = xold;
          x2 = xnew;
          y1 = yold;
          y2 = ynew;
        }
        else
        {
          x1 = xnew;
          x2 = xold;
          y1 = ynew;
          y2 = yold;
        }

        if ( (xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1) )
        {
          in_poly = !in_poly;
        }
        xold = xnew;
        yold = ynew;
      }
      return (in_poly);
    }

  }
}
