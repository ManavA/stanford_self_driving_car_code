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
 * $Id: statistics.h 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_STATISTICS_H_
#define _CLOUD_GEOMETRY_STATISTICS_H_

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include <cfloat>

namespace cloud_geometry
{

  namespace statistics
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the point indices that form the largest diagonal in a given set of points
      * \param poly the polygon message
      * \param min_idx the resultant index of the 'minimum' point
      * \param max_idx the resultant index of the 'maximum' point
      */
    inline void
      getLargestDiagonalIndices (const geometry_msgs::Polygon &poly, int &min_idx, int &max_idx)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < poly.points.size (); i++)
      {
        for (unsigned int j = i; j < poly.points.size (); j++)
        {
          double current_diagonal =
                                    (poly.points[i].x - poly.points[j].x) * (poly.points[i].x - poly.points[j].x) +
                                    (poly.points[i].y - poly.points[j].y) * (poly.points[i].y - poly.points[j].y) +
                                    (poly.points[i].z - poly.points[j].z) * (poly.points[i].z - poly.points[j].z);
          if (current_diagonal > largest_diagonal)
          {
            min_idx = i;
            max_idx = j;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the point indices that form the largest diagonal in a given set of points
      * \param points the point cloud data message
      * \param min_idx the resultant index of the 'minimum' point
      * \param max_idx the resultant index of the 'maximum' point
      */
    inline void
      getLargestDiagonalIndices (const sensor_msgs::PointCloud &points, int &min_idx, int &max_idx)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        for (unsigned int j = i; j < points.points.size (); j++)
        {
          double current_diagonal =
                                    (points.points.at (i).x - points.points.at (j).x) * (points.points.at (i).x - points.points.at (j).x) +
                                    (points.points.at (i).y - points.points.at (j).y) * (points.points.at (i).y - points.points.at (j).y) +
                                    (points.points.at (i).z - points.points.at (j).z) * (points.points.at (i).z - points.points.at (j).z);
          if (current_diagonal > largest_diagonal)
          {
            min_idx = i;
            max_idx = j;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the point indices that form the largest diagonal in a given set of points
      * \param points the point cloud data message
      * \param indices the point cloud indices that need to be used
      * \param min_idx the resultant index of the 'minimum' point
      * \param max_idx the resultant index of the 'maximum' point
      */
    inline void
      getLargestDiagonalIndices (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, int &min_idx, int &max_idx)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        for (unsigned int j = i; j < indices.size (); j++)
        {
          double current_diagonal =
                                    (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) * (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) +
                                    (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y) * (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y) +
                                    (points.points.at (indices.at (i)).z - points.points.at (indices.at (j)).z) * (points.points.at (indices.at (i)).z - points.points.at (indices.at (j)).z);
          if (current_diagonal > largest_diagonal)
          {
            min_idx = i;
            max_idx = j;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the points that form the largest diagonal in a given set of points
      * \param poly the polygon message
      * \param minP the resultant minimum point in the set
      * \param maxP the resultant maximum point in the set
      */
    inline void
      getLargestDiagonalPoints (const geometry_msgs::Polygon &poly, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < poly.points.size (); i++)
      {
        for (unsigned int j = i; j < poly.points.size (); j++)
        {
          double current_diagonal =
                                    (poly.points[i].x - poly.points[j].x) * (poly.points[i].x - poly.points[j].x) +
                                    (poly.points[i].y - poly.points[j].y) * (poly.points[i].y - poly.points[j].y) +
                                    (poly.points[i].z - poly.points[j].z) * (poly.points[i].z - poly.points[j].z);
          if (current_diagonal > largest_diagonal)
          {
            min_p.x = poly.points[i].x; min_p.y = poly.points[i].y; min_p.z = poly.points[i].z;
            max_p.x = poly.points[j].x; max_p.y = poly.points[j].y; max_p.z = poly.points[j].z;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the points that form the largest diagonal in a given set of points
      * \param points the point cloud data message
      * \param minP the resultant minimum point in the set
      * \param maxP the resultant maximum point in the set
      */
    inline void
      getLargestDiagonalPoints (const sensor_msgs::PointCloud &points, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        for (unsigned int j = i; j < points.points.size (); j++)
        {
          double current_diagonal =
                                    (points.points.at (i).x - points.points.at (j).x) * (points.points.at (i).x - points.points.at (j).x) +
                                    (points.points.at (i).y - points.points.at (j).y) * (points.points.at (i).y - points.points.at (j).y) +
                                    (points.points.at (i).z - points.points.at (j).z) * (points.points.at (i).z - points.points.at (j).z);
          if (current_diagonal > largest_diagonal)
          {
            min_p.x = points.points.at (i).x; min_p.y = points.points.at (i).y; min_p.z = points.points.at (i).z;
            max_p.x = points.points.at (j).x; max_p.y = points.points.at (j).y; max_p.z = points.points.at (j).z;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the points that form the largest diagonal in a given set of points
      * \param points the point cloud data message
      * \param indices the point cloud indices that need to be used
      * \param minP the resultant minimum point in the set
      * \param maxP the resultant maximum point in the set
      */
    inline void
      getLargestDiagonalPoints (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      double largest_diagonal = -FLT_MAX;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        for (unsigned int j = i; j < indices.size (); j++)
        {
          double current_diagonal =
                                    (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) * (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) +
                                    (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y) * (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y) +
                                    (points.points.at (indices.at (i)).z - points.points.at (indices.at (j)).z) * (points.points.at (indices.at (i)).z - points.points.at (indices.at (j)).z);
          if (current_diagonal > largest_diagonal)
          {
            min_p.x = points.points.at (indices.at (i)).x; min_p.y = points.points.at (indices.at (i)).y; min_p.z = points.points.at (indices.at (i)).z;
            max_p.x = points.points.at (indices.at (j)).x; max_p.y = points.points.at (indices.at (j)).y; max_p.z = points.points.at (indices.at (j)).z;
            largest_diagonal = current_diagonal;
          }
        }
      }
    }

    geometry_msgs::Point32 computeMedian (const sensor_msgs::PointCloud &points);
    geometry_msgs::Point32 computeMedian (const sensor_msgs::PointCloud &points, const std::vector<int> &indices);

    double computeMedianAbsoluteDeviation (const sensor_msgs::PointCloud &points, double sigma);
    double computeMedianAbsoluteDeviation (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double sigma);



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the points that form the largest diagonal in a given set of points
      * \param poly the polygon message
      * \param minP the resultant minimum point in the set
      * \param maxP the resultant maximum point in the set
      */
    inline void
      getLargestXYPoints (const geometry_msgs::Polygon &poly, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      double largest_xy = -FLT_MAX;
      for (unsigned int i = 0; i < poly.points.size (); i++)
      {
        for (unsigned int j = i; j < poly.points.size (); j++)
        {
          double current_xy = (poly.points[i].x - poly.points[j].x) * (poly.points[i].x - poly.points[j].x) +
                              (poly.points[i].y - poly.points[j].y) * (poly.points[i].y - poly.points[j].y);
          if (current_xy > largest_xy)
          {
            min_p.x = poly.points[i].x; min_p.y = poly.points[i].y; min_p.z = 0;
            max_p.x = poly.points[j].x; max_p.y = poly.points[j].y; max_p.z = 0;
            largest_xy = current_xy;
          }
        }
      }
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the points that form the largest diagonal in a given set of points
      * \param points the point cloud data message
      * \param indices the point cloud indices that need to be used
      * \param minP the resultant minimum point in the set
      * \param maxP the resultant maximum point in the set
      */
    inline void
      getLargestXYPoints (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      double largest_xy = -FLT_MAX;
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        for (unsigned int j = i; j < indices.size (); j++)
        {
          double current_xy = (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) *
                              (points.points.at (indices.at (i)).x - points.points.at (indices.at (j)).x) +
                              (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y) *
                              (points.points.at (indices.at (i)).y - points.points.at (indices.at (j)).y);
          if (current_xy > largest_xy )
          {
            min_p.x = points.points.at (indices.at (i)).x;
            min_p.y = points.points.at (indices.at (i)).y;
            min_p.z = 0;
            max_p.x = points.points.at (indices.at (j)).x;
            max_p.y = points.points.at (indices.at (j)).y;
            max_p.z = 0;
            largest_xy = current_xy;
          }
        }
      }
    }



    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centralized moment at a 3D points patch
      * \param points the point cloud data message
      * \param p the p-dimension
      * \param q the q-dimension
      * \param r the r-dimension
      */
    inline double
      computeCentralizedMoment (const sensor_msgs::PointCloud &points, double p, double q, double r)
    {
      double result = 0.0;
      for (unsigned int cp = 0; cp < points.points.size (); cp++)
        result += pow (points.points[cp].x, p) * pow (points.points[cp].y, q) * pow (points.points[cp].z, r);

      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centralized moment at a 3D points patch, using their indices.
      * \param points the point cloud data message
      * \param indices the point cloud indices that need to be used
      * \param p the p-dimension
      * \param q the q-dimension
      * \param r the r-dimension
      */
    inline double
      computeCentralizedMoment (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double p, double q, double r)
    {
      double result = 0.0;
      for (unsigned int cp = 0; cp < indices.size (); cp++)
        result += pow (points.points.at (indices.at (cp)).x, p) * pow (points.points.at (indices.at (cp)).y, q) * pow (points.points.at (indices.at (cp)).z, r);

      return (result);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given point cloud
      * \param points the point cloud message
      * \param minP the resultant minimum bounding box coordinates
      * \param maxP the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (const sensor_msgs::PointCloud &points, geometry_msgs::Point32 &minP, geometry_msgs::Point32 &maxP)
    {
      minP.x = minP.y = minP.z = FLT_MAX;
      maxP.x = maxP.y = maxP.z = -FLT_MAX;

      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        minP.x = (points.points[i].x < minP.x) ? points.points[i].x : minP.x;
        minP.y = (points.points[i].y < minP.y) ? points.points[i].y : minP.y;
        minP.z = (points.points[i].z < minP.z) ? points.points[i].z : minP.z;

        maxP.x = (points.points[i].x > maxP.x) ? points.points[i].x : maxP.x;
        maxP.y = (points.points[i].y > maxP.y) ? points.points[i].y : maxP.y;
        maxP.z = (points.points[i].z > maxP.z) ? points.points[i].z : maxP.z;
      }
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given 3D polygon
      * \param poly the polygon message
      * \param min_p the resultant minimum bounding box coordinates
      * \param max_p the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (const geometry_msgs::Polygon &poly, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      min_p.x = min_p.y = min_p.z = FLT_MAX;
      max_p.x = max_p.y = max_p.z = -FLT_MAX;

      for (unsigned int i = 0; i < poly.points.size (); i++)
      {
        min_p.x = (poly.points[i].x < min_p.x) ? poly.points[i].x : min_p.x;
        min_p.y = (poly.points[i].y < min_p.y) ? poly.points[i].y : min_p.y;
        min_p.z = (poly.points[i].z < min_p.z) ? poly.points[i].z : min_p.z;

        max_p.x = (poly.points[i].x > max_p.x) ? poly.points[i].x : max_p.x;
        max_p.y = (poly.points[i].y > max_p.y) ? poly.points[i].y : max_p.y;
        max_p.z = (poly.points[i].z > max_p.z) ? poly.points[i].z : max_p.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Determine the minimum and maximum 3D bounding box coordinates for a given point cloud, using point indices
      * \param points the point cloud message
      * \param indices the point cloud indices to use
      * \param min_p the resultant minimum bounding box coordinates
      * \param max_p the resultant maximum bounding box coordinates
      */
    inline void
      getMinMax (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &min_p, geometry_msgs::Point32 &max_p)
    {
      min_p.x = min_p.y = min_p.z = FLT_MAX;
      max_p.x = max_p.y = max_p.z = -FLT_MAX;

      for (unsigned int i = 0; i < indices.size (); i++)
      {
        min_p.x = (points.points.at (indices.at (i)).x < min_p.x) ? points.points.at (indices.at (i)).x : min_p.x;
        min_p.y = (points.points.at (indices.at (i)).y < min_p.y) ? points.points.at (indices.at (i)).y : min_p.y;
        min_p.z = (points.points.at (indices.at (i)).z < min_p.z) ? points.points.at (indices.at (i)).z : min_p.z;

        max_p.x = (points.points.at (indices.at (i)).x > max_p.x) ? points.points.at (indices.at (i)).x : max_p.x;
        max_p.y = (points.points.at (indices.at (i)).y > max_p.y) ? points.points.at (indices.at (i)).y : max_p.y;
        max_p.z = (points.points.at (indices.at (i)).z > max_p.z) ? points.points.at (indices.at (i)).z : max_p.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (const sensor_msgs::PointCloud &points, geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        if (c_idx != -1 && points.channels[c_idx].values[i] > cut_distance)
          continue;
        min_pt.x = (points.points[i].x < min_pt.x) ? points.points[i].x : min_pt.x;
        min_pt.y = (points.points[i].y < min_pt.y) ? points.points[i].y : min_pt.y;
        min_pt.z = (points.points[i].z < min_pt.z) ? points.points[i].z : min_pt.z;

        max_pt.x = (points.points[i].x > max_pt.x) ? points.points[i].x : max_pt.x;
        max_pt.y = (points.points[i].y > max_pt.y) ? points.points[i].y : max_pt.y;
        max_pt.z = (points.points[i].z > max_pt.z) ? points.points[i].z : max_pt.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (sensor_msgs::PointCloudConstPtr points, geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

      for (unsigned int i = 0; i < points->points.size (); i++)
      {
        if (c_idx != -1 && points->channels[c_idx].values[i] > cut_distance)
          continue;
        min_pt.x = (points->points[i].x < min_pt.x) ? points->points[i].x : min_pt.x;
        min_pt.y = (points->points[i].y < min_pt.y) ? points->points[i].y : min_pt.y;
        min_pt.z = (points->points[i].z < min_pt.z) ? points->points[i].z : min_pt.z;

        max_pt.x = (points->points[i].x > max_pt.x) ? points->points[i].x : max_pt.x;
        max_pt.y = (points->points[i].y > max_pt.y) ? points->points[i].y : max_pt.y;
        max_pt.z = (points->points[i].z > max_pt.z) ? points->points[i].z : max_pt.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param indices the point cloud indices to use
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

      for (unsigned int i = 0; i < indices.size (); i++)
      {
        if (c_idx != -1 && points.channels[c_idx].values[indices.at (i)] > cut_distance)
          continue;
        min_pt.x = (points.points[indices.at (i)].x < min_pt.x) ? points.points[indices.at (i)].x : min_pt.x;
        min_pt.y = (points.points[indices.at (i)].y < min_pt.y) ? points.points[indices.at (i)].y : min_pt.y;
        min_pt.z = (points.points[indices.at (i)].z < min_pt.z) ? points.points[indices.at (i)].z : min_pt.z;

        max_pt.x = (points.points[indices.at (i)].x > max_pt.x) ? points.points[indices.at (i)].x : max_pt.x;
        max_pt.y = (points.points[indices.at (i)].y > max_pt.y) ? points.points[indices.at (i)].y : max_pt.y;
        max_pt.z = (points.points[indices.at (i)].z > max_pt.z) ? points.points[indices.at (i)].z : max_pt.z;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the minimum and maximum values on each of the 3 (x-y-z) dimensions
      * in a given pointcloud, without considering points outside of a distance threshold from the laser origin
      * \param points the point cloud data message
      * \param indices the point cloud indices to use
      * \param min_pt the resultant minimum bounds
      * \param max_pt the resultant maximum bounds
      * \param c_idx the index of the channel holding distance information
      * \param cut_distance a maximum admissible distance threshold for points from the laser origin
      */
    inline void
      getMinMax (sensor_msgs::PointCloudConstPtr points, const std::vector<int> &indices, geometry_msgs::Point32 &min_pt, geometry_msgs::Point32 &max_pt,
                 int c_idx, double cut_distance)
    {
      min_pt.x = min_pt.y = min_pt.z = FLT_MAX;
      max_pt.x = max_pt.y = max_pt.z = -FLT_MAX;

      for (unsigned int i = 0; i < indices.size (); i++)
      {
        if (c_idx != -1 && points->channels[c_idx].values[indices.at (i)] > cut_distance)
          continue;
        min_pt.x = (points->points[indices.at (i)].x < min_pt.x) ? points->points[indices.at (i)].x : min_pt.x;
        min_pt.y = (points->points[indices.at (i)].y < min_pt.y) ? points->points[indices.at (i)].y : min_pt.y;
        min_pt.z = (points->points[indices.at (i)].z < min_pt.z) ? points->points[indices.at (i)].z : min_pt.z;

        max_pt.x = (points->points[indices.at (i)].x > max_pt.x) ? points->points[indices.at (i)].x : max_pt.x;
        max_pt.y = (points->points[indices.at (i)].y > max_pt.y) ? points->points[indices.at (i)].y : max_pt.y;
        max_pt.z = (points->points[indices.at (i)].z > max_pt.z) ? points->points[indices.at (i)].z : max_pt.z;
      }
    }

    void getChannelMeanStd (const sensor_msgs::PointCloud &points, int d_idx, double &mean, double &stddev);
    void getChannelMeanStd (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, int d_idx, double &mean, double &stddev);

    void selectPointsOutsideDistribution (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, int d_idx,
                                          double mean, double stddev, double alpha, std::vector<int> &inliers);
    void selectPointsInsideDistribution (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, int d_idx,
                                         double mean, double stddev, double alpha, std::vector<int> &inliers);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute both the mean and the standard deviation of an array of values
      * \param values the array of values
      * \param mean the resultant mean of the distribution
      * \param stddev the resultant standard deviation of the distribution
      */
    inline void
      getMeanStd (const std::vector<int> &values, double &mean, double &stddev)
    {
      double sum = 0, sq_sum = 0;

      for (unsigned int i = 0; i < values.size (); i++)
      {
        sum += values.at (i);
        sq_sum += values.at (i) * values.at (i);
      }
      mean = sum / values.size ();
      double variance = (double)(sq_sum - sum * sum / values.size ()) / (values.size () - 1);
      stddev = sqrt (variance);
    }

    void getTrimean (std::vector<int> values, double &trimean);

  }
}

#endif
