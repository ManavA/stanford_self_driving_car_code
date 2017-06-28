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
 * $Id: point.cpp 24968 2009-10-14 20:54:15Z wheeler $
 *
 */

/** \author Radu Bogdan Rusu */

#include <algorithm>
#include <cfloat>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/statistics.h>

namespace cloud_geometry
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the index of a specified dimension/channel in a point cloud
    * \param points the point cloud
    * \param channel_name the string defining the channel name
    */
  int
    getChannelIndex (const sensor_msgs::PointCloud &points, std::string channel_name)
  {
    for (unsigned int d = 0; d < points.channels.size (); d++)
      if (points.channels[d].name == channel_name)
        return (d);
    return (-1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the index of a specified dimension/channel in a point cloud
    * \param points the point cloud
    * \param channel_name the string defining the channel name
    */
  int
    getChannelIndex (sensor_msgs::PointCloudConstPtr points, std::string channel_name)
  {
    for (unsigned int d = 0; d < points->channels.size (); d++)
      if (points->channels[d].name == channel_name)
        return (d);
    return (-1);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the available dimensions as a space separated string
    * \param cloud the point cloud data message
    */
  std::string
    getAvailableChannels (const sensor_msgs::PointCloud &cloud)
  {
    std::string result;
    if (cloud.channels.size () == 0)
      return (result);
    unsigned int i;
    for (i = 0; i < cloud.channels.size () - 1; i++)
    {
      std::string index = cloud.channels[i].name + " ";
      result += index;
    }
    std::string index = cloud.channels[i].name;
    result += index;
    return (result);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the available dimensions as a space separated string
    * \param cloud a pointer to the point cloud data message
    */
  std::string
    getAvailableChannels (const sensor_msgs::PointCloudConstPtr& cloud)
  {
    std::string result;
    if (cloud->channels.size () == 0)
      return (result);
    unsigned int i;
    for (i = 0; i < cloud->channels.size () - 1; i++)
    {
      std::string index = cloud->channels[i].name + " ";
      result += index;
    }
    std::string index = cloud->channels[i].name;
    result += index;
    return (result);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the point indices from a cloud, whose normals are close to parallel with a given axis direction.
    * \param points the point cloud message
    * \param nx the normal X channel index
    * \param ny the normal Y channel index
    * \param nz the normal Z channel index
    * \param axis the given axis direction
    * \param eps_angle the maximum allowed difference threshold between the normal and the axis (in radians)
    * \param indices the resultant indices
    */
  void
    getPointIndicesAxisParallelNormals (const sensor_msgs::PointCloud &points, int nx, int ny, int nz, double eps_angle,
                                        const geometry_msgs::Point32 &axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points.points.size (); i++)
    {
      geometry_msgs::Point32 p;
      p.x = points.channels[nx].values[i];
      p.y = points.channels[ny].values[i];
      p.z = points.channels[nz].values[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (p, axis));
      if ( (angle < eps_angle) || ( (M_PI - angle) < eps_angle ) )
        indices.push_back (i);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the point indices from a cloud, whose normals are close to perpendicular to a given axis direction.
    * \param points the point cloud message
    * \param nx the normal X channel index
    * \param ny the normal Y channel index
    * \param nz the normal Z channel index
    * \param axis the given axis direction
    * \param eps_angle the maximum allowed difference threshold between the normal and the axis (in radians)
    * \param indices the resultant indices
    */
  void
    getPointIndicesAxisPerpendicularNormals (const sensor_msgs::PointCloud &points, int nx, int ny, int nz, double eps_angle,
                                             const geometry_msgs::Point32 &axis, std::vector<int> &indices)
  {
    // Check all points
    for (unsigned int i = 0; i < points.points.size (); i++)
    {
      geometry_msgs::Point32 p;
      p.x = points.channels[nx].values[i];
      p.y = points.channels[ny].values[i];
      p.z = points.channels[nz].values[i];
      // Compute the angle between their normal and the given axis
      double angle = acos (dot (p, axis));
      if (fabs (M_PI / 2.0 - angle) < eps_angle)
        indices.push_back (i);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param points a pointer to the point cloud message
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    * \param leaves a vector of already existing leaves (empty for the first call)
    * \param d_idx the index of the channel providing distance data (set to -1 if nonexistant)
    * \param cut_distance the maximum admissible distance of a point from the viewpoint (default: FLT_MAX)
    */
  void
    downsamplePointCloud (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, sensor_msgs::PointCloud &points_down, geometry_msgs::Point leaf_size,
                          std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1)
      cut_distance = DBL_MAX;
    // Copy the header (and thus the frame_id) + allocate enough space for points
    points_down.header = points.header;
    points_down.points.resize (points.points.size ());

    geometry_msgs::Point32 min_p, max_p, min_b, max_b, div_b;
    cloud_geometry::statistics::getMinMax (points, indices, min_p, max_p, d_idx, cut_distance);

    // Compute the minimum and maximum bounding box values
    min_b.x = (int)(floor (min_p.x / leaf_size.x));
    max_b.x = (int)(floor (max_p.x / leaf_size.x));

    min_b.y = (int)(floor (min_p.y / leaf_size.y));
    max_b.y = (int)(floor (max_p.y / leaf_size.y));

    min_b.z = (int)(floor (min_p.z / leaf_size.z));
    max_b.z = (int)(floor (max_p.z / leaf_size.z));

    // Compute the number of divisions needed along all axis
    div_b.x = (int)(max_b.x - min_b.x + 1);
    div_b.y = (int)(max_b.y - min_b.y + 1);
    div_b.z = (int)(max_b.z - min_b.z + 1);

    // Allocate the space needed
    try
    {
      if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
        leaves.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
      leaves.resize (div_b.x * div_b.y * div_b.z);
    }
    catch (std::bad_alloc)
    {
      ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                 div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
    }

    unsigned long leaves_size = div_b.x * div_b.y * div_b.z;
    if (leaves_size != leaves.size ())
      ROS_ERROR("That's odd: %lu != %zu", leaves_size, leaves.size());
    for (unsigned int cl = 0; cl < leaves_size; cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
        leaves[cl].nr_points = 0;
      }
    }

    // First pass: go over all points and insert them into the right leaf
    for (unsigned int cp = 0; cp < indices.size (); cp++)
    {
      if (d_idx != -1 && points.channels[d_idx].values[indices.at (cp)] > cut_distance)        // Use a threshold for cutting out points which are too far away
        continue;

      int i = (int)(floor (points.points[indices.at (cp)].x / leaf_size.x));
      int j = (int)(floor (points.points[indices.at (cp)].y / leaf_size.y));
      int k = (int)(floor (points.points[indices.at (cp)].z / leaf_size.z));

      int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
      leaves[idx].centroid_x += points.points[indices.at (cp)].x;
      leaves[idx].centroid_y += points.points[indices.at (cp)].y;
      leaves[idx].centroid_z += points.points[indices.at (cp)].z;
      leaves[idx].nr_points++;
    }

    // Second pass: go over all leaves and compute centroids
    int nr_p = 0;
    for (unsigned int cl = 0; cl < leaves_size; cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        points_down.points[nr_p].x = leaves[cl].centroid_x / leaves[cl].nr_points;
        points_down.points[nr_p].y = leaves[cl].centroid_y / leaves[cl].nr_points;
        points_down.points[nr_p].z = leaves[cl].centroid_z / leaves[cl].nr_points;
        nr_p++;
      }
    }
    points_down.points.resize (nr_p);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param points the point cloud message
    * \param indices a set of point indices
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    * \param leaves a vector of already existing leaves (empty for the first call)
    * \param d_idx the index of the channel providing distance data (set to -1 if nonexistant)
    * \param cut_distance the maximum admissible distance of a point from the viewpoint (default: FLT_MAX)
    */
  void
    downsamplePointCloud (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &points_down, geometry_msgs::Point leaf_size,
                          std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1)
      cut_distance = DBL_MAX;
    // Copy the header (and thus the frame_id) + allocate enough space for points
    points_down.header = points.header;
    points_down.points.resize (points.points.size ());

    geometry_msgs::Point32 min_p, max_p, min_b, max_b, div_b;
    cloud_geometry::statistics::getMinMax (points, min_p, max_p, d_idx, cut_distance);

    // Compute the minimum and maximum bounding box values
    min_b.x = (int)(floor (min_p.x / leaf_size.x));
    max_b.x = (int)(floor (max_p.x / leaf_size.x));

    min_b.y = (int)(floor (min_p.y / leaf_size.y));
    max_b.y = (int)(floor (max_p.y / leaf_size.y));

    min_b.z = (int)(floor (min_p.z / leaf_size.z));
    max_b.z = (int)(floor (max_p.z / leaf_size.z));

    // Compute the number of divisions needed along all axis
    div_b.x = (int)(max_b.x - min_b.x + 1);
    div_b.y = (int)(max_b.y - min_b.y + 1);
    div_b.z = (int)(max_b.z - min_b.z + 1);

    // Allocate the space needed
    try
    {
      if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
        leaves.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
      leaves.resize (div_b.x * div_b.y * div_b.z);
    }
    catch (std::bad_alloc)
    {
      ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                 div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
    }

    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
        leaves[cl].nr_points = 0;
      }
    }

    // First pass: go over all points and insert them into the right leaf
    for (unsigned int cp = 0; cp < points.points.size (); cp++)
    {
      if (d_idx != -1 && points.channels[d_idx].values[cp] > cut_distance)        // Use a threshold for cutting out points which are too far away
        continue;

      int i = (int)(floor (points.points[cp].x / leaf_size.x));
      int j = (int)(floor (points.points[cp].y / leaf_size.y));
      int k = (int)(floor (points.points[cp].z / leaf_size.z));

      int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
      leaves[idx].centroid_x += points.points[cp].x;
      leaves[idx].centroid_y += points.points[cp].y;
      leaves[idx].centroid_z += points.points[cp].z;
      leaves[idx].nr_points++;
    }

    // Second pass: go over all leaves and compute centroids
    int nr_p = 0;
    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        points_down.points[nr_p].x = leaves[cl].centroid_x / leaves[cl].nr_points;
        points_down.points[nr_p].y = leaves[cl].centroid_y / leaves[cl].nr_points;
        points_down.points[nr_p].z = leaves[cl].centroid_z / leaves[cl].nr_points;
        nr_p++;
      }
    }
    points_down.points.resize (nr_p);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \param points the point cloud message
    * \param indices a set of point indices
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    * \param leaves a vector of already existing leaves (empty for the first call)
    * \param d_idx the index of the channel providing distance data (set to -1 if nonexistant)
    * \param cut_distance the maximum admissible distance of a point from the viewpoint (default: FLT_MAX)
    */
  void
    downsamplePointCloud (sensor_msgs::PointCloudConstPtr points, sensor_msgs::PointCloud &points_down, geometry_msgs::Point leaf_size,
                          std::vector<Leaf> &leaves, int d_idx, double cut_distance)
  {
    if (d_idx == -1)
      cut_distance = DBL_MAX;
    // Copy the header (and thus the frame_id) + allocate enough space for points
    points_down.header = points->header;
    points_down.points.resize (points->points.size ());

    geometry_msgs::Point32 min_p, max_p, min_b, max_b, div_b;
    cloud_geometry::statistics::getMinMax (points, min_p, max_p, d_idx, cut_distance);

    // Compute the minimum and maximum bounding box values
    min_b.x = (int)(floor (min_p.x / leaf_size.x));
    max_b.x = (int)(floor (max_p.x / leaf_size.x));

    min_b.y = (int)(floor (min_p.y / leaf_size.y));
    max_b.y = (int)(floor (max_p.y / leaf_size.y));

    min_b.z = (int)(floor (min_p.z / leaf_size.z));
    max_b.z = (int)(floor (max_p.z / leaf_size.z));

    // Compute the number of divisions needed along all axis
    div_b.x = (int)(max_b.x - min_b.x + 1);
    div_b.y = (int)(max_b.y - min_b.y + 1);
    div_b.z = (int)(max_b.z - min_b.z + 1);

    // Allocate the space needed
    try
    {
      if (leaves.capacity () < div_b.x * div_b.y * div_b.z)
        leaves.reserve (div_b.x * div_b.y * div_b.z);             // fallback to x*y*z from 2*x*y*z due to memory problems
      leaves.resize (div_b.x * div_b.y * div_b.z);
    }
    catch (std::bad_alloc)
    {
      ROS_ERROR ("Failed while attempting to allocate a vector of %f (%g x %g x %g) leaf elements (%f bytes total)", div_b.x * div_b.y * div_b.z,
                 div_b.x, div_b.y, div_b.z, div_b.x * div_b.y * div_b.z * sizeof (Leaf));
    }

    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        leaves[cl].centroid_x = leaves[cl].centroid_y = leaves[cl].centroid_z = 0.0;
        leaves[cl].nr_points = 0;
      }
    }

    // First pass: go over all points and insert them into the right leaf
    for (unsigned int cp = 0; cp < points->points.size (); cp++)
    {
      if (d_idx != -1 && points->channels[d_idx].values[cp] > cut_distance)        // Use a threshold for cutting out points which are too far away
        continue;

      int i = (int)(floor (points->points[cp].x / leaf_size.x));
      int j = (int)(floor (points->points[cp].y / leaf_size.y));
      int k = (int)(floor (points->points[cp].z / leaf_size.z));

      int idx = ( (k - min_b.z) * div_b.y * div_b.x ) + ( (j - min_b.y) * div_b.x ) + (i - min_b.x);
      leaves[idx].centroid_x += points->points[cp].x;
      leaves[idx].centroid_y += points->points[cp].y;
      leaves[idx].centroid_z += points->points[cp].z;
      leaves[idx].nr_points++;
    }

    // Second pass: go over all leaves and compute centroids
    int nr_p = 0;
    for (unsigned int cl = 0; cl < leaves.size (); cl++)
    {
      if (leaves[cl].nr_points > 0)
      {
        points_down.points[nr_p].x = leaves[cl].centroid_x / leaves[cl].nr_points;
        points_down.points[nr_p].y = leaves[cl].centroid_y / leaves[cl].nr_points;
        points_down.points[nr_p].z = leaves[cl].centroid_z / leaves[cl].nr_points;
        nr_p++;
      }
    }
    points_down.points.resize (nr_p);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Downsample a Point Cloud using a voxelized grid approach
    * \note this method should not be used in fast loops as it always reallocs the std::vector<Leaf> internally (!)
    * \param points the point cloud message
    * \param points_down the resultant downsampled point cloud
    * \param leaf_size the voxel leaf dimensions
    */
  void
    downsamplePointCloud (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &points_down, geometry_msgs::Point leaf_size)
  {
    std::vector<Leaf> leaves;
    downsamplePointCloud (points, points_down, leaf_size, leaves, -1);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud object by copying the data from a given input point cloud using a set of indices
    * \param points the input point cloud message
    * \param indices a set of point indices
    * \param points the resultant/output point cloud message
    */
  void
    getPointCloud (const sensor_msgs::PointCloud &input, const std::vector<int> &indices, sensor_msgs::PointCloud &output)
  {
    output.header = input.header;
    output.points.resize (indices.size ());
    output.channels.resize (input.channels.size ());

    for (unsigned int d = 0; d < output.channels.size (); d++)
    {
      output.channels[d].name = input.channels[d].name;
      output.channels[d].values.resize (indices.size ());
    }

    // Copy the data
    for (unsigned int i = 0; i < indices.size (); i++)
    {
      output.points[i].x = input.points[indices.at (i)].x;
      output.points[i].y = input.points[indices.at (i)].y;
      output.points[i].z = input.points[indices.at (i)].z;
      for (unsigned int d = 0; d < output.channels.size (); d++)
        output.channels[d].values[i] = input.channels[d].values[indices.at (i)];
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create a new point cloud object by copying the data from a given input point cloud using a set of indices
    * \param points the input point cloud message
    * \param indices a set of point indices
    * \param points the resultant/output point cloud message
    */
  void
    getPointCloudOutside (const sensor_msgs::PointCloud &input, std::vector<int> indices, sensor_msgs::PointCloud &output)
  {
    std::vector<int> indices_outside;
    // Create the entire index list
    std::vector<int> all_indices (input.points.size ());
    for (unsigned int i = 0; i < all_indices.size (); i++)
      all_indices[i] = i;

    sort (indices.begin (), indices.end ());

    set_difference (all_indices.begin (), all_indices.end (), indices.begin (), indices.end (),
                    inserter (indices_outside, indices_outside.begin ()));

    if (indices_outside.size () == 0)
      return;

    output.header = input.header;
    output.channels.resize (input.channels.size ());

    output.points.resize (indices_outside.size ());

    for (unsigned int d = 0; d < output.channels.size (); d++)
    {
      output.channels[d].name = input.channels[d].name;
      output.channels[d].values.resize (indices_outside.size ());
    }

    // Copy the data
    for (unsigned int i = 0; i < indices_outside.size (); i++)
    {
      output.points[i].x = input.points[indices_outside.at (i)].x;
      output.points[i].y = input.points[indices_outside.at (i)].y;
      output.points[i].z = input.points[indices_outside.at (i)].z;
      for (unsigned int d = 0; d < output.channels.size (); d++)
        output.channels[d].values[i] = input.channels[d].values[indices_outside.at (i)];
    }
  }
}
