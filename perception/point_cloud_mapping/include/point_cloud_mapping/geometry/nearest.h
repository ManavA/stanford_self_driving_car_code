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
 * $Id: nearest.h 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_NEAREST_H_
#define _CLOUD_GEOMETRY_NEAREST_H_

// ROS includes
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include <Eigen/QR>

namespace cloud_geometry
{

  namespace nearest
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points and return it as a Point32 message.
      * \param points the input point cloud
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const sensor_msgs::PointCloud &points, geometry_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        centroid.x += points.points.at (i).x;
        centroid.y += points.points.at (i).y;
        centroid.z += points.points.at (i).z;
      }

      centroid.x /= points.points.size ();
      centroid.y /= points.points.size ();
      centroid.z /= points.points.size ();
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a 3D polygon and return it as a Point32 message.
      * \param poly the input polygon
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const geometry_msgs::Polygon &poly, geometry_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < poly.points.size (); i++)
      {
        centroid.x += poly.points.at (i).x;
        centroid.y += poly.points.at (i).y;
        centroid.z += poly.points.at (i).z;
      }

      centroid.x /= poly.points.size ();
      centroid.y /= poly.points.size ();
      centroid.z /= poly.points.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, geometry_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        centroid.x += points.points.at (indices.at (i)).x;
        centroid.y += points.points.at (indices.at (i)).y;
        centroid.z += points.points.at (indices.at (i)).z;
      }

      centroid.x /= indices.size ();
      centroid.y /= indices.size ();
      centroid.z /= indices.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const sensor_msgs::PointCloudConstPtr &points, const std::vector<int> &indices, geometry_msgs::Point32 &centroid)
    {
      centroid.x = centroid.y = centroid.z = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        centroid.x += points->points.at (indices.at (i)).x;
        centroid.y += points->points.at (indices.at (i)).y;
        centroid.z += points->points.at (indices.at (i)).z;
      }

      centroid.x /= indices.size ();
      centroid.y /= indices.size ();
      centroid.z /= indices.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    inline void
      computeCentroid (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, std::vector<double> &centroid)
    {
      centroid.resize (3);
      centroid[0] = centroid[1] = centroid[2] = 0;
      // For each point in the cloud
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        centroid[0] += points.points.at (indices.at (i)).x;
        centroid[1] += points.points.at (indices.at (i)).y;
        centroid[2] += points.points.at (indices.at (i)).z;
      }

      centroid[0] /= indices.size ();
      centroid[1] /= indices.size ();
      centroid[2] /= indices.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    inline void
      computeCovarianceMatrix (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &covariance_matrix, geometry_msgs::Point32 &centroid)
    {
      computeCentroid (points, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      // Sum of outer products
      // covariance_matrix (k, i)  += points_c (j, k) * points_c (j, i);
      for (unsigned int j = 0; j < points.points.size (); j++)
      {
        covariance_matrix (0, 0) += (points.points[j].x - centroid.x) * (points.points[j].x - centroid.x);
        covariance_matrix (0, 1) += (points.points[j].x - centroid.x) * (points.points[j].y - centroid.y);
        covariance_matrix (0, 2) += (points.points[j].x - centroid.x) * (points.points[j].z - centroid.z);

        covariance_matrix (1, 0) += (points.points[j].y - centroid.y) * (points.points[j].x - centroid.x);
        covariance_matrix (1, 1) += (points.points[j].y - centroid.y) * (points.points[j].y - centroid.y);
        covariance_matrix (1, 2) += (points.points[j].y - centroid.y) * (points.points[j].z - centroid.z);

        covariance_matrix (2, 0) += (points.points[j].z - centroid.z) * (points.points[j].x - centroid.x);
        covariance_matrix (2, 1) += (points.points[j].z - centroid.z) * (points.points[j].y - centroid.y);
        covariance_matrix (2, 2) += (points.points[j].z - centroid.z) * (points.points[j].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points. The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param covariance_matrix the 3x3 covariance matrix
      */
    inline void
      computeCovarianceMatrix (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &covariance_matrix)
    {
      geometry_msgs::Point32 centroid;
      computeCovarianceMatrix (points, covariance_matrix, centroid);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    inline void
      computeCovarianceMatrix (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix, geometry_msgs::Point32 &centroid)
    {
      computeCentroid (points, indices, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      for (unsigned int j = 0; j < indices.size (); j++)
      {
        covariance_matrix (0, 0) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].x - centroid.x);
        covariance_matrix (0, 1) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].y - centroid.y);
        covariance_matrix (0, 2) += (points.points[indices.at (j)].x - centroid.x) * (points.points[indices.at (j)].z - centroid.z);

        covariance_matrix (1, 0) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].x - centroid.x);
        covariance_matrix (1, 1) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].y - centroid.y);
        covariance_matrix (1, 2) += (points.points[indices.at (j)].y - centroid.y) * (points.points[indices.at (j)].z - centroid.z);

        covariance_matrix (2, 0) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].x - centroid.x);
        covariance_matrix (2, 1) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].y - centroid.y);
        covariance_matrix (2, 2) += (points.points[indices.at (j)].z - centroid.z) * (points.points[indices.at (j)].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \note The (x-y-z) centroid is also returned as a Point32 message.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      * \param centroid the computed centroid
      */
    inline void
      computeCovarianceMatrix (const sensor_msgs::PointCloudConstPtr &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix, geometry_msgs::Point32 &centroid)
    {
      computeCentroid (points, indices, centroid);

      // Initialize to 0
      covariance_matrix = Eigen::Matrix3d::Zero ();

      for (unsigned int j = 0; j < indices.size (); j++)
      {
        covariance_matrix (0, 0) += (points->points[indices.at (j)].x - centroid.x) * (points->points[indices.at (j)].x - centroid.x);
        covariance_matrix (0, 1) += (points->points[indices.at (j)].x - centroid.x) * (points->points[indices.at (j)].y - centroid.y);
        covariance_matrix (0, 2) += (points->points[indices.at (j)].x - centroid.x) * (points->points[indices.at (j)].z - centroid.z);

        covariance_matrix (1, 0) += (points->points[indices.at (j)].y - centroid.y) * (points->points[indices.at (j)].x - centroid.x);
        covariance_matrix (1, 1) += (points->points[indices.at (j)].y - centroid.y) * (points->points[indices.at (j)].y - centroid.y);
        covariance_matrix (1, 2) += (points->points[indices.at (j)].y - centroid.y) * (points->points[indices.at (j)].z - centroid.z);

        covariance_matrix (2, 0) += (points->points[indices.at (j)].z - centroid.z) * (points->points[indices.at (j)].x - centroid.x);
        covariance_matrix (2, 1) += (points->points[indices.at (j)].z - centroid.z) * (points->points[indices.at (j)].y - centroid.y);
        covariance_matrix (2, 2) += (points->points[indices.at (j)].z - centroid.z) * (points->points[indices.at (j)].z - centroid.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3x3 covariance matrix of a given set of points using their indices.
      * The result is returned as a Eigen::Matrix3d.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param covariance_matrix the 3x3 covariance matrix
      */
    inline void
      computeCovarianceMatrix (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &covariance_matrix)
    {
      geometry_msgs::Point32 centroid;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Computes the angle between the lines created from 2 points and the viewpoint: [vp->p1], [p1->p2]
      * Returns the angle (in degrees).
      * \param px X coordinate for the first point
      * \param py Y coordinate for the first point
      * \param pz Z coordinate for the first point
      * \param qx X coordinate for the second point
      * \param qy Y coordinate for the second point
      * \param qz Z coordinate for the second point
      * \param vp_x X coordinate of the viewpoint (0 by default)
      * \param vp_y Y coordinate of the viewpoint (0 by default)
      * \param vp_z Z coordinate of the viewpoint (0 by default)
      */
    inline double
      getAngleWithViewpoint (float px, float py, float pz, float qx, float qy, float qz,
                             float vp_x = 0, float vp_y = 0, float vp_z = 0)
    {  
      double dir_a[3], dir_b[3];
      dir_a[0] = vp_x - px; dir_a[1] = vp_y - py; dir_a[2] = vp_z - pz;
      dir_b[0] = qx   - px; dir_b[1] = qy   - py; dir_b[2] = qz   - pz;

      // sqrt (sqr (x) + sqr (y) + sqr (z))
      double norm_a = sqrt (dir_a[0]*dir_a[0] + dir_a[1]*dir_a[1] + dir_a[2]*dir_a[2]);
      // Check for bogus 0,0,0 points
      if (norm_a == 0) return (0);
      double norm_b = sqrt (dir_b[0]*dir_b[0] + dir_b[1]*dir_b[1] + dir_b[2]*dir_b[2]);
      if (norm_b == 0) return (0);
      // dot_product (x, y)
      double dot_pr = dir_a[0]*dir_b[0] + dir_a[1]*dir_b[1] + dir_a[2]*dir_b[2];
      if (dot_pr != dot_pr)     // Check for NaNs  
        return (0);

      return ( acos (dot_pr / (norm_a * norm_b) ) * 180.0 / M_PI);
    }

    void computeCentroid (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &centroid);
    void computeCentroid (const sensor_msgs::PointCloud &points, std::vector<int> &indices, sensor_msgs::PointCloud &centroid);

    void computePatchEigen (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values);
    void computePatchEigen (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values);
    void computePatchEigenNormalized (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values, geometry_msgs::Point32& centroid);
    void computePatchEigenNormalized (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values, geometry_msgs::Point32& centroid);

    void computePointNormal (const sensor_msgs::PointCloud &points, Eigen::Vector4d &plane_parameters, double &curvature);
    void computePointNormal (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Vector4d &plane_parameters, double &curvature);
    void computePointNormal (const sensor_msgs::PointCloudConstPtr &points, const std::vector<int> &indices, Eigen::Vector4d &plane_parameters, double &curvature);

    void computeMomentInvariants (const sensor_msgs::PointCloud &points, double &j1, double &j2, double &j3);
    void computeMomentInvariants (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double &j1, double &j2, double &j3);

    bool isBoundaryPoint (const sensor_msgs::PointCloud &points, int q_idx, const std::vector<int> &neighbors, const Eigen::Vector3d& u, const Eigen::Vector3d& v, double angle_threshold = M_PI / 2.0);

    void computePointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, int k, const geometry_msgs::PointStamped &viewpoint);
    void computePointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, double radius, const geometry_msgs::PointStamped &viewpoint);
    void computePointCloudNormals (sensor_msgs::PointCloud &points, int k, const geometry_msgs::PointStamped &viewpoint);
    void computePointCloudNormals (sensor_msgs::PointCloud &points, double radius, const geometry_msgs::PointStamped &viewpoint);

    void computeOrganizedPointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, int k, int downsample_factor, int width, int height, double max_z, 
                                            const geometry_msgs::Point32 &viewpoint);
    void computeOrganizedPointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloudConstPtr &surface, int k, int downsample_factor, int width, int height, double max_z, 
                                            const geometry_msgs::Point32 &viewpoint);
    void computeOrganizedPointCloudNormalsWithFiltering (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, int k, int downsample_factor, int width, int height, 
                                                         double max_z, double min_angle, double max_angle, const geometry_msgs::Point32 &viewpoint);

    void extractEuclideanClusters (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double tolerance, std::vector<std::vector<int> > &clusters,
                                   int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster = 1);

    void extractEuclideanClusters (const sensor_msgs::PointCloud &points, double tolerance, std::vector<std::vector<int> > &clusters,
                                   int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster = 1);
                                   
    void filterJumpEdges (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &points_filtered, int k, int width, int height, 
                          double min_angle, double max_angle, const geometry_msgs::Point32 &viewpoint);
                                                                    
  }
}

#endif
