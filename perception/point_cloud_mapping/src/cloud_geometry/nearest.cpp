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
 * $Id: nearest.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#include <Eigen/Eigenvalues>

#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

namespace cloud_geometry
{

  namespace nearest
  {

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points and return it as a PointCloud message with 1 value.
      * \param points the input point cloud
      * \param centroid the output centroid
      */
    void
      computeCentroid (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &centroid)
    {
      // Prepare the data output
      centroid.points.resize (1);
      centroid.points[0].x = centroid.points[0].y = centroid.points[0].z = 0;
      centroid.channels.resize (points.get_channels_size ());
      for (unsigned int d = 0; d < points.get_channels_size (); d++)
      {
        centroid.channels[d].name = points.channels[d].name;
        centroid.channels[d].values.resize (1);
      }

      // For each point in the cloud
      for (unsigned int i = 0; i < points.get_points_size (); i++)
      {
        centroid.points[0].x += points.points[i].x;
        centroid.points[0].y += points.points[i].y;
        centroid.points[0].z += points.points[i].z;

        for (unsigned int d = 0; d < points.get_channels_size (); d++)
          centroid.channels[d].values[0] += points.channels[d].values[i];
      }

      centroid.points[0].x /= points.get_points_size ();
      centroid.points[0].y /= points.get_points_size ();
      centroid.points[0].z /= points.get_points_size ();
      for (unsigned int d = 0; d < points.get_channels_size (); d++)
        centroid.channels[d].values[0] /= points.get_points_size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the centroid of a set of points using their indices and return it as a PointCloud message with 1 value.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param centroid the output centroid
      */
    void
      computeCentroid (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, sensor_msgs::PointCloud &centroid)
    {
      // Prepare the data output
      centroid.points.resize (1);
      centroid.points[0].x = centroid.points[0].y = centroid.points[0].z = 0;
      centroid.channels.resize (points.get_channels_size ());
      for (unsigned int d = 0; d < points.get_channels_size (); d++)
      {
        centroid.channels[d].name = points.channels[d].name;
        centroid.channels[d].values.resize (1);
      }

      // For each point in the cloud
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        centroid.points[0].x += points.points.at (indices.at (i)).x;
        centroid.points[0].y += points.points.at (indices.at (i)).y;
        centroid.points[0].z += points.points.at (indices.at (i)).z;

        for (unsigned int d = 0; d < points.get_channels_size (); d++)
          centroid.channels[d].values[0] += points.channels[d].values.at (indices.at (i));
      }

      centroid.points[0].x /= indices.size ();
      centroid.points[0].y /= indices.size ();
      centroid.points[0].z /= indices.size ();
      for (unsigned int d = 0; d < points.get_channels_size (); d++)
        centroid.channels[d].values[0] /= indices.size ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the eigenvalues and eigenvectors of a given surface patch
      * \param points the input point cloud
      * \param eigen_vectors the resultant eigenvectors
      * \param eigen_values the resultant eigenvalues
      */
    void
      computePatchEigen (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values)
    {
      geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      eigen_values = ei_symm.eigenvalues ();
      eigen_vectors = ei_symm.eigenvectors ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the eigenvalues and eigenvectors of a given surface patch
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param eigen_vectors the resultant eigenvectors
      * \param eigen_values the resultant eigenvalues
      */
    void
      computePatchEigen (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values)
    {
      geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      eigen_values = ei_symm.eigenvalues ();
      eigen_vectors = ei_symm.eigenvectors ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the eigenvalues and eigenvectors of a given surface patch from its normalized covariance matrix
      * \param points the input point cloud
      * \param eigen_vectors the resultant eigenvectors
      * \param eigen_values the resultant eigenvalues
      * \param centroid the centroid of the points
      */
    void
      computePatchEigenNormalized (const sensor_msgs::PointCloud &points, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values, geometry_msgs::Point32& centroid)
    {
      //geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, covariance_matrix, centroid);

      // Normalize the matrix by 1/N
      for (unsigned int i = 0 ; i < 3 ; i++)
      {
        for (unsigned int j = 0 ; j < 3 ; j++)
        {
          covariance_matrix(i, j) /= static_cast<double> (points.points.size ());
        }
      }

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      eigen_values = ei_symm.eigenvalues ();
      eigen_vectors = ei_symm.eigenvectors ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the eigenvalues and eigenvectors of a given surface patch from its normalized covariance matrix
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param eigen_vectors the resultant eigenvectors
      * \param eigen_values the resultant eigenvalues
      * \param centroid the centroid of the points
      */
    void
      computePatchEigenNormalized (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Matrix3d &eigen_vectors, Eigen::Vector3d &eigen_values, geometry_msgs::Point32& centroid)
    {
      //geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Normalize the matrix by 1/N
      for (unsigned int i = 0 ; i < 3 ; i++)
      {
        for (unsigned int j = 0 ; j < 3 ; j++)
        {
          covariance_matrix(i, j) /= static_cast<double> (indices.size ());
        }
      }


      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      eigen_values = ei_symm.eigenvalues ();
      eigen_vectors = ei_symm.eigenvectors ();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, and return the estimated plane parameters
      * together with the surface curvature.
      * \param points the input point cloud
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computePointNormal (const sensor_msgs::PointCloud &points, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
      Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (0, 1) * eigen_vectors (0, 1) +
                           eigen_vectors (0, 2) * eigen_vectors (0, 2));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (0, 1) / norm;
      plane_parameters (2) = eigen_vectors (0, 2) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters (1) * centroid.y + plane_parameters (2) * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computePointNormal (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
      Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      // Note: Remember to take care of the eigen_vectors ordering
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (1, 0) * eigen_vectors (1, 0) +
                           eigen_vectors (2, 0) * eigen_vectors (2, 0));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (1, 0) / norm;
      plane_parameters (2) = eigen_vectors (2, 0) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters (1) * centroid.y + plane_parameters (2) * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the Least-Squares plane fit for a given set of points, using their indices,
      * and return the estimated plane parameters together with the surface curvature.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param plane_parameters the plane parameters as: a, b, c, d (ax + by + cz + d = 0)
      * \param curvature the estimated surface curvature as a measure of
      * \f[
      * \lambda_0 / (\lambda_0 + \lambda_1 + \lambda_2)
      * \f]
      */
    void
      computePointNormal (const sensor_msgs::PointCloudConstPtr &points, const std::vector<int> &indices, Eigen::Vector4d &plane_parameters, double &curvature)
    {
      geometry_msgs::Point32 centroid;
      // Compute the 3x3 covariance matrix
      Eigen::Matrix3d covariance_matrix;
      computeCovarianceMatrix (points, indices, covariance_matrix, centroid);

      // Extract the eigenvalues and eigenvectors
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> ei_symm (covariance_matrix);
      Eigen::Vector3d eigen_values  = ei_symm.eigenvalues ();
      Eigen::Matrix3d eigen_vectors = ei_symm.eigenvectors ();

      // Normalize the surface normal (eigenvector corresponding to the smallest eigenvalue)
      // Note: Remember to take care of the eigen_vectors ordering
      double norm = sqrt ( eigen_vectors (0, 0) * eigen_vectors (0, 0) +
                           eigen_vectors (1, 0) * eigen_vectors (1, 0) +
                           eigen_vectors (2, 0) * eigen_vectors (2, 0));
      plane_parameters (0) = eigen_vectors (0, 0) / norm;
      plane_parameters (1) = eigen_vectors (1, 0) / norm;
      plane_parameters (2) = eigen_vectors (2, 0) / norm;

      // Hessian form (D = nc . p_plane (centroid here) + p)
      plane_parameters (3) = -1 * (plane_parameters (0) * centroid.x + plane_parameters (1) * centroid.y + plane_parameters (2) * centroid.z);

      // Compute the curvature surface change
      curvature = fabs ( eigen_values (0) / (eigen_values (0) + eigen_values (1) + eigen_values (2)) );
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3 moment invariants (j1, j2, j3) for a given set of points.
      * \param points the input point cloud
      * \param j1 the first moment invariant
      * \param j2 the second moment invariant
      * \param j3 the third moment invariant
      */
    void
      computeMomentInvariants (const sensor_msgs::PointCloud &points, double &j1, double &j2, double &j3)
    {
      // Compute the centroid
      geometry_msgs::Point32 centroid;
      computeCentroid (points, centroid);

      // Demean the pointset
      sensor_msgs::PointCloud points_c;
      points_c.points.resize (points.points.size ());
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        points_c.points[i].x = points.points[i].x - centroid.x;
        points_c.points[i].y = points.points[i].y - centroid.y;
        points_c.points[i].z = points.points[i].z - centroid.z;
      }

      double mu200 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 2.0, 0.0, 0.0);
      double mu020 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 2.0, 0.0);
      double mu002 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 0.0, 2.0);
      double mu110 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 1.0, 1.0, 0.0);
      double mu101 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 1.0, 0.0, 1.0);
      double mu011 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 1.0, 1.0);

      j1 = mu200 + mu020 + mu002;
      j2 = mu200*mu020 + mu200*mu002 + mu020*mu002 - mu110*mu110 - mu101*mu101 - mu011*mu011;
      j3 = mu200*mu020*mu002 + 2*mu110*mu101*mu011 - mu002*mu110*mu110 - mu020*mu101*mu101 - mu200*mu011*mu011;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Compute the 3 moment invariants (j1, j2, j3) for a given set of points, using their indices.
      * \param points the input point cloud
      * \param indices the point cloud indices that need to be used
      * \param j1 the first moment invariant
      * \param j2 the second moment invariant
      * \param j3 the third moment invariant
      */
    void
      computeMomentInvariants (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double &j1, double &j2, double &j3)
    {
      // Compute the centroid
      geometry_msgs::Point32 centroid;
      computeCentroid (points, indices, centroid);

      // Demean the pointset
      sensor_msgs::PointCloud points_c;
      points_c.points.resize (indices.size ());
      for (unsigned int i = 0; i < indices.size (); i++)
      {
        points_c.points[i].x = points.points.at (indices.at (i)).x - centroid.x;
        points_c.points[i].y = points.points.at (indices.at (i)).y - centroid.y;
        points_c.points[i].z = points.points.at (indices.at (i)).z - centroid.z;
      }

      double mu200 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 2.0, 0.0, 0.0);
      double mu020 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 2.0, 0.0);
      double mu002 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 0.0, 2.0);
      double mu110 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 1.0, 1.0, 0.0);
      double mu101 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 1.0, 0.0, 1.0);
      double mu011 = cloud_geometry::statistics::computeCentralizedMoment (points_c, 0.0, 1.0, 1.0);

      j1 = mu200 + mu020 + mu002;
      j2 = mu200*mu020 + mu200*mu002 + mu020*mu002 - mu110*mu110 - mu101*mu101 - mu011*mu011;
      j3 = mu200*mu020*mu002 + 2*mu110*mu101*mu011 - mu002*mu110*mu110 - mu020*mu101*mu101 - mu200*mu011*mu011;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Check whether a point is a boundary point in a planar patch of projected points given by indices.
      * \note A coordinate system u-v-n must be computed a-priori using \a getCoordinateSystemOnPlane
      * \param points a pointer to the input point cloud
      * \param q_idx the index of the query point in \a points
      * \param neighbors the estimated point neighbors of the query point
      * \param u the u direction
      * \param v the v direction
      * \param angle_threshold the threshold angle (default $\pi / 2.0$)
      */
    bool
      isBoundaryPoint (const sensor_msgs::PointCloud &points, int q_idx, const std::vector<int> &neighbors,
                       const Eigen::Vector3d& u, const Eigen::Vector3d& v, double angle_threshold)
    {
      if (neighbors.size () < 3)
        return (false);
      double uvn_nn[2];
      // Compute the angles between each neighbouring point and the query point itself
      std::vector<double> angles;
      angles.reserve (neighbors.size ());
      for (unsigned int i = 0; i < neighbors.size (); i++)
      {
        uvn_nn[0] = (points.points.at (neighbors.at (i)).x - points.points.at (q_idx).x) * u (0) +
                    (points.points.at (neighbors.at (i)).y - points.points.at (q_idx).y) * u (1) +
                    (points.points.at (neighbors.at (i)).z - points.points.at (q_idx).z) * u (2);
        uvn_nn[1] = (points.points.at (neighbors.at (i)).x - points.points.at (q_idx).x) * v (0) +
                    (points.points.at (neighbors.at (i)).y - points.points.at (q_idx).y) * v (1) +
                    (points.points.at (neighbors.at (i)).z - points.points.at (q_idx).z) * v (2);
        if (uvn_nn[0] == 0 && uvn_nn[1] == 0)
          continue;
        angles.push_back (cloud_geometry::angles::getAngle2D (uvn_nn));
      }
      sort (angles.begin (), angles.end ());

      // Compute the maximal angle difference between two consecutive angles
      double max_dif = DBL_MIN, dif;
      for (unsigned int i = 0; i < angles.size () - 1; i++)
      {
        dif = angles[i + 1] - angles[i];
        if (max_dif < dif)
          max_dif = dif;
      }
      // Get the angle difference between the last and the first
      dif = 2 * M_PI - angles[angles.size () - 1] + angles[0];
      if (max_dif < dif)
        max_dif = dif;

      // Check results
      if (max_dif > angle_threshold)
        return (true);
      else
        return (false);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate (in place) the point normals and surface curvatures for a given point cloud dataset (points)
      * using the data from a different point cloud (surface) for least-squares planar estimation.
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param surface the point cloud data to use for least-squares planar estimation
      * \param k use a fixed number of k-nearest neighbors for the least-squares fit
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computePointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, int k,
                                const geometry_msgs::PointStamped &viewpoint)
    {
      int nr_points = points.points.size ();
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (surface);

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < nr_points; i++)                     // Get the nearest neighbors for all the point indices in the bounds
      {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        kdtree->nearestKSearch (points.points[i], k, nn_indices, nn_distances);

        Eigen::Vector4d plane_parameters;                     // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        double curvature;
        computePointNormal (surface, nn_indices, plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points.points[i], viewpoint);

        points.channels[orig_dims + 0].values[i] = plane_parameters (0);
        points.channels[orig_dims + 1].values[i] = plane_parameters (1);
        points.channels[orig_dims + 2].values[i] = plane_parameters (2);
        points.channels[orig_dims + 3].values[i] = curvature;
      }
      delete kdtree;                                          // Delete the kd-tree
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate (in place) the point normals and surface curvatures for a given point cloud dataset (points)
      * using the data from a different point cloud (surface) for least-squares planar estimation.
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param surface the point cloud data to use for least-squares planar estimation
      * \param radius use a neighbor fixed radius search for the least-squares fit
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computePointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface, double radius,
                                const geometry_msgs::PointStamped &viewpoint)
    {
      int nr_points = points.points.size ();
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (surface);

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < nr_points; i++)                     // Get the nearest neighbors for all the point indices in the bounds
      {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        kdtree->radiusSearch (points.points[i], radius, nn_indices, nn_distances);

        Eigen::Vector4d plane_parameters;                     // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        double curvature;
        computePointNormal (surface, nn_indices, plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points.points[i], viewpoint);

        points.channels[orig_dims + 0].values[i] = plane_parameters (0);
        points.channels[orig_dims + 1].values[i] = plane_parameters (1);
        points.channels[orig_dims + 2].values[i] = plane_parameters (2);
        points.channels[orig_dims + 3].values[i] = curvature;
      }
      delete kdtree;                                          // Delete the kd-tree
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate (in place) the point normals and surface curvatures for a given point cloud dataset (points)
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param k use a fixed number of k-nearest neighbors for the least-squares fit
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computePointCloudNormals (sensor_msgs::PointCloud &points, int k, const geometry_msgs::PointStamped &viewpoint)
    {
      int nr_points = points.points.size ();
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      // Peter: timing the different pieces of this
      ros::Time ts = ros::Time::now();
      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points);
      ROS_INFO("KdTree created in %f seconds", (ros::Time::now () - ts).toSec ());

      double total_search_time = 0.0, total_normal_time = 0.0;

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < nr_points; i++)                     // Get the nearest neighbors for all the point indices in the bounds
      {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;

	ros::Time ts_search = ros::Time::now();
        kdtree->nearestKSearch (points.points[i], k, nn_indices, nn_distances);
	total_search_time += (ros::Time::now () - ts_search).toSec ();

        Eigen::Vector4d plane_parameters;                     // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        double curvature;

	ros::Time ts_normal = ros::Time::now();
        computePointNormal (points, nn_indices, plane_parameters, curvature);
	total_normal_time += (ros::Time::now () - ts_normal).toSec ();

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points.points[i], viewpoint);

        points.channels[orig_dims + 0].values[i] = plane_parameters (0);
        points.channels[orig_dims + 1].values[i] = plane_parameters (1);
        points.channels[orig_dims + 2].values[i] = plane_parameters (2);
        points.channels[orig_dims + 3].values[i] = curvature;
      }
      ROS_INFO("Breakdown: %f seconds to search, %f seconds to compute normals", total_search_time, total_normal_time);

      delete kdtree;                                          // Delete the kd-tree
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate (in place) the point normals and surface curvatures for a given point cloud dataset (points)
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param radius use a neighbor fixed radius search for the least-squares fit
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computePointCloudNormals (sensor_msgs::PointCloud &points, double radius, const geometry_msgs::PointStamped &viewpoint)
    {
      int nr_points = points.points.size ();
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (points);

#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < nr_points; i++)                     // Get the nearest neighbors for all the point indices in the bounds
      {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        kdtree->radiusSearch (points.points[i], radius, nn_indices, nn_distances);

        Eigen::Vector4d plane_parameters;                     // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        double curvature;
        computePointNormal (points, nn_indices, plane_parameters, curvature);

        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, points.points[i], viewpoint);

        points.channels[orig_dims + 0].values[i] = plane_parameters (0);
        points.channels[orig_dims + 1].values[i] = plane_parameters (1);
        points.channels[orig_dims + 2].values[i] = plane_parameters (2);
        points.channels[orig_dims + 3].values[i] = curvature;
      }
      delete kdtree;                                          // Delete the kd-tree
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate the point normals and surface curvatures for a given organized point cloud dataset (points)
      * using the data from a different point cloud (surface) for least-squares planar estimation.
      *
      * \note The main difference between this method and its \a computePointCloudNormals () sibblings is that 
      * assumptions are being made regarding the organization of points in the cloud. For example, point clouds
      * acquired using camera sensors are assumed to be row-major, and the width and height of the depth image
      * are assumed to be known, etc.
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param surface the point cloud data to use for least-squares planar estimation
      * \param k the windowing factor (i.e., how many pixels in the depth image in all directions should the neighborhood of a point contain)
      * \param downsample_factor factor for downsampling the input data, i.e., take every Nth row and column in the depth image
      * \param width the width in pixels of the depth image
      * \param height the height in pixels of the depth image
      * \param max_z maximum distance threshold (on Z) between the query point and its neighbors (set to -1 to ignore the check)
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computeOrganizedPointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface,
                                         int k, int downsample_factor, int width, int height, double max_z, 
                                         const geometry_msgs::Point32 &viewpoint)
    {
      // Reduce by a factor of N
      int nr_points = lrint (ceil (width / (double)downsample_factor)) *
                      lrint (ceil (height / (double)downsample_factor));
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      points.points.resize (nr_points);
      // Reserve space for 4 channels: nx, ny, nz, curvature
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      // Reserve enough space for the neighborhood
      std::vector<int> nn_indices ((k + k + 1) * (k + k + 1));
      Eigen::Vector4d plane_parameters;
      double curvature;

      int j = 0;
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)surface.points.size (); i++)
      {
        // Obtain the <u,v> pixel values
        int u = i / width;
        int v = i % width;

        // Get every Nth pixel in both rows, cols
        if ((u % downsample_factor != 0) || (v % downsample_factor != 0))
          continue;

        // Copy the data
        points.points[j] = surface.points[i];

        // Get all point neighbors in a k x k window
        int l = 0;
        for (int x = -k; x < k+1; x++)
        {
          for (int y = -k; y < k+1; y++)
          {
            int idx = (u+x) * width + (v+y);
            if (idx == i)
              continue;
            // If the index is not in the point cloud, continue
            if (idx < 0 || idx >= (int)surface.points.size ())
              continue;
            // If the difference in Z (depth) between the query point and the current neighbor is smaller than max_z
            if (max_z != -1)
            {
              if ( fabs (points.points[j].z - surface.points[idx].z) <  max_z )
                nn_indices[l++] = idx;
            }
            else
              nn_indices[l++] = idx;
          }
        }
        nn_indices.resize (l);
        if (nn_indices.size () < 4)
        {
          //ROS_ERROR ("Not enough neighboring indices found for point %d (%f, %f, %f).", i, surface.points[i].x, surface.points[i].y, surface.points[i].z);
          continue;
        }

        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        cloud_geometry::nearest::computePointNormal (surface, nn_indices, plane_parameters, curvature);
        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, surface.points[i], viewpoint);

        points.channels[orig_dims + 0].values[j] = plane_parameters (0);
        points.channels[orig_dims + 1].values[j] = plane_parameters (1);
        points.channels[orig_dims + 2].values[j] = plane_parameters (2);
        points.channels[orig_dims + 3].values[j] = curvature;
        j++;
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate the point normals and surface curvatures for a given organized point cloud dataset (points)
      * using the data from a different point cloud (surface) for least-squares planar estimation.
      *
      * \note The main difference between this method and its \a computePointCloudNormals () sibblings is that 
      * assumptions are being made regarding the organization of points in the cloud. For example, point clouds
      * acquired using camera sensors are assumed to be row-major, and the width and height of the depth image
      * are assumed to be known, etc.
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param surface the point cloud data to use for least-squares planar estimation
      * \param k the windowing factor (i.e., how many pixels in the depth image in all directions should the neighborhood of a point contain)
      * \param downsample_factor factor for downsampling the input data, i.e., take every Nth row and column in the depth image
      * \param width the width in pixels of the depth image
      * \param height the height in pixels of the depth image
      * \param max_z maximum distance threshold (on Z) between the query point and its neighbors (set to -1 to ignore the check)
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computeOrganizedPointCloudNormals (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloudConstPtr &surface,
                                         int k, int downsample_factor, int width, int height, double max_z, 
                                         const geometry_msgs::Point32 &viewpoint)
    {
      // Reduce by a factor of N
      int nr_points = lrint (ceil (width / (double)downsample_factor)) *
                      lrint (ceil (height / (double)downsample_factor));
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      points.points.resize (nr_points);
      // Reserve space for 4 channels: nx, ny, nz, curvature
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      // Reserve enough space for the neighborhood
      std::vector<int> nn_indices ((k + k + 1) * (k + k + 1));
      Eigen::Vector4d plane_parameters;
      double curvature;

      int j = 0;
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)surface->points.size (); i++)
      {
        // Obtain the <u,v> pixel values
        int u = i / width;
        int v = i % width;

        // Get every Nth pixel in both rows, cols
        if ((u % downsample_factor != 0) || (v % downsample_factor != 0))
          continue;

        // Copy the data
        points.points[j] = surface->points[i];

        // Get all point neighbors in a k x k window
        int l = 0;
        for (int x = -k; x < k+1; x++)
        {
          for (int y = -k; y < k+1; y++)
          {
            int idx = (u+x) * width + (v+y);
            if (idx == i)
              continue;
            // If the index is not in the point cloud, continue
            if (idx < 0 || idx >= (int)surface->points.size ())
              continue;
            // If the difference in Z (depth) between the query point and the current neighbor is smaller than max_z
            if (max_z != -1)
            {
              if ( fabs (points.points[j].z - surface->points[idx].z) <  max_z )
                nn_indices[l++] = idx;
            }
            else
              nn_indices[l++] = idx;
          }
        }
        nn_indices.resize (l);
        if (nn_indices.size () < 4)
        {
          //ROS_ERROR ("Not enough neighboring indices found for point %d (%f, %f, %f).", i, surface->points[i].x, surface->points[i].y, surface->points[i].z);
          continue;
        }

        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        cloud_geometry::nearest::computePointNormal (surface, nn_indices, plane_parameters, curvature);
        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, surface->points[i], viewpoint);

        points.channels[orig_dims + 0].values[j] = plane_parameters (0);
        points.channels[orig_dims + 1].values[j] = plane_parameters (1);
        points.channels[orig_dims + 2].values[j] = plane_parameters (2);
        points.channels[orig_dims + 3].values[j] = curvature;
        j++;
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Estimate the point normals and surface curvatures for a given organized point cloud dataset (points)
      * using the data from a different point cloud (surface) for least-squares planar estimation.
      *
      * \note The main difference between this method and its \a computePointCloudNormals () sibblings is that 
      * assumptions are being made regarding the organization of points in the cloud. For example, point clouds
      * acquired using camera sensors are assumed to be row-major, and the width and height of the depth image
      * are assumed to be known, etc.
      * \param points the input point cloud (on output, 4 extra channels will be added: \a nx, \a ny, \a nz, and \a curvatures)
      * \param surface the point cloud data to use for least-squares planar estimation
      * \param k the windowing factor (i.e., how many pixels in the depth image in all directions should the neighborhood of a point contain)
      * \param downsample_factor factor for downsampling the input data, i.e., take every Nth row and column in the depth image
      * \param width the width in pixels of the depth image
      * \param height the height in pixels of the depth image
      * \param max_z maximum distance threshold (on Z) between the query point and its neighbors (set to -1 to ignore the check)
      * \param min_angle the minimum angle allowed between the viewpoint ray and the line formed by two subsequent points(used for filtering jump edge data)
      * \param max_angle the maximum angle allowed between the viewpoint ray and the line formed by two subsequent points(used for filtering jump edge data)
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      computeOrganizedPointCloudNormalsWithFiltering (sensor_msgs::PointCloud &points, const sensor_msgs::PointCloud &surface,
                                                      int k, int downsample_factor, int width, int height, double max_z, 
                                                      double min_angle, double max_angle, 
                                                      const geometry_msgs::Point32 &viewpoint)
    {
      // Reduce by a factor of N
      int nr_points = lrint (ceil (width / (double)downsample_factor)) *
                      lrint (ceil (height / (double)downsample_factor));
      int orig_dims = points.channels.size ();
      points.channels.resize (orig_dims + 4);                     // Reserve space for 4 channels: nx, ny, nz, curvature
      points.channels[orig_dims + 0].name = "nx";
      points.channels[orig_dims + 1].name = "ny";
      points.channels[orig_dims + 2].name = "nz";
      points.channels[orig_dims + 3].name = "curvatures";
      points.points.resize (nr_points);
      // Reserve space for 4 channels: nx, ny, nz, curvature
      for (unsigned int d = orig_dims; d < points.channels.size (); d++)
        points.channels[d].values.resize (nr_points);

      // Reserve enough space for the neighborhood
      std::vector<int> nn_indices ((k + k + 1) * (k + k + 1));
      Eigen::Vector4d plane_parameters;
      double curvature;

      int j = 0;
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)surface.points.size (); i++)
      {
        // Obtain the <u,v> pixel values
        int u = i / width;
        int v = i % width;

        // Get every Nth pixel in both rows, cols
        if ((u % downsample_factor != 0) || (v % downsample_factor != 0))
          continue;

        // Copy the data
        points.points[j] = surface.points[i];

        // Get all point neighbors in a k x k window
        int l = 0;
        for (int x = -k; x < k+1; x++)
        {
          for (int y = -k; y < k+1; y++)
          {
            int idx = (u+x) * width + (v+y);
            if (idx == i)
              continue;

            // If the index is not in the point cloud, continue
            if (idx < 0 || idx >= (int)surface.points.size ())
              continue;

            // Get the angle between the lines formed with the viewpoint
            double angle = getAngleWithViewpoint (points.points[j].x, points.points[j].y, points.points[j].z,
                                                  surface.points[idx].x, surface.points[idx].y, surface.points[idx].z,
                                                  viewpoint.x, viewpoint.y, viewpoint.z);
            if (angle < min_angle || angle > max_angle)
              continue;

            // If the difference in Z (depth) between the query point and the current neighbor is smaller than max_z
            if (max_z != -1)
            {
              if ( fabs (points.points[j].z - surface.points[idx].z) <  max_z )
                nn_indices[l++] = idx;
            }
            else
              nn_indices[l++] = idx;
         }
        }
        nn_indices.resize (l);
        if (nn_indices.size () < 4)
        {
          //ROS_ERROR ("Not enough neighboring indices found for point %d (%f, %f, %f).", i, surface.points[i].x, surface.points[i].y, surface.points[i].z);
          continue;
        }

        // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
        cloud_geometry::nearest::computePointNormal (surface, nn_indices, plane_parameters, curvature);
        cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, surface.points[i], viewpoint);

        points.channels[orig_dims + 0].values[j] = plane_parameters (0);
        points.channels[orig_dims + 1].values[j] = plane_parameters (1);
        points.channels[orig_dims + 2].values[j] = plane_parameters (2);
        points.channels[orig_dims + 3].values[j] = curvature;
        j++;
      }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Filter jump edges in an organized point cloud dataset (e.g., acquired using TOF or dense stereo, etc)
      *
      * \NOTE the input point cloud size is assumed to have (width*height) number of points, in order!
      * \param points the input organized point cloud data
      * \param points_filtered the output filtered point cloud data
      * \param k the windowing factor (i.e., how many pixels in the depth image in all directions should the neighborhood of a point contain)
      * \param width the width in pixels of the depth image
      * \param height the height in pixels of the depth image
      * \param min_angle the minimum angle allowed between the viewpoint ray and the line formed by two subsequent points(used for filtering jump edge data)
      * \param max_angle the maximum angle allowed between the viewpoint ray and the line formed by two subsequent points(used for filtering jump edge data)
      * \param viewpoint the viewpoint where the cloud was acquired from (used for normal flip)
      */
    void
      filterJumpEdges (const sensor_msgs::PointCloud &points, sensor_msgs::PointCloud &points_filtered, int k, int width, int height, 
                       double min_angle, double max_angle, const geometry_msgs::Point32 &viewpoint)
    {
      // Copy the header, and reserve the correct number of channels and points, etc
      points_filtered.header = points.header;
      points_filtered.points.resize (points.points.size ());
      points_filtered.channels.resize (points.channels.size ());
      for (unsigned int d = 0; d < points.channels.size (); d++)
      {
        points_filtered.channels[d].name = points.channels[d].name;
        points_filtered.channels[d].values.resize (points.channels[d].values.size ());
      }

      int j = 0;
#pragma omp parallel for schedule(dynamic)
      for (int i = 0; i < (int)points.points.size (); i++)
      {
        // Obtain the <u,v> pixel values
        int u = i / width;
        int v = i % width;

        // Get all point neighbors in a k x k window
        bool valid_point = true;
        for (int x = -k; x < k+1; x++)
        {
          if (!valid_point) 
            break;
          for (int y = -k; y < k+1; y++)
          {
            int idx = (u+x) * width + (v+y);
            if (idx == i)
              continue;

            // If the index is not in the point cloud, continue
            if (idx < 0 || idx >= (int)points.points.size ())
              continue;

            // Get the angle between the lines formed with the viewpoint
            double angle = getAngleWithViewpoint (points.points[i].x, points.points[i].y, points.points[i].z,
                                                  points.points[idx].x, points.points[idx].y, points.points[idx].z,
                                                  viewpoint.x, viewpoint.y, viewpoint.z);
            if (angle < min_angle || angle > max_angle)
            {
              valid_point = false;
              break;
            }

          }
        }

        if (valid_point)
        {
          // Copy the data
          points_filtered.points[j] = points.points[i];
          for (unsigned int d = 0; d < points.channels.size (); d++)
            points_filtered.channels[d].values[j] = points.channels[d].values[j];
          j++;
        }
      }
      points_filtered.points.resize (j);
      for (unsigned int d = 0; d < points.channels.size (); d++)
        points_filtered.channels[d].values.resize (j);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
      * angular deviation
      * \NOTE: assumes normalized point normals !
      * \param points the point cloud message
      * \param indices a list of point indices to use
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters containing point indices
      * \param nx_idx the index of the channel containing the X component of the normal (use -1 to disable the check)
      * \param ny_idx the index of the channel containing the Y component of the normal (use -1 to disable the check)
      * \param nz_idx the index of the channel containing the Z component of the normal (use -1 to disable the check)
      * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
      *                  (only valid if nx_idx && ny_idx && nz_idx != -1)
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      extractEuclideanClusters (const sensor_msgs::PointCloud &points, const std::vector<int> &indices, double tolerance,
                                std::vector<std::vector<int> > &clusters,
                                int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points, indices);

      int nr_points = indices.size ();
      // Create a bool vector of processed point indices, and initialize it to false
      std::vector<bool> processed;
      processed.resize (nr_points, false);

      std::vector<int> nn_indices;
      std::vector<float> nn_distances;
      // Process all points in the indices vector
      for (int i = 0; i < nr_points; i++)
      {
        if (processed[i])
          continue;

        std::vector<int> seed_queue;
//        seed_queue.reserve (indices.size ());
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          // Search for sq_idx
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)       // nn_indices[0] should be sq_idx
          {
            if (processed.at (nn_indices[j]))                         // Has this point been processed before ?
              continue;

            processed[nn_indices[j]] = true;
            if (nx_idx != -1)                                         // Are point normals present ?
            {
              // [-1;1]
              double dot_p = points.channels[nx_idx].values[indices.at (i)] * points.channels[nx_idx].values[indices.at (nn_indices[j])] +
                             points.channels[ny_idx].values[indices.at (i)] * points.channels[ny_idx].values[indices.at (nn_indices[j])] +
                             points.channels[nz_idx].values[indices.at (i)] * points.channels[nz_idx].values[indices.at (nn_indices[j])];
              if ( fabs (acos (dot_p)) < eps_angle )
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
            // If normal information is not present, perform a simple Euclidean clustering
            else
            {
              processed[nn_indices[j]] = true;
              seed_queue.push_back (nn_indices[j]);
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          std::vector<int> r (seed_queue.size ());
          for (unsigned int j = 0; j < seed_queue.size (); j++)
            r[j] = indices.at (seed_queue[j]);

          sort (r.begin (), r.end ());
          r.erase (unique (r.begin (), r.end ()), r.end ());

          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Decompose a region of space into clusters based on the euclidean distance between points, and the normal
      * angular deviation
      * \NOTE: assumes normalized point normals !
      * \param points the point cloud message
      * \param tolerance the spatial tolerance as a measure in the L2 Euclidean space
      * \param clusters the resultant clusters containing point indices
      * \param nx_idx the index of the channel containing the X component of the normal (use -1 to disable the check)
      * \param ny_idx the index of the channel containing the Y component of the normal (use -1 to disable the check)
      * \param nz_idx the index of the channel containing the Z component of the normal (use -1 to disable the check)
      * \param eps_angle the maximum allowed difference between normals in degrees for cluster/region growing
      *                  (only valid if nx_idx && ny_idx && nz_idx != -1)
      * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
      */
    void
      extractEuclideanClusters (const sensor_msgs::PointCloud &points, double tolerance,
                                std::vector<std::vector<int> > &clusters,
                                int nx_idx, int ny_idx, int nz_idx, double eps_angle, unsigned int min_pts_per_cluster)
    {
      // Create a tree for these points
      cloud_kdtree::KdTree* tree = new cloud_kdtree::KdTreeANN (points);

      int nr_points = points.points.size ();
      // Create a bool vector of processed point indices, and initialize it to false
      std::vector<bool> processed;
      processed.resize (nr_points, false);

      std::vector<int> nn_indices;
      std::vector<float> nn_distances;
      // Process all points in the indices vector
      for (int i = 0; i < nr_points; i++)
      {
        if (processed[i])
          continue;

        std::vector<int> seed_queue;
        int sq_idx = 0;
        seed_queue.push_back (i);

        processed[i] = true;

        while (sq_idx < (int)seed_queue.size ())
        {
          // Search for sq_idx
          tree->radiusSearch (seed_queue.at (sq_idx), tolerance, nn_indices, nn_distances);

          for (unsigned int j = 1; j < nn_indices.size (); j++)       // nn_indices[0] should be sq_idx
          {
            if (processed.at (nn_indices[j]))                         // Has this point been processed before ?
              continue;

            processed[nn_indices[j]] = true;
            if (nx_idx != -1)                                         // Are point normals present ?
            {
              // [-1;1]
              double dot_p = points.channels[nx_idx].values[i] * points.channels[nx_idx].values[nn_indices[j]] +
                             points.channels[ny_idx].values[i] * points.channels[ny_idx].values[nn_indices[j]] +
                             points.channels[nz_idx].values[i] * points.channels[nz_idx].values[nn_indices[j]];
              if ( fabs (acos (dot_p)) < eps_angle )
              {
                processed[nn_indices[j]] = true;
                seed_queue.push_back (nn_indices[j]);
              }
            }
            // If normal information is not present, perform a simple Euclidean clustering
            else
            {
              processed[nn_indices[j]] = true;
              seed_queue.push_back (nn_indices[j]);
            }
          }

          sq_idx++;
        }

        // If this queue is satisfactory, add to the clusters
        if (seed_queue.size () >= min_pts_per_cluster)
        {
          std::vector<int> r (seed_queue.size ());
          for (unsigned int j = 0; j < seed_queue.size (); j++)
            r[j] = seed_queue[j];

          // Remove duplicates
          sort (r.begin (), r.end ());
          r.erase (unique (r.begin (), r.end ()), r.end ());

          clusters.push_back (r);
        }
      }

      // Destroy the tree
      delete tree;
    }
	

  }
}
