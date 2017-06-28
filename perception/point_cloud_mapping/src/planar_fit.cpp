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
 * $Id: planar_fit.cpp 30923 2010-02-09 22:47:11Z rusu $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b planar_fit attempts to fit the best plane to a given PointCloud message, using several different techniques.
This node should be used for testing different planar segmentation strategies.

 **/

// ROS core
#include <ros/node_handle.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
// For the normals visualization
#include <visualization_msgs/Marker.h>

#include <tf/transform_listener.h>

// Cloud kd-tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/intersections.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/rransac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/rmsac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/sac_model_normal_plane.h>

#include <angles/angles.h>

using namespace std;

class PlanarFit
{
  protected:
    ros::NodeHandle node_;

  public:

    // ROS messages
    sensor_msgs::PointCloud cloud_down_, cloud_plane_, cloud_outliers_;
    ros::Publisher normals_pub_;
    ros::Publisher plane_pub_;
    ros::Publisher outliers_pub_;
    ros::Publisher vis_pub_;
    ros::Subscriber cloud_sub_ ;

    tf::TransformListener tf_;

    // Kd-tree stuff
    cloud_kdtree::KdTree *kdtree_;
    vector<vector<int> > points_indices_;

    // Parameters
    double radius_;
    int k_;

    // Additional downsampling parameters
    int downsample_;
    geometry_msgs::Point leaf_width_;

    vector<cloud_geometry::Leaf> leaves_;

    // Do not use normals (0), use radius search (0) or use K-nearest (1) for point neighbors
    int radius_or_knn_;

    // If normals_fidelity_, then use the original cloud data to estimate the k-neighborhood and thus the normals
    int normals_fidelity_;

    double sac_distance_threshold_;
    int sac_maximum_iterations_;
    int sac_use_normals_;
    double sac_normal_distance_weight_;	// for finding the plane
    double sac_normal_inlier_distance_weight_; // for selecting inliers

    // Euclidean clustering
    int use_clustering_;      // use clustering (1) or not (0)
    double euclidean_cluster_angle_tolerance_, euclidean_cluster_distance_tolerance_;
    int euclidean_cluster_min_pts_;

    // Normal line markers
    int publish_normal_lines_as_markers_;
    double max_dist_;
    string distances_channel_, cloud_topic_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlanarFit () : node_ ("~"), cloud_topic_ ("/tilt_laser_cloud")
    {
      node_.param ("search_radius_or_knn", radius_or_knn_, 1);   // none (0), radius (1) or k (2) nearest neighbor search
      node_.param ("search_radius", radius_, 0.01);              // 1cm radius by default
      node_.param ("search_k_closest", k_, 20);                  // 20 k-neighbors by default

      node_.param ("downsample", downsample_, 1);                        // downsample cloud before normal estimation
      node_.param ("downsample_leaf_width_x", leaf_width_.x, 0.02);      // 2cm radius by default
      node_.param ("downsample_leaf_width_y", leaf_width_.y, 0.02);      // 2cm radius by default
      node_.param ("downsample_leaf_width_z", leaf_width_.z, 0.02);      // 2cm radius by default
      node_.param ("downsample_max_dist", max_dist_, -1.0);              // use all data by default
      node_.param ("downsample_distances_channel", distances_channel_, string ("distances"));              // use all data by default

      node_.param ("normals_high_fidelity", normals_fidelity_, 1);       // compute the downsampled normals from the original data

      node_.param ("sac_distance_threshold", sac_distance_threshold_, 0.02);   // 2 cm threshold
      node_.param ("sac_maximum_iterations", sac_maximum_iterations_, 500);    // maximum 500 SAC iterations
      node_.param ("sac_use_normals", sac_use_normals_, 0);                    // use normals in SAC
      node_.param ("sac_normal_distance_weight", sac_normal_distance_weight_, 0.05);   // weight normals by .05 in the SAC distance
      node_.param ("sac_normal_inlier_distance_weight", sac_normal_inlier_distance_weight_, 0.05);   // weight normals by .05 in the SAC inlier retrieval

      node_.param ("use_clustering", use_clustering_, 0);
      node_.param ("euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_, 15.0);
      euclidean_cluster_angle_tolerance_ = angles::from_degrees (euclidean_cluster_angle_tolerance_);
      node_.param ("euclidean_cluster_min_pts", euclidean_cluster_min_pts_, 500);
      node_.param ("euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_, 0.05);

      node_.param ("publish_normal_lines_as_markers", publish_normal_lines_as_markers_, 1);

      ROS_INFO("[PLANAR] sac_use_normals is %d", sac_use_normals_);

      cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud> (cloud_topic_, 1, &PlanarFit::cloud_cb, this);
      normals_pub_ = node_.advertise<sensor_msgs::PointCloud> ("/planar_fit/normals", 1);
      plane_pub_ = node_.advertise<sensor_msgs::PointCloud> ("/planar_fit/plane", 1);
      outliers_pub_ = node_.advertise<sensor_msgs::PointCloud> ("/planar_fit/outliers", 1);

      // A channel to visualize the normals as cute little lines
      //node_.advertise<PolyLine> ("/planar_fit/normal_lines", 1);
      vis_pub_ = node_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlanarFit () { }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get the view point from where the scans were taken in the incoming PointCloud message frame
      * \param cloud_frame the point cloud message TF frame
      * \param viewpoint_cloud the resultant view point in the incoming cloud frame
      * \param tf a pointer to a TransformListener object
      */
    void
      getCloudViewPoint (const string &cloud_frame, geometry_msgs::PointStamped &viewpoint_cloud, tf::TransformListener &tf)
    {
      // Figure out the viewpoint value in the point cloud frame
      geometry_msgs::PointStamped viewpoint_laser;
      viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
      // Set the viewpoint in the laser coordinate system to 0, 0, 0
      viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;

      try
      {
        tf.transformPoint (cloud_frame, viewpoint_laser, viewpoint_cloud);
        ROS_INFO ("Cloud view point in frame %s is: %g, %g, %g.", cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
      catch (...)
      {
        // Default to 0.05, 0, 0.942768 (base_link, ~base_footprint)
        viewpoint_cloud.point.x = 0.05; viewpoint_cloud.point.y = 0.0; viewpoint_cloud.point.z = 0.942768;
        ROS_WARN ("Could not transform a point from frame %s to frame %s! Defaulting to <%f, %f, %f>",
                  viewpoint_laser.header.frame_id.c_str (), cloud_frame.c_str (),
                  viewpoint_cloud.point.x, viewpoint_cloud.point.y, viewpoint_cloud.point.z);
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      updateParametersFromServer ()
    {
      if (node_.hasParam ("search_radius_or_knn")) node_.getParam ("search_radius_or_knn", radius_or_knn_);
      if (node_.hasParam ("search_radius")) node_.getParam ("search_radius", radius_);
      if (node_.hasParam ("search_k_closest")) node_.getParam ("search_k_closest", k_);

      if (node_.hasParam ("downsample")) node_.getParam ("downsample", downsample_);
      if (node_.hasParam ("downsample_leaf_width_x")) node_.getParam ("downsample_leaf_width_x", leaf_width_.x);
      if (node_.hasParam ("downsample_leaf_width_y")) node_.getParam ("downsample_leaf_width_y", leaf_width_.y);
      if (node_.hasParam ("downsample_leaf_width_z")) node_.getParam ("downsample_leaf_width_z", leaf_width_.z);

      if (node_.hasParam ("normals_high_fidelity")) node_.getParam ("normals_high_fidelity", normals_fidelity_);

      if (node_.hasParam ("sac_distance_threshold")) node_.getParam ("sac_distance_threshold", sac_distance_threshold_);
      if (node_.hasParam ("sac_maximum_iterations")) node_.getParam ("sac_maximum_iterations", sac_maximum_iterations_);
      if (node_.hasParam ("sac_use_normals")) node_.getParam ("sac_use_normals", sac_use_normals_);                                   // use normals in SAC
      if (node_.hasParam ("sac_normal_distance_weight")) node_.getParam ("sac_normal_distance_weight", sac_normal_distance_weight_);  // weight normals by .05 in the SAC distance
      if (node_.hasParam ("sac_normal_inlier_distance_weight")) node_.getParam ("sac_normal_inlier_distance_weight", sac_normal_inlier_distance_weight_);  // weight normals by .05 in the SAC inlier retrieval

      if (node_.hasParam ("use_clustering")) node_.getParam ("use_clustering", use_clustering_);
      if (node_.hasParam ("euclidean_cluster_min_pts")) node_.getParam ("euclidean_cluster_min_pts", euclidean_cluster_min_pts_);
      if (node_.hasParam ("euclidean_cluster_distance_tolerance")) node_.getParam ("euclidean_cluster_distance_tolerance", euclidean_cluster_distance_tolerance_);
      if (node_.hasParam ("euclidean_cluster_angle_tolerance"))
      {
        node_.getParam ("euclidean_cluster_angle_tolerance", euclidean_cluster_angle_tolerance_);
        euclidean_cluster_angle_tolerance_ = angles::from_degrees (euclidean_cluster_angle_tolerance_);
      }
      if (node_.hasParam ("publish_normal_lines_as_markers")) node_.getParam ("publish_normal_lines_as_markers", publish_normal_lines_as_markers_);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Callback
    void cloud_cb (const sensor_msgs::PointCloud::ConstPtr& cloud_msg_)
    {
      updateParametersFromServer ();

      #warning "The message callback gives const ptr. The processing requires NON-CONST, therefore we have to copy the data"
      sensor_msgs::PointCloud cloud_ = *cloud_msg_;

      ROS_INFO ("Received %d data points in frame %s with %d channels (%s).", (int)cloud_.points.size (), cloud_.header.frame_id.c_str (),
                (int)cloud_.channels.size (), cloud_geometry::getAvailableChannels (cloud_).c_str ());
      if (cloud_.points.size () == 0)
      {
        ROS_ERROR ("No data points found. Exiting...");
        return;
      }

      cloud_down_.header = cloud_.header;
      cloud_down_.channels.clear();

      //ROS_INFO ("Original: %d channels, Downsampled: %d channels", cloud_.channels.size(), cloud_down_.channels.size());

      ros::Time ts = ros::Time::now ();
      // Figure out the viewpoint value in the cloud_frame frame
      geometry_msgs::PointStamped viewpoint_cloud;
      getCloudViewPoint (cloud_.header.frame_id, viewpoint_cloud, tf_);

      // ---------------------------------------------------------------------------------------------------------------
      // ---[ Downsample and/or estimate point normals
      if (downsample_ != 0)
      {
        int d_idx = cloud_geometry::getChannelIndex (cloud_, distances_channel_);
        ros::Time ts1 = ros::Time::now ();

        // Downsample addon for points in the stereo frame (z = distance)
        vector<int> indices_down (cloud_.points.size ());
        if (max_dist_ != -1.0)
          for (size_t i = 0; i < indices_down.size (); ++i)
          {
            if (distances_channel_ == "z" && cloud_.points[i].z < max_dist_)
              indices_down[i] = i;
          }
        else
          for (size_t i = 0; i < indices_down.size (); ++i)
            indices_down[i] = i;
        try
        {
          // We sacrifice functionality for speed. Use a fixed 3D grid to downsample the data instead of an octree structure
          ROS_DEBUG ("Downsampling dataset with %f %f %f, using a maximum distance of %f.", leaf_width_.x, leaf_width_.y, leaf_width_.z, max_dist_);
          cloud_geometry::downsamplePointCloud (cloud_, indices_down, cloud_down_, leaf_width_, leaves_, d_idx, max_dist_);  // -1 means use all data
        }
        catch (std::bad_alloc)
        {
          ROS_ERROR ("Not enough memory to create a simple downsampled representation. Change the downsample_leaf_width parameters.");
          return;
        }
        ROS_DEBUG ("Downsampling enabled. Number of points left: %d / %d in %g seconds.", (int)cloud_down_.points.size (), (int)cloud_.points.size (), (ros::Time::now () - ts1).toSec ());

        if (radius_or_knn_ == 1)             // Use a radius search
        {
          if (normals_fidelity_)
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud_, radius_, viewpoint_cloud);  // Estimate point normals in the original point cloud using a fixed radius search
          else
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, radius_, viewpoint_cloud);          // Estimate point normals in the downsampled point cloud using a fixed radius search
        }
        else if (radius_or_knn_ == 2)        // Use a k-nearest neighbors search
        {
          if (normals_fidelity_)
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud_, k_, viewpoint_cloud);       // Estimate point normals in the original point cloud using a K-nearest search
          else
            cloud_geometry::nearest::computePointCloudNormals (cloud_down_, k_, viewpoint_cloud);               // Estimate point normals in the downsampled point cloud using a K-nearest search
        }
        normals_pub_.publish (cloud_down_);
        //if (publish_normal_lines_as_markers_)
        //  publishNormalLines (cloud_down_, 0, 1, 2, 0.01, 0.0005);
      } // if (downsample)
      else
      {
        if (radius_or_knn_ == 1)             // Use a radius search
          cloud_geometry::nearest::computePointCloudNormals (cloud_, radius_, viewpoint_cloud);    // Estimate point normals using a fixed radius search
        else if (radius_or_knn_ == 2)        // Use a k-nearest neighbors search
          cloud_geometry::nearest::computePointCloudNormals (cloud_, k_, viewpoint_cloud);         // Estimate point normals using a K-nearest search
        normals_pub_.publish( cloud_);
        //if (publish_normal_lines_as_markers_)
        //  publishNormalLines (cloud_, first_normal_index, first_normal_index+1, first_normal_index+2, 0.01, 0.0005);
      }
      ROS_DEBUG ("Normals estimated in %g seconds.", (ros::Time::now () - ts).toSec ());


      // ---------------------------------------------------------------------------------------------------------------
      // ---[ Fit a planar model
      vector<int> inliers;
      vector<double> coeff;
      int total_nr_points;
      ts = ros::Time::now ();
      if (downsample_ != 0)
      {
        total_nr_points = cloud_down_.points.size ();

        // Break into clusters
        if (use_clustering_)
        {
          ros::Time ts1 = ros::Time::now ();
          vector<vector<int> > clusters;

          if (radius_or_knn_ != 0)                  // did we estimate normals ?
            cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          else                                      // if not, set nx_idx to -1 and perform a pure Euclidean clustering
            cloud_geometry::nearest::extractEuclideanClusters (cloud_down_, euclidean_cluster_distance_tolerance_, clusters, -1, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          ROS_DEBUG ("Clustering done. Number of clusters extracted: %d in %g seconds.", (int)clusters.size (), (ros::Time::now () - ts1).toSec ());

          ts = ros::Time::now ();
          fitSACPlane (&cloud_down_, clusters[clusters.size () - 1], inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model
        }
        else
          fitSACPlane (&cloud_down_, inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model

        cloud_geometry::getPointCloud (cloud_down_, inliers, cloud_plane_);
        cloud_geometry::getPointCloudOutside (cloud_down_, inliers, cloud_outliers_);
      }
      else
      {
        total_nr_points = cloud_.points.size ();

        // Break into clusters
        if (use_clustering_)
        {
          ros::Time ts1 = ros::Time::now ();
          vector<vector<int> > clusters;

          if (radius_or_knn_ != 0)                  // did we estimate normals ?
            cloud_geometry::nearest::extractEuclideanClusters (cloud_, euclidean_cluster_distance_tolerance_, clusters, 0, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          else                                      // if not, set nx_idx to -1 and perform a pure Euclidean clustering
            cloud_geometry::nearest::extractEuclideanClusters (cloud_, euclidean_cluster_distance_tolerance_, clusters, -1, 1, 2, euclidean_cluster_angle_tolerance_, euclidean_cluster_min_pts_);
          ROS_DEBUG ("Clustering done. Number of clusters extracted: %d in %g seconds.", (int)clusters.size (), (ros::Time::now () - ts1).toSec ());

          ts = ros::Time::now ();
          fitSACPlane (&cloud_, clusters[clusters.size () - 1], inliers, coeff, viewpoint_cloud, sac_distance_threshold_);    // Fit the plane model
        }
        else
          fitSACPlane (&cloud_, inliers, coeff, viewpoint_cloud, sac_distance_threshold_);       // Fit the plane model

        cloud_geometry::getPointCloud (cloud_, inliers, cloud_plane_);
        cloud_geometry::getPointCloudOutside (cloud_, inliers, cloud_outliers_);
      }
      ROS_INFO ("Planar model found with %d / %d inliers in %g seconds.\n", (int)inliers.size (), total_nr_points, (ros::Time::now () - ts).toSec ());

      plane_pub_.publish ( cloud_plane_);
      outliers_pub_.publish ( cloud_outliers_);

      //dbug
      if (publish_normal_lines_as_markers_)
	      publishNormalLines (cloud_plane_, 0, 1, 2, 0.01, 0.0005);
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> &inliers, vector<double> &coeff,
                   const geometry_msgs::PointStamped &viewpoint_cloud, double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model;
      sample_consensus::SACModelNormalPlane *normal_model = new sample_consensus::SACModelNormalPlane ();
	  
      if (sac_use_normals_)
	    {
    	  normal_model->setNormalDistanceWeight(sac_normal_distance_weight_);
    	  model = (sample_consensus::SACModelPlane *) normal_model;
    	}
      else
    	  model = new sample_consensus::SACModelPlane ();

      //      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
      //      sample_consensus::SAC *sac             = new sample_consensus::RRANSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RRANSAC*>(sac)->setFractionNrPretest (10);
      //      sample_consensus::SAC *sac             = new sample_consensus::RMSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RMSAC*>(sac)->setFractionNrPretest (10);

      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (sac_maximum_iterations_);
      model->setDataSet (points);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
    	  // NOTE (kk): can reweight normals for inlier selection
      	normal_model->setNormalDistanceWeight (sac_normal_inlier_distance_weight_);
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->points.at (inliers[0]), viewpoint_cloud);

        ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
                   coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
	//        model->projectPointsInPlace (inliers, coeff);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Find a plane model in a point cloud with SAmple Consensus methods
      * \param points the point cloud message
      * \param indices a subset of point indices to use
      * \param inliers the resultant planar inliers
      * \param coeff the resultant plane coefficients
      * \param viewpoint_cloud a point to the pose where the data was acquired from (for normal flipping)
      * \param dist_thresh the maximum allowed distance threshold of an inlier to the model
      * \param min_pts the minimum number of points allowed as inliers for a plane model
      */
    bool
      fitSACPlane (sensor_msgs::PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const geometry_msgs::PointStamped &viewpoint_cloud, double dist_thresh)
    {
      // Create and initialize the SAC model
      sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();

      //      sample_consensus::SAC *sac             = new sample_consensus::RANSAC (model, dist_thresh);
      //      sample_consensus::SAC *sac             = new sample_consensus::RRANSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RRANSAC*>(sac)->setFractionNrPretest (10);
      //      sample_consensus::SAC *sac             = new sample_consensus::RMSAC (model, dist_thresh);
      //      reinterpret_cast<sample_consensus::RMSAC*>(sac)->setFractionNrPretest (10);

      sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, dist_thresh);
      sac->setMaxIterations (sac_maximum_iterations_);
      model->setDataSet (points, indices);

      // Search for the best plane
      if (sac->computeModel (0))
      {
        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->points.at (inliers[0]), viewpoint_cloud);

        //ROS_INFO ("Found a planar model supported by %d inliers: [%g, %g, %g, %g]", (int)inliers.size (),
        //           coeff[0], coeff[1], coeff[2], coeff[3]);

        // Project the inliers onto the model
	//        model->projectPointsInPlace (inliers, coeff);
      }
      else
      {
        ROS_ERROR ("Could not compute a plane model.");
        return (false);
      }

      delete sac;
      delete model;
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Publish the normals as cute little lines
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param nx_idx the index of the x normal
      * \param ny_idx the index of the y normal
      * \param nz_idx the index of the z normal
      * \param length the length of the normal lines
      * \param width the width of the normal lines
      */
    void
      publishNormalLines (const sensor_msgs::PointCloud &points, int nx_idx, int ny_idx, int nz_idx, double length, double width)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = points.header.frame_id;  //"base_link"

      ROS_INFO("marker.header.frame_id = %s", marker.header.frame_id.c_str());

      marker.header.stamp = ros::Time();
      marker.ns = "normal_lines_ns";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = width;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

      int nr_points = points.points.size ();

      marker.points.resize (2 * nr_points);
      for (int i = 0; i < nr_points; i++)
      {
        marker.points[2*i].x = points.points[i].x;
        marker.points[2*i].y = points.points[i].y;
        marker.points[2*i].z = points.points[i].z;
        marker.points[2*i+1].x = points.points[i].x + points.channels[nx_idx].values[i] * length;
        marker.points[2*i+1].y = points.points[i].y + points.channels[ny_idx].values[i] * length;
        marker.points[2*i+1].z = points.points[i].z + points.channels[nz_idx].values[i] * length;
      }

      vis_pub_.publish ( marker);
      ROS_INFO ("Published %d normal lines", nr_points);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Publish the normals as cute little lines
      * \NOTE: assumes normalized point normals !
      * \param points pointer to the point cloud message
      * \param indices indices of the point cloud normals to publish
      * \param nx_idx the index of the x normal
      * \param ny_idx the index of the y normal
      * \param nz_idx the index of the z normal
      * \param length the length of the normal lines
      * \param width the width of the normal lines
      */
    void
      publishNormalLines (const sensor_msgs::PointCloud &points, vector<int> &indices, int nx_idx, int ny_idx, int nz_idx, double length, double width)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = points.header.frame_id;  //"base_link"

      ROS_INFO("marker.header.frame_id = %s", marker.header.frame_id.c_str());

      marker.header.stamp = ros::Time();
      marker.ns = "normal_lines_ns";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = width;
      marker.scale.y = 1;
      marker.scale.z = 1;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

      int nr_points = indices.size ();

      marker.points.resize (2 * nr_points);
      for (int i = 0; i < nr_points; i++)
      {
        marker.points[2*i].x = points.points[indices[i]].x;
        marker.points[2*i].y = points.points[indices[i]].y;
        marker.points[2*i].z = points.points[indices[i]].z;
        marker.points[2*i+1].x = points.points[indices[i]].x + points.channels[nx_idx].values[indices[i]] * length;
        marker.points[2*i+1].y = points.points[indices[i]].y + points.channels[ny_idx].values[indices[i]] * length;
        marker.points[2*i+1].z = points.points[indices[i]].z + points.channels[nz_idx].values[indices[i]] * length;
      }

      vis_pub_.publish ( marker);
      ROS_INFO ("Published %d normal lines", nr_points);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv,"planar_fit");

  PlanarFit p;

  ros::spin ();

  return (0);
}
/* ]--- */

