/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef EXTRA_FEATURES_H
#define EXTRA_FEATURES_H

#include <multibooster/multibooster.h>
#include <descriptors_2d/descriptors_2d.h>
#include <descriptors_3d/all_descriptors.h>
#include "point_cloud_mapping/kdtree/kdtree.h"
#include "point_cloud_mapping/kdtree/kdtree_ann.h"
#include <sensor_msgs/PointCloud.h>
#include <tr1/memory>
#include <tr1/random>

void floatToEigen(const std::vector<float>& vec, Eigen::VectorXf* eig);
typedef std::vector< std::vector<float> > vvf;
double sampleFromGaussian(double stdev, std::tr1::mt19937& mt);

//! Class to put a set of point clouds into canonical orientation.  TODO: add robot pose as a parameter so that the remaining ambiguity (the principal component can have a sign flip) is resolved.
class D3DCloudOrienter
{
 public:
  bool computed_;
  //! Matrices are k x 3 containing the points in (x, y, z) rows.  z is up, x is the long direction, y is the short direction.
  std::vector<Eigen::MatrixXf> oriented_clouds_; 

  D3DCloudOrienter();
  //! Clears oriented_clouds_ and resets computed_.
  void clear();
  //! Computes the new oriented clouds and stores in oriented_clouds_.
  void orientClouds(const sensor_msgs::PointCloud& data,
		    const std::vector<const std::vector<int>*>& interest_region_indices);
  std::string getName();
  
 private:
  Eigen::MatrixXf orientCloud(const sensor_msgs::PointCloud& data,
			      const std::vector<int>& interest_region_indices);
};

//! Class to project canonically-oriented clusters into a virtual orthographic camera image plane.
//! Depth images currently have an ambiguity as to which side they see.
class D3DCloudProjector
{
 public:
  bool debug_;
  bool computed_;
  //! Intensity projections of the clusters.
  std::vector<IplImage*> intensity_projections_;
  //! Depth projections of the clusters.
  std::vector<IplImage*> depth_projections_;
  
  D3DCloudProjector(int axis_of_projection, float pixels_per_meter, std::tr1::shared_ptr<D3DCloudOrienter> orienter);
  ~D3DCloudProjector();
  //! Computes the depth and intensity projection of the given clouds.
  void projectClouds(const sensor_msgs::PointCloud& data, const std::vector< const std::vector<int>* >& interest_region_indices);
  //! Clears orienter_, intensity_projections_, and depth_projections_. Resets computed_.
  void clear();
  std::string getName();

  
 private:
  //! 0 == x, 1 == y, 2 == z, where x, y, and z are the canonical orientation defined in D3DCloudOrienter.
  int axis_of_projection_;
  //! Resolution of the projection.
  float pixels_per_meter_;
  //! Object that puts the clusters in canonical orientation.
  std::tr1::shared_ptr<D3DCloudOrienter> orienter_;

  //! Computes the intensity and depth projections of cluster id in orienter_, then pushes them back on to intensity_projections_ and depth_projections_.
  void projectCloud(int id, const sensor_msgs::PointCloud& data, const std::vector<int>& interest_region_indices);	       
};


class CloudProjection : public Descriptor3D
{
public:
  //! If false, accumulate points into each bin; otherwise get average intensity.
  bool intensity_;
  //! Which principal component to use.  0, 1, or 2; 0 is the axis of largest variation.
  size_t axis_;
  int rows_;
  int cols_;
  float pixels_per_meter_;
  //! the percent of the way from min_u to max_u that the u_offset should be set to.  u = right, v = down.
  float u_offset_pct_;
  //! the percent of the way from min_v to max_v that the v_offset should be set to.
  float v_offset_pct_;
  //! The projected images for each cluster.  Cleared by clearShared().  You must call this function for each new pointcloud!
  std::vector<IplImage*> imgs_;

  CloudProjection(bool intensity, size_t axis, int rows, int cols, float pixels_per_meter,
		  float u_offset_pct = 0.5, float v_offset_pct = 0.5);

  std::string getName() const;
  //! Clears the projected images.
  void clearShared();

protected:
  void doComputation(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		     __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		     __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts,
		     __attribute__((unused)) std::vector<std::vector<float> >& results) {};
  void doComputation(const sensor_msgs::PointCloud& data,
		     cloud_kdtree::KdTree& data_kdtree,
		     const std::vector<const std::vector<int>*>& interest_region_indices,
		     std::vector<std::vector<float> >& results);
  int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		 __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts) {return 0;}
  
  int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		 __attribute__((unused)) const std::vector<const std::vector<int>*>& interest_region_indices) {return 0;}

  IplImage* computeProjection(const sensor_msgs::PointCloud& data,
			      const std::vector<int>& interest_region_indices);
};

class D3DRandomProjector : public Descriptor3D
{
public:
  std::tr1::shared_ptr<Descriptor3D> descriptor_;
  int seed_;
  std::tr1::mt19937 mt_;
  Eigen::MatrixXf projector_;
  D3DRandomProjector(std::tr1::shared_ptr<Descriptor3D> desc, unsigned int dim, int seed);
  std::string getName() const;

  void clearShared();
protected:
  virtual void doComputation(__attribute__((unused)) const sensor_msgs::PointCloud& data,
			     __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
			     __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts,
			     __attribute__((unused)) std::vector<std::vector<float> >& results) {};
  virtual void doComputation(const sensor_msgs::PointCloud& data,
			     cloud_kdtree::KdTree& data_kdtree,
			     const std::vector<const std::vector<int>*>& interest_region_indices,
			     std::vector<std::vector<float> >& results);
  virtual int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
			 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
			 __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts) {return 0;}
  
  virtual int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
			 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
			 __attribute__((unused)) const std::vector<const std::vector<int>*>& interest_region_indices) {return 0;}
};

class CloudProjectionHog : public Descriptor3D
{
 public:
  std::tr1::shared_ptr<CloudProjection> cp_;
  std::tr1::shared_ptr<HogWrapper> hog_;
  int kernel_size_u_;
  int kernel_size_v_;  

  CloudProjectionHog(std::tr1::shared_ptr<CloudProjection> cp, std::tr1::shared_ptr<HogWrapper> hog,
		     int kernel_size_u_, int kernel_size_v_);
  void clearShared();
  std::string getName() const;

 protected:
  void doComputation(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		     __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		     __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts,
		     __attribute__((unused)) std::vector<std::vector<float> >& results) {};
  void doComputation(const sensor_msgs::PointCloud& data,
		     cloud_kdtree::KdTree& data_kdtree,
		     const std::vector<const std::vector<int>*>& interest_region_indices,
		     std::vector<std::vector<float> >& results);
  int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		 __attribute__((unused)) const std::vector<const geometry_msgs::Point32*>& interest_pts) {return 0;}
  
  int precompute(__attribute__((unused)) const sensor_msgs::PointCloud& data,
		 __attribute__((unused)) cloud_kdtree::KdTree& data_kdtree,
		 __attribute__((unused)) const std::vector<const std::vector<int>*>& interest_region_indices) {return 0;}
};
  

#endif //EXTRA_FEATURES_H
