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


#ifndef CLUSTER_DESCRIPTORS_H_
#define CLUSTER_DESCRIPTORS_H_

#include <multibooster/multibooster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.hpp>
#include <Eigen/Eigen>
#include <pipeline/pipeline.h>
#include <tr1/random>




class PointCloudInterface : public pipeline::ComputeNode
{
 public:
 PointCloudInterface() : ComputeNode() {}
  virtual void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud) = 0;
  virtual void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity) = 0;
  virtual boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const = 0;
  virtual boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const = 0;
};

class PlaneFittingCloudOrienter : public PointCloudInterface
{
 public:
  boost::shared_ptr<Eigen::MatrixXf> input_cloud_;
  boost::shared_ptr<Eigen::VectorXf> input_intensity_;
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;

  PlaneFittingCloudOrienter(int num_iterations = 100, float tolerance = 0.1);
  void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud);
  void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity);
  boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const;
  
 private:
  //! How close to the plane a point must be to count.
  float tolerance_;
  //! How many RANSAC iterations to run.
  int num_iterations_;

  //! Run RANSAC to find the dominant line in the XY plane.
  Eigen::VectorXf fitPlane(const Eigen::MatrixXf& cloud) const;
  std::string _getName() const;
  void _flush();
  void _compute();
};
  

//! Class to put a set of point clouds into canonical orientation.  TODO: add robot pose as a parameter so that the remaining ambiguity (the principal component can have a sign flip) is resolved.
class CloudOrienter : public PointCloudInterface
{
 public:
  //! Matrices are k x 3 containing the points in (x, y, z) rows.  z is up, x is the long direction, y is the short direction.
  boost::shared_ptr<Eigen::MatrixXf> input_cloud_;
  boost::shared_ptr<Eigen::VectorXf> input_intensities_;
  boost::shared_ptr<Eigen::MatrixXf> output_cloud_;
  
  CloudOrienter();
  void setInputCloud(boost::shared_ptr<Eigen::MatrixXf> cloud);
  void setInputIntensity(boost::shared_ptr<Eigen::VectorXf> intensity);
  boost::shared_ptr<Eigen::MatrixXf> getOutputCloud() const;
  boost::shared_ptr<Eigen::VectorXf> getOutputIntensity() const;
  
 protected:
  virtual std::string _getName() const;
  virtual void _flush();
  virtual void _compute();
};

//! Class to put a set of point clouds into canonical orientation.  TODO: add robot pose as a parameter so that the remaining ambiguity (the principal component can have a sign flip) is resolved.
class HoughCloudOrienter : public CloudOrienter
{
 public:
  HoughCloudOrienter();

 protected:
  virtual std::string _getName() const;
  virtual void _compute();
};

//! Class to project canonically-oriented clusters into a virtual orthographic camera image plane.
//! Depth images currently have an ambiguity as to which side they see.
class CloudProjector : public pipeline::ComputeNode
{
 public:
  IplImage* intensity_projection_;
  IplImage* depth_projection_;
  
  CloudProjector(int axis_of_projection, float pixels_per_meter, boost::shared_ptr<PointCloudInterface> orienter, int smoothing = 3, int min_width = 0, int min_height = 0);
  ~CloudProjector();
  
 private:
  //! 0 == x, 1 == y, 2 == z, where x, y, and z are the canonical orientation defined in CloudOrienter.
  int axis_of_projection_;
  //! Resolution of the projection.
  float pixels_per_meter_;
  //! cvSmooth parameter: size of the Gaussian kernel to use.
  int smoothing_;
  //! 0 indicates no minimum size.  Usually this is good to use in conjuction with HogArray, which needs a minimum size image to work with.
  int min_width_;
  //! 0 indicates no minimum size.
  int min_height_;
  //! Object that puts the clusters in canonical orientation.
  boost::shared_ptr<PointCloudInterface> orienter_;

  std::string _getName() const;
  void _flush();
  void _compute();
  void _display() const;
};

//! Puts x,y,z coordinates into spin coordinates (alpha, beta = z) for spin image computation.
class CloudSpinner : public pipeline::ComputeNode
{
 public:
  //! orienter_->oriented_clouds_ stored in spin coordinates, (sqrt(x^2 + y^2), z) pairs.
  boost::shared_ptr<Eigen::MatrixXf> spin_coords_;

  CloudSpinner(boost::shared_ptr<PointCloudInterface> orienter);
  
 private:
  boost::shared_ptr<PointCloudInterface> orienter_;

  std::string _getName() const;
  void _flush();
  void _compute();
};

class SpinImage : public pipeline::DescriptorNode
{
 public:
  //! row-major.
  boost::shared_ptr<Eigen::VectorXf> vectorized_spin_image_;
  
  SpinImage(boost::shared_ptr<CloudSpinner> spinner, float pixels_per_meter, int num_rows, int num_cols);
  ~SpinImage();
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<CloudSpinner> spinner_;
  float pixels_per_meter_;
  int num_rows_;
  int num_cols_;
  IplImage* ipl_;
  

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush();
  void _compute();
  std::string _getName() const;
  void _display() const;
};

class Whitener : public pipeline::DescriptorNode
{
 public:
  Whitener(boost::shared_ptr<pipeline::DescriptorNode> node);
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<pipeline::DescriptorNode> node_;
  boost::shared_ptr<Eigen::VectorXf> whitened_;
  
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush();
  void _compute();
  std::string _getName() const;
};

class OrientedBoundingBoxSize : public pipeline::DescriptorNode
{
 public:
  //! x, y, z.
  boost::shared_ptr<Eigen::VectorXf> bbox_size_;

  OrientedBoundingBoxSize(boost::shared_ptr<PointCloudInterface> orienter);
  int getDescriptorLength() const;

 private:
  boost::shared_ptr<PointCloudInterface> orienter_;

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  void _flush() {bbox_size_.reset();}
  void _compute();
  std::string _getName() const {return std::string("OrientedBoundingBoxSize");}
  void _display() const;
};

//! Computes HOG descriptors on a variable-sized image at locations defined as percentages of maximum size in u and v.
//! If the window falls off the edge of the image, then the window is shifted so that it does not fall off the edge.
class HogArray : public pipeline::ComputeNode
{
 public:
  //! Percent of the way from u=0 to u=num_cols to compute hog.
  std::vector<float> u_offset_pcts_;
  //! Percent of the way from v=0 to v=num_rows to compute hog.
  std::vector<float> v_offset_pcts_;
  //! The result of hog computation.
  std::vector< boost::shared_ptr<Eigen::VectorXf> > descriptors_;
  //! Input for the image to work on.
  boost::shared_ptr<CloudProjector> projector_;

  HogArray(boost::shared_ptr<CloudProjector> projector, const std::vector<float>& u_offset_pcts, const std::vector<float>& v_offset_pcts,
	   cv::Size win_size, cv::Size block_size, cv::Size block_stride, cv::Size cell_size, int num_bins);
  ~HogArray();
  int getDescriptorLength() const;
  
 private:
  cv::Size win_size_;
  cv::Size block_size_;
  cv::Size block_stride_;
  cv::Size cell_size_;
  int num_bins_;
  //! OpenCV HOG computation object.
  cv::HOGDescriptor hog_;
  //! Locations to compute HOG, derived from {u,v}_offset_pcts_ and stored for use by HogWindow's display function.
  std::vector<cv::Point> coords_;
  IplImage* img8u_;
  
  std::string _getName() const;
  void _flush();
  void _compute();

  friend class HogWindow;
};  

class HogWindow : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> hog_descriptor_;

  HogWindow(size_t window_number, boost::shared_ptr<HogArray> hog_array);
  int getDescriptorLength() const;
  
 private:
  boost::shared_ptr<HogArray> hog_array_;
  size_t window_number_;

  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  std::string _getName() const;
  void _display() const;
  void _flush();
  void _compute();
};

class RandomProjector : public pipeline::DescriptorNode
{
 public:
  boost::shared_ptr<Eigen::VectorXf> projected_descriptor_;

  //! Slow constructor that generates the projection matrix.
  RandomProjector(int output_dim, int seed, boost::shared_ptr<pipeline::DescriptorNode> descriptor);
  //! Fast constructor that gets the pre-generated projection matrix from another RandomProjector.
  //RandomProjector(boost::shared_ptr<Eigen::MatrixXf> projector, boost::shared_ptr<pipeline::DescriptorNode> descriptor);

  static boost::shared_ptr<Eigen::MatrixXf> generateProjectionMatrix(int input_dim, int output_dim, int seed);
  RandomProjector(boost::shared_ptr<Eigen::MatrixXf> projector, boost::shared_ptr<pipeline::DescriptorNode> descriptor);
  int getDescriptorLength() const;

 private:
  int seed_;
  int output_dim_;
  boost::shared_ptr<pipeline::DescriptorNode> descriptor_;
  boost::shared_ptr<Eigen::MatrixXf> projector_;

  static double sampleFromGaussian(std::tr1::mt19937& mersenne_twister, double stdev);
  boost::shared_ptr<Eigen::VectorXf> _getDescriptor() const;
  std::string _getName() const;
  void _flush();
  void _compute();
};

class MultiBoosterObjectConstructor : public pipeline::ComputeNode
{
 public:
  std::vector< boost::shared_ptr<pipeline::DescriptorNode> > descriptor_nodes_;
  //boost::shared_ptr<Object> object_;
  Object* object_;
  
  MultiBoosterObjectConstructor(std::vector< boost::shared_ptr<pipeline::DescriptorNode> > descriptor_nodes);
  void display() const {std::cout << "No display functionality for " << _getName() << std::endl;}

 private:
  std::string _getName() const;
  void _flush();
  void _compute();
};

class MultiBoosterNode : public pipeline::ComputeNode
{
 public:
  //! Response vector in order of booster_->class_map_.
  Eigen::VectorXf response_;

  MultiBoosterNode(MultiBooster* booster,
		   boost::shared_ptr<MultiBoosterObjectConstructor> obj_constructor);

 private:
  MultiBooster* booster_;
  boost::shared_ptr<MultiBoosterObjectConstructor> constructor_;

  
  std::string _getName() const;
  void _flush();
  void _compute();
};

/* class DescriptorPipeline : public pipeline::Pipeline */
/* { */
/*  public: */
/*   DescriptorPipeline(int num_components, int num_threads); */
/*   //! clouds.size() <= num_components_. */
/*   void setInputs(const std::vector< boost::shared_ptr<Eigen::MatrixXf> >& clouds, */
/* 		 const std::vector< boost::shared_ptr<Eigen::VectorXf> >& intensities); */
/*   //! For the special case of a single pipeline component. */
/*   void setInput(boost::shared_ptr<Eigen::MatrixXf> cloud, */
/* 		boost::shared_ptr<Eigen::VectorXf> intensity); */
/*   std::vector<std::string> getDescriptorNames() const; */
/*   //! Returns the number of bytes in all the descriptors in a single pipeline component. */
/*   int getNumBytes() const; */
/* /\*   std::vector< std::vector< boost::shared_ptr<Eigen::VectorXf> > > *\/ */
/* /\*     computeDescriptors(boost::shared_ptr<Eigen::MatrixXf> clouds, *\/ */
/* /\* 		       boost::shared_ptr<Eigen::VectorXf> intensities); *\/ */

  
/*   std::vector< boost::shared_ptr<pipeline::DescriptorNode> > getOutputDescriptorNodes(size_t id); */
/*   std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > getOutputDescriptorNodes(); */
  
/*  private: */
/*   //! The number of pipeline components, i.e. the number of clusters to handle at once. */
/*   int num_components_; */
/*   //! Number of threads to use. */
/*   int num_threads_; */
/*   //! output_nodes_[i] is the vector of all output descriptor nodes for component i. */
/*   std::vector< std::vector< boost::shared_ptr<pipeline::DescriptorNode> > > output_nodes_; */
/*   //! cloud_orienters_[i] is the cloud orienter (only input) for component i. */
/*   std::vector< boost::shared_ptr<PointCloudInterface> > cloud_orienters_; */

/* }; */


/* void generateClusterDescriptorPipeline(std::vector< boost::shared_ptr<Eigen::MatrixXf> > clouds, */
/* 				       std::vector< boost::shared_ptr<Eigen::VectorXf> > intensities, */
/* 				       std::vector< boost::shared_ptr<pipeline::ComputeNode> >* nodes, */
/* 				       std::vector< boost::shared_ptr<pipeline::DescriptorNode> >* descriptor_nodes); */

#endif // CLUSTER_DESCRIPTORS_H_
