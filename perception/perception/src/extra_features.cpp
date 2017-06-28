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


#include "extra_features.h"

using namespace std;
using namespace Eigen;
using namespace std::tr1;


double sampleFromGaussian(double stdev, std::tr1::mt19937& mt) {
  double sum = 0;
  for(size_t i=0; i<12; ++i) {
    sum += 2 * stdev * (double)mt() / (double)mt.max() - stdev;
  }
  return sum / 2;
}


void floatToEigen(const vector<float>& vec, VectorXf* eig) {
  assert(!vec.empty());
  *eig = VectorXf::Zero(vec.size());
  for(size_t i=0; i<vec.size(); ++i) {
    (*eig)[i] = vec[i];
  }
}

D3DCloudProjector::D3DCloudProjector(int axis_of_projection, float pixels_per_meter, std::tr1::shared_ptr<D3DCloudOrienter> orienter) :
  debug_(false),
  computed_(false),
  axis_of_projection_(axis_of_projection),
  pixels_per_meter_(pixels_per_meter),
  orienter_(orienter)
{
}

D3DCloudProjector::~D3DCloudProjector() {
  for(size_t i=0; i<intensity_projections_.size(); ++i)
    cvReleaseImage(&intensity_projections_[i]);
  for(size_t i=0; i<depth_projections_.size(); ++i)
    cvReleaseImage(&depth_projections_[i]);
}

void D3DCloudProjector::clear() {
  if(!computed_) //If these things are already cleared, don't bother.
    return;
  
  for(size_t i=0; i<intensity_projections_.size(); ++i)
    cvReleaseImage(&intensity_projections_[i]);
  for(size_t i=0; i<depth_projections_.size(); ++i)
    cvReleaseImage(&depth_projections_[i]);

  intensity_projections_.clear();
  depth_projections_.clear();

  computed_ = false;
}

std::string D3DCloudProjector::getName() {
  ostringstream oss;
  oss << orienter_->getName() << "_D3DCloudProjector_axis" << axis_of_projection_ << "_ppm" << pixels_per_meter_;
  return oss.str();
}

void D3DCloudProjector::projectClouds(const sensor_msgs::PointCloud& data, const std::vector< const std::vector<int>* >& interest_region_indices) {
  if(!(intensity_projections_.empty() && depth_projections_.empty()))
    cerr << "D3DCloudProjector is projecting more clouds before the old ones have been cleared!" << endl;
  assert(!computed_);
  
  if(!orienter_->computed_)
    orienter_->orientClouds(data, interest_region_indices);

  assert(orienter_->oriented_clouds_.size() == interest_region_indices.size());
  for(size_t i=0; i<interest_region_indices.size(); ++i)
    projectCloud(i, data, *interest_region_indices[i]);

  computed_ = true;
}

void D3DCloudProjector::projectCloud(int id, const sensor_msgs::PointCloud& data, const std::vector<int>& interest_region_indices) {
  MatrixXf& oriented = orienter_->oriented_clouds_[id];

  // -- Get a copy of the projected points.
  MatrixXf projected(oriented.rows(), 2);
  int c=0;
  for(int i=0; i<3; ++i) {
    if(i == axis_of_projection_)
      continue;
    projected.col(c) = oriented.col(i);
    ++c;
  }

  // -- Transform into pixel units.  projected is currently in meters, centered at 0.
  //projected *= pixels_per_meter_;
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) *= pixels_per_meter_;
    projected(i, 1) *= pixels_per_meter_;
  }
  
  
  // -- Find min and max of u and v.  TODO: noise sensitivity?
  // u is the col number in the image plane, v is the row number.
  float min_v = FLT_MAX;
  float min_u = FLT_MAX;
  float max_v = -FLT_MAX;
  float max_u = -FLT_MAX;
  for(int i=0; i<projected.rows(); ++i) {
    float u = projected(i, 0);
    float v = projected(i, 1);
    if(u < min_u)
      min_u = u;
    if(u > max_u)
      max_u = u;
    if(v < min_v)
      min_v = v;
    if(v > max_v)
      max_v = v;
  }

  // -- Translate to coordinate system where (0,0) is the upper right of the image.
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) -= min_u;
    projected(i, 1) = max_v - projected(i, 1);
  }
  
  // -- Get the max depth.
  float max_depth = -FLT_MAX;
  float min_depth = FLT_MAX;
  for(int i=0; i<oriented.rows(); ++i) {
    if(oriented(i, axis_of_projection_) > max_depth)
      max_depth = oriented(i, axis_of_projection_);
    if(oriented(i, axis_of_projection_) < min_depth)
      min_depth = oriented(i, axis_of_projection_);
  }

  // -- Compute the normalized depths.  Depths are between 0 and 1, with 1 meaning closest and 0 meaning furthest.
  VectorXf depths = oriented.col(axis_of_projection_);
  if(axis_of_projection_ == 1)
    depths = -depths;
  depths = depths.cwise() - depths.minCoeff();
  depths = depths / depths.maxCoeff();
  
      
  
  // -- Fill the IplImages.
  assert(sizeof(float) == 4);
  CvSize size = cvSize(ceil(max_u - min_u), ceil(max_v - min_v));
  IplImage* acc = cvCreateImage(size, IPL_DEPTH_32F, 1);
  IplImage* intensity = cvCreateImage(size, IPL_DEPTH_32F, 1);
  IplImage* depth = cvCreateImage(size, IPL_DEPTH_32F, 1);
  cvSetZero(acc);
  cvSetZero(depth);
  cvSetZero(intensity);
  assert(projected.rows() == (int)interest_region_indices.size());
  for(int i=0; i<projected.rows(); ++i) {
    int row = floor(projected(i, 1));
    int col = floor(projected(i, 0));

    // Update accumulator.
    assert(interest_region_indices[i] < (int)data.channels[0].values.size() && (int)interest_region_indices[i] >= 0);
    ((float*)(acc->imageData + row * acc->widthStep))[col]++;

    // Add to intensity values.
    float val = (float)data.channels[0].values[interest_region_indices[i]] / 255.0 * (3.0 / 4.0) + 0.25;
    assert(val <= 1.0 && val >= 0.0);
    ((float*)(intensity->imageData + row * intensity->widthStep))[col] += val;

    // Add to depth values.
    ((float*)(depth->imageData + row * depth->widthStep))[col] += depths(i); //
  }
  
  // -- Normalize by the number of points falling in each pixel.
  for(int v=0; v<acc->height; ++v) {
    float* intensity_ptr = (float*)(intensity->imageData + v * intensity->widthStep);
    float* depth_ptr = (float*)(depth->imageData + v * depth->widthStep);
    float* acc_ptr = (float*)(acc->imageData + v * acc->widthStep);
    for(int u=0; u<acc->width; ++u) {
      if(*acc_ptr == 0) {
	*intensity_ptr = 0;
	*depth_ptr = 0;
      }
      else {
	*intensity_ptr = *intensity_ptr / *acc_ptr;
	*depth_ptr = *depth_ptr / *acc_ptr;
      }

      intensity_ptr++;
      depth_ptr++;
      acc_ptr++;
    }
  }

  // -- Store images.
  depth_projections_.push_back(depth);
  intensity_projections_.push_back(intensity);

  // -- Debugging.
  if(debug_) {
    float scale = 10;
    IplImage* intensity_big = cvCreateImage(cvSize(((float)intensity->width)*scale, ((float)intensity->height)*scale), intensity->depth, intensity->nChannels);
    cvResize(intensity, intensity_big, CV_INTER_AREA);

    IplImage* depth_big = cvCreateImage(cvSize(((float)depth->width)*scale, ((float)depth->height)*scale), depth->depth, depth->nChannels);
    cvResize(depth, depth_big, CV_INTER_AREA);

    CVSHOW("Intensity Image", intensity_big);
    CVSHOW("Depth Image", depth_big);
    cvWaitKey(0);
    cvDestroyWindow("Intensity Image");
    cvDestroyWindow("Depth Image");
  }
  
  // -- Clean up.
  cvReleaseImage(&acc);
}


D3DCloudOrienter::D3DCloudOrienter() :
  computed_(false)
{
}

void D3DCloudOrienter::clear() {
  oriented_clouds_.clear();
  computed_ = false;
}

std::string D3DCloudOrienter::getName() {
  ostringstream oss;
  oss << "D3DCloudOrienter" << endl;
  return oss.str();
}

void D3DCloudOrienter::orientClouds(const sensor_msgs::PointCloud& data,
				 const std::vector<const std::vector<int>*>& interest_region_indices)
{
  if(!oriented_clouds_.empty())
    cerr << "D3DCloudOrienter is orienting more clouds before old ones have been cleared!" << endl;
  for(size_t i=0; i<interest_region_indices.size(); ++i)
    oriented_clouds_.push_back(orientCloud(data, *interest_region_indices[i]));
  computed_ = true;
}


MatrixXf D3DCloudOrienter::orientCloud(const sensor_msgs::PointCloud& data,
				    const std::vector<int>& interest_region_indices)
{
  // -- Put cluster points into matrix form.
  MatrixXf points(interest_region_indices.size(), 3);
  for(size_t i=0; i<interest_region_indices.size(); ++i) {
    points(i, 0) = data.points[interest_region_indices[i]].x;
    points(i, 1) = data.points[interest_region_indices[i]].y;
    points(i, 2) = data.points[interest_region_indices[i]].z;
  }

  // -- Subtract off the mean of the points.
  VectorXf pt_mean = points.colwise().sum() / (float)points.rows();
  for(int i=0; i<points.rows(); ++i)
    points.row(i) -= pt_mean.transpose();

  // -- Flatten to z == 0.
  MatrixXf X = points;
  X.col(2) = VectorXf::Zero(X.rows());
  MatrixXf Xt = X.transpose();
  
  // -- Find the long axis.
  // Start with a random vector.
  VectorXf pc = VectorXf::Zero(3);
  pc(0) = 1; //Chosen by fair dice roll.
  pc(1) = 1;
  pc.normalize();
  
  // Power method.
  VectorXf prev = pc;
  double thresh = 1e-4;
  int ctr = 0;
  while(true) { 
    prev = pc;
    pc =  Xt * (X * pc);
    pc.normalize();
    ctr++;
    if((pc - prev).norm() < thresh)
      break;
  }
  assert(abs(pc(2)) < 1e-4);
  
  // -- Find the short axis.
  VectorXf shrt = VectorXf::Zero(3);
  shrt(1) = -pc(0);
  shrt(0) = pc(1);
  assert(abs(shrt.norm() - 1) < 1e-4);
  assert(abs(shrt.dot(pc)) < 1e-4);
  
  // -- Build the basis of normalized coordinates.
  MatrixXf basis = MatrixXf::Zero(3,3);
  basis.col(0) = pc;
  basis.col(1) = shrt;
  basis(2,2) = 1.0;
  assert(abs(basis.col(0).dot(basis.col(1))) < 1e-4);
  assert(abs(basis.col(0).norm() - 1) < 1e-4);
  assert(abs(basis.col(1).norm() - 1) < 1e-4);
  assert(abs(basis.col(2).norm() - 1) < 1e-4);

  // -- Rotate and return.
  MatrixXf oriented = points * basis;
  return oriented;
}


void CloudProjection::clearShared() {
  for(size_t i=0; i<imgs_.size(); ++i)
    cvReleaseImage(&imgs_[i]);
  imgs_.clear();
}
    
CloudProjection::CloudProjection(bool intensity, size_t axis,
				 int rows, int cols, float pixels_per_meter,
				 float u_offset_pct, float v_offset_pct) :
  intensity_(intensity),
  axis_(axis),
  rows_(rows),
  cols_(cols),
  pixels_per_meter_(pixels_per_meter),
  u_offset_pct_(u_offset_pct),
  v_offset_pct_(v_offset_pct)
{
  if(!intensity_) { 
    cerr << "intensity_ must be true." << endl;
    throw;
  }
  result_size_ = rows * cols;
  result_size_defined_ = true;
}

string CloudProjection::getName() const {
  ostringstream oss;
  oss << "CloudProjection_intensity" << intensity_ << "_axis" << axis_
      << "_rows" << rows_ << "_cols" << cols_
      << "_pixels_per_meter" << pixels_per_meter_
      << "_offset" << u_offset_pct_ << "x" << v_offset_pct_;
  return oss.str();
}

void CloudProjection::doComputation(const sensor_msgs::PointCloud& data,
				    cloud_kdtree::KdTree& data_kdtree,
				    const vector<const std::vector<int>*>& interest_region_indices,
				    vector<vector<float> >& results)
{
  assert(results.size() == interest_region_indices.size());
  assert(imgs_.size() == 0);
  results.resize(interest_region_indices.size());
  imgs_.resize(interest_region_indices.size());
    
  for(size_t i=0; i<interest_region_indices.size(); ++i) {
    imgs_[i] = computeProjection(data, *interest_region_indices[i]);
  }

  // -- Show the first image and pause if we are in debug mode.
  if(debug_) { 
    cout << "Displaying " << getName() << endl;
    IplImage* img = imgs_[0];
    float scale = 10;
    IplImage* img_big = cvCreateImage(cvSize(((float)img->width)*scale, ((float)img->height)*scale), img->depth, img->nChannels);
    cvResize(img, img_big, CV_INTER_AREA);

    IplImage* forsave = cvCloneImage(img_big);
    cvConvertScale(forsave, forsave, 255, 0);
    cvSaveImage((getName() + ".png").c_str(), forsave); //TODO: Remove this.
    cvReleaseImage(&forsave);
    
    cvNamedWindow("Intensity image");
    cvShowImage("Intensity image", img_big);
    cvWaitKey();
    cvDestroyWindow("Intensity image");
    cvReleaseImage(&img_big);
  }
  
  // -- Dump the images to results.
  for(size_t i=0; i<results.size(); ++i) {
    results[i].resize(result_size_);
    int idx = 0;
    for(int v=0; v<rows_; ++v) {
      float* img_ptr = (float*)(imgs_[i]->imageData + v * imgs_[i]->widthStep);
      for(int u=0; u<cols_; ++u) {
	results[i][idx] = *img_ptr;
	img_ptr++;
	idx++;
      }
    }
  }
}

IplImage* CloudProjection::computeProjection(const sensor_msgs::PointCloud& data,
					     const std::vector<int>& interest_region_indices)
{
  // -- Put cluster points into matrix form.
  MatrixXf points(interest_region_indices.size(), 3);
  for(size_t i=0; i<interest_region_indices.size(); ++i) {
    points(i, 0) = data.points[interest_region_indices[i]].x;
    points(i, 1) = data.points[interest_region_indices[i]].y;
    points(i, 2) = data.points[interest_region_indices[i]].z;
  }

  // -- Subtract off the mean and flatten to z=0 to prepare for PCA.
  MatrixXf X = points;
  X.col(2) = VectorXf::Zero(X.rows());
  VectorXf pt_mean = X.colwise().sum() / (float)X.rows();
  for(int i=0; i<X.rows(); ++i) {
    X.row(i) -= pt_mean.transpose();
  }
  MatrixXf Xt = X.transpose();
  
  // -- Find the long axis.
  // Start with a random vector.
  VectorXf pc = VectorXf::Zero(3);
  pc(0) = 1; //Chosen by fair dice roll.
  pc(1) = 1;
  pc.normalize();
  
  // Power method.
  VectorXf prev = pc;
  double thresh = 1e-4;
  int ctr = 0;
  while(true) { 
    prev = pc;
    pc =  Xt * (X * pc);
    pc.normalize();
    ctr++;
    if((pc - prev).norm() < thresh)
      break;
  }
  assert(abs(pc(2)) < 1e-4);
  
  // -- Find the short axis.
  VectorXf shrt = VectorXf::Zero(3);
  shrt(1) = -pc(0);
  shrt(0) = pc(1);
  assert(abs(shrt.norm() - 1) < 1e-4);
  assert(abs(shrt.dot(pc)) < 1e-4);
  
  // -- Build the basis of normalized coordinates.
  MatrixXf basis = MatrixXf::Zero(3,3);
  basis.col(0) = pc;
  basis.col(1) = shrt;
  basis(2,2) = -1.0;
  assert(abs(basis.col(0).dot(basis.col(1))) < 1e-4);
  assert(abs(basis.col(0).norm() - 1) < 1e-4);
  assert(abs(basis.col(1).norm() - 1) < 1e-4);
  assert(abs(basis.col(2).norm() - 1) < 1e-4);


  // -- Put the cluster into normalized coordinates, and choose which axis to project on.
  MatrixXf projected_basis(3, 2);
  if(axis_ == 0) { 
    projected_basis.col(0) = basis.col(1);
    projected_basis.col(1) = basis.col(2);
  }
  else if(axis_ == 1) { 
    projected_basis.col(0) = basis.col(0);
    projected_basis.col(1) = basis.col(2);
  }
  else if(axis_ == 2) { 
    projected_basis.col(0) = basis.col(0);
    projected_basis.col(1) = basis.col(1);
  }
  MatrixXf projected = points * projected_basis;
    
  // -- Transform into pixel units.
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) *= pixels_per_meter_;
    projected(i, 1) *= pixels_per_meter_;
  }

  // -- Find min and max of u and v.  TODO: noise sensitivity?
  float min_v = FLT_MAX;
  float min_u = FLT_MAX;
  float max_v = -FLT_MAX;
  float max_u = -FLT_MAX;
  for(int i=0; i<projected.rows(); ++i) {
    float u = projected(i, 0);
    float v = projected(i, 1);
    if(u < min_u)
      min_u = u;
    if(u > max_u)
      max_u = u;
    if(v < min_v)
      min_v = v;
    if(v > max_v)
      max_v = v;
  }

  // -- Shift the origin based on {u,v}_offset_pct. 
  //    u_offset_pct_ is the percent of the way from min_u to max_u that the
  //    u_offset should be set to.  If this makes the window fall outside min_u or max_u,
  //    then shift the window so that it is inside.
  float u_offset = u_offset_pct_ * (max_u - min_u) + min_u;
  float v_offset = v_offset_pct_ * (max_v - min_v) + min_v;

  if(u_offset_pct_ > 0.5 && u_offset + cols_ / 2 > max_u)
    u_offset = max_u - cols_ / 2 + 1;
  if(u_offset_pct_ < 0.5 && u_offset - cols_ / 2 < min_u)
    u_offset = min_u + cols_ / 2 - 1;

  if(v_offset_pct_ > 0.5 && v_offset + rows_ / 2 > max_v)
    v_offset = max_v - rows_ / 2 + 1;
  if(v_offset_pct_ < 0.5 && v_offset - rows_ / 2 < min_v)
    v_offset = min_v + rows_ / 2 - 1;

  
  for(int i=0; i<projected.rows(); ++i) {
    projected(i, 0) -= u_offset - (float)cols_ / 2.0;
    projected(i, 1) -= v_offset - (float)rows_ / 2.0;
  }
  
  // -- Fill the IplImages.
  assert(sizeof(float) == 4);
  IplImage* acc = cvCreateImage(cvSize(cols_, rows_), IPL_DEPTH_32F, 1);
  IplImage* img = cvCreateImage(cvSize(cols_, rows_), IPL_DEPTH_32F, 1);
  cvSetZero(acc);
  cvSetZero(img);
  for(int i=0; i<projected.rows(); ++i) {
    int row = floor(projected(i, 1));
    int col = floor(projected(i, 0));
    if(row >= rows_ || col >= cols_ || row < 0 || col < 0)
      continue;

    float intensity = (float)data.channels[0].values[interest_region_indices[i]] / 255.0 * (3.0 / 4.0) + 0.25;
    //cout << i << ": " << interest_region_indices[i] << "/" << data.channels[0].values.size() << " " << (float)data.channels[0].values[interest_region_indices[i]] << " " << intensity << endl;
    assert(interest_region_indices[i] < (int)data.channels[0].values.size() && (int)interest_region_indices[i] >= 0);
    assert(intensity <= 1.0 && intensity >= 0.0);
    ((float*)(img->imageData + row * img->widthStep))[col] += intensity;
    ((float*)(acc->imageData + row * acc->widthStep))[col]++;
  }
  
  // -- Normalize by the number of points falling in each pixel.
  for(int v=0; v<rows_; ++v) {
    float* img_ptr = (float*)(img->imageData + v * img->widthStep);
    float* acc_ptr = (float*)(acc->imageData + v * acc->widthStep);
    for(int u=0; u<cols_; ++u) {
      if(*acc_ptr == 0)
	*img_ptr = 0;
      else
	*img_ptr = *img_ptr / *acc_ptr;

      img_ptr++;
      acc_ptr++;
    }
  }

  // -- Clean up and return.
  cvReleaseImage(&acc);
  return img;  
}

void D3DRandomProjector::clearShared() {
  descriptor_->clearShared();
}

D3DRandomProjector::D3DRandomProjector(shared_ptr<Descriptor3D> desc, unsigned int dim, int seed) :
  descriptor_(desc),
  seed_(seed),
  mt_(std::tr1::mt19937(seed))
{
  // -- MT is working.
//   double accum = 0;
//   double accum2 = 0;
//   double num_samples = 100000;
//   for(int i=0; i<num_samples; ++i) {
//     double val = sampleFromGaussian(20, mt_);
//     accum += val;
//     accum2 += val * val;
//   }
//   cout << "Variance is " << accum2 / num_samples - (accum / num_samples) * (accum / num_samples) << endl;
  

  result_size_ = dim;
  result_size_defined_ = true;

  // -- Each entry in the projector is drawn from the standard normal.
  // http://books.google.com/books?id=6Ewlh_lNo4sC&lpg=PP9&ots=JrJ9sqV0a5&dq=random%20projection&lr=&pg=PA2#v=twopage&q=&f=false
  projector_ = MatrixXf::Zero(dim, descriptor_->getResultSize());
  srand(seed_);
  for(int i=0; i<projector_.rows(); ++i)
    for(int j=0; j<projector_.cols(); ++j)
      projector_(i,j) = sampleFromGaussian(1, mt_); //TODO: Use a better normal distribution sampler?
  mt_.seed(seed_);
}

string D3DRandomProjector::getName() const {
  ostringstream oss;
  oss << descriptor_->getName();
  oss << "_randomProjection_dim" << result_size_ << "_seed" << seed_;
  return oss.str();
}

void D3DRandomProjector::doComputation(const sensor_msgs::PointCloud& data,
				    cloud_kdtree::KdTree& data_kdtree,
				    const vector<const std::vector<int>*>& interest_region_indices,
				    vector<vector<float> >& results)
{
  assert(results.size() == interest_region_indices.size());
  vvf orig_results;
  descriptor_->compute(data, data_kdtree, interest_region_indices, orig_results); //TODO: Make descriptors cache their results.
  results = vvf(orig_results.size());
  for(size_t i=0; i<orig_results.size(); ++i) {
    if(orig_results[i].empty()) {
      results[i] = vector<float>(0);
      continue;
    }
      
    // -- Project the feature.
    VectorXf vec;
    floatToEigen(orig_results[i], &vec);
    VectorXf projected = projector_ * vec;

    // -- Put into results.
    results[i] = vector<float>(projected.rows());
    for(size_t j=0; j<results[i].size(); ++j) {
      results[i][j] = projected(j);
    }
  }
}


CloudProjectionHog::CloudProjectionHog(shared_ptr<CloudProjection> cp, shared_ptr<HogWrapper> hog,
				       int kernel_size_u, int kernel_size_v) :
  cp_(cp),
  hog_(hog),
  kernel_size_u_(kernel_size_u),
  kernel_size_v_(kernel_size_v)
{
  result_size_ = hog_->getSize();
  result_size_defined_ = true;
}

void CloudProjectionHog::clearShared()
{
  cp_->clearShared();
}

std::string CloudProjectionHog::getName() const
{
  ostringstream oss;
  oss << "CloudProjectionHog_kernel" << kernel_size_u_ << "x" << kernel_size_v_ << "_" << cp_->getName() << "_" << hog_->getName();
  return oss.str();
}

void CloudProjectionHog::doComputation(const sensor_msgs::PointCloud& data,
				       cloud_kdtree::KdTree& data_kdtree,
				       const vector<const std::vector<int>*>& interest_region_indices,
				       vector< vector<float> >& results)
{
  assert(results.size() == interest_region_indices.size());

  // -- Compute the Cloud Projections.
  cp_->compute(data, data_kdtree, interest_region_indices, results);
  assert(cp_->imgs_.size() == interest_region_indices.size());
  results.clear();
  results.resize(interest_region_indices.size());

  // -- Smooth the images.
  vector<IplImage*> smoothed(cp_->imgs_.size(), NULL);
  for(size_t i=0; i<smoothed.size(); ++i) {
    IplImage* src = cp_->imgs_[i];
    assert(src->nChannels == 1);
    IplImage* img = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, src->nChannels);
    cvConvertScale(src, img, 255);
    cvSmooth(img, img, CV_GAUSSIAN, kernel_size_u_, kernel_size_v_);
    //cvConvertScale(img, img, 2); //TODO: Get rid of this.2
    smoothed[i] = img;
  }
  
  // -- Compute the HOG descriptors for each image.
  for(size_t i=0; i<smoothed.size(); ++i) {

    // Choose a keypoint so that the whole intensity image is used.
    vector<cv::KeyPoint> kp(1);
    kp[0].pt.x = smoothed[i]->width / 2;
    kp[0].pt.y = smoothed[i]->height / 2;

    // Compute and copy in results.
    vvf tmp_res;
    hog_->compute(smoothed[i], kp, tmp_res);

    if(tmp_res[0].empty()) {
      results[i].clear();
      continue;
    }

    results[i].resize(result_size_);
    for(size_t j=0; j<tmp_res[0].size(); ++j)
      results[i][j] = tmp_res[0][j];
  }

  // -- Debugging
  if(debug_) {
    cout << "Displaying " << getName() << endl;

    float scale = 10;
    IplImage* vis = cvCreateImage(cvSize(smoothed[0]->width * scale, smoothed[0]->height * scale), smoothed[0]->depth, smoothed[0]->nChannels);
    cvResize(smoothed[0], vis, CV_INTER_NN);
    string savename = getName().substr(0, 250); // Crop the name, since cvSaveImage dies otherwise.
    cvSaveImage((savename + ".png").c_str(), vis); //TODO: Remove this.
    cvReleaseImage(&vis);

    
    //    cout << "Debugging " << result_size_ << " dim " << getName() << " feature." << endl;
//     cout << "Feature: ";
//     for(size_t i=0; i<results[0].size(); ++i)
//       cout << results[0][i] << " ";
//     cout << endl;
  }

  // -- Clean up.
  for(size_t i=0; i<smoothed.size(); ++i)
    cvReleaseImage(&smoothed[i]);
}
  
