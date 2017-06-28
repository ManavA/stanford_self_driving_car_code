/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <descriptors_3d/generic/spin_image_generic.h>

using namespace std;

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpinImageGeneric::SpinImageGeneric()
{
  spin_axes_ = NULL;
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
SpinImageGeneric::~SpinImageGeneric()
{
}

void SpinImageGeneric::display(const vector<float>& spin_image) const
{
  if(spin_image.empty()) {
    ROS_WARN("No spin image to display.");
    return;
  }
  
  assert(spin_image.size() == nbr_rows_ * nbr_cols_);

  CvSize sz = cvSize(nbr_cols_, nbr_rows_);
  IplImage* img = cvCreateImage(sz, IPL_DEPTH_8U, 1);
  
  for(int y=0; y<img->height; ++y) { 
    uchar* ptr = (uchar*)img->imageData + y * img->widthStep;
    for(int x=0; x<img->width; ++x, ++ptr) { 
      *ptr = spin_image[x + y * nbr_cols_] * 255;
    }
  }

  float scale = 40;
  IplImage* img_big = cvCreateImage(cvSize(((float)img->width)*scale, ((float)img->height)*scale), img->depth, img->nChannels);
  cvResize(img, img_big, CV_INTER_AREA);


  ostringstream oss;
  oss << nbr_rows_ * row_res_ << "x" << nbr_cols_ * col_res_ << " m, " << row_res_ << "x" << col_res_ << "pix/m" << endl;
  cvNamedWindow(oss.str().c_str());
  cvShowImage(oss.str().c_str(), img_big);
  cvWaitKey();
  cvDestroyWindow(oss.str().c_str());
  cvReleaseImage(&img);
  cvReleaseImage(&img_big);
}

// --------------------------------------------------------------
/* See function definition */
// --------------------------------------------------------------
void SpinImageGeneric::computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                                                  const std::vector<int>& neighbor_indices,
                                                  const unsigned int interest_sample_idx,
                                                  vector<float>& result) const
{
  const Eigen::Vector3d* curr_spin_axis = (*spin_axes_)[interest_sample_idx];
  if (curr_spin_axis == NULL)
  {
    ROS_DEBUG("SpinImageGeneric::computeNeighborhoodFeature() no spin axis for sample %u", interest_sample_idx);
    return;
  }

  // Clear out result for counting
  result.resize(result_size_);
  for (size_t i = 0 ; i < result_size_ ; i++)
  {
    result[i] = 0.0;
  }

  const Eigen::Vector3d& center_pt = spin_image_centers_[interest_sample_idx];

  // Offset into row number (center point is middle of the spin image)
  const unsigned int row_offset = nbr_rows_ / 2;

  float max_bin_count = 1.0; // init to 1.0 so avoid divide by 0 if no neighbor points
  unsigned int nbr_neighbors = neighbor_indices.size();
  for (unsigned int i = 0 ; i < nbr_neighbors ; i++)
  {
    // Create vector from center point to neighboring point
    const geometry_msgs::Point32& curr_neighbor_pt = data.points.at(neighbor_indices[i]);
    Eigen::Vector3d neighbor_vec;
    neighbor_vec[0] = curr_neighbor_pt.x - center_pt[0];
    neighbor_vec[1] = curr_neighbor_pt.y - center_pt[1];
    neighbor_vec[2] = curr_neighbor_pt.z - center_pt[2];
    const double neighbor_vec_norm = neighbor_vec.norm();

    // ----------------------------------------
    // Scalar projection of neighbor_vec onto spin axis (unit length)
    const double axis_projection = neighbor_vec_norm * neighbor_vec.dot(*curr_spin_axis);
    // Computed signed bin along the axis
    const int signed_row_nbr = static_cast<int> (floor(axis_projection / row_res_));
    // Offset the bin
    const int curr_row = signed_row_nbr + row_offset;

    // ----------------------------------------
    // Two vectors a and b originating from the same point form a parallelogram
    // |a x b| is the area Q of a parallelogram with base |a| and height h (Q = |a|h)
    // http://en.wikipedia.org/wiki/Cross_product
    // a = spin axis (unit length)
    // b = neighbor_vec
    // h = Q / |a| = |a x b| / |a| = |a x b|
    const unsigned int curr_col = static_cast<unsigned int> (floor((curr_spin_axis->cross(
        neighbor_vec)).norm() / col_res_));

    // Increment appropriate spin image cell.
    // First verify the point is contained in the image
    if (curr_row >= 0 && static_cast<unsigned int> (curr_row) < nbr_rows_ && curr_col < nbr_cols_)
    {
      // Compute vectorized cell number
      size_t cell_nbr = static_cast<size_t> (curr_row * nbr_cols_ + curr_col);
      result[cell_nbr] += 1.0;

      // Update counter for cell with most points
      if (result[cell_nbr] > max_bin_count)
      {
        max_bin_count = result[cell_nbr];
      }
    }
  }

  // Normalize all cells between 0 and 1
  for (size_t i = 0 ; i < result_size_ ; i++)
  {
    result[i] /= max_bin_count;
  }

  if(debug_)
    display(result);
}
