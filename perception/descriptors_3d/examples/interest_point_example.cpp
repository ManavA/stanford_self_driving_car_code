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

#include <stdlib.h>

#include <vector>
#include <iostream>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// include all descriptors for you
#include <descriptors_3d/all_descriptors.h>

using namespace std;

void createPointCloud(sensor_msgs::PointCloud& data);

// --------------------------------------------------------------
/*!
 * \brief Example program demonstrating how to use the desciptors_3d package.
 */
// --------------------------------------------------------------
int main()
{
  // ----------------------------------------------
  // Read point cloud data and create Kd-tree that represents points.
  // We will compute features for all points in the point cloud.
  sensor_msgs::PointCloud data;
  createPointCloud(data);
  cloud_kdtree::KdTreeANN data_kdtree(data);
  vector<const geometry_msgs::Point32*> interest_pts(data.points.size());
  for (size_t i = 0 ; i < data.points.size() ; i++)
  {
    interest_pts[i] = &(data.points[i]);
  }

  // ----------------------------------------------
  // SpectralAnalysis is not a descriptor, it is a class that holds
  // intermediate data to be used/shared by different descriptors.
  // It performs eigen-analyis of local neighborhoods and extracts
  // eigenvectors and values.  Here, we set it to look at neighborhoods
  // within a radius of 5.0 around each interest point.
  SpectralAnalysis sa(5.0);

  // ----------------------------------------------
  // Examines the eigenvalues for a local neighborhood scatter matrix
  // to give features that represent the local shape (how flat, how linear,
  // how scattered) of the neighborhood.
  // This descriptor uses information from spectral analysis.
  ShapeSpectral shape_spectral(sa);

  // ----------------------------------------------
  // Compute a spin image centered at each interest point.
  // The spin axis here is the +z direction (0,0,1).
  // The spin image resolution has 1.0 square cells.
  // Note that the number of rows in the spin image MUST be odd
  // (see documentation).
  // The last argument is used when computing spin image for
  // regions of points, since we are computing a descriptor
  // for single points it should be false.
  SpinImageCustom spin_image1(0, 0, 1, 1.0, 1.0, 5, 4, false);

  // ----------------------------------------------
  // Compute another spin image centered at each interest point.
  // The spin axis here is about each interest points' estimated normal.
  // We can either recompute the normals using a different radius than
  // used with spectral_shape (by instantiating another SpectralAnalysis),
  // or we can use the same normals computed from shape_spectral.
  SpinImageNormal spin_image2(1.0, 1.0, 5, 4, false, sa);

  // ----------------------------------------------
  // Compares the locally estimated normal and tangent vectors around
  // each interest point against the specified reference direction (+z).
  // The feature value is cos(theta), where theta is the angle between
  // the normal/tangent and the reference direction
  OrientationNormal o_normal(0, 0, 1, sa);
  OrientationTangent o_tangent(0, 0, 1, sa);

  // ----------------------------------------------
  // The feature is simply the z coordinate for each interest point
  Position position;

  // ----------------------------------------------
  // Computes the bounding box in the principle component space
  // of all points within radius 6.0.  That is, feature values
  // are the lengths of the box along the 3 component directions.
  BoundingBoxSpectral bbox_spectral(6.0, sa);

  // ----------------------------------------------
  // Put all descriptors into a vector
  vector<Descriptor3D*> descriptors_3d;
  descriptors_3d.push_back(&shape_spectral);
  descriptors_3d.push_back(&spin_image1);
  descriptors_3d.push_back(&spin_image2);
  descriptors_3d.push_back(&o_normal);
  descriptors_3d.push_back(&o_tangent);
  descriptors_3d.push_back(&position);
  descriptors_3d.push_back(&bbox_spectral);

  // ----------------------------------------------
  // Iterate over each descriptor and compute features for each point in the point cloud.
  // The compute() populates a vector of vector of floats, i.e. a feature vector for each
  // interest point.  If the features couldn't be computed successfully for an interest point,
  // its feature vector has size 0
  unsigned int nbr_descriptors = descriptors_3d.size();
  vector<vector<vector<float> > > all_descriptor_results(nbr_descriptors);
  for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
  {
    descriptors_3d[i]->compute(data, data_kdtree, interest_pts, all_descriptor_results[i]);
  }

  // ----------------------------------------------
  // Print out the bounding box dimension features for the first point 0
  vector<float>& pt0_bbox_features = all_descriptor_results[6][0];
  cout << "Bounding box features:";
  for (size_t i = 0 ; i < pt0_bbox_features.size() ; i++)
  {
    cout << " " << pt0_bbox_features[i];
  }
  cout << endl;
}

// Generates random point cloud
void createPointCloud(sensor_msgs::PointCloud& data)
{
  unsigned int nbr_pts = 5000;
  data.points.resize(nbr_pts);

  for (unsigned int i = 0 ; i < nbr_pts ; i++)
  {
    data.points[i].x = rand() % 50;
    data.points[i].y = rand() % 50;
    data.points[i].z = rand() % 50;
  }
}

