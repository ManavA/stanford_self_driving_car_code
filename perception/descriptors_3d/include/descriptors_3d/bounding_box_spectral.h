#ifndef __D3D_BOUNDING_BOX_SPECTRAL_H__
#define __D3D_BOUNDING_BOX_SPECTRAL_H__
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

#include <vector>

#include <Eigen/Core>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>

#include <descriptors_3d/generic/neighborhood_feature.h>
#include <descriptors_3d/shared/spectral_analysis.h>

// --------------------------------------------------------------
/*!
 * \file bounding_box_spectral.h
 *
 * \brief A BoundingBoxSpectral descriptor computes the dimensions of the
 *        3-D box that encloses a group of points in the neighborhood's
 *        principle component space
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A BoundingBoxSpectral descriptor to compute the dimensions
 *        of the bounding box in principle component space that
 *        encloses a neighborhood of points within a radius of
 *        the interest point/region.
 *
 * The compute features are in order: [a,b,c] where a is the length
 * along the principle eigenvector, b is the length along the middle
 * eigenvector, and c is the length along the smallest eigenvector.
 */
// --------------------------------------------------------------
class BoundingBoxSpectral: public NeighborhoodFeature
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the bounding box feature with the specified
     *        radius to define the neighborhood and spectral information
     *
     * When computing the feature for an interest region of points, the
     * bounding box can either be the box that encloses the given region
     * of points (indicated by -negative value), or from the neighboring points
     * within the specified radius from the region's centroid (indicated
     * by positive value).
     *
     * \param bbox_radius The radius from the interest point/region to define
     *                    the neighborhood that defines the bounding box
     * \param spectral_information Class to retrieve spectral information for the
     *                             point cloud during Descriptor3D::compute()
     */
    // --------------------------------------------------------------
    BoundingBoxSpectral(double bbox_radius, SpectralAnalysis& spectral_information);

    // --------------------------------------------------------------
    /*!
     * \brief Clears any already-computed spectral information
     */
    // --------------------------------------------------------------
    virtual void clearShared();

    // --------------------------------------------------------------
    /*!
     * \brief Returns a name that is unique for any given setting of the parameters.
     *        
     */
    // --------------------------------------------------------------
    std::string getName() const;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the spectral information (eigenvectors), necessary
     *        to project the points into principle component space
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts The list of interest points to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the spectral information (eigenvectors), necessary
     *        to project the points into principle component space
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts The list of interest regions to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const std::vector<int>*>& interest_region_indices);

    // --------------------------------------------------------------
    /*!
     * \brief Projects the given neighborhood in principle component space and then
     *        computes its bounding box
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param neighbor_indices The list of indices in data that constitute the neighborhood
     * \param interest_sample_idx The index of the interest point/region that is being
     *                            processed from Descriptor3D::compute()
     * \param result The vector to hold the computed bounding box dimensions
     */
    // --------------------------------------------------------------
    virtual void
    computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                               const std::vector<int>& neighbor_indices,
                               const unsigned int interest_sample_idx,
                               std::vector<float>& result) const;

  private:
    /*! \brief The smallest eigenvector for each interest point/region */
    const std::vector<const Eigen::Vector3d*>* eig_vecs_min_;

    /*! \brief The middle eigenvector for each interest point/region */
    const std::vector<const Eigen::Vector3d*>* eig_vecs_mid_;

    /*! \brief The biggest eigenvector for each interest point/region */
    const std::vector<const Eigen::Vector3d*>* eig_vecs_max_;

    /*! \brief The container the holds spectral information for the point cloud */
    SpectralAnalysis* spectral_information_;
};

#endif
