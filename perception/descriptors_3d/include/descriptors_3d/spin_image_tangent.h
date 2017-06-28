#ifndef __D3D_SPIN_IMAGE_TANGENT_H__
#define __D3D_SPIN_IMAGE_TANGENT_H__
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

#include <descriptors_3d/generic/spin_image_generic.h>
#include <descriptors_3d/shared/spectral_analysis.h>

// --------------------------------------------------------------
/*!
 * \file spin_image_tangent.h
 *
 * \brief A spin image descriptor using local tangent as the
 *        spinning axis
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A SpinImageTangent descriptor computes a spin image spinning
 *        around the locally extracted tangent vector.
 *
 * The tangent vector is the "beta" axis as described in Johnson & Hebert 1999.
 *
 * Example spin image definition with 3 rows and 4 cols: \n
 *   beta                 \n
 *    ^                   \n
 *    |_ _ _ _            \n
 *    |_|_|_|_|           \n
 *    x_|_|_|_|           \n
 *    |_|_|_|_|           \n
 *    -----------> alpha  \n
 * (x = center point of spin image, beta = [tangent vector])
 *
 * The center point of the spin image is the given interest point or
 * the centroid of given regions of interest points
 */
// --------------------------------------------------------------
class SpinImageTangent: public SpinImageGeneric
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the spin image descriptor to use the given specifications
     *
     * \warning The number of rows (nbr_rows) must be odd
     *
     * \param row_res The cell resolution along the beta axis
     * \param col_res The cell resolution along the alpha axis
     * \param nbr_rows The number of cells along the beta axis
     * \param nbr_cols The number of cells along the alpha axis
     * \param use_interest_regions_only When computing for interest regions,
     *                                  true indicates to use only the points within
     *                                  the interest region to compute the spin image
     * \param spectral_information The class to retrieve the tangent vectors from for
     *                             each interest point/region
     */
    // --------------------------------------------------------------
    SpinImageTangent(const double row_res,
                     const double col_res,
                     const unsigned int nbr_rows,
                     const unsigned int nbr_cols,
                     const bool use_interest_regions_only,
                     SpectralAnalysis& spectral_information);

    // --------------------------------------------------------------
    /*!
     * \brief Clears any already-computed spectral information
     */
    // --------------------------------------------------------------
    virtual void clearShared();

    std::string getName() const;
    
  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes/retrieves the tangent for each interest point
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts  The list of interest points to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief Computes/retrieves the tangent for each interest region
     *
     * \param data The point cloud to process from Descriptor3D::compute()
     * \param data_kdtree The efficient neighborhood data structure
     * \param interest_pts  The list of interest points to be processed
     *
     * \return 0 on success, otherwise negative value on error
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const std::vector<int>*>& interest_region_indices);
  private:
    SpectralAnalysis* spectral_information_;
};

#endif
