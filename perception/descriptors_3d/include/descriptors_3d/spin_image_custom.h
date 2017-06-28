#ifndef __D3D_SPIN_IMAGE_CUSTOM_H__
#define __D3D_SPIN_IMAGE_CUSTOM_H__
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

// --------------------------------------------------------------
/*!
 * \file spin_image_custom.h
 *
 * \brief A spin image descriptor with custom spinning axis
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A SpinImageCustom descriptor computes a spin image spinning
 *        around a custom reference direction/axis.
 *
 * The custom axis is the "beta" axis as described in Johnson & Hebert 1999.
 *
 * Example spin image definition with 3 rows and 4 cols: \n
 *   beta                 \n
 *    ^                   \n
 *    |_ _ _ _            \n
 *    |_|_|_|_|           \n
 *    x_|_|_|_|           \n
 *    |_|_|_|_|           \n
 *    -----------> alpha  \n
 * (x = center point of spin image, beta = [custom reference direction]
 *
 * The center point of the spin image is the given interest point or
 * the centroid of given regions of interest points
 */
// --------------------------------------------------------------
class SpinImageCustom: public SpinImageGeneric
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the spin image descriptor to use the given specifications
     *
     * \warning The number of rows (nbr_rows) must be odd
     *
     * \param ref_x The x dimension of the beta vector
     * \param ref_y The y dimension of the beta vector
     * \param ref_z The z dimension of the beta vector
     * \param row_res The cell resolution along the beta axis
     * \param col_res The cell resolution along the alpha axis
     * \param nbr_rows The number of cells along the beta axis
     * \param nbr_cols The number of cells along the alpha axis
     * \param use_interest_regions_only When computing for interest regions,
     *                                  true indicates to use only the points within
     *                                  the interest region to compute the spin image
     */
    // --------------------------------------------------------------
    SpinImageCustom(const double ref_x,
                    const double ref_y,
                    const double ref_z,
                    const double row_res,
                    const double col_res,
                    const unsigned int nbr_rows,
                    const unsigned int nbr_cols,
                    const bool use_interest_regions_only);

    // --------------------------------------------------------------
    /*!
     * \brief This descriptor uses no shared precomputation, so this method
     *        has no affect
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
     * \brief Defines the spin axis to be the custom direction for each interest point
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
     * \brief Defines the spin axis to be the custom direction for each interest region
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
    /*! \brief The custom spinning axis */
    Eigen::Vector3d custom_axis_;

    /*! \brief The custom axis duplicated for each interest point/region */
    std::vector<const Eigen::Vector3d*> custom_axis_duplicated_;
};

#endif
