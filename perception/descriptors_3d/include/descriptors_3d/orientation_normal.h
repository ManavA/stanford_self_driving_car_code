#ifndef __D3D_ORIENTATION_NORMAL_H__
#define __D3D_ORIENTATION_NORMAL_H__
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

#include <descriptors_3d/generic/orientation_generic.h>
#include <descriptors_3d/shared/spectral_analysis.h>

// --------------------------------------------------------------
/*!
 * \file orientation_normal.h
 *
 * \brief An Orientation descriptor using locally extracted normals as
 *        the local direction.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief An OrientationNormal descriptor uses extracted local
 *        normals around each interest point/region to use as the
 *        local directions.
 *
 * \warning This descriptor ignores the sign of the extracted normal and
 *          the computed feature is always between 0 and 1
 *
 * TODO: use sensor location so the extracted directions have meaningful signs
 */
// --------------------------------------------------------------
class OrientationNormal: public OrientationGeneric
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the orientation descriptor with given
     *        reference direction information and spectral information
     *
     * \param ref_x The x dimension of the reference direction
     * \param ref_y The y dimension of the reference direction
     * \param ref_z The z dimension of the reference direction
     * \param spectral_information The class to retrieve the normals from for
     *                             each interest point/region
     */
    // --------------------------------------------------------------
    OrientationNormal(const double ref_x,
                      const double ref_y,
                      const double ref_z,
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
     * \brief Extracts the normals around each interest point
     *
     * \see Descriptor3D::precompute()
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief Extracts the normals around each interest region
     *
     * \see Descriptor3D::precompute()
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const std::vector<int>*>& interest_region_indices);
  private:
    SpectralAnalysis* spectral_information_;
};

#endif
