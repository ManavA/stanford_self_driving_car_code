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


#ifndef __D3D_SPIN_IMAGE_GENERIC_H__
#define __D3D_SPIN_IMAGE_GENERIC_H__
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
#include <Eigen/Geometry>

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/cvaux.hpp>
#include <opencv/highgui.h>

#include <descriptors_3d/generic/neighborhood_feature.h>

// --------------------------------------------------------------
/*!
 * \file spin_image_generic.h
 *
 * \brief A SpinImageGeneric descriptor is the base abstract class for
 *        computing the feature described in: \n
 *        Johnson and Hebert, "Using Spin-Images for Efficient Object
 *        Recognition in Cluttered 3-D Scenes", PAMI 1999.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief SpinImageGeneric is the base class for descriptors that
 *        compute spin images as described in: \n
 *        Johnson and Hebert, "Using Spin-Images for Efficient Object
 *        Recognition in Cluttered 3-D Scenes", PAMI 1999.
 *
 * See the inheriting class' descriptions
 */
// --------------------------------------------------------------
class SpinImageGeneric: public NeighborhoodFeature
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor
     */
    // --------------------------------------------------------------
    SpinImageGeneric();

    virtual ~SpinImageGeneric() = 0;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the spin image descriptor for the given neighborhood of
     *        points
     *
     * \param data The overall point cloud data
     * \param neighbor_indices List of indices in data that constitute the neighborhood
     * \param interest_sample_idx The interest point/region that is being processed.
     * \param result The vector to hold the resulting spin image feature vector
     */
    // --------------------------------------------------------------
    virtual void
    computeNeighborhoodFeature(const sensor_msgs::PointCloud& data,
                               const std::vector<int>& neighbor_indices,
                               const unsigned int interest_sample_idx,
                               std::vector<float>& result) const;

    void display(const std::vector<float>& spin_image) const;

    /*! \brief The spinning (beta) axis for each interest point/region */
    const std::vector<const Eigen::Vector3d*>* spin_axes_;

    /*! \brief The point that the image is spinning around */
    std::vector<Eigen::Vector3d> spin_image_centers_;

    /*! \brief The cell resolution along the beta axis */
    double row_res_;

    /*! \brief The cell resolution along the alpha axis */
    double col_res_;

    /*! \brief The number of cells along the beta axis */
    unsigned int nbr_rows_;

    /*! \brief The number of cells along the alpha axis */
    unsigned int nbr_cols_;
};

#endif
