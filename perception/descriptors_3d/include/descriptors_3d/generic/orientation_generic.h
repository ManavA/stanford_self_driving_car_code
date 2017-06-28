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


#ifndef __D3D_ORIENTATION_GENERIC_H__
#define __D3D_ORIENTATION_GENERIC_H__
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

#include <descriptors_3d/descriptor_3d.h>

// --------------------------------------------------------------
/*!
 * \file orientation_generic.h
 *
 * \brief An OrientationGeneric descriptor computes the angle between a local
 *        direction (data dependent) and a specified reference direction.
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief OrientationGeneric is an abstract base class for descriptors
 *        that compute the angle between a locally extracted direction
 *        and a specified reference direction. the Orientation descriptor
 *
 * The computed feature is the cosine of the angle between the two vectors.
 * See the inheriting class for the definition of the extracted local
 * direction.
 */
// --------------------------------------------------------------
class OrientationGeneric: public Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor
     */
    // --------------------------------------------------------------
    OrientationGeneric();

    virtual ~OrientationGeneric() = 0;

  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Computes the angle between the local around each interest point
     *        and the reference direction
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const geometry_msgs::Point32*>& interest_pts,
                               std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the angle between the local around each interest region
     *        and the reference direction
     *
     * \see Descriptor3D::compute
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices,
                               std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes the angle for an interest point/region
     *
     * \param interest_sample_idx The interest point/region being processed
     * \param result Container to hold cosine(theta)
     */
    // --------------------------------------------------------------
    virtual void computeOrientation(const unsigned int interest_sample_idx,
                                    std::vector<float>& result) const;

    /*! \brief Container of the extracted local direction for each interest point/region */
    const std::vector<const Eigen::Vector3d*>* local_directions_;

    /*! \brief The reference direction to compare the local direction against */
    Eigen::Vector3d reference_direction_;

    /*! \brief The negative of reference_direction_ */
    Eigen::Vector3d reference_direction_flipped_;
};

#endif
