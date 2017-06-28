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


#ifndef __D3D_SPECTRAL_ANALYSIS_H__
#define __D3D_SPECTRAL_ANALYSIS_H__
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

#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>
#include <point_cloud_mapping/geometry/nearest.h>

// --------------------------------------------------------------
/*!
 * \file spectral_analysis.h
 *
 * \brief An auxiliary class used by descriptors that require the results
 *        from spectral analysis of a given volume of points
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief A SpectralAnalysis performs eigen-analysis on neighborhoods
 *        of point clouds and saves the resulting eigenvectors and
 *        eigenvalues
 *
 * SpectralAnalysis does NOT inherit from Descriptor3D.  Instead,
 * this class is meant to hold intermediate computations to be shared
 * with requiring Descriptor3Ds that use spectral information such
 * that computation is not unnecessarily repeated.
 */
// --------------------------------------------------------------
class SpectralAnalysis
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Instantiates the class to perform eigen-analysis on
     *        neighborhoods of points around each interest point/region
     *        within the specified support radius.
     *
     * After instantiation, NO other methods need to be called before
     * passing this to a Descriptor3D for computation.
     *
     * TODO add sensor location (for aligning normals to viewpoint)
     *
     * \param support_radius The radius to define the local neighborhood around
     *                       each interest point/region.  For interest regions,
     *                       a negative value indicates to use the region itself
     *                       for spectral analysis
     */
    // --------------------------------------------------------------
    SpectralAnalysis(){}
    SpectralAnalysis(double support_radius);

    ~SpectralAnalysis();

    // --------------------------------------------------------------
    /*!
     * \brief Clears & frees previously computed spectral data
     *
     * This function should be called when calling Descriptor3D::compute()
     * on DIFFERENT point clouds.
     */
    // --------------------------------------------------------------
    void clearSpectral();

    // ===================================================================
    /*! \name Accessors */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief Returns flag if spectral information has been computed
     */
    // --------------------------------------------------------------
    inline bool isSpectralComputed() const
    {
      return spectral_computed_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved normals (smallest eigenvector)
     *        estimated for each interest point/region
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getNormals() const
    {
      return normals_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the support radius.
     */
    // --------------------------------------------------------------
    inline double getRadius() const
    {
      return support_radius_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved tangents (biggest eigenvector) estimated
     *        for each interest point/region
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getTangents() const
    {
      return tangents_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved middle/2nd eigenvector estimated for
     *        each interest ponit/region
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getMiddleEigenVectors() const
    {
      return middle_eig_vecs_;
    }

    // --------------------------------------------------------------
    /*!
     * \brief Returns the saved eigenvalues of the covariance matrix for each
     *        interest point/region
     */
    // --------------------------------------------------------------
    inline const std::vector<const Eigen::Vector3d*>& getEigenValues() const
    {
      return eigenvalues_;
    }
    //@}

    // --------------------------------------------------------------
    /*!
     * \brief Performs eigen-decomposition for each neighborhood around
     *        the interest points
     *
     * This method does NOT need to be called by the user before passing
     * to a Descriptor3D
     */
    // --------------------------------------------------------------
    int analyzeInterestPoints(const sensor_msgs::PointCloud& data,
                              cloud_kdtree::KdTree& data_kdtree,
                              const std::vector<const geometry_msgs::Point32*>& interest_pts);

    // --------------------------------------------------------------
    /*!
     * \brief Performs eigen-decomposition for each neighborhood in/around
     *        the interest regions
     *
     * This method does NOT need to be called by the user before passing
     * to a Descriptor3D
     */
    // --------------------------------------------------------------
    int analyzeInterestRegions(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices);

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Common method that computes the eigen-vector/values of the
     *        scattered matrix constructed from the given neighborhood
     */
    // --------------------------------------------------------------
    void computeSpectralInfo(const sensor_msgs::PointCloud& data,
                             const std::vector<int>& curr_region_indices,
                             const size_t idx);

    /*! \brief The radius used to define the local neighborhood */
    double support_radius_;

    /*! \brief Flag indicating if eigen-decomposition has been performed */
    bool spectral_computed_;

    /*! \brief The smallest extracted eigenvectors */
    std::vector<const Eigen::Vector3d*> normals_;

    /*! \brief The biggest extracted eigenvectors */
    std::vector<const Eigen::Vector3d*> tangents_;

    /*! \brief The 2nd/middle extracted eigenvectors */
    std::vector<const Eigen::Vector3d*> middle_eig_vecs_;

    /*! \brief The extracted eigenvalues */
    std::vector<const Eigen::Vector3d*> eigenvalues_;
};

#endif
