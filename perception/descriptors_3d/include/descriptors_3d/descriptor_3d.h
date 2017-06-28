#ifndef __D3D_DESCRIPTORS_3D_H__
#define __D3D_DESCRIPTORS_3D_H__
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
#include <set>

#include <boost/shared_array.hpp>

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cvaux.hpp>

#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/kdtree/kdtree.h>

// --------------------------------------------------------------
/*!
 * \file descriptor_3d.h
 *
 * \brief The abstract base class for all feature descriptors
 *        that operate on 3-D data
 */
// --------------------------------------------------------------

// --------------------------------------------------------------
/*!
 * \brief Descriptor3D is the abstract base class for all descriptors
 *        that operate on 3-D data.  All inheriting classes define
 *        all necessary parameters in their constructor.  Hence,
 *        after instantiation, all descriptors can compute feature
 *        values through the compute() method.  The number of feature
 *        values the descriptor generates on success is given by
 *        getResultSize()
 */
// --------------------------------------------------------------
class Descriptor3D
{
  public:
    // --------------------------------------------------------------
    /*!
     * \brief Abstract constructor
     */
    // --------------------------------------------------------------
    Descriptor3D();

    virtual ~Descriptor3D() = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Clears the shared information this Descriptor3D is using
     *        so the information is computed from scratch on the next
     *        compute() call
     */
    // --------------------------------------------------------------
    virtual void clearShared() = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each specified interest point
     *
     * See the inherited class constructor for the type of features computed
     * and necessary parameters
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_pts List of interest points to compute features for
     * \param results Vector to hold computed vector of features for each interest point.
     *                If the features could not be computed for an interest point i, then
     *                results[i].size() == 0
     */
    // --------------------------------------------------------------
    void compute(const sensor_msgs::PointCloud& data,
                 cloud_kdtree::KdTree& data_kdtree,
                 const std::vector<const geometry_msgs::Point32*>& interest_pts,
                 std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Computes feature values for each interest region of points
     *
     * See the inherited class constructor for the type of features computed
     * and necessary parameters
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_region_indices List of groups of indices into data that represent an interest region
     * \param results Vector to hold computed vector of features for each interest region.
     *                If the features could not be computed for an interest region i, then
     *                results[i].size() == 0
     */
    // --------------------------------------------------------------
    void compute(const sensor_msgs::PointCloud& data,
                 cloud_kdtree::KdTree& data_kdtree,
                 const std::vector<const std::vector<int>*>& interest_region_indices,
                 std::vector<std::vector<float> >& results);

    // --------------------------------------------------------------
    /*!
     * \brief Returns the number of feature values this descriptor computes on success
     *
     * \return the number of feature values this descriptor computes on success
     */
    // --------------------------------------------------------------
    inline unsigned int getResultSize() const
    {
      return result_size_;
    }
    
    // --------------------------------------------------------------
    /*!
     * \brief Returns a string that is unique for the current param settings.
     *
     * \return the name of this feature.
     */
    // --------------------------------------------------------------
    virtual std::string getName() const = 0;
    
    // ===================================================================
    /*! \name Utility methods */
    // ===================================================================
    //@{
    // --------------------------------------------------------------
    /*!
     * \brief Utility function to compute multiple descriptor feature around
     *        interest points and concatenate the results into a single vector
     *
     * This method clears out any pre-computed shared information among the
     * descriptors before calculating the new features.
     *
     * \param data See Descriptor3D::compute
     * \param data_kdtree See Descriptor3D::compute
     * \param interest_pts See Descriptor3D::compute
     * \param descriptors_3d List of various feature descriptors to compute on each interest point
     * \param concatenated_features List containing the concatenated features from the descriptors if
     *                              they were ALL successful for the interest point.  If one descriptor
     *                              failed for interest point i, then concatenated_features[i].get() == NULL
     *
     * \return The total number of concatenated feature values
     */
    // --------------------------------------------------------------
    static unsigned int
    computeAndConcatFeatures(const sensor_msgs::PointCloud& data,
                             cloud_kdtree::KdTree& data_kdtree,
                             const std::vector<const geometry_msgs::Point32*>& interest_pts,
                             std::vector<Descriptor3D*>& descriptors_3d,
                             std::vector<boost::shared_array<const float> >& concatenated_features);
    // --------------------------------------------------------------
    /*!
     * \brief Utility function to compute multiple descriptor feature around
     *        interest regions and concatenate the results into a single vector
     *
     * This method clears out any pre-computed shared information among the
     * descriptors before calculating the new features.
     *
     * \param data See Descriptor3D::compute
     * \param data_kdtree See Descriptor3D::compute
     * \param interest_region_indices See Descriptor3D::compute
     * \param descriptors_3d List of various feature descriptors to compute on each interest point
     * \param concatenated_features List containing the concatenated features from the descriptors if
     *                              they were ALL successful for the interest region.  If one descriptor
     *                              failed for interest region i, then concatenated_features[i].get() == NULL
     *
     * \return The total number of concatenated feature values
     */
    // --------------------------------------------------------------
    static unsigned int
    computeAndConcatFeatures(const sensor_msgs::PointCloud& data,
                             cloud_kdtree::KdTree& data_kdtree,
                             const std::vector<const std::vector<int>*>& interest_region_indices,
                             std::vector<Descriptor3D*>& descriptors_3d,
                             std::vector<boost::shared_array<const float> >& concatenated_features);
    //@}

    //! Whether to display the features as they are computed.
    bool debug_;
  protected:
    // --------------------------------------------------------------
    /*!
     * \brief Method to do any setup pre-computation necessary for the
     *        inheriting descriptor.
     *
     *  Example: estimating normals for each interest point.
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_pts List of interest points for feature computation
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const geometry_msgs::Point32*>& interest_pts) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Method to do any setup pre-computation necessary for the
     *        inheriting descriptor.
     *
     *  Example: estimating normals for each interest point.
     *
     * \param data Point cloud of the data
     * \param data_kdtree K-D tree representation of data
     * \param interest_region_indices  List of groups of indices into data
     *                                 that represent an interest region for
     *                                 feature computation
     */
    // --------------------------------------------------------------
    virtual int precompute(const sensor_msgs::PointCloud& data,
                           cloud_kdtree::KdTree& data_kdtree,
                           const std::vector<const std::vector<int>*>& interest_region_indices) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Does the actual computation after any pre-computations
     *
     * \see Descriptor3D::compute()
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const geometry_msgs::Point32*>& interest_pts,
                               std::vector<std::vector<float> >& results) = 0;

    // --------------------------------------------------------------
    /*!
     * \brief Does the actual computation after any pre-computations
     *
     * \see Descriptor3D::compute()
     */
    // --------------------------------------------------------------
    virtual void doComputation(const sensor_msgs::PointCloud& data,
                               cloud_kdtree::KdTree& data_kdtree,
                               const std::vector<const std::vector<int>*>& interest_region_indices,
                               std::vector<std::vector<float> >& results) = 0;

    /*! \brief The number of feature values the inheriting descriptor computes on success */
    unsigned int result_size_;

    /*! \brief Flag if the inheriting descriptor has defined the result size */
    bool result_size_defined_;

  private:
    // --------------------------------------------------------------
    /*!
     * \brief Concatenates the resulting feature descriptor values
     *
     * \param all_descriptor_results The results for each descriptor
     * \param nbr_samples The number of interest points/regions
     * \param nbr_concatenated_vals The total length of all concatenated features from
     *                              each descriptor
     * \param concatenated_features The concatenated features.  NULL indicates could not
     *                              successfully compute descriptor for sample
     */
    // --------------------------------------------------------------
    static void
    concatenateFeatures(const std::vector<std::vector<std::vector<float> > >& all_descriptor_results,
                        const unsigned int nbr_samples,
                        const unsigned int nbr_concatenated_vals,
                        std::vector<boost::shared_array<const float> >& concatenated_features);
};

#endif
