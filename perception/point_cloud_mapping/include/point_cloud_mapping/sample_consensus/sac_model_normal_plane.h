/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: sac_model_oriented_plane.h 10961 2009-02-11 00:20:50Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu, Jared Glover */

#ifndef _SAMPLE_CONSENSUS_SACMODELNORMALPLANE_H_
#define _SAMPLE_CONSENSUS_SACMODELNORMALPLANE_H_

#include <geometry_msgs/Point32.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>
#include <point_cloud_mapping/sample_consensus/model_types.h>

namespace sample_consensus
{
  /** \brief A Sample Consensus Model class for normal 3D plane segmentation.
    */
  class SACModelNormalPlane : public SACModelPlane
  {
    public:

      // Constructor
      SACModelNormalPlane()
	{
	  normal_distance_weight_ = 0.0;
	  eps_angle_ = 0.0;
	  eps_dist_ = 0.0;
	}

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the normal angular distance weight
        * \param w the relative weight (between 0 and 1) to give to the angular distance (0 to pi/2) between point normals and the plane normal.  (The Euclidean distance will have weight 1-w.)
        */
      void setNormalDistanceWeight (double w) { this->normal_distance_weight_ = w; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the axis along which we need to search for a plane perpendicular to
        * \param ax a pointer to the axis.
        */
      void setAxis (const geometry_msgs::Point32 &ax)
      {
        this->axis_.x = ax.x;
        this->axis_.y = ax.y;
        this->axis_.z = ax.z;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the angle epsilon (delta) threshold
        * \param ea the maximum allowed difference between the plane normal and the given axis
        */
      void setEpsAngle (double ea) { this->eps_angle_ = ea; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the distance we expect the plane to be from the origin
        * \param d distance from the template plane to the origin
        */
      void setDist (double d) { this->dist_ = d; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the distance epsilon (delta) threshold
        * \param delta the maximum allowed deviation from the template distance from the origin
        */
      void setEpsDist (double delta) { this->eps_dist_ = delta; }

      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (SACMODEL_NORMAL_PLANE). */
      virtual int getModelType () { return (SACMODEL_NORMAL_PLANE); }

    protected:
      double normal_distance_weight_;
      geometry_msgs::Point32 axis_;
      double dist_;
      double eps_angle_;
      double eps_dist_;
  };
}

#endif
