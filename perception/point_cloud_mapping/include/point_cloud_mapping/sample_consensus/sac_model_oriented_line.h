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
 * $Id: sac_model_oriented_line.h 20633 2009-08-04 07:19:09Z tfoote $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _SAMPLE_CONSENSUS_SACMODELORIENTEDLINE_H_
#define _SAMPLE_CONSENSUS_SACMODELORIENTEDLINE_H_

#include <geometry_msgs/Point32.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/sac_model_line.h>
#include <point_cloud_mapping/sample_consensus/model_types.h>

namespace sample_consensus
{
  /** \brief A Sample Consensus Model class for oriented 3D line segmentation.
    */
  class SACModelOrientedLine : public SACModelLine
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the axis along which we need to search for a line
        * \param ax a pointer to the axis
        */
      void
        setAxis (const geometry_msgs::Point32 &ax)
      {
        this->axis_.x = ax.x;
        this->axis_.y = ax.y;
        this->axis_.z = ax.z;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the angle epsilon (delta) threshold
        * \param ea the maximum allowed threshold between the line direction and the given axis
        */
      void setEpsAngle (double ea) { this->eps_angle_ = ea; }

      virtual void getDistancesToModel (const std::vector<double> &model_coefficients, std::vector<double> &distances);
      virtual void selectWithinDistance (const std::vector<double> &model_coefficients, double threshold, std::vector<int> &inliers);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return an unique id for this model (SACMODEL_ORIENTED_LINE). */
      virtual int getModelType () { return (SACMODEL_ORIENTED_LINE); }

    protected:
      geometry_msgs::Point32 axis_;
      double eps_angle_;
  };
}

#endif
