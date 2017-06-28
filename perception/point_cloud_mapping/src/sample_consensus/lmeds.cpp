/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: lmeds.cpp 16379 2009-05-29 19:20:46Z hsujohnhsu $
 *
 */

/** \author Radu Bogdan Rusu */

#include <algorithm>
#include <cfloat>
#include <point_cloud_mapping/sample_consensus/lmeds.h>

namespace sample_consensus
{
  ////////////////////////////////////////////////////////////////////////////////
  /** \brief LMedS (Least Medians of Squares) main constructor
    * \param model a Sample Consensus model
    * \param threshold distance to model threshold
    */
  LMedS::LMedS (SACModel *model, double threshold) : SAC (model)
  {
    this->threshold_ = threshold;
    // Maximum number of trials before we give up.
    this->max_iterations_ = 50;

    this->iterations_ = 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief LMedS (Least Medians of Squares) main constructor
    * \param model a Sample Consensus model
    */
  LMedS::LMedS (SACModel* model) : SAC (model) { }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Compute the actual model and find the inliers
    * \param debug enable/disable on-screen debug information
    */
  bool
    LMedS::computeModel (int debug)
  {
    iterations_ = 0;
    double d_best_penalty = DBL_MAX;

    std::vector<int> best_model;
    std::vector<int> best_inliers;
    std::vector<int> selection;
    std::vector<double> distances;

    // Iterate
    while (iterations_ < max_iterations_)
    {
      // Get X samples which satisfy the model criteria
      sac_model_->getSamples (iterations_, selection);

      if (selection.size () == 0) break;

      // Search for inliers in the point cloud for the current plane model M
      sac_model_->computeModelCoefficients (selection);

      double d_cur_penalty = 0;
      // d_cur_penalty = sum (min (dist, threshold))

      // Iterate through the 3d points and calculate the distances from them to the model
      sac_model_->getDistancesToModel (sac_model_->getModelCoefficients (), distances);
      if (distances.size () == 0)
      {
        iterations_ += 1;
        continue;
      }

      std::sort (distances.begin (), distances.end ());

      // d_cur_penalty = median (distances)
      int mid = sac_model_->getIndices ()->size () / 2;

      // Do we have a "middle" point or should we "estimate" one ?
      if (sac_model_->getIndices ()->size () % 2 == 0)
        d_cur_penalty = (sqrt (distances[mid-1]) + sqrt (distances[mid])) / 2;
      else
        d_cur_penalty = sqrt (distances[mid]);

      // Better match ?
      if (d_cur_penalty < d_best_penalty)
      {
        d_best_penalty = d_cur_penalty;
        best_model = selection;
      }

      iterations_ += 1;
      if (debug > 1)
        std::cerr << "[LMedS::computeModel] Trial " << iterations_ << " out of "<< max_iterations_ << ". Best penalty is : " << d_best_penalty << "." << std::endl;
    }

    if (best_model.size () != 0)
    {
      // Classify the data points into inliers and outliers
      // Sigma = 1.4826 * (1 + 5 / (n-d)) * sqrt (M)
      // NOTE: See "Robust Regression Methods for Computer Vision: A Review"
//      double sigma = 1.4826 * (1 + 5 / (sac_model_->getIndices ()->size () - best_model.size ())) * sqrt (d_best_penalty);
//      double threshold = 2.5 * sigma;

      sac_model_->computeModelCoefficients (best_model);
      // Iterate through the 3d points and calculate the distances from them to the model
      std::vector<double> distances;
      sac_model_->getDistancesToModel (sac_model_->getModelCoefficients (), distances);
      if (distances.size () == 0)
        ROS_WARN ("Distances to model _after_ model estimation have size 0!");

      best_inliers.resize (sac_model_->getIndices ()->size ());
      int n_inliers_count = 0;
      for (unsigned int i = 0; i < sac_model_->getIndices ()->size (); i++)
      {
//        if (distances[i] <= threshold)
        if (distances[i] <= threshold_)
        {
          best_inliers[n_inliers_count] = sac_model_->getIndices ()->at (i);
          n_inliers_count++;
        }
      }
      best_inliers.resize (n_inliers_count);

      if (debug > 0)
        std::cerr << "[LMedS::computeModel] Model found: " << n_inliers_count << " inliers" << std::endl;
      sac_model_->setBestModel (best_model);
      sac_model_->setBestInliers (best_inliers);
      return (true);
    }
    else
      if (debug > 0)
        std::cerr << "[LMedS::computeModel] Unable to find a solution!" << std::endl;
    return (false);
  }
}
