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
 * $Id: test_plane_fit.cpp 24267 2009-09-25 04:10:20Z gerkey $
 *
 */

/** \author Radu Bogdan Rusu */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud.h>

#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/rransac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/rmsac.h>
#include <point_cloud_mapping/sample_consensus/mlesac.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

using namespace sample_consensus;

TEST (LMedS, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new LMedS (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RANSAC, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new RANSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (MSAC, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new MSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (MLESAC, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new MLESAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RRANSAC, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new RRANSAC (model, 0.2);
  reinterpret_cast<RRANSAC*>(sac)->setFractionNrPretest (50);

  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RMSAC, SACModelPlane)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 0;      points.points[0].y = 1.5708; points.points[0].z = 0.75;
  points.points[1].x = 0;      points.points[1].y = 4.7124; points.points[1].z = 0.75;
  points.points[2].x = 0.7854; points.points[2].y = 1.5708; points.points[2].z = 0.75;
  points.points[3].x = 1.5708; points.points[3].y = 0;      points.points[3].z = 0.75;
  points.points[4].x = 1.5708; points.points[4].y = 1.5708; points.points[4].z = 0.75;
  points.points[5].x = 2.3562; points.points[5].y = 0;      points.points[5].z = 0.75;
  points.points[6].x = 2.3562; points.points[6].y = 3.1416; points.points[6].z = 0.75;
  points.points[7].x = 3.1416; points.points[7].y = 0;      points.points[7].z = 0.75;
  points.points[8].x = 3.1416; points.points[8].y = 4.7124; points.points[8].z = 0.75;
  points.points[9].x = 4;      points.points[9].y = 2;      points.points[9].z = 3;

  SACModel *model = new SACModelPlane ();
  SAC *sac        = new RMSAC (model, 0.2);
  reinterpret_cast<RMSAC*>(sac)->setFractionNrPretest (50);

  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  // Changing this EXPECT to an ASSERT because when it does fail, the test
  // subsequently crashes with a vector 'std::out_of_range' exception.
  //EXPECT_EQ (result, true);
  ASSERT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Plane coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  //printf ("Plane coefficients (refined): %f %f %f %f\n", coeff_ref[0], coeff_ref[1], coeff_ref[2], coeff_ref[3]);
  EXPECT_EQ (coeff[0], 0);
  EXPECT_EQ (coeff[1], 0);
  if (coeff[2] > 0)
  {
    EXPECT_EQ (coeff[2], 1);
    EXPECT_EQ (coeff[3], -0.75);
  }
  else
  {
    EXPECT_EQ (coeff[2], -1);
    EXPECT_EQ (coeff[3], 0.75);
  }

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

/* ---[ */
int
  main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
/* ]--- */
