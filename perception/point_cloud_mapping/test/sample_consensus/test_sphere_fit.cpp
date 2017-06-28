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
 * $Id: test_sphere_fit.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
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
#include <point_cloud_mapping/sample_consensus/sac_model_sphere.h>

using namespace sample_consensus;

TEST (LMedS, SACModelSphere)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 1.7068; points.points[0].y = 1.0684; points.points[0].z = 2.2147;
  points.points[1].x = 2.4708; points.points[1].y = 2.3081; points.points[1].z = 1.1736;
  points.points[2].x = 2.7609; points.points[2].y = 1.9095; points.points[2].z = 1.3574;
  points.points[3].x = 2.8016; points.points[3].y = 1.6704; points.points[3].z = 1.5009;
  points.points[4].x = 1.8517; points.points[4].y = 2.0276; points.points[4].z = 1.0112;
  points.points[5].x = 1.8726; points.points[5].y = 1.3539; points.points[5].z = 2.7523;
  points.points[6].x = 2.5179; points.points[6].y = 2.3218; points.points[6].z = 1.2074;
  points.points[7].x = 2.4026; points.points[7].y = 2.5114; points.points[7].z = 2.7588;
  points.points[8].x = 2.6999; points.points[8].y = 2.5606; points.points[8].z = 1.5571;
  points.points[9].x = 0;      points.points[9].y = 0;      points.points[9].z = 0;

  SACModel *model = new SACModelSphere ();
  SAC *sac        = new LMedS (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Sphere coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_NEAR (coeff[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff[3], 0.99, 1e-1);

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  EXPECT_NEAR (coeff_ref[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[3], 0.99, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RANSAC, SACModelSphere)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 1.7068; points.points[0].y = 1.0684; points.points[0].z = 2.2147;
  points.points[1].x = 2.4708; points.points[1].y = 2.3081; points.points[1].z = 1.1736;
  points.points[2].x = 2.7609; points.points[2].y = 1.9095; points.points[2].z = 1.3574;
  points.points[3].x = 2.8016; points.points[3].y = 1.6704; points.points[3].z = 1.5009;
  points.points[4].x = 1.8517; points.points[4].y = 2.0276; points.points[4].z = 1.0112;
  points.points[5].x = 1.8726; points.points[5].y = 1.3539; points.points[5].z = 2.7523;
  points.points[6].x = 2.5179; points.points[6].y = 2.3218; points.points[6].z = 1.2074;
  points.points[7].x = 2.4026; points.points[7].y = 2.5114; points.points[7].z = 2.7588;
  points.points[8].x = 2.6999; points.points[8].y = 2.5606; points.points[8].z = 1.5571;
  points.points[9].x = 0;      points.points[9].y = 0;      points.points[9].z = 0;

  SACModel *model = new SACModelSphere ();
  SAC *sac        = new RANSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Sphere coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_NEAR (coeff[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff[3], 0.99, 1e-1);

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  EXPECT_NEAR (coeff_ref[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[3], 0.99, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (MSAC, SACModelSphere)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 1.7068; points.points[0].y = 1.0684; points.points[0].z = 2.2147;
  points.points[1].x = 2.4708; points.points[1].y = 2.3081; points.points[1].z = 1.1736;
  points.points[2].x = 2.7609; points.points[2].y = 1.9095; points.points[2].z = 1.3574;
  points.points[3].x = 2.8016; points.points[3].y = 1.6704; points.points[3].z = 1.5009;
  points.points[4].x = 1.8517; points.points[4].y = 2.0276; points.points[4].z = 1.0112;
  points.points[5].x = 1.8726; points.points[5].y = 1.3539; points.points[5].z = 2.7523;
  points.points[6].x = 2.5179; points.points[6].y = 2.3218; points.points[6].z = 1.2074;
  points.points[7].x = 2.4026; points.points[7].y = 2.5114; points.points[7].z = 2.7588;
  points.points[8].x = 2.6999; points.points[8].y = 2.5606; points.points[8].z = 1.5571;
  points.points[9].x = 0;      points.points[9].y = 0;      points.points[9].z = 0;

  SACModel *model = new SACModelSphere ();
  SAC *sac        = new MSAC (model, 0.2);
  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Sphere coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_NEAR (coeff[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff[3], 0.99, 1e-1);

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  EXPECT_NEAR (coeff_ref[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[3], 0.99, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RRANSAC, SACModelSphere)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 1.7068; points.points[0].y = 1.0684; points.points[0].z = 2.2147;
  points.points[1].x = 2.4708; points.points[1].y = 2.3081; points.points[1].z = 1.1736;
  points.points[2].x = 2.7609; points.points[2].y = 1.9095; points.points[2].z = 1.3574;
  points.points[3].x = 2.8016; points.points[3].y = 1.6704; points.points[3].z = 1.5009;
  points.points[4].x = 1.8517; points.points[4].y = 2.0276; points.points[4].z = 1.0112;
  points.points[5].x = 1.8726; points.points[5].y = 1.3539; points.points[5].z = 2.7523;
  points.points[6].x = 2.5179; points.points[6].y = 2.3218; points.points[6].z = 1.2074;
  points.points[7].x = 2.4026; points.points[7].y = 2.5114; points.points[7].z = 2.7588;
  points.points[8].x = 2.6999; points.points[8].y = 2.5606; points.points[8].z = 1.5571;
  points.points[9].x = 0;      points.points[9].y = 0;      points.points[9].z = 0;

  SACModel *model = new SACModelSphere ();
  SAC *sac        = new RRANSAC (model, 0.2);
  reinterpret_cast<RRANSAC*>(sac)->setFractionNrPretest (50);

  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Sphere coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_NEAR (coeff[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff[3], 0.99, 1e-1);

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  EXPECT_NEAR (coeff_ref[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[3], 0.99, 1e-1);

  int nr_points_left = sac->removeInliers ();
  EXPECT_EQ (nr_points_left, 1);

  delete sac;
  delete model;
}

TEST (RMSAC, SACModelSphere)
{
  sensor_msgs::PointCloud points;
  points.points.resize (10);

  points.points[0].x = 1.7068; points.points[0].y = 1.0684; points.points[0].z = 2.2147;
  points.points[1].x = 2.4708; points.points[1].y = 2.3081; points.points[1].z = 1.1736;
  points.points[2].x = 2.7609; points.points[2].y = 1.9095; points.points[2].z = 1.3574;
  points.points[3].x = 2.8016; points.points[3].y = 1.6704; points.points[3].z = 1.5009;
  points.points[4].x = 1.8517; points.points[4].y = 2.0276; points.points[4].z = 1.0112;
  points.points[5].x = 1.8726; points.points[5].y = 1.3539; points.points[5].z = 2.7523;
  points.points[6].x = 2.5179; points.points[6].y = 2.3218; points.points[6].z = 1.2074;
  points.points[7].x = 2.4026; points.points[7].y = 2.5114; points.points[7].z = 2.7588;
  points.points[8].x = 2.6999; points.points[8].y = 2.5606; points.points[8].z = 1.5571;
  points.points[9].x = 0;      points.points[9].y = 0;      points.points[9].z = 0;

  SACModel *model = new SACModelSphere ();
  SAC *sac        = new RMSAC (model, 0.2);
  reinterpret_cast<RMSAC*>(sac)->setFractionNrPretest (50);

  model->setDataSet (&points);
  EXPECT_EQ ((int)model->getCloud ()->points.size (), 10);

  bool result = sac->computeModel ();
  EXPECT_EQ (result, true);

  std::vector<int> inliers = sac->getInliers ();
  EXPECT_EQ ((int)inliers.size (), 9);

  std::vector<double> coeff;
  sac->computeCoefficients (coeff);
  EXPECT_EQ ((int)coeff.size (), 4);
  //printf ("Sphere coefficients: %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3]);
  EXPECT_NEAR (coeff[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff[3], 0.99, 1e-1);

  std::vector<double> coeff_ref;
  sac->refineCoefficients (coeff_ref);
  EXPECT_EQ ((int)coeff_ref.size (), 4);
  EXPECT_NEAR (coeff_ref[0], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[1], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[2], 2.0, 1e-1);
  EXPECT_NEAR (coeff_ref[3], 0.99, 1e-1);

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
