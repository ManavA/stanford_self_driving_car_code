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

#ifndef CURVESMOOTHER_H
#define CURVESMOOTHER_H

#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <cmath>

#include <poly_traj.h>
#include <driving_common/Trajectory2D.h>

namespace vlr {

class CurveSmoother {

public:
  CurveSmoother();
  ~CurveSmoother();

public:
  class point2d {
  public:
    double x, y;
    const point2d operator+(const point2d& other) const {
      point2d res;
      res.x = x + other.x;
      res.y = y + other.y;
      return res;
    }

    const point2d operator-(const point2d& other) const {
      point2d res;
      res.x = x - other.x;
      res.y = y - other.y;
      return res;
    }

    const point2d operator*(double scalar) const {
      point2d res;
      res.x = scalar * x;
      res.y = scalar * y;
      return res;
    }
  };

public:
  void sample_as_bezier(int degree, int dense, std::vector<CurvePoint>& points, std::vector<CurvePoint>* splinepoints);

  bool clothoideSpline(const std::vector<CurvePoint>& points, double theta0, double kappa0, double s, int n_lookahead, std::vector<CurvePoint>& splinepoints);
  void sampleLinearEquidist(const std::vector<CurvePoint>& points, const std::vector<bool>& ignore, double point_dist, std::vector<CurvePoint>& linepoints);

  void bez_to_points(int degree, int npoints, const double* coeff, double* points);
  double curvatureAt0(const std::vector<CurvePoint>& p, size_t idx, int32_t degree);
  double BezierArcLength(point2d p1, point2d p2, point2d p3, point2d p4);
  void sampleCubicBezierEquidist(const std::vector<CurvePoint>& control_polygon, double point_dist, std::vector<CurvePoint>& sampled_points);

private:
  double curvature_0(double* bez_x, double* bez_y, int degree);
  void subdiv(int degree, double* coeff, double t, double* bleft, double* bright);
  double area(double* p1, double* p2, double* p3);

  void bezier2points(int degree, int l, const double* bez, int dense, double* points, int* point_num);
  double hornbez(int degree, const double* coeff, double t);
  point2d hornbez(int degree, const point2d* p, double t);

  void bezierCurvatures(double* bez_x, double* bez_y, int degree, int l, int dense, double* curvatures);

  void bezier_kappas(double* bez_x, double* bez_y, int degree, int l, int dense, double* curvatures);
  void derivative(double t, double* bez_x, double* bez_y, double& x, double& y) const;
  void derivative2(double t, double* bez_x, double* bez_y, double& x, double& y) const;

  void sampleLineLinearEquidist(std::vector<CurvePoint>::const_iterator pit0, std::vector<CurvePoint>::const_iterator pit1, double point_dist, std::vector<
      CurvePoint>& linepoints);

  void sampleCubicBezierPoints(int npoints, const point2d* control_polygon, double& s, std::vector<CurvePoint>& sampled_points);

  double simpson(double a, double b, int n_limit, double tolerance);
  double bezierArcLengthFunction(double t) {
    return sqrt(std::abs(q5_ + t * (q4_ + t * (q3_ + t * (q2_ + t * q1_)))));
  }

  static const unsigned int cs_temp_buf_size_ = 50000;

  double* xp_, *yp_;
  double* alpha_, *beta_, *gamma_;
  double* up_, *low_;
  double* ergx_, *ergy_, *curvatures_;
  double* bez_x_, *bez_y_;
  double* lookahead_x_, *lookahead_y_;
  CurvePoint cp_;

private:
  static const double ARC_LENGTH_TOLERANCE = 0.0000001; // Application specific tolerance
  double q1_, q2_, q3_, q4_, q5_; // These belong to balf()
};

} // namespace vlr

#endif // CURVESMOOTHER_H_
