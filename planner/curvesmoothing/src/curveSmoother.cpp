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


#include <cmath>
#include <global.h>
#include <poly_traj.h>
#include <driving_common/Trajectory2D.h>

#include <curveSmoother.h>

namespace vlr {

const uint32_t CurveSmoother::cs_temp_buf_size_;

CurveSmoother::CurveSmoother() {
  xp_ = new double[cs_temp_buf_size_];
  yp_ = new double[cs_temp_buf_size_];
  alpha_ = new double[cs_temp_buf_size_];
  beta_ = new double[cs_temp_buf_size_];
  gamma_ = new double[cs_temp_buf_size_];
  up_ = new double[cs_temp_buf_size_];
  low_ = new double[cs_temp_buf_size_];
  ergx_ = new double[cs_temp_buf_size_];
  ergy_ = new double[cs_temp_buf_size_];
  curvatures_ = new double[cs_temp_buf_size_];
  bez_x_ = new double[cs_temp_buf_size_];
  bez_y_ = new double[cs_temp_buf_size_];
  lookahead_x_ = new double[cs_temp_buf_size_];
  lookahead_y_ = new double[cs_temp_buf_size_];

  cp_.kappa_prime = 0;
}

CurveSmoother::~CurveSmoother() {
  delete[] xp_;
  delete[] yp_;
  delete[] alpha_;
  delete[] beta_;
  delete[] gamma_;
  delete[] up_;
  delete[] low_;
  delete[] ergx_;
  delete[] ergy_;
  delete[] curvatures_;
  delete[] bez_x_;
  delete[] bez_y_;

}

void CurveSmoother::sample_as_bezier(int degree, int dense, std::vector<CurvePoint>& points, std::vector<CurvePoint>* splinepoints) {
  int l;
  uint32_t num_points, numResPoints;
  double* knots = NULL;

  num_points = std::min((uint32_t) ((points.size() >> 1) << 1), cs_temp_buf_size_ - 1);

  // TODO: 4 is only valid for cubic curves...
  if (num_points < 4) {
    printf("Not enough points (min 4)!\n");
    return;
  }

  // number of intervals
  l = (int) num_points - 1;
  splinepoints->clear();

  for (uint32_t i = 0; i < num_points; i++) {
    xp_[i] = points[i].x;
    yp_[i] = points[i].y;
  }

  l = (num_points - 1) / 3;

  bezier2points(degree, l, xp_, dense, ergx_, (int*) &numResPoints);
  bezier2points(degree, l, yp_, dense, ergy_, (int*) &numResPoints);

  bezierCurvatures(xp_, yp_, degree, l, dense, curvatures_);

  splinepoints->resize(numResPoints);

  static double s = 0;
  for (uint32_t i = 0; i < numResPoints; i++) {
    if (i == 0) {
      (*splinepoints)[i].s = 0;
    }
    else {
      s += hypot(ergx_[i] - ergx_[i - 1], ergy_[i] - ergy_[i - 1]);
      (*splinepoints)[i].s = s;
    }
    (*splinepoints)[i].x = ergx_[i];
    (*splinepoints)[i].y = ergy_[i];
    if (i == 0 || (ergy_[i] - ergy_[i - 1] < 0.0000001 && ergx_[i] - ergx_[i - 1] < 0.0000001 && i != num_points - 1)) {
      (*splinepoints)[i].theta = atan2(ergy_[i + 1] - ergy_[i], ergx_[i + 1] - ergx_[i]);
    }
    else {
      //      (*splinepoints)[i].theta = 0;
      (*splinepoints)[i].theta = atan2(ergy_[i] - ergy_[i - 1], ergx_[i] - ergx_[i - 1]);
    }
    (*splinepoints)[i].kappa = curvatures_[i];
  }

  free(knots);
}

void CurveSmoother::bezierCurvatures(double* bez_x, double* bez_y, int degree, int l, int dense, double* curvatures) {
  double bleftx[50], blefty[50], brightx[50], brighty[50];
  double coeffx[50], coeffy[50];

  int ldeg = degree * l;
  int num_curvatures = dense + 1;
  int curvature_index = 0;
  double* res_segment = curvatures;

  double delt = 1.0 / (double) dense;

  for (int i = 0; i < ldeg; i += degree) {

    for (int k = 0; k <= degree; k++) {
      coeffx[k] = bez_x[i + k];
      coeffy[k] = bez_y[i + k];
    }

    for (double t = 0.0; t < 0.5; t = t + delt) {
      subdiv(degree, coeffx, t, bleftx, brightx);
      subdiv(degree, coeffy, t, blefty, brighty);

      curvatures[curvature_index] = curvature_0(brightx, brighty, degree);
      curvature_index++;
    }

    for (double t = 0.5; t < 0.999; t = t + delt) {
      subdiv(degree, coeffx, t, bleftx, brightx);
      subdiv(degree, coeffy, t, blefty, brighty);

      /* minus sign since order of polygon
       traversal is reversed!
       */
      curvatures[curvature_index] = -curvature_0(bleftx, blefty, degree);
      curvature_index++;
    }
    res_segment += num_curvatures;
  }
}

// computes curvature of Bezier curve at t=0

double CurveSmoother::curvatureAt0(const std::vector<CurvePoint>& p, size_t idx, int32_t degree) {
  double b0[2], b1[2], b2[2];
  double dist;

  b0[0] = p[idx].x;
  b0[1] = p[idx].y;
  idx++;
  b1[0] = p[idx].x;
  b1[1] = p[idx].y;
  idx++;
  b2[0] = p[idx].x;
  b2[1] = p[idx].y;
  idx++;

  dist = sqrt((b1[0] - b0[0]) * (b1[0] - b0[0]) + (b1[1] - b0[1]) * (b1[1] - b0[1]));

  return (2.0 * (degree - 1) * area(b0, b1, b2) / (degree * dist * dist * dist));
}

double CurveSmoother::curvature_0(double* bez_x, double* bez_y, int degree) {
  double b0[2], b1[2], b2[2];
  double dist;

  b0[0] = bez_x[0];
  b1[0] = bez_x[1];
  b2[0] = bez_x[2];
  b0[1] = bez_y[0];
  b1[1] = bez_y[1];
  b2[1] = bez_y[2];

  dist = sqrt((b1[0] - b0[0]) * (b1[0] - b0[0]) + (b1[1] - b0[1]) * (b1[1] - b0[1]));

  return (2.0 * (degree - 1) * area(b0, b1, b2) / (degree * dist * dist * dist));
}

/*
 subdivides bezier curve at parameter value t.
 Output: left and right polygon with respective weights.
 Ordering of right polygon is reversed.
 */
void CurveSmoother::subdiv(int degree, double* coeff, double t, double* bleft, double* bright) {
  int r, i;
  double t1;

  t1 = 1.0 - t;

  /*
   first, obtain right subpolygon from rat de Casteljau
   */

  for (i = 0; i <= degree; i++)
    bright[i] = coeff[i];

  for (r = 1; r <= degree; r++)
    for (i = 0; i <= degree - r; i++) {
      bright[i] = (t1 * bright[i] + t * bright[i + 1]);
    }

  /*
   use same as above in order to get left half. Idea:
   reverse ordering; then the above yields left half.
   */

  t = 1.0 - t;
  t1 = 1.0 - t;
  for (i = 0; i <= degree; i++)
    bleft[degree - i] = coeff[i];

  for (r = 1; r <= degree; r++)
    for (i = 0; i <= degree - r; i++) {
      bleft[i] = (t1 * bleft[i] + t * bleft[i + 1]);
    }

}

void CurveSmoother::bezier2points(int degree, int l, const double* bez, int dense, double* points, int* point_num) {

  double aux[50];
  double res_tmp[1000];

  int ldeg = degree * l;
  int num_bez_points = dense + 1;
  double* res_segment = points;

  *point_num = 0;
  for (int i = 0; i < ldeg; i += degree) {

    memset(aux, 255, 50 * sizeof(double));

    for (int k = 0; k <= degree; k++) {
      aux[k] = bez[i + k];
    }

    memset(res_tmp, 255, 1000 * sizeof(double));

    bez_to_points(degree, dense, aux, res_tmp);
    memcpy(res_segment, res_tmp, num_bez_points * sizeof(double));

    *point_num += num_bez_points;
    res_segment += num_bez_points;
    //    bez_to_points(degree, dense, aux, res_segment);
    //    *point_num += num_bez_points;
    //    res_segment += num_bez_points;

    //    fprintf(psfile, "%f %f moveto\n", scale_x * (points_x[0] - value[0]), scale_y * (points_y[0] - value[2]));
    //
    //    for (j = 1; j <= dense; j++) {
    //      fprintf(psfile, "%f %f lineto\n", scale_x * (points_x[j] - value[0]), scale_y * (points_y[j] - value[2]));
    //    }
  }
}

// Converts Bezier curve into point sequence
//
// Input:   degree:  degree of curve.
// npoints: # of coordinates to be generated. (counting from 0!)
// coeff:   coordinates of control polygon.
// Output:  points:  coordinates of points on curve.

void CurveSmoother::bez_to_points(int degree, int npoints, const double* coeff, double* points) {
  int i;

  if (npoints < 2) return;
  npoints--;

  double delt = 1.0 / (double) npoints;
  double t = 0.0;

  for (i = 0; i <= npoints; i++) {
    points[i] = hornbez(degree, coeff, t);
    t = t + delt;
  }
}

// uses  Horner's scheme to compute one coordinate
// value of a  Bezier curve. Has to be called
// for each coordinate  (x,y, and/or z) of a control polygon.
// Input:   degree: degree of curve.
// coeff:  array with coefficients of curve.
// t:      parameter value.
// Output: coordinate value.

double CurveSmoother::hornbez(int degree, const double* coeff, double t) {
  int i;
  int n_choose_i;
  double fact, t1, aux;

  t1 = 1.0 - t;
  fact = 1.0;
  n_choose_i = 1;

  aux = coeff[0] * t1;
  for (i = 1; i < degree; i++) {
    fact = fact * t;
    n_choose_i = n_choose_i * (degree - i + 1) / i;
    aux = (aux + fact * n_choose_i * coeff[i]) * t1;
  }

  aux = aux + fact * t * coeff[degree];

  return aux;
}

CurveSmoother::point2d CurveSmoother::hornbez(int degree, const CurveSmoother::point2d* p, double t) {
  int i;
  int n_choose_i;
  double fact, t1;
  point2d aux;

  t1 = 1.0 - t;
  fact = 1.0;
  n_choose_i = 1;

  aux.x = p[0].x * t1;
  aux.y = p[0].y * t1;
  for (i = 1; i < degree; i++) {
    fact = fact * t;
    n_choose_i = n_choose_i * (degree - i + 1) / i;
    aux.x = (aux.x + fact * n_choose_i * p[i].x) * t1;
    aux.y = (aux.y + fact * n_choose_i * p[i].y) * t1;
  }

  aux.x = aux.x + fact * t * p[degree].x;
  aux.y = aux.y + fact * t * p[degree].y;

  return aux;
}

// find area of 2D triangle p1,p2,p3
double CurveSmoother::area(double* p1, double* p2, double* p3) {
  return ((p2[0] - p1[0]) * (p3[1] - p1[1]) - (p2[1] - p1[1]) * (p3[0] - p1[0])) / 2.0;
}

void CurveSmoother::derivative(double t, double* px, double* py, double& x, double& y) const {
  x = 3 * (px[3] - 3 * px[2] + 3 * px[1] - px[0]) * t * t + 2 * (3 * px[2] - 6 * px[1] + 3 * px[0]) * t + 3 * (px[1] - px[0]);
  y = 3 * (py[3] - 3 * py[2] + 3 * py[1] - py[0]) * t * t + 2 * (3 * py[2] - 6 * py[1] + 3 * py[0]) * t + 3 * (py[1] - py[0]);
}

void CurveSmoother::derivative2(double t, double* px, double* py, double& x, double& y) const {
  x = 6 * (px[3] - 3 * px[2] + 3 * px[1] - px[0]) * t + 2 * (3 * px[2] - 6 * px[1] + 3 * px[0]);
  y = 6 * (py[3] - 3 * py[2] + 3 * py[1] - py[0]) * t + 2 * (3 * py[2] - 6 * py[1] + 3 * py[0]);
}

inline void CurveSmoother::sampleLineLinearEquidist(std::vector<CurvePoint>::const_iterator pit0, std::vector<CurvePoint>::const_iterator pit1,
    double point_dist, std::vector<CurvePoint>& linepoints) {

  const double& x0 = (*pit0).x;
  const double& y0 = (*pit0).y;
  const double& x1 = (*pit1).x;
  const double& y1 = (*pit1).y;

  double len = hypot(x1 - x0, y1 - y0);

  // minimum is to return the input points
  uint32_t num_points = std::max(2u, uint32_t(len / point_dist + 1.5));

  double delta_x = (x1 - x0) / (num_points - 1);
  double delta_y = (y1 - y0) / (num_points - 1);

  //  double num_points_f = std::max(2., len / point_dist+1);
  //  double last_dist = hypot(x1 - (x0 + (num_points - 1) * delta_x), y1 - (y0 + (num_points - 1) * delta_y));
  //  double delta_s = hypot(delta_x, delta_y);
  //  printf("delta_s: %f, point_dist: %f, num_points: %u, num_points_f: %f, last_dist: %f\n", delta_s, point_dist, num_points, num_points_f, last_dist);

  for (uint32_t j = 0; j < num_points; j++) {
    cp_.x = x0 + j * delta_x;
    cp_.y = y0 + j * delta_y;
    linepoints.push_back(cp_);
  }
}

void CurveSmoother::sampleLinearEquidist(const std::vector<CurvePoint>& points, const std::vector<bool>& ignore, double point_dist,
    std::vector<CurvePoint>& linepoints) {
  linepoints.clear();
  if (points.size() < 2) {
    return;
  }

  std::vector<CurvePoint>::const_iterator pit0 = points.begin();
  std::vector<CurvePoint>::const_iterator pit1 = pit0 + 1;
  std::vector<bool>::const_iterator iit = ignore.begin();

  for (; pit1 != points.end(); pit0++, pit1++, iit++) {
    if (*iit) {
      linepoints.push_back((*pit0));
      continue;
    }
    sampleLineLinearEquidist(pit0, pit1, point_dist, linepoints);
    if ((pit1 + 1) != points.end()) {
      linepoints.pop_back();
    }
  }
}

void CurveSmoother::sampleCubicBezierPoints(int npoints, const point2d* control_polygon, double& s, std::vector<CurvePoint>& sampled_points) {
  int i;

  if (npoints < 2) {
    return;
  }
  npoints--;

  double delt = 1.0 / (double) npoints;
  double t = 0.0;

  double coeff_x[4], bleft_x[4], bright_x[4];
  double coeff_y[4], bleft_y[4], bright_y[4];

  coeff_x[0] = control_polygon[0].x;
  coeff_y[0] = control_polygon[0].y;
  coeff_x[1] = control_polygon[1].x;
  coeff_y[1] = control_polygon[1].y;
  coeff_x[2] = control_polygon[2].x;
  coeff_y[2] = control_polygon[2].y;
  coeff_x[3] = control_polygon[3].x;
  coeff_y[3] = control_polygon[3].y;

  CurvePoint cp;
  cp.kappa_prime = 0;

  double last_ds = 0;
  for (i = 0; i <= npoints; i++) {
    point2d point_sample = hornbez(3, control_polygon, t);

    cp.x = point_sample.x;
    cp.y = point_sample.y;

    // subdivide at t to calculate curvature
    subdiv(3, coeff_x, t, bleft_x, bright_x);
    subdiv(3, coeff_y, t, bleft_y, bright_y);

    if (t < 0.5) {
      cp.theta = atan2(bright_y[1] - bright_y[0], bright_x[1] - bright_x[0]);
      cp.kappa = curvature_0(bright_x, bright_y, 3);
    }
    else {
      // minus sign since order of polygon traversal is reversed!
      cp.theta = atan2(bleft_y[0] - bleft_y[1], bleft_x[0] - bleft_x[1]);
      cp.kappa = curvature_0(bleft_x, bleft_y, 3);
      cp.kappa = 0.0 - cp.kappa;
    }

    point2d p1, p2, p3, p4;
    p4.x = bleft_x[0];
    p4.y = bleft_y[0];
    p3.x = bleft_x[1];
    p3.y = bleft_y[1];
    p2.x = bleft_x[2];
    p2.y = bleft_y[2];
    p1.x = bleft_x[3];
    p1.y = bleft_y[3];

    //   printf("p4: (%f, %f) <-> cp: (%f, %f)\n", p4.x, p4.y, cp.x, cp.y);
    double ds = BezierArcLength(p1, p2, p3, p4);
    double dds = ds - last_ds;
    last_ds = ds;
    s += dds;
    cp.s = s;

    sampled_points.push_back(cp);
    t = t + delt;
  }
}

void CurveSmoother::sampleCubicBezierEquidist(const std::vector<CurvePoint>& control_polygon, double point_dist, std::vector<CurvePoint>& sampled_points) {

  double s = 0;
  point2d p[4];
  sampled_points.clear();

  for (uint32_t i3 = 0; i3 < (uint32_t) control_polygon.size() - 1; i3 += 3) {

    // get arc length of original Bezier interval to see how many points
    // we have to sample
    p[0].x = control_polygon[i3].x;
    p[0].y = control_polygon[i3].y;
    p[1].x = control_polygon[i3 + 1].x;
    p[1].y = control_polygon[i3 + 1].y;
    p[2].x = control_polygon[i3 + 2].x;
    p[2].y = control_polygon[i3 + 2].y;
    p[3].x = control_polygon[i3 + 3].x;
    p[3].y = control_polygon[i3 + 3].y;
    double ds = BezierArcLength(p[0], p[1], p[2], p[3]);

    // minimum is to return the input points
    uint32_t num_points = std::max(2u, uint32_t(ds / point_dist + 1.5));
    printf("i3: %u, num_points: %u\n", i3, num_points);
    sampleCubicBezierPoints(num_points - 1, p, s, sampled_points);

    // remove last point since it's too close to next interval's start point
    if (i3 != control_polygon.size() - 4) {
      sampled_points.pop_back();
      //     s = (*(--sampled_points.end())).s;
    }
  }
}

void nextPointCCS(double x_ccs, double kappa0, double dkappa, double& y_ccs, double& theta_ccs, double& kappa_ccs) {
  y_ccs = 0.5 * kappa0 * (x_ccs * x_ccs) + 1.0 / 6.0 * dkappa * (x_ccs * x_ccs * x_ccs); // f(x0_next)

  double slope = kappa0 * x_ccs + 0.5 * dkappa * (x_ccs * x_ccs);
  theta_ccs = atan(slope); // atan(dy/dx(x0_next))

  double term2 = 1.0 / (1 + slope * slope);
  double errorFactor = sqrt(term2 * term2 * term2); // 1/(1 + (kappa0*xNext) + .5*dkappa*xNext^2)^2)^(3/2)
  kappa_ccs = errorFactor * (dkappa * x_ccs + kappa0);
}

bool CurveSmoother::clothoideSpline(const std::vector<CurvePoint>& points_orig, double theta0_orig, double kappa0_orig, double s, int n_lookahead,
    std::vector<CurvePoint>& splinepoints) {
  double dkappa = 0;
  double x0_next_ccs, y0_next_ccs, theta0_next_ccs;
  double x0_next, y0_next, theta0_next, kappa0_next;
  double theta0, kappa0;

  int num_points = points_orig.size();

  if ((num_points <= 1) || (n_lookahead > num_points) || n_lookahead > (int) cs_temp_buf_size_) {
    return false;
  }

  std::vector<CurvePoint> points;// = points_orig;
  splinepoints = points_orig;

  for (int32_t bf = 0; bf <= 0; bf++) {

    int32_t istart, iend, istep;
    if (bf % 2) {
      istart = num_points - 2;
      iend = -1;
      istep = -1;
      theta0=splinepoints[splinepoints.size()-1].theta;
      kappa0=splinepoints[splinepoints.size()-1].kappa;
    }
    else {
      istart = 0;
      iend = num_points - 1;
      istep = 1;
      theta0 = theta0_orig;
      kappa0 = kappa0_orig;
   }

    points = splinepoints;
    splinepoints.clear();

    double x0 = points[istart].x;
    double y0 = points[istart].y;

    cp_.s = s;
    cp_.x = points[istart].x;
    cp_.y = points[istart].y;
    cp_.theta = normalizeAngle(theta0);
    cp_.kappa = kappa0;
    splinepoints.push_back(cp_);

    for (int32_t i = istart; i != iend; i += istep) {
      if(std::abs(iend - i) > n_lookahead-1) {
//      if (i <= num_points - n_lookahead - 1) {
        // Transform to CS of next point
        for (int32_t j = 0; j < n_lookahead; j++) {
          lookahead_x_[j] = cos(-theta0) * (points[i + istep + j*istep].x - x0) - sin(-theta0) * (points[i + istep + j*istep].y - y0);
          lookahead_y_[j] = sin(-theta0) * (points[i + istep + j*istep].x - x0) + cos(-theta0) * (points[i + istep + j*istep].y - y0);
        }

        // Least Squares

        // dkappa/ds = 6/sum(xi^6)*sum(xi^3*(yi-kappa0*xi^2/2))
        // sum xi^6
        double sum1 = 0, sum2 = 0;
        for (int32_t k = 0; k < n_lookahead; k++) {
          sum1 += lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k];
          sum2 += (lookahead_x_[k] * lookahead_x_[k] * lookahead_x_[k]) * (lookahead_y_[k] - (kappa0 * (lookahead_x_[k] * lookahead_x_[k]) / 2.));
        }

        dkappa = 6 / sum1 * sum2;
        x0_next_ccs = lookahead_x_[0];
      }
      else {
        x0_next_ccs = lookahead_x_[i - (num_points - n_lookahead) + 1]; // From main for
      }

      // Calculate smoothed values for next point

      // Calculate coordinates of next point in current CS
      nextPointCCS(x0_next_ccs, kappa0, dkappa, y0_next_ccs, theta0_next_ccs, kappa0_next);

      // Calculate coordinates of next point in original CS
      x0_next = (cos(theta0) * x0_next_ccs - sin(theta0) * y0_next_ccs) + x0; // Rotate then translate back
      y0_next = (sin(theta0) * x0_next_ccs + cos(theta0) * y0_next_ccs) + y0;
      theta0_next = theta0 + theta0_next_ccs;

        // Store values for next loop
      if(std::abs(iend - i) > n_lookahead-1) {
        x0 = x0_next;
        y0 = y0_next;
        theta0 = normalizeAngle(theta0_next);
        kappa0 = kappa0_next;
      }
      // Store values for return
      cp_.s += hypot(x0_next - cp_.x, y0_next - cp_.y);
      cp_.x = x0_next;
      cp_.y = y0_next;
      cp_.theta = normalizeAngle(theta0_next);
      cp_.kappa = kappa0_next;
      splinepoints.push_back(cp_);

      if(std::abs(iend - i) <= n_lookahead-1) {
        theta0_next = theta0;
        kappa0_next = kappa0;
        //        printf("%i. theta: %f; kappa: %f\n", i, theta0, kappa0);
      }
    }
  }

//  for (int32_t i = 0; i < (int32_t) splinepoints.size(); i++) {
//    CurvePoint& p = splinepoints[i];
//    printf("%i. %f %f %f %f %f\n", i, p.s, p.x, p.y, p.theta, p.kappa);
//  }

  return true;
}

//---------------------------------------------------------------------------
// NOTES:       TOLERANCE is a maximum error ratio
//                      if n_limit isn't a power of 2 it will be act like the next higher
//                      power of two.
//double CurveSmoother::simpson(double(*f)(double), double a, double b, int n_limit, double tolerance) {
double CurveSmoother::simpson(double a, double b, int n_limit, double tolerance) {
  int n = 1;
  double multiplier = (b - a) / 6.0;
  double endsum = bezierArcLengthFunction(a) + bezierArcLengthFunction(b);
  double interval = (b - a) / 2.0;
  double asum = 0;
  double bsum = bezierArcLengthFunction(a + interval);
  double est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  double est0 = 2 * est1;

  while (n < n_limit && (std::abs(est1) > 0 && std::abs((est1 - est0) / est1) > tolerance)) {
    n *= 2;
    multiplier /= 2;
    interval /= 2;
    asum += bsum;
    bsum = 0;
    est0 = est1;
    double interval_div_2n = interval / (2.0 * n);

    for (int i = 1; i < 2 * n; i += 2) {
      double t = a + i * interval_div_2n;
      bsum += bezierArcLengthFunction(t); //f(t);
    }

    est1 = multiplier * (endsum + 2 * asum + 4 * bsum);
  }

  return est1;
}

//
//---------------------------------------------------------------------------
//
double CurveSmoother::BezierArcLength(point2d p1, point2d p2, point2d p3, point2d p4) {
  point2d k1, k2, k3, k4;

  k1 = p1 * (-1) + (p2 - p3) * 3 + p4;
  k2 = (p1 + p3) * 3 - p2 * 6;
  k3 = (p2 - p1) * 3;
  k4 = p1;

  q1_ = 9.0 * (k1.x * k1.x + k1.y * k1.y);
  q2_ = 12.0 * (k1.x * k2.x + k1.y * k2.y);
  q3_ = 3.0 * (k1.x * k3.x + k1.y * k3.y) + 4.0 * (k2.x * k2.x + k2.y * k2.y);
  q4_ = 4.0 * (k2.x * k3.x + k2.y * k3.y);
  q5_ = k3.x * k3.x + k3.y * k3.y;

  return simpson(0, 1, 1024, 0.001);
}

} // namespace vlr

