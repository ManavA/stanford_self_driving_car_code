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


#ifndef MATH_MISC_
#define MATH_MISC_

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <sstream>

#include <vlrException.h>
#include <sys/time.h>

namespace vlr {

 const double TWO_PI = 6.28318530717958647692;

  // a tiny value
 const double EPS = 1E-5;

  // Conversion factor for degrees to radians
 const double  deg2rad=0.0174532925199432957692;

// Conversion factor for radians to degrees
 const double  rad2deg=57.295779513082320876;

/// clamp the given variable x to passed range [a,b]
template <class T> inline T vlr_clamp(const T x, const T a, const T b) {
  if (x < a) return a;
  else return (x>b)?b:x;
}

/// returns true if the given variable x is in passed range [a,b]
template <class T> inline bool vlr_within(const T x, const T a, const T b) {
  return (x>=a && x<=b);
}

/// return the sign of a number
template <class T> inline T vlr_sign(const T x) {
  return ((x != T(0))? x/fabs(x) : T(1));
}

/// returns the integral digits of the specified number x
template <class T> inline int vlr_trunc(const T x) {
  return (int)x;
}

/// rounds a value to the nearest integer.
template <class T> inline int vlr_round(const T x) {
  if (x >= 0)
    return (int)(x + T(0.5));
  else
    return (int)(x - T(0.5));
}

template <class T> inline	T vlr_lerp(const T& t0, const T& t1, float a) {
  return t0 * (1.0f - a) + t1 * a;
}

/// return (a^2 + b^2)^(1/2) without over/underflow
template <class T> inline T vlr_pythag(T a, T b) {
  T absa=std::abs(a); T absb=std::abs(b);
  if (absa > absb) return absa*sqrt(1.0+(absb/absa)*(absb/absa));
  else return (absb==T(0) ? T(0) : absb*sqrt(T(1)+(absa/absb)*(absa/absb)));
}

/// square a number
template <class T> inline T vlr_sqr(const T x) { return x*x; }

/// cube a number
template <class T> inline T vlr_cube(const T x) { return x*x*x; }

/// take number to the 4th power
template <class T> inline T vlr_quart(const T x) { return x*x*x*x; }

/// take number to the 5th power
template <class T> inline T vlr_quint(const T x) { return x*x*x*x*x; }

/// a number^3/2
template <class T> inline T vlr_cuberoot(const T x) { return cube(sqrt(x)); }

/// return the maximum
template <class T> inline T vlr_max(const T x1, const T x2) {
  return (x1>x2)?x1:x2;
}

/// return the minimum
template <class T> inline T vlr_min(const T x1, const T x2) {
  return (x1<x2)?x1:x2;
}

/// swap values
template <class T> inline void vlr_swap(T& x1, T& x2) {
  T temp = x1; x1 = x2; x2 = temp;
}

/// compare floating point numbers (for sorts)
template <class T> inline int vlr_compare(const void * a, const void * b) {
  if (*(T*)a < *(T*)b) return -1;
  else if (*(T*)a == *(T*)b) return 0;
  else return 1;
}

// random number functions

/// Returns a sample from a normal distribution
template <class T> inline T vlr_normal_rand(T mean = T(0), T stdev = T(1)) {
  const double norm = 1.0/(RAND_MAX + 1.0);
  double u = 1.0 - std::rand()*norm;
  double v = std::rand()*norm;
  double z = std::sqrt(-2.0*log(u))*std::cos(2.0*M_PI*v);
  return T(mean + stdev*z);
}

/*
/// Returns a sample from a multivariate distribution
template <class T, int M> Vec<T,M> multivariateGauss(const Vec<T,M>& mean = Vec<T,M>(0),
                                                     const SMat<T,M>& cov = SMat<T,M>(1)) {
  Vec<T,M> v;
  SMat<T,M> S(cov);
  int n=S.choleskyDecomp();
  assert(n==0);
  S.transpose();
  v.normalRand();
  v = S*v;
  v += mean;
  return v;
}
*/

/// Returns a sample from a uniform distribution
template <class T> inline T vlr_uniform_rand(T lower = T(0), T upper = T(1)) {
  return lower + T(double(upper - lower)*std::rand()/(RAND_MAX+1.0));
}

/// Seed pseudo-random number generator
#ifdef _WIN32
inline void seedRand(void) {
  SYSTEMTIME SystemTime; GetSystemTime(&SystemTime);
unsigned int n = int(SystemTime.wSecond*1000000 + SystemTime.wMilliseconds); std::srand(n);
}
#else
inline void vlr_seed_rand(void) {
  timeval tv; gettimeofday(&tv,NULL);
  unsigned int n = int(tv.tv_sec*1000000 + tv.tv_usec); std::srand(n);
}
#endif

/// Absolute value
inline double vlr_abs(const double& x) { return std::fabs(x); }
inline float vlr_abs(const float& x) { return (float)std::fabs(static_cast<double>(x)); }
inline int vlr_abs(const int& x) { return std::abs(x); }
inline long int vlr_abs(const long int& x) { return std::labs(x); }

/// Square root
template <class T> inline T vlr_sqrt(const T& x) {
  return static_cast<T>(std::sqrt(static_cast<double>(x)));
}

/// x by the power of y
template <class T, class U> inline T vlr_pow(const T& x, const U& y) {
  return static_cast<T>(std::pow(static_cast<double>(x),static_cast<double>(y)));
}

/// x by the power of y
template<> inline float vlr_pow<float,int>(const float &x, const int &y)
{
  float ret=1;
  for (int i=0;i<y;i++) ret*=x;
  return ret;
}

template <class T> inline T vlr_safe_acos(T costheta)
{
  if (costheta>T(1)) costheta=T(1);
  if (costheta<T(-1)) costheta=T(-1);
  return (T)acos(double(costheta));
}

/// converts cartesian coordinates to spherical coordinates
template <class T> inline void vlr_cartesian_to_spherical(const T& x, const T& y, const T& z, T& r, T& theta, T& phi)
{
  r = sqrt(vlr_sqr(x)+vlr_sqr(y)+vlr_sqr(z));
  if(r<EPS) {
    r = theta = phi = 0;
  }
  else {
    theta = atan2(y,x);
    phi = acos(z/r);
  }
}

/// converts spherical coordinates to cartesian coordinates
template <class T> inline void vlr_spherical_to_cartesian(const T& r, const T& theta, const T& phi, T& x, T& y, T& z)
{
  float ct = cos(theta);
  float st = sin(theta);
  float sp = sin(phi);
  float cp = cos(phi);
  x = r*ct*sp;
  y = r*st*sp;
  z = r*cp;
}

/// normalizes angle to a [-PI,PI] interval
template <class T> inline T normalizeAngle(T angle) {

  if (angle >= -T(M_PI) && angle < T(M_PI)) {
    return angle;
  }

  int multiplier = (int)(angle / T(TWO_PI));
  angle = angle - multiplier*T(TWO_PI);
  if (angle >= T(M_PI)) {angle -= T(TWO_PI);}
  if (angle < -T(M_PI)) {angle += T(TWO_PI);}

  return angle;
}

template <typename float_t> inline float_t deltaAngle(const float_t& angle1, const float_t& angle2)
{
  float_t delta = normalizeAngle(angle2 - angle1);
  return delta > M_PI ? TWO_PI - delta : delta;
}

/// r in [0,inf), theta in [0,2PI), and phi in [0,PI],
template <class T> inline void vlr_normalize_spherical(T& theta, T& phi)
{
  int multiplier;

  // normalize phi
  if (phi < T(0) || phi > T(M_PI)) {
    multiplier = (int)(phi / T(TWO_PI));
    phi = phi - multiplier*T(TWO_PI);
    if (phi > T(M_PI)) {
      phi = T(TWO_PI)-phi;
      theta += T(M_PI);
    }
    else if (phi < T(0)) {
      phi*=-1;
      theta += T(M_PI);
    }
  }

  // normalize theta
  theta = vlr_normalize_theta(theta);
}

} // namespace vlr

#endif // _MATH_MISC_H_
