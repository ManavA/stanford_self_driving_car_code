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


#ifndef AW_GEOMETRY_3D_H
#define AW_GEOMETRY_3D_H

#include <Eigen/Dense>

namespace vlr {

// two vectors, a and b, starting from c
float dot(const Eigen::Vector3f &a, const Eigen::Vector3f &b);
// two vectors, a and b
Eigen::Vector3f cross(const Eigen::Vector3f &a, const Eigen::Vector3f &b);
// two vectors, a and b, starting from c
Eigen::Vector3f cross(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c);
float dist2(const Eigen::Vector3f &a, const Eigen::Vector3f &b);
float dist(const Eigen::Vector3f &a, const Eigen::Vector3f &b);
// linear interpolation
Eigen::Vector3f lerp(float t, const Eigen::Vector3f &a, const Eigen::Vector3f &b);
// is the ball centered at b with radius r
// fully within the box centered at bc, with radius br?
bool ball_within_bounds(const Eigen::Vector3f &b, float r,
                        const Eigen::Vector3f &bc, float br);
// is the ball centered at b with radius r
// fully within the box centered from min to max?
bool ball_within_bounds(const Eigen::Vector3f &b, float r,
                        const Eigen::Vector3f &min,
                        const Eigen::Vector3f &max);
// does the ball centered at b, with radius r,
// intersect the box centered at bc, with radius br?
bool bounds_overlap_ball(const Eigen::Vector3f &b, float r,
                         const Eigen::Vector3f &bc, float br);
bool bounds_overlap_ball(const Eigen::Vector3f &b, float r,
                         const Eigen::Vector3f &min, const Eigen::Vector3f &max);
// calculate barycentric coordinates of the point p
// (already on the triangle plane) with normal vector n
// and two edge vectors v1 and v2,
// starting from a common vertex t0
void bary_fast(const Eigen::Vector3f& p, const Eigen::Vector3f& n,
               const Eigen::Vector3f &t0, const Eigen::Vector3f& v1,
               const Eigen::Vector3f& v2, float &b1, float &b2, float &b3);
bool closer_on_lineseg(const Eigen::Vector3f &x, Eigen::Vector3f &cp, const Eigen::Vector3f &a,
                       const Eigen::Vector3f &b, float &d2);
void distance_point_line(const Eigen::Vector3f &x, const Eigen::Vector3f &a,
                         const Eigen::Vector3f &b, float &d2, Eigen::Vector3f &cp);
void distance_point_tri(const Eigen::Vector3f &x, const Eigen::Vector3f &t1,
                        const Eigen::Vector3f &t2, const Eigen::Vector3f &t3,
                        float &d2, Eigen::Vector3f &cp);
bool closer_on_tri(const Eigen::Vector3f &x, Eigen::Vector3f &cp,
                   const Eigen::Vector3f &t1, const Eigen::Vector3f &t2,
                   const Eigen::Vector3f &t3, float &d2);
// calculate the intersection of a line going through p
// to direction dir with a plane spanned by t1,t2,t3
// (modified from Graphics Gems, p.299)
bool line_plane_X(const Eigen::Vector3f& p, const Eigen::Vector3f& dir,
                  const Eigen::Vector3f& t1, const Eigen::Vector3f& t2,
                  const Eigen::Vector3f& t3,
                  Eigen::Vector3f &x, float &dist);
bool line_plane_X(const Eigen::Vector3f& p, const Eigen::Vector3f& dir,
                  const Eigen::Vector3f& nrm, float d, Eigen::Vector3f &x, float &dist);
// calculate barycentric coordinates of the point p
// on triangle t1 t2 t3
void bary(const Eigen::Vector3f& p,
          const Eigen::Vector3f& t1, const Eigen::Vector3f& t2, const Eigen::Vector3f& t3,
          float &b1, float &b2, float &b3);
// calculate barycentric coordinates for the intersection of
// a line starting from p, going to direction dir, and the plane
// of the triangle t1 t2 t3
bool bary(const Eigen::Vector3f& p, const Eigen::Vector3f& dir,
          const Eigen::Vector3f& t1, const Eigen::Vector3f& t2, const Eigen::Vector3f& t3,
          float &b1, float &b2, float &b3);
// calculate the intersection of a line starting from p,
// going to direction dir, and the triangle t1 t2 t3
bool line_tri_X(const Eigen::Vector3f& p, const Eigen::Vector3f& dir,
                const Eigen::Vector3f& t1, const Eigen::Vector3f& t2, const Eigen::Vector3f& t3,
                Eigen::Vector3f& x, float& d);

} // namespace vlr

#endif // AW_GEOMETRY_3D_H
