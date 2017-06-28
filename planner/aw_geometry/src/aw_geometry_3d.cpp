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


#include <iostream>
#include <Eigen/Dense>
#include <aw_geometry_3d.h>

using namespace Eigen;

namespace vlr {

//== CONSTANTS ========================================================
#define INSIDE  false
#define OUTSIDE true

// two vectors, a and b, starting from c
float dot(const Vector3f& a, const Vector3f& b) {
  return a.dot(b);
}

// two vectors, a and b
Vector3f cross(const Vector3f& a, const Vector3f& b) {
  return a.cross(b);
}

// two vectors, a and b, starting from c
Vector3f cross(const Vector3f& a, const Vector3f& b, const Vector3f& c) {
  float a0 = a(0) - c(0);
  float a1 = a(1) - c(1);
  float a2 = a(2) - c(2);
  float b0 = b(0) - c(0);
  float b1 = b(1) - c(1);
  float b2 = b(2) - c(2);
  return Vector3f(a1 * b2 - a2 * b1, a2 * b0 - a0 * b2, a0 * b1 - a1 * b0);
}

float dist2(const Vector3f& a, const Vector3f& b) {
  float x = a(0) - b(0);
  float y = a(1) - b(1);
  float z = a(2) - b(2);
  return x * x + y * y + z * z;
}

float dist(const Vector3f& a, const Vector3f& b) {
  return sqrtf(dist2(a, b));
}

// linear interpolation
Vector3f lerp(float t, const Vector3f& a, const Vector3f& b) {
  float v[3];
  float u = 1.0 - t;
  v[0] = u * a(0) + t * b(0);
  v[1] = u * a(1) + t * b(1);
  v[2] = u * a(2) + t * b(2);
  return Vector3f(v[0], v[1], v[2]);
}

// is the ball centered at b with radius r
// fully within the box centered at bc, with radius br?
bool ball_within_bounds(const Vector3f& b, float r, const Vector3f& bc, float br) {
  r -= br;
  if ((b(0) - bc(0) <= r) || (bc(0) - b(0) <= r) || (b(1) - bc(1) <= r) || (bc(1) - b(1) <= r) || (b(2) - bc(2) <= r) || (bc(2) - b(2) <= r)) return false;
  return true;
}

// is the ball centered at b with radius r
// fully within the box centered from min to max?
bool ball_within_bounds(const Vector3f& b, float r, const Vector3f& min, const Vector3f& max) {
  if ((b(0) - min(0) <= r) || (max(0) - b(0) <= r) || (b(1) - min(1) <= r) || (max(1) - b(1) <= r) || (b(2) - min(2) <= r) || (max(2) - b(2) <= r)) return false;
  return true;
}

// does the ball centered at b, with radius r,
// intersect the box centered at bc, with radius br?
bool bounds_overlap_ball(const Vector3f& b, float r, const Vector3f& bc, float br) {
  float sum = 0.0, tmp;
  if ((tmp = bc(0) - br - b(0)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  else if ((tmp = b(0) - (bc(0) + br)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  if ((tmp = bc(1) - br - b(1)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  else if ((tmp = b(1) - (bc(1) + br)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  if ((tmp = bc(2) - br - b(2)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  else if ((tmp = b(2) - (bc(2) + br)) > 0.0) {
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  return (sum < r * r);
}

bool bounds_overlap_ball(const Vector3f& b, float r, const Vector3f& min, const Vector3f& max) {
  float sum = 0.0, tmp;
  if (b(0) < min(0)) {
    tmp = min(0) - b(0);
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  else if (b(0) > max(0)) {
    tmp = b(0) - max(0);
    if (tmp > r) return false;
    sum += tmp * tmp;
  }
  if (b(1) < min(1)) {
    tmp = min(1) - b(1);
    sum += tmp * tmp;
  }
  else if (b(1) > max(1)) {
    tmp = b(1) - max(1);
    sum += tmp * tmp;
  }
  r *= r;
  if (sum > r) return false;
  if (b(2) < min(2)) {
    tmp = min(2) - b(2);
    sum += tmp * tmp;
  }
  else if (b(2) > max(2)) {
    tmp = b(2) - max(2);
    sum += tmp * tmp;
  }
  return (sum < r);
}

// calculate barycentric coordinates of the point p
// (already on the triangle plane) with normal vector n
// and two edge vectors v1 and v2,
// starting from a common vertex t0
void bary_fast(const Vector3f& p, const Vector3f& n, const Vector3f& t0, const Vector3f& v1, const Vector3f& v2, float &b1, float &b2, float &b3) {
  // see bary above
  int i = 0;
  if (n(1) > n(0)) i = 1;
  if (n(2) > n(i)) {
    // ignore z
    float d = 1.0 / (v1(0) * v2(1) - v1(1) * v2(0));
    float x0 = (p(0) - t0(0));
    float x1 = (p(1) - t0(1));
    b1 = (x0 * v2(1) - x1 * v2(0)) * d;
    b2 = (v1(0) * x1 - v1(1) * x0) * d;
  }
  else if (i == 0) {
    // ignore x
    float d = 1.0 / (v1(1) * v2(2) - v1(2) * v2(1));
    float x0 = (p(1) - t0(1));
    float x1 = (p(2) - t0(2));
    b1 = (x0 * v2(2) - x1 * v2(1)) * d;
    b2 = (v1(1) * x1 - v1(2) * x0) * d;
  }
  else {
    // ignore y
    float d = 1.0 / (v1(2) * v2(0) - v1(0) * v2(2));
    float x0 = (p(2) - t0(2));
    float x1 = (p(0) - t0(0));
    b1 = (x0 * v2(0) - x1 * v2(2)) * d;
    b2 = (v1(2) * x1 - v1(0) * x0) * d;
  }
  b3 = 1.0 - b1 - b2;
}

bool closer_on_lineseg(const Vector3f& x, Vector3f& cp, const Vector3f& a, const Vector3f& b, float &d2) {
  Vector3f ba(b(0) - a(0), b(1) - a(1), b(2) - a(2));
  Vector3f xa(x(0) - a(0), x(1) - a(1), x(2) - a(2));

  float xa_ba = dot(xa, ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba < 0.0) {
    float nd = dist2(x, a);
    if (nd < d2) {
      cp = a;
      d2 = nd;
      return true;
    }
    return false;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  float fact = xa_ba / ba.squaredNorm();
  if (fact >= 1.0) {
    float nd = dist2(x, b);
    if (nd < d2) {
      cp = b;
      d2 = nd;
      return true;
    }
    return false;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  float nd = xa.squaredNorm() - xa_ba * fact;
  if (nd < d2) {
    d2 = nd;
    cp(0) = a(0) + fact * ba(0);
    cp(1) = a(1) + fact * ba(1);
    cp(2) = a(2) + fact * ba(2);
    return true;
  }
  return false;
}

void distance_point_line(const Vector3f& x, const Vector3f& a, const Vector3f& b, float &d2, Vector3f& cp) {
  Vector3f ba(b(0) - a(0), b(1) - a(1), b(2) - a(2));
  Vector3f xa(x(0) - a(0), x(1) - a(1), x(2) - a(2));

  float xa_ba = dot(xa, ba);

  // if the dot product is negative, the point is closest to a
  if (xa_ba < 0.0) {
    d2 = dist2(x, a);
    cp = a;
    return;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  float fact = xa_ba / ba.squaredNorm();
  if (fact >= 1.0) {
    d2 = dist2(x, b);
    cp = b;
    return;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  d2 = xa.squaredNorm() - xa_ba * fact;
  cp(0) = a(0) + fact * ba(0);
  cp(1) = a(1) + fact * ba(1);
  cp(2) = a(2) + fact * ba(2);
  return;
}

void distance_point_tri(const Vector3f& x, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, float &d2, Vector3f& cp) {
  // calculate the normal and distance from the plane
  Vector3f v1(t2(0) - t1(0), t2(1) - t1(1), t2(2) - t1(2));
  Vector3f v2(t3(0) - t1(0), t3(1) - t1(1), t3(2) - t1(2));
  Vector3f n = cross(v1, v2);
  float n_inv_mag2 = 1.0 / n.squaredNorm();
  float tmp = (x(0) - t1(0)) * n(0) + (x(1) - t1(1)) * n(1) + (x(2) - t1(2)) * n(2);
  float distp2 = tmp * tmp * n_inv_mag2;

  // calculate the barycentric coordinates of the point
  // (projected onto tri plane) with respect to v123
  float b1, b2, b3;
  float f = tmp * n_inv_mag2;
  Vector3f pp(x(0) - f * n(0), x(1) - f * n(1), x(2) - f * n(2));
  bary_fast(pp, n, t1, v1, v2, b1, b2, b3);

  // all non-negative, the point is within the triangle
  if (b1 >= 0.0 && b2 >= 0.0 && b3 >= 0.0) {
    d2 = distp2;
    cp = pp;
    return;
  }

  // look at the signs of the barycentric coordinates
  // if there are two negative signs, the positive
  // one tells the vertex that's closest
  // if there's one negative sign, the opposite edge
  // (with endpoints) is closest

  if (b1 < 0.0) {
    if (b2 < 0.0) {
      d2 = dist2(x, t3);
      cp = t3;
    }
    else if (b3 < 0.0) {
      d2 = dist2(x, t2);
      cp = t2;
    }
    else {
      distance_point_line(x, t2, t3, d2, cp);
    }
  }
  else if (b2 < 0.0) {
    if (b3 < 0.0) {
      d2 = dist2(x, t1);
      cp = t1;
    }
    else {
      distance_point_line(x, t1, t3, d2, cp);
    }
  }
  else {
    distance_point_line(x, t1, t2, d2, cp);
  }
  return;
}

bool closer_on_tri(const Vector3f& x, Vector3f& cp, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, float &d2) {
  // calculate the normal and distance from the plane
  Vector3f v1(t2(0) - t1(0), t2(1) - t1(1), t2(2) - t1(2));
  Vector3f v2(t3(0) - t1(0), t3(1) - t1(1), t3(2) - t1(2));
  Vector3f n = cross(v1, v2);
  float n_inv_mag2 = 1.0 / n.squaredNorm();
  float tmp = (x(0) - t1(0)) * n(0) + (x(1) - t1(1)) * n(1) + (x(2) - t1(2)) * n(2);
  float distp2 = tmp * tmp * n_inv_mag2;
  if (distp2 >= d2) return false;

  // calculate the barycentric coordinates of the point
  // (projected onto tri plane) with respect to v123
  float b1, b2, b3;
  float f = tmp * n_inv_mag2;
  Vector3f pp(x(0) - f * n(0), x(1) - f * n(1), x(2) - f * n(2));
  bary_fast(pp, n, t1, v1, v2, b1, b2, b3);

  // all non-negative, the point is within the triangle
  if (b1 >= 0.0 && b2 >= 0.0 && b3 >= 0.0) {
    d2 = distp2;
    cp = pp;
    return true;
  }

  // look at the signs of the barycentric coordinates
  // if there are two negative signs, the positive
  // one tells the vertex that's closest
  // if there's one negative sign, the opposite edge
  // (with endpoints) is closest

  if (b1 < 0.0) {
    if (b2 < 0.0) {
      float nd = dist2(x, t3);
      if (nd < d2) {
        d2 = nd;
        cp = t3;
        return true;
      }
      else {
        return false;
      }
    }
    else if (b3 < 0.0) {
      float nd = dist2(x, t2);
      if (nd < d2) {
        d2 = nd;
        cp = t2;
        return true;
      }
      else {
        return false;
      }
    }
    else return closer_on_lineseg(x, cp, t2, t3, d2);
  }
  else if (b2 < 0.0) {
    if (b3 < 0.0) {
      float nd = dist2(x, t1);
      if (nd < d2) {
        d2 = nd;
        cp = t1;
        return true;
      }
      else {
        return false;
      }
    }
    else return closer_on_lineseg(x, cp, t1, t3, d2);
  }
  else return closer_on_lineseg(x, cp, t1, t2, d2);
}

// calculate the intersection of a line going through p
// to direction dir with a plane spanned by t1,t2,t3
// (modified from Graphics Gems, p.299)
bool line_plane_X(const Vector3f& p, const Vector3f& dir, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, Vector3f& x, float &dist) {
  // note: normal doesn't need to be unit vector
  Vector3f nrm = cross(t1, t2, t3);
  float tmp = dot(nrm, dir);
  if (tmp == 0.0) {
    std::cerr << "Cannot intersect plane with a parallel line" << std::endl;
    return false;
  }
  // d  = -dot(nrm,t1)
  // t  = - (d + dot(p,nrm))/dot(dir,nrm)
  // is = p + dir * t
  x = dir;
  dist = (dot(nrm, t1) - dot(nrm, p)) / tmp;
  x *= dist;
  x += p;
  if (dist < 0.0) dist = -dist;
  return true;
}

bool line_plane_X(const Vector3f& p, const Vector3f& dir, const Vector3f& nrm, float d, Vector3f& x, float &dist) {
  float tmp = dot(nrm, dir);
  if (tmp == 0.0) {
    std::cerr << "Cannot intersect plane with a parallel line" << std::endl;
    return false;
  }
  x = dir;
  dist = -(d + dot(nrm, p)) / tmp;
  x *= dist;
  x += p;
  if (dist < 0.0) dist = -dist;
  return true;
}

// calculate barycentric coordinates of the point p
// on triangle t1 t2 t3
void bary(const Vector3f& p, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, float &b1, float &b2, float &b3) {
  // figure out the plane onto which to project the vertices
  // by calculating a cross product and finding its largest dimension
  // then use Cramer's rule to calculate two of the
  // barycentric coordinates
  // e.g., if the z coordinate is ignored, and v1 = t1-t3, v2 = t2-t3
  // b1 = det(gx(0)g v2(0); x(1) v2(1)) / det(v1(0) v2(0); v1(1) v2(1))
  // b2 = det(gv1(0)g x(0); v1(1) x(1)) / det(v1(0) v2(0); v1(1) v2(1))
  float v10 = t1(0) - t3(0);
  float v11 = t1(1) - t3(1);
  float v12 = t1(2) - t3(2);
  float v20 = t2(0) - t3(0);
  float v21 = t2(1) - t3(1);
  float v22 = t2(2) - t3(2);
  float c[2];
  c[0] = fabs(v11 * v22 - v12 * v21);
  c[1] = fabs(v12 * v20 - v10 * v22);
  int i = 0;
  if (c[1] > c[0]) i = 1;
  if (fabs(v10 * v21 - v11 * v20) > c[i]) {
    // ignore z
    float d = 1.0f / (v10 * v21 - v11 * v20);
    float x0 = (p(0) - t3(0));
    float x1 = (p(1) - t3(1));
    b1 = (x0 * v21 - x1 * v20) * d;
    b2 = (v10 * x1 - v11 * x0) * d;
  }
  else if (i == 0) {
    // ignore x
    float d = 1.0f / (v11 * v22 - v12 * v21);
    float x0 = (p(1) - t3(1));
    float x1 = (p(2) - t3(2));
    b1 = (x0 * v22 - x1 * v21) * d;
    b2 = (v11 * x1 - v12 * x0) * d;
  }
  else {
    // ignore y
    float d = 1.0f / (v12 * v20 - v10 * v22);
    float x0 = (p(2) - t3(2));
    float x1 = (p(0) - t3(0));
    b1 = (x0 * v20 - x1 * v22) * d;
    b2 = (v12 * x1 - v10 * x0) * d;
  }
  b3 = 1.0f - b1 - b2;
}

// calculate barycentric coordinates for the intersection of
// a line starting from p, going to direction dir, and the plane
// of the triangle t1 t2 t3
bool bary(const Vector3f& p, const Vector3f& dir, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, float &b1, float &b2, float &b3) {
  Vector3f x;
  float d;
  if (!line_plane_X(p, dir, t1, t2, t3, x, d)) return false;
  bary(x, t1, t2, t3, b1, b2, b3);

  return true;
}

// calculate the intersection of a line starting from p,
// going to direction dir, and the triangle t1 t2 t3
bool line_tri_X(const Vector3f& p, const Vector3f& dir, const Vector3f& t1, const Vector3f& t2, const Vector3f& t3, Vector3f& x, float& d) {
  float b1, b2, b3;
  Vector3f x_temp;
  float d_temp;
  if (!line_plane_X(p, dir, t1, t2, t3, x_temp, d_temp)) return false;

  bary(x_temp, t1, t2, t3, b1, b2, b3);
  // all non-negative, the point is within the triangle
  if (b1 >= 0.0 && b2 >= 0.0 && b3 >= 0.0) {
    x = x_temp;
    d = d_temp;
    return true;
  }
  return false;
}

} // namespace vlr
