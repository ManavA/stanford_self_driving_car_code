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


#include <assert.h>
#include <aw_geometry_2d.h>

using namespace Eigen;

namespace vlr {

#define INSIDE  false
#define OUTSIDE true

double dist2(const Vector2d& a, const Vector2d& b)
{
  double x = a(0)-b(0);
  double y = a(1)-b(1);
  return x*x + y*y;
}

bool closer_on_line(const Vector2d& x, const Vector2d& a, const Vector2d& b, double &d2, Vector2d& cp)
{
  Vector2d ba(b(0)-a(0), b(1)-a(1));
  Vector2d xa(x(0)-a(0), x(1)-a(1));

  double xa_ba = xa.dot(ba);
  // if the dot product is negative, the point is closest to a
  if (xa_ba < 0.0) {
    double nd = dist2(x,a);
    if (nd < d2) { cp = a; d2 = nd; return true; }
    return false;
  }

  // if the dot product is greater than squared segment length,
  // the point is closest to b
  double fact = xa_ba/ba.squaredNorm();
  if (fact >= 1.0) {
    float nd = dist2(x,b);
    if (nd < d2) { cp = b; d2 = nd; return true; }      return false;
  }

  // take the squared dist x-a, squared dot of x-a to unit b-a,
  // use Pythagoras' rule
  double nd = xa.squaredNorm() - xa_ba*fact;
  if (nd < d2) {
    d2 = nd;
    cp(0) = a(0) + fact * ba(0);
    cp(1) = a(1) + fact * ba(1);
    return true;
  }
  return false;
}


// is the circle centered at b with radius r
// fully within the rectangle centered at bc, with radius br?
bool circle_within_bounds(const Vector2d& b, double r, const Vector2d& bc, double br)
{
  r -= br;
  if ((b(0)  - bc(0) <= r) ||
      (bc(0) - b(0)  <= r) ||
      (b(1)  - bc(1) <= r) ||
      (bc(1) - b(1)  <= r)) return false;
  return true;
}

// is the circle centered at b with radius r
// fully within the rectangle centered from min to max?
bool circle_within_bounds(const Vector2d& b, double r, const Vector2d& min, const Vector2d& max)
{
  if ((b(0) - min(0) <= r) ||
    (max(0) - b(0) <= r) ||
    (b(1) - min(1) <= r) ||
    (max(1) - b(1) <= r)) return false;
  return true;
}

// does the circle centered at b, with radius r,
// intersect the rectangle centered at bc, with radius br?
bool bounds_overlap_circle(const Vector2d& b, double r, const Vector2d& bc, double br)
{
  double r2 = r*r;
  double R_max_x,R_max_y;
  double R_min_x,R_min_y;

  /* Translate coordinates, placing the circle at the origin. */
  R_max_x = bc(0)+br-b(0);  R_max_y = bc(1)+br-b(1);
  R_min_x = bc(0)-br-b(0);  R_min_y = bc(1)-br-b(1);

  if (R_max_x < 0)      /* R to left of circle center */
    if ( R_max_y < 0)     /* R in lower left corner */
        return ((R_max_x * R_max_x + R_max_y * R_max_y) < r2);
    else if ( R_min_y > 0)  /* R in upper left corner */
        return ((R_max_x * R_max_x +  R_min_y *  R_min_y) < r2);
    else          /* R due West of circle */
        return(abs(R_max_x) < r);
   else if ( R_min_x > 0)    /* R to right of circle center */
      if ( R_max_y < 0)   /* R in lower right corner */
          return (( R_min_x *  R_min_x + R_max_y * R_max_y) < r2);
    else if ( R_min_y > 0)    /* R in upper right corner */
        return (( R_min_x *  R_min_x +  R_min_y *  R_min_y) < r2);
    else        /* R due East of circle */
        return ( R_min_x < r);
   else        /* R on circle vertical centerline */
      if ( R_max_y < 0)   /* R due South of circle */
        return (abs(R_max_y) < r);
    else if ( R_min_y > 0)    /* R due North of circle */
        return ( R_min_y < r);
    else        /* R contains circle centerpoint */
        return(true);
}

//// does the circle centered at b, with radius r,
//// intersect the rectangle centered at bc, with radius br?
//bool bounds_overlap_circle(const Vector2d& b, double r, const Vector2d& bc, double br)
//  {
//    double sum = 0.0, tmp;
//    if        ((tmp = bc(0)-br - b(0)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    } else if ((tmp = b(0) - (bc(0)+br)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    }
//    if        ((tmp = bc(1)-br - b(1)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    } else if ((tmp = b(1) - (bc(1)+br)) > 0.0) {
//      if (tmp>r) return false; sum += tmp*tmp;
//    }
//    return (sum < r*r);
//  }

 /* Which of the four edges is point P outside of? */
long bevel_1d(const Vector2d& p)
{
  long outcode = 0;
  if (p(0) >  .5) outcode |= 0x01;
  if (p(0) < -.5) outcode |= 0x02;
  if (p(1) >  .5) outcode |= 0x04;
  if (p(1) < -.5) outcode |= 0x08;
  return outcode;
}

/* Which of the four corner lines is point P outside of? */
long bevel_2d(const Vector2d& p)
{
  long outcode = 0;
  if ( p(0) + p(1) > 1.0) outcode |= 0x01;
  if ( p(0) - p(1) > 1.0) outcode |= 0x02;
  if (-p(0) + p(1) > 1.0) outcode |= 0x04;
  if (-p(0) - p(1) > 1.0) outcode |= 0x08;
  return outcode;
}

// 2D linear interpolation
Vector2d lerp(double t, const Vector2d& a, const Vector2d& b)
{
  double v[2];
  double u = 1.0 - t;
  v[0]=u*a(0)+t*b(0);
  v[1]=u*a(1)+t*b(1);
  return Vector2d(v[0],v[1]);
}

/* Test the point "alpha" of the way from P1 to P2 */
/* See if it is on a edge of the rectangle         */
/* Consider only faces in "mask"                   */
bool point_on_edge(const Vector2d& p1, const Vector2d& p2, double alpha, long mask)
{
  Vector2d line_point;
  line_point = lerp(alpha, p1, p2);
  long l = bevel_1d(line_point) & mask;
  return (l==0?INSIDE:OUTSIDE);
}

/* Compute intersection of P1 --> P2 line segment with edge lines  */
/* Then test intersection point to see if it is on cube face       */
/* Consider only face planes in "outcode_diff"                     */
/* Note: Zero bits in "outcode_diff" means edge is outside of      */
bool segment_on_edge(const Vector2d& p1, const Vector2d& p2, long outcode_diff)
{
  if (0x01 & outcode_diff)
    if (point_on_edge(p1,p2,( .5-p1(0))/(p2(0)-p1(0)),0xE) == INSIDE) return INSIDE;
  if (0x02 & outcode_diff)
    if (point_on_edge(p1,p2,(-.5-p1(0))/(p2(0)-p1(0)),0xD) == INSIDE) return INSIDE;
  if (0x04 & outcode_diff)
    if (point_on_edge(p1,p2,( .5-p1(1))/(p2(1)-p1(1)),0xB) == INSIDE) return INSIDE;
  if (0x08 & outcode_diff)
    if (point_on_edge(p1,p2,(-.5-p1(1))/(p2(1)-p1(1)),0x7) == INSIDE) return INSIDE;
  return OUTSIDE;
}


// the main routine
// true if line t1,t2 is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
//
bool line_outside_of_rect(const Vector2d& c, double s, const Vector2d& t1, const Vector2d& t2)
{

  long v1_test,v2_test;

  // First compare both points tih all four rectangle edges
  // If any point is inside the rectangle, return immediately!
  Vector2d v1((t1(0)-c(0))/s, (t1(1)-c(1))/s);
  if (!(v1_test = bevel_1d(v1))) return INSIDE;
  Vector2d v2((t2(0)-c(0))/s, (t2(1)-c(1))/s);
  if (!(v2_test = bevel_1d(v2))) return INSIDE;
  // If both points were outside of one or more edges,
  // return immediately with a trivial rejection!
  if ((v1_test & v2_test) != 0) return OUTSIDE;

  // Now do the same trivial rejection test for the four corner lines
  v1_test |= bevel_2d(v1) << 8;
  v2_test |= bevel_2d(v2) << 8;
  if ((v1_test & v2_test) != 0) return OUTSIDE;

  /* If point 1 and 2, as a pair, cannot be trivially rejected    */
  /* by the above tests, then see if the v1-->v2 segment          */
  /* intersects the rectangle.                                    */
  /* Pass to the intersection algorithm the "OR" of the outcode   */
  /* bits, so that only those rectangle edges which are spanned   */
  /* by each triangle edge need be tested.                        */
  if (segment_on_edge(v1,v2,v1_test|v2_test) == INSIDE) return INSIDE;

//  if (point_on_edge(v1,v2,( .5-v1(0))/(v2(0)-v1(0))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,(-.5-v1(0))/(v2(0)-v1(0))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,( .5-v1(1))/(v2(1)-v1(1))) == INSIDE) return INSIDE;
//  if (point_on_edge(v1,v2,(-.5-v1(1))/(v2(1)-v1(1))) == INSIDE) return INSIDE;

//  /* No line touched the rectangle                                   */
//  /* We're done...there was no intersection.                         */
  return OUTSIDE;
}

// true if line p is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool point_outside_of_rect(const Vector2d& c, double s, const Vector2d& p)
{
  Vector2d v1((p(0)-c(0))/s, (p(1)-c(1))/s);
  if (!bevel_1d(v1)) return INSIDE;
  return OUTSIDE;
}

// true if rectangle 1 intersects rectangle 2
// c == center
// theta == angle
// w == width
// l == length
bool rect_rect_X(const Vector2d& c1, double theta1, double w1, double l1,
                 const Vector2d& c2, double theta2, double w2, double l2) {

  double x,y,sintheta2,costheta2;
  double tx, ty, scale_x, scale_y,sintheta1,costheta1;
  assert(l1);assert(w1);
  assert(l2);assert(w2);

  sintheta2 = sin(theta2);
  costheta2 = cos(theta2);

  // calculate the coordinates of all four corners of rectangle 2
  double rect2_x[4];
  double rect2_y[4];

  x = -l2/2.0; y = -w2/2.0;
  rect2_x[0] = costheta2*(x) - sintheta2*(y) + c2(0);
  rect2_y[0] = sintheta2*(x) + costheta2*(y) + c2(1);

  x = l2/2.0; y = -w2/2.0;
  rect2_x[1] = costheta2*(x) - sintheta2*(y) + c2(0);
  rect2_y[1] = sintheta2*(x) + costheta2*(y) + c2(1);

  x = l2/2.0; y = w2/2.0;
  rect2_x[2] = costheta2*(x) - sintheta2*(y) + c2(0);
  rect2_y[2] = sintheta2*(x) + costheta2*(y) + c2(1);

  x = -l2/2.0; y = w2/2.0;
  rect2_x[3] = costheta2*(x) - sintheta2*(y) + c2(0);
  rect2_y[3] = sintheta2*(x) + costheta2*(y) + c2(1);

  // normalize all coordinates so that rectangle 1 ist axis aligned and of length 1
  tx = c1(0);
  ty = c1(1);
  scale_x = l1;
  scale_y = w1;
  sintheta1 = sin(-theta1);
  costheta1 = cos(-theta1);

  for(int i=0;i<4;++i) {
    // transform coordinates of rectangle 2
    x = rect2_x[i] - tx;
    y = rect2_y[i] - ty;
    rect2_x[i] = (costheta1*(x) - sintheta1*(y))/scale_x;
    rect2_y[i] = (sintheta1*(x) + costheta1*(y))/scale_y;
  }

  Vector2d t0(rect2_x[0], rect2_y[0]);
  Vector2d t1(rect2_x[1], rect2_y[1]);
  Vector2d t2(rect2_x[2], rect2_y[2]);
  Vector2d t3(rect2_x[3], rect2_y[3]);

  // perform line rectangle intersection for all four edges of rectangle 2
  Vector2d c(0.0, 0.0);
  bool boutside;

  boutside = line_outside_of_rect(c, 1, t0, t1);
  if(!boutside) return true;

  boutside = line_outside_of_rect(c, 1, t1, t2);
  if(!boutside) return true;

  boutside = line_outside_of_rect(c, 1, t2, t3);
  if(!boutside) return true;

  boutside = line_outside_of_rect(c, 1, t3, t0);
  if(!boutside) return true;

  return false;
}

// true if line intersects rectangle
// line t1-->t2
// c == center
// theta == angle
// w == width
// l == length
bool line_rect_X(const Vector2d& t1, const Vector2d& t2,
                 const Vector2d& c, double theta, double w, double l) {
  double x,y,sintheta,costheta;
  double tx, ty, scale_x, scale_y;
  Vector2d t1n, t2n;
  assert(l);assert(w);

  // normalize all coordinates so that rectangle 1 ist axis aligned and of length 1
  tx = c(0);
  ty = c(1);
  scale_x = l;
  scale_y = w;
  sintheta = sin(-theta);
  costheta = cos(-theta);

  // transform coordinates of t1
  x = t1(0) - tx;
  y = t1(1) - ty;
  t1n(0) = (costheta*(x) - sintheta*(y)) / scale_x;
  t1n(1) = (sintheta*(x) + costheta*(y)) / scale_y;

  // transform coordinates of t2
  x = t2(0) - tx;
  y = t2(1) - ty;
  t2n(0) = (costheta*(x) - sintheta*(y)) / scale_x;
  t2n(1) = (sintheta*(x) + costheta*(y)) / scale_y;

  // perform line rectangle intersection for all four edges of rectangle 2
  Vector2d cn(0.0, 0.0);
  bool boutside = line_outside_of_rect(cn, 1, t1n, t2n);
  if(!boutside) return true;
  return false;
}

} // namespace vlr
