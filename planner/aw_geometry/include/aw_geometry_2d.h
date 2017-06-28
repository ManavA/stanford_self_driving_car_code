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


#ifndef AW_GEOMETRY_2D_H
#define AW_GEOMETRY_2D_H

#include <Eigen/Dense>

namespace vlr {

// squared euclidean distance
double dist2(const Eigen::Vector2d &a, const Eigen::Vector2d &b);
// Calculates the closest point to x on a line segmente a-->b
// if the distance to closest point is smaller than d2, both
// d2 and cp will be overwritten
bool closer_on_line(const Eigen::Vector2d &x, const Eigen::Vector2d &a, const Eigen::Vector2d &b, double &d2, Eigen::Vector2d &cp);
// is the circle centered at b with radius r
// fully within the rectangle centered at bc, with radius br?
bool circle_within_bounds(const Eigen::Vector2d &b, double r, const Eigen::Vector2d &bc, double br);
// is the circle centered at b with radius r
// fully within the rectangle centered from min to max?
bool circle_within_bounds(const Eigen::Vector2d &b, double r, const Eigen::Vector2d &min, const Eigen::Vector2d &max);
// does the circle centered at b, with radius r,
// intersect the rectangle centered at bc, with radius br?
bool bounds_overlap_circle(const Eigen::Vector2d &b, double r, const Eigen::Vector2d &bc, double br);
// Which of the four edges is point P outside of?
long bevel_1d(const Eigen::Vector2d &p);
// Which of the four edge lines is point P outside of?
long bevel_2d(const Eigen::Vector2d &p);
// 2D linear interpolation
Eigen::Vector2d lerp(double t, const Eigen::Vector2d &a, const Eigen::Vector2d &b);
// Test the point "alpha" of the way from P1 to P2
// See if it is on a edge of the rectangle
// Consider only faces in "mask"
bool point_on_edge(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, double alpha, long mask);
// Compute intersection of P1 --> P2 line segment with edge lines
// Then test intersection point to see if it is on cube face
// Consider only face planes in "outcode_diff"
// Note: Zero bits in "outcode_diff" means edge is outside of
bool segment_on_edge(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, long outcode_diff);
// true if line t1,t2 is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool line_outside_of_rect(const Eigen::Vector2d &c, double s, const Eigen::Vector2d &t1, const Eigen::Vector2d &t2);
// true if line p is outside a rectangle
// centered at c with length of a side s,
// false if the line intersects rectangle
bool point_outside_of_rect(const Eigen::Vector2d &c, double s, const Eigen::Vector2d &p);
// true if rectangle 1 intersects rectangle 2
// c == center
// theta == angle
// w == width
// l == length
bool rect_rect_X(const Eigen::Vector2d &c1, double theta1, double w1, double l1,
                 const Eigen::Vector2d &c2, double theta2, double w2, double l2);
// true if line intersects rectangle
// line t1-->t2
// c == center
// theta == angle
// w == width
// l == length
bool line_rect_X(const Eigen::Vector2d &t1, const Eigen::Vector2d &t2,
                 const Eigen::Vector2d &c, double theta, double w, double l);

} // namespace vlr

#endif // AW_GEOMETRY_2D_H
