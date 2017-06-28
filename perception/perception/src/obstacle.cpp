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


#include <obstacle.h>
#include <utils.h>
#include <grid.h>
#include <iostream>
#include <perception.h>

using namespace std;
using namespace dgc;
using namespace Eigen;

//#define FILL_DEBUG
//#define FILL_DEBUG_2

namespace drc = driving_common;

namespace perception {

dgc_pose_t zero_pose = {0,0,0,0,0,0};

Obstacle::Obstacle(int id, Perception& perception) :
  x_center_(0),
  y_center_(0),
  z_center_(0),
  lane_(NULL),
  rndfDist_(-1.0),
  matched_(false),
  perception_(perception),
  id(id),
  pose(zero_pose),
  length(0.0),
  width(0.0),
  type(OBSTACLE_UNKNOWN),
  type_this_frame_(OBSTACLE_UNKNOWN),
  classified_this_frame_(false),
  response_(VectorXf::Zero(getClassNames().size())) {
}

Obstacle::Obstacle(const Obstacle& o) :
  x_center_(o.x_center_),
  y_center_(o.y_center_),
  z_center_(o.z_center_),
  lane_(o.lane_),
  rndfDist_(o.rndfDist_),
  matched_(o.matched_),
  perception_(o.perception_),
  id (o.id),
  pose(o.pose),
  length(o.length),
  width(o.width),
  type(o.type),
  type_this_frame_(o.type_this_frame_),
  classified_this_frame_(o.classified_this_frame_),
  response_(o.response_)
  // import not to copy points_ here
{


}

Obstacle::~Obstacle() {

}

  // bresenham
void drawLine(const Grid<PerceptionCell>& grid, int32_t x1, int32_t y1, int32_t x2, int32_t y2, int value, bool* edge) {
  int32_t X1, Y1;
  int32_t X2, Y2;
  int32_t increment;
  bool usingYindex = false;
  int deltaX, deltaY;
  int dTerm;
  int incrE, incrNE;
  int XIndex, YIndex;
  int flipped;

  if(std::abs((double)(y2 - y1) / (double)(x2 - x1)) > 1) {
     usingYindex = true;
  }

  if(usingYindex) {
    Y1 = x1;
    X1 = y1;
    Y2 = x2;
    X2 = y2;
  }
  else {
    X1 = x1;
    Y1 = y1;
    X2 = x2;
    Y2 = y2;
  }

  if((x2 - x1) * (y2 - y1) < 0) {
    flipped = 1;
    Y1 = -Y1;
    Y2 = -Y2;
  }
  else {
    flipped = 0;
  }

  if(X2 > X1)
    increment = 1;
  else
    increment = -1;

  deltaX = X2-X1;
  deltaY = Y2-Y1;

  incrE = 2 * deltaY * increment;
  incrNE = 2 * (deltaY - deltaX) * increment;
  dTerm = (2 * deltaY - deltaX) * increment;

  XIndex = X1;
  YIndex = Y1;

  while (XIndex != X2) {

    XIndex += increment;
    if(dTerm < 0 || (increment < 0 && dTerm <= 0))
      dTerm += incrE;
    else {
      dTerm += incrNE;
      YIndex += increment;
    }

    int x, y;
    if(usingYindex) {
      y = XIndex;
      x = YIndex;
      if(flipped)
        x = -x;
    }
    else {
      x = XIndex;
      y = YIndex;
      if(flipped)
        y = -y;
    }

    PerceptionCell* cell = grid.getRCLocal(x, y);
    if (cell) {
      cell->last_dynamic = value;
#ifdef FILL_DEBUG
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
#endif
    } else {
      *edge = true;
    }
  }
}

// works for convex objects that don't cross grid boundaries (which can happen if grid is recentered)
void fastFillBoundary(const Grid<PerceptionCell>& grid, int32_t start_r, int32_t start_c, uint16_t fill) {

  PerceptionCell* cell = grid.getRCLocalUnsafe(start_r, start_c);

  uint32_t bytes_per_cell = sizeof(*cell);
  uint32_t bytes_per_row = bytes_per_cell * grid.cols_;

  char* last_dynamic = (char*)&cell->last_dynamic;
  char* right_dynamic = last_dynamic + bytes_per_cell;
  char* top = 0;
  char* bottom = 0;

  // fill left half of row
  while (*((unsigned short*)last_dynamic) < fill) {
    *((unsigned short*)last_dynamic) = fill;
    last_dynamic -= bytes_per_cell;

    if (!top) {
      char* tmp = last_dynamic + bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        top = tmp;
    }

    if (!bottom) {
      char* tmp = last_dynamic - bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        bottom = tmp;
    }

    if (top && bottom) {
      while (*((unsigned short*)last_dynamic) < fill) {
        *((unsigned short*)last_dynamic) = fill;
        last_dynamic -= bytes_per_cell;
      }
      break;
    }
  }

  // fill right half of row
  last_dynamic = right_dynamic;
  while (*((unsigned short*)last_dynamic) < fill) {
    *((unsigned short*)last_dynamic) = fill;
    last_dynamic += bytes_per_cell;

    if (!top) {
      char* tmp = last_dynamic + bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        top = tmp;
    }

    if (!bottom) {
      char* tmp = last_dynamic - bytes_per_row;
      if (*(unsigned short*)tmp < fill)
        bottom = tmp;
    }

    if (top && bottom) {
      while (*((unsigned short*)last_dynamic) < fill) {
        *((unsigned short*)last_dynamic) = fill;
        last_dynamic += bytes_per_cell;
      }
      break;
    }
  }


  // fill top half
  while (top) {
    last_dynamic = top;
    right_dynamic = last_dynamic + bytes_per_cell;

    top = 0;

    // fill left half of row
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;

      if (!top) {
        char* tmp = last_dynamic + bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          top = tmp;
        last_dynamic -= bytes_per_cell;
      } else {
        last_dynamic -= bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic -= bytes_per_cell;
        }
        break;
      }
    }

    // fill right half of row
    last_dynamic = right_dynamic;
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;
      if (!top) {
        char* tmp = last_dynamic + bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          top = tmp;
        last_dynamic += bytes_per_cell;
      } else {
        last_dynamic += bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic += bytes_per_cell;
        }
        break;
      }
    }
  }

  // fill bottom half
  while (bottom) {
    last_dynamic = bottom;
    right_dynamic = last_dynamic + bytes_per_cell;

    bottom = 0;

    // fill left half of row
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;

      if (!bottom) {
        char* tmp = last_dynamic - bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          bottom = tmp;
        last_dynamic -= bytes_per_cell;
      } else {
        last_dynamic -= bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic -= bytes_per_cell;
        }
        break;
      }
    }

    // fill right half of row
    last_dynamic = right_dynamic;
    while (*((unsigned short*)last_dynamic) < fill) {
      *((unsigned short*)last_dynamic) = fill;
      if (!bottom) {
        char* tmp = last_dynamic - bytes_per_row;
        if (*(unsigned short*)tmp < fill)
          bottom = tmp;
        last_dynamic += bytes_per_cell;
      } else {
        last_dynamic += bytes_per_cell;
        while (*((unsigned short*)last_dynamic) < fill) {
          *((unsigned short*)last_dynamic) = fill;
          last_dynamic += bytes_per_cell;
        }
        break;
      }
    }
  }
}

void fillBoundary(const Grid<PerceptionCell>& grid, int32_t start_r, int32_t start_c, int32_t fill) {
  // fill first row
#ifdef FILL_DEBUG_2
  int count = 0;
#endif
  int32_t r = start_r;
  int32_t c = start_c;
  int32_t top_c = start_c;
  int32_t bottom_c = start_c;
  bool found_top_c = false;
  bool found_bottom_c = false;

  PerceptionCell* cell = grid.getRCLocal(r, c);
  PerceptionCell* neighbor_cell = NULL;

  // fill left
  while ((cell) && (cell->last_dynamic < fill)) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = grid.getRCLocal(r+1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = grid.getRCLocal(r-1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c--;
    cell = grid.getRCLocal(r, c);
  }

  // fill right
  c = start_c + 1;
  cell = grid.getRCLocal(r, c);
  while ((cell) && (cell->last_dynamic < fill)) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = grid.getRCLocal(r+1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = grid.getRCLocal(r-1, c);
      if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c++;
    cell = grid.getRCLocal(r, c);
  }


  // fill up
  while (found_top_c) {
    found_top_c = false;
    c = top_c;
    int right_c = top_c + 1;
    r++;
    cell = grid.getRCLocal(r, c);

    // fill left
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = grid.getRCLocal(r+1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_top_c = true;
          top_c = c;
        }
      }

      c--;
      cell = grid.getRCLocal(r, c);
    }

    // fill right
    c = right_c;
    cell = grid.getRCLocal(r, c);
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = grid.getRCLocal(r+1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_top_c = true;
          top_c = c;
        }
      }

      c++;
      cell = grid.getRCLocal(r, c);
    }
  }

  // fill down
  r = start_r;
  while (found_bottom_c) {
    found_bottom_c = false;
    c = bottom_c;
    int right_c = bottom_c + 1;
    r--;
    cell = grid.getRCLocal(r, c);

    // fill left
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = grid.getRCLocal(r-1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c--;
      cell = grid.getRCLocal(r, c);
    }

    // fill right
    c = right_c;
    cell = grid.getRCLocal(r, c);
    while ((cell) && (cell->last_dynamic < fill)) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = grid.getRCLocal(r-1, c);
        if ((neighbor_cell) && (neighbor_cell->last_dynamic < fill)) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c++;
      cell = grid.getRCLocal(r, c);
    }
  }
}

void fillBoundaryUnsafe(const Grid<PerceptionCell>& grid, int32_t start_r, int32_t start_c, int32_t fill) {
  // fill first row
#ifdef FILL_DEBUG_2
  int count = 0;
#endif
  int32_t r = start_r;
  int32_t c = start_c;
  int32_t top_c = start_c;
  int32_t bottom_c = start_c;
  bool found_top_c = false;
  bool found_bottom_c = false;

  PerceptionCell* cell = grid.getRCLocalUnsafe(r, c);
  PerceptionCell* neighbor_cell = NULL;

  // fill left
  while (cell->last_dynamic < fill) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = grid.getRCLocalUnsafe(r+1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = grid.getRCLocalUnsafe(r-1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c--;
    cell = grid.getRCLocalUnsafe(r, c);
  }

  // fill right
  c = start_c + 1;
  cell = grid.getRCLocalUnsafe(r, c);
  while (cell->last_dynamic < fill) {
    cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
    cell->min = 2.0; cell->max = 3.0;
    obstacles_s->cell[obstacles_s->num] = cell;
    if (obstacles_s->num<MAX_NUM_POINTS) {
      obstacles_s->num++;
    }
    count++;
#endif

    if (!found_top_c) {
      neighbor_cell = grid.getRCLocalUnsafe(r+1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_top_c = true;
        top_c = c;
      }
    }

    if (!found_bottom_c) {
      neighbor_cell = grid.getRCLocalUnsafe(r-1, c);
      if (neighbor_cell->last_dynamic < fill) {
        found_bottom_c = true;
        bottom_c = c;
      }
    }

    c++;
    cell = grid.getRCLocalUnsafe(r, c);
  }


  // fill up
  while (found_top_c) {
    found_top_c = false;
    c = top_c;
    int right_c = top_c + 1;
    r++;
    cell = grid.getRCLocalUnsafe(r, c);

    // fill left
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = grid.getRCLocalUnsafe(r+1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_top_c = true;
          top_c = c;
        }
      }

      c--;
      cell = grid.getRCLocalUnsafe(r, c);
    }

    // fill right
    c = right_c;
    cell = grid.getRCLocalUnsafe(r, c);
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_top_c) {
        neighbor_cell = grid.getRCLocalUnsafe(r+1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_top_c = true;
          top_c = c;
        }
      }

      c++;
      cell = grid.getRCLocalUnsafe(r, c);
    }
  }

  // fill down
  r = start_r;
  while (found_bottom_c) {
    found_bottom_c = false;
    c = bottom_c;
    int right_c = bottom_c + 1;
    r--;
    cell = grid.getRCLocalUnsafe(r, c);

    // fill left
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = grid.getRCLocalUnsafe(r-1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c--;
      cell = grid.getRCLocalUnsafe(r, c);
    }

    // fill right
    c = right_c;
    cell = grid.getRCLocalUnsafe(r, c);
    while (cell->last_dynamic < fill) {
      cell->last_dynamic = fill;
#ifdef FILL_DEBUG_2
      cell->min = 2.0; cell->max = 3.0;
      obstacles_s->cell[obstacles_s->num] = cell;
      if (obstacles_s->num<MAX_NUM_POINTS) {
        obstacles_s->num++;
      }
      count++;
#endif

      if (!found_bottom_c) {
        neighbor_cell = grid.getRCLocalUnsafe(r-1, c);
        if (neighbor_cell->last_dynamic < fill) {
          found_bottom_c = true;
          bottom_c = c;
        }
      }

      c++;
      cell = grid.getRCLocalUnsafe(r, c);
    }
  }
}

void Obstacle::markDynamic(const Grid<PerceptionCell>& grid, dgc_perception_map_cells_p obstacles_s, unsigned short counter, double velocity) {
  dgc_transform_t t;
  dgc_transform_identity(t);

  dgc_transform_rotate_z(t, pose.yaw);

  double buffer = 0.5;
  double lookahead = 0.0;
  if (velocity > 1.0) {
    lookahead = std::max(velocity, 10.0) * 0.5; // assume this is not going to hit anything in the next half second
  }

  double unused_z = 0;
  double w = (length/2.0) + buffer;
  double w1 = w +  lookahead;
  double l1 = (width/2.0)  + buffer;
  double w2 = w + lookahead;
  double l2 = -l1;
  double w3 = -w;
  double l3 = -l1;
  double w4 = -w;
  double l4 = l1;

  dgc_transform_point(&w1, &l1, &unused_z, t);
  dgc_transform_point(&w2, &l2, &unused_z, t);
  dgc_transform_point(&w3, &l3, &unused_z, t);
  dgc_transform_point(&w4, &l4, &unused_z, t);

  double x1 = pose.x + w1;
  double y1 = pose.y + l1;

  double x2 = pose.x + w2;
  double y2 = pose.y + l2;

  double x3 = pose.x + w3;
  double y3 = pose.y + l3;

  double x4 = pose.x + w4;
  double y4 = pose.y + l4;

  int16_t r, c, r1, c1, r2, c2, r3, c3, r4, c4;
  grid.xyToRCLocal(pose.x, pose.y, &r, &c);
  grid.xyToRCLocal(x1, y1, &r1, &c1);
  grid.xyToRCLocal(x2, y2, &r2, &c2);
  grid.xyToRCLocal(x3, y3, &r3, &c3);
  grid.xyToRCLocal(x4, y4, &r4, &c4);

  bool edge = false;
  drawLine(grid, r1, c1, r2, c2, counter, &edge);
  drawLine(grid, r2, c2, r3, c3, counter, &edge);
  drawLine(grid, r3, c3, r4, c4, counter, &edge);
  drawLine(grid, r4, c4, r1, c1, counter, &edge);

  int min_r = r1;
  int min_c = c1;

  if (r2 < min_r) {
    min_r = r2;
    min_c = c2;
  }

  if (r3 < min_r) {
    min_r = r3;
    min_c = c3;
  }

  if (r4 < min_r) {
    min_r = r4;
    min_c = c4;
  }

  if (edge) {
    fillBoundary(grid, r, c, counter);
  }
  else {
    fillBoundaryUnsafe(grid, r, c, counter);
  }
}

bool Obstacle::getCenterOfPoints(double *x, double *y)
{
  if (points_.size() == 0) {
    populatePoints();
  }

  if(x_center_ != 0.0 || y_center_ != 0.0) {
    *x = x_center_;
    *y = y_center_;
    return true;
  }

  *x = 0;
  *y = 0;
  int count = points_.size();
  if (count < 1)
    return false;

  for (int i=0; i<count; i++) {
    *x = *x + points_[i].x;
    *y = *y + points_[i].y;
  }

  *x = *x / (double)count;
  *y = *y / (double)count;
  x_center_ = *x;
  y_center_ = *y;
  return true;
}

bool Obstacle::getCenterOfPoints(double *x, double *y, double *z)
{
  if (points_.size() == 0) {
    populatePoints();
  }

  if(x_center_ != 0.0 || y_center_ != 0.0 || z_center_ != 0.0) {
    *x = x_center_;
    *y = y_center_;
    *z = z_center_;
  }

  *x = 0;
  *y = 0;
  *z = 0;
  int count = points_.size();
  if (count < 1)
    return false;

  for (int i=0; i<count; i++) {
    *x = *x + points_[i].x;
    *y = *y + points_[i].y;
    *z = *z + points_[i].z;
  }

  *x = *x / (double)count;
  *y = *y / (double)count;
  *z = *z / (double)count;
  x_center_ = *x;
  y_center_ = *y;
  z_center_ = *z;
  return true;
}

void Obstacle::merge(const Obstacle& o) {
  pose.x = (pose.x + o.pose.x) / 2.0;
  pose.y = (pose.y + o.pose.y) / 2.0;
  pose.z = (pose.z + o.pose.z) / 2.0;
  x_center_ = 0.0;
  y_center_ = 0.0;
  z_center_ = 0.0;

  if ((points_.size() > 0) && (o.points_.size() > 0)) {
    points_.insert(points_.end(), o.points_.begin(), o.points_.end());
  } else {
    points_.clear();
  }
}

void Obstacle::populatePoints() {
  points_.clear();
  point3d_t pt;
  pt.x = pose.x;
  pt.y = pose.y;
  pt.z = pose.z;
  points_.push_back(pt);
}

std::vector<point3d_t>& Obstacle::getPoints() {
  if (points_.size() == 0) {
    populatePoints();
  }

  return points_;
}
  
GridObstacle::GridObstacle(int id, const Grid<PerceptionCell>& grid, Perception& perception) :
  Obstacle(id, perception),
  grid_(grid) {
}

GridObstacle::GridObstacle (const GridObstacle& o) :
  Obstacle(o),
  grid_(o.grid_),
  cells_(o.cells_) {
}

GridObstacle::~GridObstacle() {

}

int GridObstacle::getSize() {
  return cells_.size();
}

void GridObstacle::clear() {
  cells_.clear();
}

//void GridObstacle::markDynamic(Grid* grid, dgc_perception_map_cells_p obstacles, unsigned short counter)  {
//  for (unsigned int i=0; i<cells_.size(); i++) {
//    cells_[i]->last_dynamic = counter;
//  }
//}

void GridObstacle::merge(const GridObstacle& o) {
  Obstacle::merge(o);
  cells_.insert(cells_.end(), o.cells_.begin(), o.cells_.end());
}

void GridObstacle::addCell(PerceptionCell& cell) {
  cells_.push_back(&cell);
}

std::vector<PerceptionCell*>& GridObstacle::getCells() {
  return cells_;
}

void GridObstacle::pointsInCell(const PerceptionCell& cell, std::vector<point3d_t>& points) {
#ifdef USE_GRID_SEGMENTER
  points.reserve(perception_.cellToPoints().count((uintptr_t) &cell));
  std::pair<Perception::map_type::iterator, Perception::map_type::iterator> p = perception_.cellToPoints().equal_range((uintptr_t) &cell);
  point3d_t point;
  for (Perception::map_type::iterator it = p.first; it != p.second; it++) {
    laser_point_t* pt = (laser_point_p) (it->second);
    drc::GlobalPose robot_pose = perception_.pose(pt->timestamp);
    point.x = pt->point.x * CM_TO_METER_FACTOR + robot_pose.x();
    point.y = pt->point.y * CM_TO_METER_FACTOR + robot_pose.y();
    point.z = pt->point.z * CM_TO_METER_FACTOR + robot_pose.z();
    point.intensity = pt->intensity;
    points.push_back(point);
  }
#endif
}

void GridObstacle::populatePoints() {
  points_.clear();

  point3d_t pt;
  pt.z = 0;
  for (unsigned int i=0; i < cells_.size(); i++) {
    pointsInCell(*cells_[i], points_);

    grid_.cellToXY(cells_[i], &pt.y, &pt.x);
    points_.push_back(pt);
  }
}

float GridObstacle::maxHeight() {
  float max_height = -1.0;

  for (size_t i=0; i < cells_.size(); i++) {
    max_height = std::max(max_height, cells_[i]->max-cells_[i]->min);
  }
  return max_height;
}


LaserObstacle::LaserObstacle(int id, Perception& perception) : Obstacle(id, perception) {

}

LaserObstacle::LaserObstacle (const LaserObstacle& o) :
    Obstacle(o),
    min_z_(1e9),
    max_z_(-1e9)
{
  points_ = o.points_;
}

LaserObstacle::~LaserObstacle() {

}

int LaserObstacle::getSize() {
  return points_.size();
}

void LaserObstacle::clear() {
  points_.clear();
}

void LaserObstacle::merge(const LaserObstacle& o) {
  Obstacle::merge(o);
}

//void LaserObstacle::markDynamic(Grid<PerceptionCell>* grid, dgc_perception_map_cells_p obstacles, unsigned short counter) {
//  std::vector<point3d_t>& points = getPoints();
//  for (unsigned int i=0; i < points.size(); i++) {
//    PerceptionCell* cell = (PerceptionCell*)grid_get_xy(grid, points[i].x, points[i].y);
//    if (cell != NULL) {
//      cell->last_dynamic = counter;
//    }
//  }
//}

void LaserObstacle::populatePoints() {

}

void LaserObstacle::addPoint(laser_point_p pt) {
  point3d_t point;
  drc::GlobalPose robot_pose = perception_.pose(pt->timestamp);
  point.x = pt->point.x * CM_TO_METER_FACTOR + robot_pose.x();
  point.y = pt->point.y * CM_TO_METER_FACTOR + robot_pose.y();
  point.z = pt->point.z * CM_TO_METER_FACTOR + robot_pose.z();
  point.intensity = pt->intensity;

  max_z_ = std::max(max_z_, point.z);
  min_z_ = std::min(min_z_, point.z);

  points_.push_back(point);
}

float LaserObstacle::maxHeight() {
  return (max_z_ - min_z_);
}

void LaserObstacle::reserve(int count) {
  points_.reserve(count);
}

RadarObservation::RadarObservation() {

}

RadarObservation::~RadarObservation() {

}


}

