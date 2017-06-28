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


#include <perception.h>

namespace perception {

void grid_line_core(ivec2_t start, ivec2_t end, grid_line_p line) {
  int dx, dy, incr1, incr2, d, x, y, xend, yend, xdirflag, ydirflag;

  int cnt = 0;

  dx = abs(end.x - start.x);
  dy = abs(end.y - start.y);

  if (dy <= dx) {
    d = 2 * dy - dx;
    incr1 = 2 * dy;
    incr2 = 2 * (dy - dx);
    if (start.x > end.x) {
      x = end.x;
      y = end.y;
      ydirflag = (-1);
      xend = start.x;
    }
    else {
      x = start.x;
      y = start.y;
      ydirflag = 1;
      xend = end.x;
    }
    line->grid[cnt].x = x;
    line->grid[cnt].y = y;
    if (cnt < line->max) cnt++;
    if (((end.y - start.y) * ydirflag) > 0) {
      while (x < xend) {
        x++;
        if (d < 0) {
          d += incr1;
        }
        else {
          y++;
          d += incr2;
        }
        line->grid[cnt].x = x;
        line->grid[cnt].y = y;
        if (cnt < line->max) cnt++;
      }
    }
    else {
      while (x < xend) {
        x++;
        if (d < 0) {
          d += incr1;
        }
        else {
          y--;
          d += incr2;
        }
        line->grid[cnt].x = x;
        line->grid[cnt].y = y;
        if (cnt < line->max) cnt++;
      }
    }
  }
  else {
    d = 2 * dx - dy;
    incr1 = 2 * dx;
    incr2 = 2 * (dx - dy);
    if (start.y > end.y) {
      y = end.y;
      x = end.x;
      yend = start.y;
      xdirflag = (-1);
    }
    else {
      y = start.y;
      x = start.x;
      yend = end.y;
      xdirflag = 1;
    }
    line->grid[cnt].x = x;
    line->grid[cnt].y = y;
    if (cnt < line->max) cnt++;
    if (((end.x - start.x) * xdirflag) > 0) {
      while (y < yend) {
        y++;
        if (d < 0) {
          d += incr1;
        }
        else {
          x++;
          d += incr2;
        }
        line->grid[cnt].x = x;
        line->grid[cnt].y = y;
        if (cnt < line->max) cnt++;
      }
    }
    else {
      while (y < yend) {
        y++;
        if (d < 0) {
          d += incr1;
        }
        else {
          x--;
          d += incr2;
        }
        line->grid[cnt].x = x;
        line->grid[cnt].y = y;
        if (cnt < line->max) cnt++;
      }
    }
  }
  line->numgrids = cnt;
}

void grid_line(ivec2_t start, ivec2_t end, grid_line_p line) {
  int i, j;
  int half;
  ivec2_t v;
  grid_line_core(start, end, line);
  if (start.x != line->grid[0].x || start.y != line->grid[0].y) {
    half = line->numgrids / 2;
    for (i = 0, j = line->numgrids - 1; i < half; i++, j--) {
      v = line->grid[i];
      line->grid[i] = line->grid[j];
      line->grid[j] = v;
    }
  }
}

} // namespace perception
