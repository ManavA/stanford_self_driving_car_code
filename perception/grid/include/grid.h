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

#ifndef GRID_H_
#define GRID_H_

#include <cmath>
#include <global.h>
#include <cell.h>

namespace perception {

template <class T>
class Grid {
public:
  Grid(double resolution, int16_t rows, int16_t cols, const T* default_value) :
                  resolution_(resolution), rows_(rows), cols_(cols),
                  map_r0_(0), map_c0_(0), array_r0_(0), array_c0_(0) {

    cell_ = new T[rows_*cols_];

    if (default_value) {
     default_value_ = *default_value;
    }

    clear();
  }

  Grid(const Grid& other) {
    // realloc memory if necessary
    if (other.rows_ != rows_ || other.cols_ != cols_) {
      delete cell_;
      cell_ = new T[other.rows_*other.cols_];
    }

    // copy grid properties
    resolution_ = other.resolution_;
    rows_ = other.rows_;
    cols_ = other.cols_;
    map_r0_ = other.map_r0_;
    map_c0_ = other.map_c0_;
    array_r0_ = other.array_r0_;
    array_c0_ = other.array_c0_;
    default_value_ = other.default_value_;

    // copy grid cells
    memcpy(cell_, other.cell_, rows_* cols_*sizeof(T));
  }

  ~Grid() {
    if(cell_) {delete[] cell_;}
  }

  inline int32_t int_floor(double x) const {
    return ((int32_t) (x + 100000.0)) - 100000;
  }

  // void* grid_get_xy(dgc::dgc_grid_t* grid, float x, float y)
  template <class CT>
  T* getXY(CT x, CT y) const {
    int16_t c = (int16_t) floor(x / resolution_) - map_c0_;
    int16_t r = (int16_t) floor(y / resolution_) - map_r0_;
    if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}

    r = wrap(r + array_r0_, rows_);
    c = wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
  }

  T* getRCGlobal(int16_t r, int16_t c) const {
    r -= map_r0_;
    c -= map_c0_;
    if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}

    r = wrap(r + array_r0_, rows_);
    c = wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
  }

  T* getRCLocalUnsafe(int16_t r, int16_t c) const {
    r = wrap(r + array_r0_, rows_);
    c = wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
  }

  // void* grid_get_rc(dgc::dgc_grid_t* grid, int r, int c);
  T* getRCLocal(int16_t r, int16_t c) const {
    if (r < 0 || c < 0 || r >= rows_ || c >= cols_) {return NULL;}
    r = wrap(r + array_r0_, rows_);
    c = wrap(c + array_c0_, cols_);
    return &cell_[r * cols_ + c];
  }

//  void grid_xy_to_rc(dgc::dgc_grid_t* grid, float x, float y, int *r, int *c);
  template <class CT>
  void xyToRCLocal(CT x, CT y, int16_t* r, int16_t* c) const {
    *c = int_floor(x / resolution_) - map_c0_;
    *r = int_floor(y / resolution_) - map_r0_;
  }

// void grid_rc_to_xy(dgc::dgc_grid_t* grid, int r, int c, float* x, float* y);
  template <class CT>
  void rcLocalToXY(int16_t r, int16_t c, CT* x, CT* y) const {
    *x = (map_c0_ + c + 0.5) * resolution_;
    *y = (map_r0_ + r + 0.5) * resolution_;
  }

  // void cell_to_coord(dgc_grid_p grid, dgc_perception_map_cell_p cell, short *x, short *y);
  void cellToRCLocal(const T* cell, int16_t* r, int16_t* c) const {

    int32_t n = cell - cell_;
    *r = n / cols_;
    *c = n - *r * cols_;

    *r -= array_r0_;
    *c -= array_c0_;
    if (*r < 0) {*r += rows_;}
    if (*c < 0) {*c += cols_;}
  }

  // void cell_to_coord(dgc_grid_p grid, dgc_perception_map_cell_t* cell, double *x, double *y);
  template <class CT>
  void cellToXY(const T* cell, CT* x, CT* y) const {

    int32_t n = cell - cell_;
    int16_t r = n / cols_;
    int16_t c = n - r * cols_;

    r -= array_r0_;
    c -= array_c0_;
    if (r < 0) r += rows_;
    if (c < 0) c += cols_;

    *x = (map_c0_ + c + 0.5) * resolution_;
    *y = (map_r0_ + r + 0.5) * resolution_;
  }

  void clear() {
    for (int32_t i = 0; i < rows_ * cols_; i++) {
      cell_[i].clear();
      memcpy(&cell_[i], &default_value_, sizeof(T));
    }
  }

  template <class CT>
  bool recenter(CT x, CT y) {
    int16_t corner_r = int_floor(y / resolution_) - rows_ / 2;
    int16_t corner_c = int_floor(x / resolution_) - cols_ / 2;

    int16_t dr = corner_r - map_r0_;
    int16_t dc = corner_c - map_c0_;

    if (dr == 0 && dc == 0) {return false;}
    if (abs(dr) >= rows_ || abs(dc) >= cols_) {
      clear();
      map_r0_ = corner_r;
      map_c0_ = corner_c;
      array_r0_ = 0;
      array_c0_ = 0;
    }
    else {
      if (dr > 0) {
        for (int16_t i = 0; i < dr; i++) {addRowNorth();}
      }
      else if (dr < 0) {
        for (int16_t i = 0; i < abs(dr); i++) {addRowSouth();}
      }
      if (dc > 0) {
        for (int16_t i = 0; i < dc; i++) {addColumnEast();}
      }
      else if (dc < 0) {
        for (int16_t i = 0; i < abs(dc); i++) {addColumnWest();}
      }
    }

    return true;
  }

private:
  void addColumnEast() {
      for (int16_t r = 0; r < rows_; r++) {
      T* cell = &cell_[r * cols_ + array_c0_];
      cell->clear();
      memcpy(cell, &default_value_, sizeof(T));
    }

    map_c0_++;
    array_c0_++;
    if (array_c0_ == cols_) {array_c0_ = 0;}
  }

  void addColumnWest() {
    int16_t new_array_c0 = array_c0_ - 1;
    if (new_array_c0 < 0) {new_array_c0 = cols_ - 1;}

    for (int16_t r = 0; r < rows_; r++) {
      T* cell = &cell_[r * cols_ + new_array_c0];
      cell->clear();
      memcpy(cell, &default_value_, sizeof(T));
    }

    map_c0_--;
    array_c0_ = new_array_c0;
  }

  void addRowNorth() {
    for (int16_t c = 0; c < cols_; c++) {
      T* cell = &cell_[array_r0_ * cols_ + c];
      cell->clear();
      memcpy(cell, &default_value_, sizeof(T));
    }

    map_r0_++;
    array_r0_++;
    if (array_r0_ == rows_) {array_r0_ = 0;}
  }

  void addRowSouth() {
    int16_t new_array_r0 = array_r0_ - 1;
    if (new_array_r0 < 0) {new_array_r0 = rows_ - 1;}

    for (int16_t c = 0; c < cols_; c++) {
      T* cell = &cell_[new_array_r0 * cols_ + c];
      cell->clear();
      memcpy(cell, &default_value_, sizeof(T));
    }

    map_r0_--;
    array_r0_ = new_array_r0;
  }


  static int16_t wrap(int16_t x, int16_t max) {
    if (x >= max) {
      while (x >= max)
        x -= max;
    }
    else if (x < 0) {
      while (x < 0)
        x += max;
    }
    return x;
  }

//  int32_t wrap(int32_t x, int32_t max) {
//    if (x >= max) {
//      return (x % max);
//    }
//    else if (x < 0) {
//      return ((x + max) % max);
//    }
//    return x;
//  }

public:
  double resolution_;              // map resolution in meters
  int16_t rows_, cols_;                 // size of grid
  int16_t map_r0_, map_c0_;             // grid coordinates of lower left corner of map
  int16_t array_r0_, array_c0_;         // position of lower left corner in array

private:
  T* cell_;                     // actual map data
  T default_value_;            // default value for new cells
};




} //namespace perception

#endif
