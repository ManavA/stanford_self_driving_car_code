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


#include <string>
#include <global.h>
#include <comp_stdio.h>
#include <grid.h>
#include <lltransform.h>
#include <vlrException.h>
#include <highgui.h>

#include "laserMap.h"

using namespace dgc;

namespace vlr {

TerrainTile::TerrainTile(int32_t rows, int32_t cols) : rows_(rows), cols_(cols) {

  cell_ = new TerrainTileCell*[cols_];

  for (int32_t x = 0; x < cols_; x++) {
    cell_[x] = new TerrainTileCell[rows_];
    for (int32_t y = 0; y < rows_; y++) {
      cell_[x][y].z = 1e6;
      cell_[x][y].current_highest = -1e6;
    }
  }

  num_pixels_ = TERRAIN_TILE_SIZE * TERRAIN_TILE_SIZE;
  rgb_buf_ = new uint8_t[3*num_pixels_];
  memset(rgb_buf_, 0, 3*num_pixels_);
}

TerrainTile::~TerrainTile() {
  double stdev_sum = 0.0;
  int32_t stdev_count = 0;
  double stdev_max = 0.0;
  for (int32_t x = 0; x < cols_; x++) {
    for (int32_t y = 0; y < rows_; y++) {
      //stdev_sum += cell_[x][y].stdev;
      if (cell_[x][y].stdev > 0) {
        stdev_count++;
        stdev_sum += cell_[x][y].stdev;
        if (cell_[x][y].stdev > stdev_max) {stdev_max = cell_[x][y].stdev;}
      }
    }
  }
  //printf("SUM: %f\n", stdev_sum);
  //printf("%d cells had an average stdev of %f, max of %f\n", stdev_count, 1.0 * stdev_sum / stdev_count, stdev_max);
  for (int32_t x = 0; x < cols_; x++) {delete cell_[x];}
  delete[] cell_;
  delete[] rgb_buf_;
}

bool TerrainTile::load(std::string& filename) {

  cio::FILE* fp = cio::fopen(filename.c_str(), "r");
  if (!fp) {return false;}

  filename_ = filename;

  int32_t temp_cols, temp_rows;
  cio::fread(&temp_cols, sizeof(int32_t), 1, fp);
  cio::fread(&temp_rows, sizeof(int32_t), 1, fp);
  if (temp_cols != cols_ || temp_rows != rows_) {
    throw VLRException("Error: tile is not the right size");
  }

  cio::fread(&utm_x0_, sizeof(double), 1, fp);
  cio::fread(&utm_y0_, sizeof(double), 1, fp);
  char utmzone[5];
  cio::fread(&utmzone, 3, 1, fp);
  utmzone[4] = '\0';
  utm_zone_ = utmzone;
  cio::fread(&resolution_, sizeof(double), 1, fp);

  for (int32_t i = 0; i < cols_; i++) {
    cio::fread(cell_[i], rows_ * sizeof(TerrainTileCell), 1, fp);
  }

  cio::fclose(fp);

  return true;
}

void TerrainTile::do_average() {
  min_z_ = 1e9;
  for (int32_t x = 0; x < cols_; x++)
    for (int32_t y = 0; y < rows_; y++) {
      if (cell_[x][y].z_count > 0) {
        cell_[x][y].z /= (double) cell_[x][y].z_count;
        if (cell_[x][y].z < min_z_) {min_z_ = cell_[x][y].z;}
      }
      if (cell_[x][y].i_count > 0) {cell_[x][y].intensity /= (double) cell_[x][y].i_count;}
    }
}

void TerrainTile::save(std::string& filename) {
  cio::FILE* fp = cio::fopen(filename.c_str(), "w");
  if (!fp) {
    throw VLRException("Cannot write to file " + filename);
  }

  cio::fwrite(&cols_, sizeof(int32_t), 1, fp);
  cio::fwrite(&rows_, sizeof(int32_t), 1, fp);

  cio::fwrite(&utm_x0_, sizeof(double), 1, fp);
  cio::fwrite(&utm_y0_, sizeof(double), 1, fp);
  char utmzone[4];
  sprintf(utmzone, "%s", utm_zone_.c_str());
  cio::fwrite(&utmzone, 3, 1, fp);
  cio::fwrite(&resolution_, sizeof(double), 1, fp);

  for (int32_t i = 0; i < cols_; i++) {
    cio::fwrite(cell_[i], rows_ * sizeof(TerrainTileCell), 1, fp);
  }
  cio::fclose(fp);


  memset(rgb_buf_, 0, 3*num_pixels_);

  for (int32_t i = 0; i < cols_; i++) {
    for (int32_t j = 0; j < rows_; j++) {
      int32_t count = cell_[i][j].i_count;
      if (count > 0) {
        int32_t intensity = (int32_t)(cell_[i][j].intensity / count);
        intensity *= 1.0;
        if (intensity > 255) {intensity = 255;}
        else if (intensity < 0) {intensity = 0;}
        rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i)] = (uint8_t) intensity;
        rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (uint8_t)(.5 * intensity);
        rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (uint8_t)(.2 * intensity);
      }
      /* int32_t z_ness = (int32_t) (30 * pow((cell_[i][j].z / cell_[i][j].z_count), 2));
       if(cell_[i][j].z_count > 1)
       z_ness = (int32_t) (100 * (cell_[i][j].current_highest - cell_[i][j].z));
       else
       z_ness = 0;
       z_ness = (int32_t) (2 * cell_[i][j].stdev);
       if(z_ness > 255) z_ness = 255;
       //rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (uint8_t) z_ness;
       z_ness = (int32_t) (1.0 * cell_[i][j].z_response);
       if(z_ness > 255) z_ness = 255;
       //rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (uint8_t) z_ness;

       z_ness = .01 * pow(cell_[i][j].intensity / cell_[i][j].i_count, 2) / cell_[i][j].stdev;
       if(z_ness > 255) z_ness = 255;
       //rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (uint8_t) z_ness;

       if(cell_[i][j].z_count > 0) {
       float height = cell_[i][j].current_highest - cell_[i][j].z;
       if(height > 1) height = 1;
       rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = 0 + 1 * ((uint8_t) (255 * height));

       int32_t z_ness = (int32_t) (10 * pow(cell_[i][j].z_response / (.000001 + cell_[i][j].current_highest - cell_[i][j].z), -1));
       z_ness = 10 * cell_[i][j].z_response;
       z_ness = 2 * (cell_[i][j].z_response / (height + .05));
       if(z_ness > 255) z_ness = 255;

       rgb_buf_[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = 0 + 1 * ((uint8_t) z_ness);
       }
       */
    }
  }

  std::string imagename = filename + ".png";

  printf("NEW FILE NAME: %s\n", imagename.c_str());
//  dgc_image_write_raw(rgb_buf_, TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, imagename.c_str());
  cv::Mat tmp_mat(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, CV_8UC3, rgb_buf_);
  cv::imwrite(imagename, tmp_mat);
}

} // namespace vlr
