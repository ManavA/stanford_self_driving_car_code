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


#include <grid.h>
#include <comp_stdio.h>
#include <lltransform.h>
#include <terrainmap.h>
#include <opencv2/core/core.hpp>
#include <highgui.h>
#include <string.h>

namespace vlr {

static unsigned char *rgb = NULL;

int terrain_tile::load(char *filename) {
  cio::FILE *fp;
  int i, temp_cols, temp_rows;

  fp = cio::fopen(filename, "r");
  if(fp == NULL)
    return -1;
  
  strcpy(this->filename, filename);
  
  cio::fread(&temp_cols, sizeof(int), 1, fp);
  cio::fread(&temp_rows, sizeof(int), 1, fp);
  if(temp_cols != cols || temp_rows != rows) {
    throw VLRException("tile is not the right size");
  }

  cio::fread(&utm_x0, sizeof(double), 1, fp);
  cio::fread(&utm_y0, sizeof(double), 1, fp);
  cio::fread(&utmzone, 3, 1, fp);
  utmzone[4] = '\0';
  cio::fread(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    cio::fread(cell[i], rows * sizeof(terrain_tile_cell), 1, fp);
  
  cio::fclose(fp);

  return 0;
}

int vision_tile::load(char *filename) {
  cio::FILE *fp;
  int i, temp_cols, temp_rows;

  fp = cio::fopen(filename, "r");
  if(fp == NULL)
    return -1;
  
  strcpy(this->filename, filename);
  
  cio::fread(&temp_cols, sizeof(int), 1, fp);
  cio::fread(&temp_rows, sizeof(int), 1, fp);
  if(temp_cols != cols || temp_rows != rows) {
    throw VLRException("Vision tile is not the right size");
  }

  cio::fread(&utm_x0, sizeof(double), 1, fp);
  cio::fread(&utm_y0, sizeof(double), 1, fp);
  cio::fread(&utmzone, 3, 1, fp);
  utmzone[4] = '\0';
  cio::fread(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    cio::fread(cell[i], rows * sizeof(vision_tile_cell), 1, fp);
  
  cio::fclose(fp);

  return 0;
}

void terrain_tile::do_average(void)
{
  int x, y;
  
  min_z = 1e9;
  for(x = 0; x < cols; x++)
    for(y = 0; y < rows; y++) {
      if(cell[x][y].z_count > 0) {
	cell[x][y].z /= (double)cell[x][y].z_count;
	if(cell[x][y].z < min_z)
	  min_z = cell[x][y].z;
      }
      if(cell[x][y].i_count > 0)
	cell[x][y].intensity /= (double)cell[x][y].i_count;
    }
}

int vision_tile::save(char *filename) {
  cio::FILE *fp;
  int i, j;

  fp = cio::fopen(filename, "w");
  if(fp == NULL)
    return -1;
  
  cio::fwrite(&cols, sizeof(int), 1, fp);
  cio::fwrite(&rows, sizeof(int), 1, fp);

  cio::fwrite(&utm_x0, sizeof(double), 1, fp);
  cio::fwrite(&utm_y0, sizeof(double), 1, fp);
  cio::fwrite(&utmzone, 3, 1, fp);
  cio::fwrite(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    cio::fwrite(cell[i], rows * sizeof(vision_tile_cell), 1, fp);
  cio::fclose(fp);

  int numPixels = TERRAIN_TILE_SIZE * TERRAIN_TILE_SIZE;
  if(!rgb)
  	rgb = (unsigned char *)calloc(numPixels*3, 1);

  for(i = 0; i < numPixels * 3; i++)
	rgb[i] = 0;
  for(i = 0; i < cols; i++) {
	for(j = 0; j < rows; j++) {
		int intensity = (int) (255 * cell[i][j].blur);
		if(intensity > 255) intensity = 255;
		int lane = (int) (1000 * cell[i][j].line_lat);
		if(lane > 255) lane = 255;
		rgb[3 * (j * TERRAIN_TILE_SIZE + i)] = (unsigned char) intensity;
		rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) (lane);
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) (.2 * intensity);
	}
  }
  
  char imagename[100];
  strncpy(imagename, filename, 29);
  strcpy(imagename + 29, ".png");
  printf("NEW FILE NAME: %s\n", imagename);
  cv::Mat tmp_mat(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, CV_8UC3, rgb);
  cv::imwrite(imagename, tmp_mat);
  //dgc_image_write_raw(rgb, TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, imagename);
  return 0;
}

int terrain_tile::save(char *filename) {
  cio::FILE *fp;
  int i, j;

  fp = cio::fopen(filename, "w");
  if(fp == NULL)
    return -1;
  
  cio::fwrite(&cols, sizeof(int), 1, fp);
  cio::fwrite(&rows, sizeof(int), 1, fp);

  cio::fwrite(&utm_x0, sizeof(double), 1, fp);
  cio::fwrite(&utm_y0, sizeof(double), 1, fp);
  cio::fwrite(&utmzone, 3, 1, fp);
  cio::fwrite(&resolution, sizeof(double), 1, fp);

  for(i = 0; i < cols; i++)
    cio::fwrite(cell[i], rows * sizeof(terrain_tile_cell), 1, fp);
  cio::fclose(fp);

  int numPixels = TERRAIN_TILE_SIZE * TERRAIN_TILE_SIZE;
  if(!rgb)
  	rgb = (unsigned char *)calloc(numPixels*3, 1);

  for(i = 0; i < numPixels * 3; i++)
	rgb[i] = 0;
  for(i = 0; i < cols; i++) {
	for(j = 0; j < rows; j++) {
		int count = cell[i][j].i_count;
		if(count > 0) {
			int intensity = (int) (cell[i][j].intensity / count);
			intensity *= 1.0;
			if(intensity > 255) intensity = 255;
			if(intensity < 0) intensity = 0;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i)] = (unsigned char) intensity;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) (.5 * intensity);
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) (.2 * intensity);
		}
		/* int z_ness = (int) (30 * pow((cell[i][j].z / cell[i][j].z_count), 2));
		if(cell[i][j].z_count > 1)
			z_ness = (int) (100 * (cell[i][j].current_highest - cell[i][j].z));
		else
			z_ness = 0;
		z_ness = (int) (2 * cell[i][j].stdev);
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = (unsigned char) z_ness;
		z_ness = (int) (1.0 * cell[i][j].z_response);
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) z_ness;

		z_ness = .01 * pow(cell[i][j].intensity / cell[i][j].i_count, 2) / cell[i][j].stdev;
		if(z_ness > 255) z_ness = 255;
		//rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = (unsigned char) z_ness;

		if(cell[i][j].z_count > 0) {
			float height = cell[i][j].current_highest - cell[i][j].z;
			if(height > 1) height = 1;
			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 1] = 0 + 1 * ((unsigned char) (255 * height));

			int z_ness = (int) (10 * pow(cell[i][j].z_response / (.000001 + cell[i][j].current_highest - cell[i][j].z), -1));
			z_ness = 10 * cell[i][j].z_response;
			z_ness = 2 * (cell[i][j].z_response / (height + .05));
			if(z_ness > 255) z_ness = 255;

			rgb[3 * (j * TERRAIN_TILE_SIZE + i) + 2] = 0 + 1 * ((unsigned char) z_ness);
		}
    */
	}
  }
  
  char imagename[100];
  strncpy(imagename, filename, 29);
  strcpy(imagename + 29, ".png");
  printf("NEW FILE NAME: %s\n", imagename);
  cv::Mat tmp_mat(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, CV_8UC3, rgb);
  cv::imwrite(imagename, tmp_mat);
//  dgc_image_write_raw(rgb, TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE, imagename);
  return 0;
}

vision_tile::vision_tile(int rows, int cols) {
  int x, y;

  this->rows = rows;
  this->cols = cols;
  cell = (vision_tile_cell **)calloc(cols, sizeof(vision_tile_cell *));
  dgc::dgc_test_alloc(cell);
  for(x = 0; x < cols; x++) {
    cell[x] = (vision_tile_cell *)calloc(rows, sizeof(vision_tile_cell));
    dgc::dgc_test_alloc(cell);
    for(y = 0; y < rows; y++) {
      cell[x][y].curb = 0;
      cell[x][y].blur = 0;
    }
  }
}

terrain_tile::terrain_tile(int rows, int cols) {
  int x, y;

  this->rows = rows;
  this->cols = cols;
  cell = (terrain_tile_cell **)calloc(cols, sizeof(terrain_tile_cell *));
  dgc::dgc_test_alloc(cell);
  for(x = 0; x < cols; x++) {
    cell[x] = (terrain_tile_cell *)calloc(rows, sizeof(terrain_tile_cell));
    dgc::dgc_test_alloc(cell);
    for(y = 0; y < rows; y++) {
      cell[x][y].z = 1e6;
      cell[x][y].current_highest = -1e6;
    }
  }
}

vision_tile::~vision_tile() {
  int x;
  for(x = 0; x < cols; x++)
    free(cell[x]);
  free(cell);
}

terrain_tile::~terrain_tile() {
  int x, y;
  
  //printf("hi!\n");
  double stdev_sum = 0.0;
  int stdev_count = 0;
  double stdev_max = 0.0;
  for(x = 0; x < cols; x++) {
    for(y = 0; y < rows; y++) {
	//stdev_sum += cell[x][y].stdev;
	if(cell[x][y].stdev > 0) {
		stdev_count++;
		stdev_sum += cell[x][y].stdev;
		if(cell[x][y].stdev > stdev_max)
			stdev_max = cell[x][y].stdev;
	}
    }
  }
  //printf("SUM: %f\n", stdev_sum);
  //printf("%d cells had an average stdev of %f, max of %f\n", stdev_count, 1.0 * stdev_sum / stdev_count, stdev_max);
  for(x = 0; x < cols; x++)
    free(cell[x]);
  free(cell);
}

} //namespace vlr
