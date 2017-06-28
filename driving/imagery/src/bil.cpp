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


#include <string.h>
#include <cstdio>
#include <cmath>
#include <iostream>
#include <global.h>
#include <lltransform.h>

#include <bil.h>

using namespace dgc;
namespace vlr {

int edge_count[16] = { 0, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 0 };
int edge_vertices[4][4] = { { 0, 0, 1, 0 }, { 1, 0, 1, 1 }, { 0, 1, 1, 1 }, { 0, 0, 0, 1 } };
int edge_indices[16][4] = { { 0, 0, 0, 0 }, { 0, 3, 0, 0 }, { 0, 1, 0, 0 }, { 1, 3, 0, 0 }, { 1, 2, 0, 0 }, { 0, 1, 2, 3 }, { 0, 2, 0, 0 }, { 2, 3, 0, 0 }, {
    2, 3, 0, 0 }, { 0, 2, 0, 0 }, { 1, 2, 0, 3 }, { 1, 2, 0, 0 }, { 1, 3, 0, 0 }, { 0, 1, 0, 0 }, { 0, 3, 0, 0 }, { 0, 0, 0, 0 } };

void free_shortmap(shortmap_p *map) {
  int i;

  for (i = 0; i < (*map)->cols; i++)
    free((*map)->map[i]);
  free(*map);
  *map = NULL;
}

shortmap_p read_shortmap_bil(const std::string& filename) {
  shortmap_t* map = NULL;
  char line[1000];
  FILE *fp;
  short int *mapdata;
  int i, r, c;
  void *dummy;

  if (filename.compare(filename.size() - 4, 4, ".BIL") != 0) {
    throw VLRException("File " + filename + std::string(" does not have .BIL extension."));
  }
  std::string header_filename = filename.substr(0, filename.size() - 4) + ".HDR";
  std::string worldfile_filename = filename.substr(0, filename.size() - 4) + ".BLW";
  std::cout << "Reading map " << filename << "...";

  map = (shortmap_t*) calloc(1, sizeof(shortmap_t));
  dgc_test_alloc(map);

  fp = fopen(header_filename.c_str(), "r");
  if (fp == NULL) {
    free(map);
    throw VLRException("Could not open file " + header_filename);
  }
  dummy = fgets(line, 1000, fp);
  dummy = fgets(line, 1000, fp);
  if (fgets(line, 1000, fp) != line) {
    std::cout << "Error reading from map file " << header_filename << std::endl;
  }
  else {
    if (sscanf(line, "%*s %d", &map->rows) != 1) {
      std::cout << "Format error oin map file " << header_filename << std::endl;
    }
  }
  if (fgets(line, 1000, fp) != line) {
    std::cout << "Error reading from map file " << header_filename;
  }
  else if (sscanf(line, "%*s %d", &map->cols) != 1) {
    std::cout << "Format error in map file " << header_filename;
  }
  fclose(fp);

  fp = fopen(worldfile_filename.c_str(), "r");
  if (fp == NULL) {
    free(map);
    throw VLRException("Could not open file " + worldfile_filename);
  }
  if (fscanf(fp, "%lf\n", &map->resolution_degrees) != 1) {
    std::cout << "Error reading from map file " << worldfile_filename << std::endl;
  }
  if (fscanf(fp, "%*f\n%*f\n%*f\n%lf\n%lf", &map->min_lon, &map->min_lat) != 2) {
    std::cout << "Error reading from map file " << worldfile_filename << std::endl;
  }
  fclose(fp);

  map->min_lat -= map->resolution_degrees * (map->rows - 0.5);
  map->min_lon -= map->resolution_degrees * 0.5;

  map->y_resolution = map->resolution_degrees * M_PI / 180.0 * 6378100.0;
  map->x_resolution = map->y_resolution * cos(dgc_d2r(map->min_lat));

  latLongToUtm(map->min_lat, map->min_lon, &map->min_x, &map->min_y, map->utmzone);

  mapdata = (short int *) calloc(map->rows * map->cols, sizeof(short int));
  dgc_test_alloc(mapdata);

  fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    free(map);
    free(mapdata);
    throw VLRException("Could not open file " + filename);
  }

  map->map = (short int **) calloc(map->cols, sizeof(short int *));
  dgc_test_alloc(map->map);
  for (i = 0; i < map->cols; i++) {
    map->map[i] = (short int *) calloc(map->rows, sizeof(short int));
    dgc_test_alloc(map->map[i]);
  }

  if (fread(mapdata, sizeof(short int), map->rows * map->cols, fp) != (size_t) map->rows * map->cols) {
    fclose(fp);
    throw VLRException("Could not read map data from file " + filename);
  }

  fclose(fp);

  for (r = 0; r < map->rows; r++) {
    for (c = 0; c < map->cols; c++) {
      map->map[c][map->rows - r - 1] = mapdata[r * map->cols + c];
    }
  }

  free(mapdata);
  std::cout << "done.\n";
  return map;
}

void shortmap_to_pgm(shortmap_p map, const std::string& filename) {
  FILE *fp;
  float min_z = 1e6, max_z = -1e6;
  int x, y;
  unsigned char c;

  for (y = 0; y < map->rows; y++)
    for (x = 0; x < map->cols; x++)
      if (map->map[x][y] > max_z) max_z = map->map[x][y];
      else if (map->map[x][y] < min_z) min_z = map->map[x][y];

  fp = fopen(filename.c_str(), "w");
  if (!fp) {
    throw VLRException("Could not open file " + filename + std::string(" for writing"));
  }
  fprintf(fp, "P5\n%d %d\n255\n", map->cols, map->rows);
  for (y = 0; y < map->rows; y++)
    for (x = 0; x < map->cols; x++) {
      c = (unsigned char) ((map->map[x][map->rows - y - 1] - min_z) / (max_z - min_z) * 255.0);
      fwrite(&c, 1, 1, fp);
    }
  fclose(fp);
}

charmap_p read_charmap_bil(const std::string& filename) {
  char header_filename[200];
  charmap_p map;
  char line[1000];
  FILE *fp;
  unsigned char *mapdata;
  int i, r, c;
  void *dummy;

  if (filename.compare(filename.size() - 4, 4, ".BIL") != 0) {
    throw VLRException("File " + filename + std::string(" does not have .BIL extension."));
  }

  strcpy(header_filename, filename.c_str());
  strcpy(header_filename + strlen(header_filename) - 4, ".HDR");
  fprintf(stderr, "Reading map %s... ", filename.c_str());

  map = (charmap_p) calloc(1, sizeof(charmap_t));
  dgc_test_alloc(map);

  fp = fopen(header_filename, "r");
  if (fp == NULL) {
    dgc_error("Could not open file %s\n", header_filename);
    return NULL;
  }
  dummy = fgets(line, 1000, fp);
  dummy = fgets(line, 1000, fp);
  if (fgets(line, 1000, fp) != line) dgc_warning("Error reading from %s", header_filename);
  else if (sscanf(line, "%*s %d", &map->rows) != 1) dgc_warning("Formatting error in file %s", header_filename);
  if (fgets(line, 1000, fp) != line) dgc_warning("Error reading from %s", header_filename);
  else if (sscanf(line, "%*s %d", &map->cols) != 1) dgc_warning("Formatting error in file %s", header_filename);
  fclose(fp);

  map->resolution = 30.0;

  mapdata = (unsigned char *) calloc(map->rows * map->cols, sizeof(unsigned char));
  dgc_test_alloc(mapdata);

  map->map = (unsigned char **) calloc(map->cols, sizeof(unsigned char *));
  dgc_test_alloc(map->map);
  for (i = 0; i < map->cols; i++) {
    map->map[i] = (unsigned char *) calloc(map->rows, sizeof(unsigned char));
    dgc_test_alloc(map->map[i]);
  }

  fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    throw VLRException("Could not open file " + filename);
  }
  if (fread(mapdata, sizeof(unsigned char), map->rows * map->cols, fp) != (size_t) map->rows * map->cols) {
    fclose(fp);
    throw VLRException("Could not read map data from file " + filename);
  }
  fclose(fp);

  for (r = 0; r < map->rows; r++) {
    for (c = 0; c < map->cols; c++) {
      map->map[c][map->rows - r - 1] = mapdata[r * map->cols + c];
    }
  }

  free(mapdata);
  std::cout << "done.\n";
  return map;
}

void charmap_to_ppm(charmap_p map, const std::string& filename) {
  FILE *fp;
  int x, y;

  fp = fopen(filename.c_str(), "w");
  if (fp == NULL) dgc_die("Error: could not open file %s for writing.\n", filename.c_str());
  fprintf(fp, "P6\n%d %d\n255\n", map->cols, map->rows);
  for (y = 0; y < map->rows; y++)
    for (x = 0; x < map->cols; x++) {
      switch (map->map[x][y]) {
        case 11: /* water */
          fprintf(fp, "%c%c%c", 0, 0, 255);
          break;
        case 12: /* snow */
          fprintf(fp, "%c%c%c", 255, 255, 255);
          break;
        case 21:
        case 22:
        case 23: /* urban */
          fprintf(fp, "%c%c%c", 255, 0, 0);
          break;
        case 31:
        case 32:
        case 33: /* barren */
          fprintf(fp, "%c%c%c", 211, 211, 211);
          break;
        case 41:
        case 42:
        case 43:
        case 61: /* vegetated */
          fprintf(fp, "%c%c%c", 0, 100, 0);
          break;
        case 51:
        case 71: /* shrubland and grassland */
          fprintf(fp, "%c%c%c", 205, 133, 63);
          break;
        case 81:
        case 82:
        case 83:
        case 84:
        case 85: /* farmland */
          fprintf(fp, "%c%c%c", 255, 255, 0);
          break;
        case 91:
        case 92: /* wetlands */
          fprintf(fp, "%c%c%c", 64, 224, 208);
          break;
        default:
          fprintf(fp, "%c%c%c", 255, 255, 255);
          break;
      }
    }
  fclose(fp);
}

void compute_normal(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, float *nx, float *ny, float *nz) {
  float v1x, v1y, v1z, v2x, v2y, v2z;
  float len;

  v1x = x2 - x1;
  v1y = y2 - y1;
  v1z = z2 - z1;
  v2x = x3 - x1;
  v2y = y3 - y1;
  v2z = z3 - z1;
  *nx = v1y * v2z - v1z * v2y;
  *ny = -v1x * v2z + v1z * v2x;
  *nz = v1x * v2y - v1y * v2x;
  len = sqrt((*nx) * (*nx) + (*ny) * (*ny) + (*nz) * (*nz));
  if (len != 0) {
    *nx /= len;
    *ny /= len;
    *nz /= len;
  }
}

vector_t **compute_map_normals(char *map_filename, shortmap_p map) {
  vector_t **normal;
  int x, y;
  float totalx, totaly, totalz, nx, ny, nz, len;
  char normal_filename[200];
  FILE *fp;
  char temp;
  char *buffer;
  /* allocate normal memory */
  normal = (vector_t **) calloc(map->cols, sizeof(vector_t *));
  dgc_test_alloc(normal);
  for (x = 0; x < map->cols; x++) {
    normal[x] = (vector_t *) calloc(map->rows, sizeof(vector_t));
    dgc_test_alloc(normal[x]);
  }

  /* create normal file, if necessary */
  strcpy(normal_filename, map_filename);
  strcpy(normal_filename + strlen(normal_filename) - 4, ".NML");
  fp = fopen(normal_filename, "r");
  if (fp != NULL) {
    /* file already exists */
    buffer = (char *) calloc(map->rows, 3);
    dgc_test_alloc(buffer);
    for (x = 0; x < map->cols; x++) {
      if (fread(buffer, map->rows, 3, fp) != 3) dgc_die("Couldn't read from %s", normal_filename);
      for (y = 0; y < map->rows; y++) {
        normal[x][y].x = (float) buffer[y * 3] / 100.0;
        normal[x][y].y = (float) buffer[y * 3 + 1] / 100.0;
        normal[x][y].z = (float) buffer[y * 3 + 2] / 100.0;
      }
    }
    free(buffer);
    fclose(fp);
  }
  else {
    /* compute normals and write to file */
    fp = fopen(normal_filename, "w");
    fprintf(stderr, "Writing normals to file %s\n", normal_filename);
    fprintf(stderr, "Computing normals... ");
    for (x = 0; x < map->cols; x++)
      for (y = 0; y < map->rows; y++) {
        totalx = 0;
        totaly = 0;
        totalz = 0;
        if (x > 0 && y > 0) {
          compute_normal(0, 0, map->map[x - 1][y - 1], map->x_resolution, 0, map->map[x][y - 1], map->x_resolution, map->y_resolution, map->map[x][y], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
          compute_normal(0, 0, map->map[x - 1][y - 1], map->x_resolution, map->y_resolution, map->map[x][y], 0, map->y_resolution, map->map[x - 1][y], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
        }
        if (x > 0 && y < map->rows - 1) {
          compute_normal(0, 0, map->map[x - 1][y], map->x_resolution, 0, map->map[x][y], map->x_resolution, map->y_resolution, map->map[x][y + 1], &nx, &ny,
              &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
          compute_normal(0, 0, map->map[x - 1][y], map->x_resolution, map->y_resolution, map->map[x][y + 1], 0, map->y_resolution, map->map[x - 1][y + 1], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
        }
        if (x < map->cols - 1 && y > 0) {
          compute_normal(0, 0, map->map[x][y - 1], map->x_resolution, 0, map->map[x + 1][y - 1], map->x_resolution, map->y_resolution, map->map[x + 1][y], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
          compute_normal(0, 0, map->map[x][y - 1], map->x_resolution, map->y_resolution, map->map[x + 1][y], 0, map->y_resolution, map->map[x][y], &nx, &ny,
              &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
        }
        if (x < map->cols - 1 && y < map->rows - 1) {
          compute_normal(0, 0, map->map[x][y], map->x_resolution, 0, map->map[x + 1][y], map->x_resolution, map->y_resolution, map->map[x + 1][y + 1], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
          compute_normal(0, 0, map->map[x][y], map->x_resolution, map->y_resolution, map->map[x + 1][y + 1], 0, map->y_resolution, map->map[x][y + 1], &nx,
              &ny, &nz);
          totalx += nx;
          totaly += ny;
          totalz += nz;
        }
        len = sqrt(totalx * totalx + totaly * totaly + totalz * totalz);
        normal[x][y].x = totalx / len;
        normal[x][y].y = totaly / len;
        normal[x][y].z = totalz / len;
        temp = (char) rint(normal[x][y].x * 100.0);
        fwrite(&temp, 1, 1, fp);
        temp = (char) rint(normal[x][y].y * 100.0);
        fwrite(&temp, 1, 1, fp);
        temp = (char) rint(normal[x][y].z * 100.0);
        fwrite(&temp, 1, 1, fp);
      }
    fprintf(stderr, "done.\n");
    fclose(fp);
  }
  return normal;
}

#define INTERP(min,max,val) (((val)-(min))/((max)-(min)))

inline void interp_edge(shortmap_p map, float isolevel, int x1, int y1, float *x2, float *y2, int edge) {
  float frac;

  frac = INTERP(map->map[x1+edge_vertices[edge][0]][y1+edge_vertices[edge][1]],
      map->map[x1+edge_vertices[edge][2]][y1+edge_vertices[edge][3]],
      isolevel);
  if (edge_vertices[edge][0] == edge_vertices[edge][2]) *x2 = x1 + edge_vertices[edge][0];
  else *x2 = x1 + frac;
  if (edge_vertices[edge][1] == edge_vertices[edge][3]) *y2 = y1 + edge_vertices[edge][1];
  else *y2 = y1 + frac;
}

inline void marchsq_cell(shortmap_p map, int x, int y, float isolevel, marchinggrid_results_p results, int *num_lines, int *max_lines) {
  int i, sqindex = 0;

  if (x < 0 || y < 0 || x >= map->cols - 1 || y >= map->rows - 1) return;
  if (map->map[x][y] < isolevel) sqindex |= 1;
  if (map->map[x + 1][y] < isolevel) sqindex |= 2;
  if (map->map[x + 1][y + 1] < isolevel) sqindex |= 4;
  if (map->map[x][y + 1] < isolevel) sqindex |= 8;
  for (i = 0; i < edge_count[sqindex]; i++) {
    if (*num_lines == *max_lines) {
      *max_lines += 1000;
      results->line = (marchinggrid_line_p) realloc(results->line, *max_lines * sizeof(marchinggrid_line_t));
    }
    interp_edge(map, isolevel, x, y, &results->line[*num_lines].x1, &results->line[*num_lines].y1, edge_indices[sqindex][i * 2]);
    interp_edge(map, isolevel, x, y, &results->line[*num_lines].x2, &results->line[*num_lines].y2, edge_indices[sqindex][i * 2 + 1]);
    (*num_lines)++;
  }
}

void marching_squares(shortmap_p map, float isolevel, marchinggrid_results_p results) {
  int x, y;
  int num_lines, max_lines;

  num_lines = 0;
  max_lines = 1000;
  results->line = (marchinggrid_line_p) calloc(max_lines, sizeof(marchinggrid_line_t));
  dgc_test_alloc(results->line);
  for (x = 0; x < map->cols; x++)
    for (y = 0; y < map->rows; y++)
      marchsq_cell(map, x, y, isolevel, results, &num_lines, &max_lines);
  results->num_lines = num_lines;
}

} // namespace vlr
