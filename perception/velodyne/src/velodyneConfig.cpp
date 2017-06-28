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


#include <inttypes.h>
#include <global.h>
#include <defaultcal.h>
#include <velodyneConfig.h>

using namespace dgc;

namespace velodyne {

typedef struct {
  double angle;
  int idx;
} beam_angle_t;

Config::Config() {
  autoConfig();
  recomputeAngles();
  findBeamOrder();
}

Config::~Config() {

}

static int beamCompare(const void *a, const void *b) {
  static beam_angle_t v1, v2;
  v1 = *(beam_angle_t *) a;
  v2 = *(beam_angle_t *) b;
  if (v1.angle > v2.angle) return 1;
  else if (v1.angle == v2.angle) return 0;
  else return -1;
}

void Config::findBeamOrder() {
  beam_angle_t beams[VELO_NUM_LASERS];

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    beams[i].angle = vert_angle[i];
    beams[i].idx = i;
  }

  qsort(beams, VELO_NUM_LASERS, sizeof(beam_angle_t), beamCompare);

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    beam_order[i] = beams[i].idx;
    inv_beam_order[beams[i].idx] = i;
  }
}

void Config::integrateOffset(const dgc_transform_t offset) {
  dgc_transform_copy(offset_, offset);
}

void Config::recomputeAngles() {
  int i, j;
  double angle = 0;
  double laser_offset_yaw = 0.0;

  for (i = 0; i < 64; i++) {
    cos_vert_angle[i] = cos(vert_angle[i]);
    sin_vert_angle[i] = sin(vert_angle[i]);

    for (j = 0; j < VELO_NUM_TICKS; j++) {
      angle = vlr::normalizeAngle(laser_offset_yaw - j / (float) VELO_NUM_TICKS * 2.0 * M_PI + rot_angle[i]);
      cos_rot_angle[j][i] = cos(angle);
      sin_rot_angle[j][i] = sin(angle);
    }
  }
  for (j = 0; j < VELO_NUM_TICKS; j++) {
    angle = vlr::normalizeAngle(laser_offset_yaw - j / (float) VELO_NUM_TICKS * 2.0 * M_PI);
    cos_enc_angle[j] = cos(angle);
    sin_enc_angle[j] = sin(angle);
  }
}

void Config::autoConfig() {
  global_range_offset = 0;
  range_multiplier = 1.0;

  for (int32_t i = 0; i < VELO_NUM_LASERS; i++) {
    rot_angle[i] = dgc::dgc_d2r(DEFAULT_VELO_ROT_ANGLE[i]);
    vert_angle[i] = dgc::dgc_d2r(DEFAULT_VELO_VERT_ANGLE[i]);
    range_offset[i] = 0;
    laser_enabled[i] = 1;
  }
}

#define MAX_LINE_LENGTH    512

bool Config::readIntensity(const std::string& filename) {
  FILE *iop;
  int i, j;

  if(filename.empty()) { // not using calibrated intensities so skip this
    return true;
  }

  char *expanded_filename = NULL;
  expanded_filename = dgc_expand_filename(filename.c_str());
  if (expanded_filename == NULL) {
    fprintf(stderr, "[31;1m# ERROR: could not expand filename %s[0m\n", filename.c_str());
    return false;
  }
  else if ((iop = fopen(expanded_filename, "r")) == 0) {
    fprintf(stderr, "[31;1m# ERROR: could not open velodyne intensity calibration file %s[0m\n", filename.c_str());
    return false;
  }
  fprintf(stderr, "# INFO: read velodyne intensity calibration file %s\n", filename.c_str());
  free(expanded_filename);

  int dummy = fscanf(iop, "%d %d\n", &min_intensity, &max_intensity);
  for (i = 0; i < 64; i++) {
    for (j = 0; j < 256; j++) {
      dummy = fscanf(iop, "%lf ", &intensity_map[i][j]);
      float expanded = (intensity_map[i][j] - min_intensity) / (max_intensity - min_intensity);
      if (expanded < 0) expanded = 0;
      if (expanded > 1) expanded = 1;
      intensity_map[i][j] = (unsigned char) (255 * expanded);
    }
  }
  fclose(iop);
  //printf("New min:%d    New max: %d\n", min_intensity, max_intensity);
  return true;
}

bool Config::readCalibration(const std::string& filename) {
  FILE * iop;

  int FEnd;
  int linectr = 0;
  int n, id, enabled;
  int i, j;
  double rcf, hcf, hoff, voff, dist, distX, distY;

  char command[MAX_LINE_LENGTH];
  char line[MAX_LINE_LENGTH];
  char str1[MAX_LINE_LENGTH];
  char str2[MAX_LINE_LENGTH];
  char str3[MAX_LINE_LENGTH];
  char str4[MAX_LINE_LENGTH];
  char str5[MAX_LINE_LENGTH];
  char str6[MAX_LINE_LENGTH];
  char str7[MAX_LINE_LENGTH];
  char str8[MAX_LINE_LENGTH];
  char str9[MAX_LINE_LENGTH];
  float range[64];
  char *expanded_filename = NULL;

  min_intensity = 0;
  max_intensity = 255;
  for (i = 0; i < 64; i++) {
    for (j = 0; j < 256; j++) {
      intensity_map[i][j] = j;
    }
  }
  for (n = 0; n < 64; n++) {
    range[n] = 0.0;
  }
  range_multiplier = 1.0;
  spin_start = VELO_SPIN_START;

  expanded_filename = dgc_expand_filename(filename.c_str());
  if (expanded_filename == NULL) {
    fprintf(stderr, "[31;1m# ERROR: could not expand filename %s[0m\n", filename.c_str());
    return false;
  }
  else if ((iop = fopen(expanded_filename, "r")) == 0) {
    fprintf(stderr, "[31;1m# ERROR: could not open velodyne calibration file %s[0m\n", filename.c_str());
    return false;
  }
  fprintf(stderr, "# INFO: read velodyne calibration file %s\n", filename.c_str());
  free(expanded_filename);

  FEnd = 0;
  do {
    if (fgets(line, MAX_LINE_LENGTH, iop) == NULL) FEnd = 1;
    else {
      linectr++;
      if (sscanf(line, "%s", command) == 0) {
        fclose(iop);
        return false;
      }
      else {
        if (command[0] != '#') {
          n = sscanf(line, "%s %s %s %s %s %s %s %s %s", str1, str2, str3, str4, str5, str6, str7, str8, str9);
          if (n == 9) {
            id = atoi(str1);
            rcf = atof(str2);
            hcf = atof(str3);
            dist = atof(str4);
            distX = atof(str5);
            distY = atof(str6);
            voff = atof(str7);
            hoff = atof(str8);
            enabled = atoi(str9);
            if (id < 0 || id > 63) {
              fprintf(stderr, "[31;1m# ERROR: wrong id '%d' in line %d[0m\n", id, linectr);
              fclose(iop);
              return false;
            }
            else {
              rot_angle[id] = dgc_d2r(rcf);
              vert_angle[id] = dgc_d2r(hcf);
              range_offset[id] = dist;
              range_offsetX[id] = distX;
              range_offsetY[id] = distY;
              laser_enabled[id] = enabled;
              v_offset[id] = voff;
              h_offset[id] = hoff;
            }
          }
          else if (n == 2) {
            if (!strcasecmp(str1, "RANGE_MULTIPLIER")) {
              range_multiplier = atof(str2);
            }
            else if (!strcasecmp(str1, "SPIN_START")) {
              spin_start = atoi(str2);
            }
            else {
              fprintf(stderr, "[31;1m# ERROR: unknown keyword '%s' in line %d[0m\n", str1, linectr);
              fclose(iop);
              return false;
            }
          }
          else {
            fprintf(stderr, "[31;1m# ERROR: error in line %d: %s[0m\n", linectr, line);
            fclose(iop);
            return false;
          }
        }
      }
    }
  } while (!FEnd);

  fclose(iop);

  recomputeAngles();
  findBeamOrder();

  return true;
}

void Config::printCalibrationData() {
  int i;

  printf("\ndouble VELO_ROT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", dgc_r2d(rot_angle[4 * i + 0]), dgc_r2d(rot_angle[4 * i + 1]),
        dgc_r2d(rot_angle[4 * i + 2]), dgc_r2d(rot_angle[4 * i + 3]));
  }
  printf("                                  };\n");

  printf("double VELO_VERT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n", dgc_r2d(vert_angle[4 * i + 0]),
        dgc_r2d(vert_angle[4 * i + 1]), dgc_r2d(vert_angle[4 * i + 2]), dgc_r2d(vert_angle[4 * i + 3]));
  }
  printf("                                  };\n");

  printf("int VELO_RANGE_OFFSET2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %3f,%3f,%3f,%3f,\n", range_offset[4 * i + 0], range_offset[4 * i + 1], range_offset[4
        * i + 2], range_offset[4 * i + 3]);
  }
  printf("                                  };\n");

  printf("char VELO_LASER_ENABLED2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %2d,%2d,%2d,%2d,\n", laser_enabled[4 * i + 0], laser_enabled[4 * i + 1],
        laser_enabled[4 * i + 2], laser_enabled[4 * i + 3]);
  }
  printf("                                  };\n");
}

} // namespace velodyne
