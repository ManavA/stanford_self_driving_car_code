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


#include <sys/dir.h>
#include <global.h>
#include <gl_support.h>
#include <textures.h>
#include <lltransform.h>
#include <imagery.h>
#include <bil.h>
#include <compassrose.h>
#include <fstream>

using namespace dgc;

namespace drc = driving_common;

namespace vlr {

Imagery::Imagery(const std::string& imagery_root) :
                     min_res_({COLOR_MIN_RES, COLOR_MIN_RES, TOPO_MIN_RES, LASER_MIN_RES, GSAT_MIN_RES, DARPA_MIN_RES, BW_MIN_RES}),
                     max_res_({COLOR_MAX_RES, COLOR_MAX_RES, TOPO_MAX_RES, LASER_MAX_RES, GSAT_MAX_RES, DARPA_MAX_RES, BW_MAX_RES}),
                     cache_(imagery_root), imagery_root_(imagery_root), current_imagery_type_(NONE), ned_(NULL) {

  rose_texture_.loadRGBAFromBytes(compassrose_data, 512, 0, 255, 255, 1);
  has_imagery_type_.resize(num_types_);
}

Imagery::~Imagery() {

}

int isnt_dirlink(const struct direct* entry) {
  if (strcmp(entry->d_name, "..") == 0) return false;
  else return true;
}

bool Imagery::insideSubdir(const std::string& dir, double lat, double lon) {

  std::string filename = dir + "/bound.txt";
  std::ifstream file(filename);

  if (!file.is_open()) {return false;}

  double min_lat, min_lon, max_lat, max_lon;
  file >> min_lat;
  file >> min_lon;
  file >> max_lat;
  file >> max_lon;
  
  if(!file.good()) {
    file.close();
    throw VLRException( "Formatting error in bound.txt!" );
  }
  file.close();
  if (lat > min_lat && lat < max_lat && lon > min_lon && lon < max_lon) {return true;}
  return false;
}

bool Imagery::detectSubdir(double lat, double lon) {
  static double last_lat = 0, last_lon = 0, last_check = 0;
  static bool firsttime_scandir_error = true;
  std::string filename;
  struct dirent **namelist;
  int n, found_subdir = 0;
  struct stat file_stat;
  static bool first = true;
  double current_time;

  /* dont check more than once per second */
  current_time = drc::Time::current();
  if (current_time - last_check < 1.0) {return false;}

  /* only check if it is the first time, or we have jumped a lot */
  if (!first && !detected_subdir_.empty() && fabs(lat - last_lat) < 0.0005 && fabs(lon - last_lon) < 0.0005) {
    last_lon = lon;
    last_lat = lat;
    last_check = current_time;
    return false;
  }

  if (imagery_root_.compare(0, 7, "http://") == 0) {
    has_imagery_type_[NONE] = true;
    has_imagery_type_[COLOR] = true;
    has_imagery_type_[TOPO] = true;
    has_imagery_type_[LASER] = true;
    has_imagery_type_[GSAT] = true;
    has_imagery_type_[DARPA] = true;
    has_imagery_type_[BW] = true;
    cache_.setVersion(1);
    return false;
  }
  else if (imagery_root_.compare(0, 6, "tcp://") == 0) {
    has_imagery_type_[NONE] = true;
    has_imagery_type_[COLOR] = true;
    has_imagery_type_[TOPO] = true;
    has_imagery_type_[LASER] = true;
    has_imagery_type_[GSAT] = true;
    has_imagery_type_[DARPA] = true;
    has_imagery_type_[BW] = true;
    cache_.setVersion(2);
    return false;
  }
  else {
    cache_.setVersion(0);
  }

  if (!detected_subdir_.empty()) {
    filename = imagery_root_ + "/" + detected_subdir_;
    if (insideSubdir(filename, lat, lon)) {
      last_lon = lon;
      last_lat = lat;
      last_check = current_time;
      return false;
    }
  }

  n = scandir(imagery_root_.c_str(), &namelist, isnt_dirlink, NULL);
  if (n < 0) {
    if (firsttime_scandir_error) {
      perror("scandir");
      firsttime_scandir_error = false;
    }
  }
  else {
    while (n--) {
      filename = imagery_root_ + "/" + namelist[n]->d_name;
      stat(filename.c_str(), &file_stat);
      if (S_ISDIR(file_stat.st_mode)) {
        filename = imagery_root_ + "/" + namelist[n]->d_name;
        if (insideSubdir(filename, lat, lon)) {
          found_subdir = true;
          detected_subdir_ = namelist[n]->d_name;
          std::cout << "Using imagery directory " << detected_subdir_ << ".\n";

            // load new NED file
          if (ned_ != NULL) {free_shortmap(&ned_);}
          std::string ned_filename;
          ned_filename = imagery_root_ + "/" + detected_subdir_ + "/ned_/NED1.BIL";
          try{
            ned_ = read_shortmap_bil(ned_filename.c_str());
          }
          catch(vlr::Ex<>& e) {
            std::cout << e.what() << std::endl;
          }

            // check imagery availability
          has_imagery_type_[NONE] = true;

          std::string dirname = imagery_root_ + "/" + detected_subdir_ + "/200";
          has_imagery_type_[COLOR] = has_imagery_type_[TOPO] = dgc::dgc_file_exists(dirname.c_str());

          dirname = imagery_root_ + "/" + detected_subdir_ + "/laser";
          has_imagery_type_[LASER] = dgc::dgc_file_exists(dirname.c_str());

          dirname = imagery_root_ + "/" + detected_subdir_ + "/gsat";
          has_imagery_type_[GSAT] = dgc::dgc_file_exists(dirname.c_str());

          dirname = imagery_root_ + "/" + detected_subdir_ + "/darpa";
          has_imagery_type_[DARPA] = dgc::dgc_file_exists(dirname.c_str());

          has_imagery_type_[BW] = false;
        }
      }
      free(namelist[n]);
    }
    free(namelist);
  }

  first = 0;
  last_check = drc::Time::current();
  last_lon = lon;
  last_lat = lat;
  return 1;
}

void Imagery::cycleType() {
  if (detected_subdir_.empty()) return;
  bool current_supported = false;
  do {
    current_imagery_type_ = Type_t((int32_t)current_imagery_type_+1);
    if ((uint32_t)current_imagery_type_ >= num_types_) {current_imagery_type_ = NONE;}
    current_supported = has_imagery_type_[current_imagery_type_];
    std::cout << "type: " << (int32_t)current_imagery_type_ << "supported: " << (current_supported ? "YES" : "NO") << std::endl;
  } while (!current_supported);
}

bool Imagery::detectSubdirUtm(double utm_x, double utm_y, const std::string& utmzone) {
  double lat, lon;

  utmToLatLong(utm_x, utm_y, utmzone, &lat, &lon);
  return detectSubdir(lat, lon);
}

void Imagery::drawRoseTexture(double x, double y, double rotation, double scale, double zoom) {
  char str[50];

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(rotation, 0, 0, 1);
  glScalef(scale, scale, scale);
  rose_texture_.draw(-0.5 * rose_texture_.imageWidth(), -0.5 * rose_texture_.imageHeight(), 0.5 * rose_texture_.imageWidth(), 0.5 * rose_texture_.imageHeight(), 1);
  glPopMatrix();

  glLineWidth(2);
  glBegin(GL_LINES);
  glColor3f(1, 1, 0);
  glVertex2f(x - 65, 10);
  glVertex2f(x + 65, 10);
  glVertex2f(x - 65, 15);
  glVertex2f(x - 65, 5);
  glVertex2f(x + 65, 15);
  glVertex2f(x + 65, 5);
  glEnd();
  glPushMatrix();
  glColor3f(1, 1, 1);
  sprintf(str, "%.2f meters", 130 / zoom);
  glTranslatef(x - stroke_string_width(GLUT_STROKE_ROMAN, str) * 0.1 / 2.0, 15, 0);
  glScalef(0.1, 0.1, 1);
  render_stroke_string(GLUT_STROKE_ROMAN, str);
  glPopMatrix();
  glLineWidth(1);
}

void Imagery::drawCompassRose(double window_width, double window_height, double rotation, double zoom, Imagery::RoseLocation_t location) {
  set_display_mode_2D((int) window_width, (int) window_height);
  if (location == BOTTOM_LEFT) {drawRoseTexture(70, 80, dgc_r2d(rotation), 0.5, zoom);}
  else if (location == BOTTOM_RIGHT) {drawRoseTexture(window_width - 70, 80, dgc_r2d(rotation), 0.5, zoom);}
}

inline float lookup_height(shortmap_p ned_, double utm_x, double utm_y) {
  double x, y, t, u;
  int x1, y1, x2, y2;
  float h;

  if (ned_ == NULL) return 0;

  x = (utm_x - ned_->min_x) / ned_->x_resolution;
  y = (utm_y - ned_->min_y) / ned_->y_resolution;

  x1 = (int) floor(x);
  y1 = (int) floor(y);
  x2 = x1 + 1;
  y2 = y1 + 1;

  t = x - x1;
  u = y - y1;

  if (x1 < 0 || y1 < 0 || x2 >= ned_->cols - 1 || y2 >= ned_->rows - 1) return 0;
  else {
    h = (1 - t) * (1 - u) * ned_->map[x1][y1] + t * (1 - u) * ned_->map[x2][y1] + t * u * ned_->map[x2][y2] + (1 - t) * u * ned_->map[x1][y2];
    return h;
  }
}

#define NED_N 30

void draw_wireframe_ned3D(double x1, double y1, double x2, double y2, double origin_x, double origin_y, shortmap_p ned_, double height_factor) {
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;

  xd = (x2 - x1) / NED_N;
  yd = (y2 - y1) / NED_N;

  for (x_i = 0; x_i <= NED_N; x_i++)
    for (y_i = 0; y_i <= NED_N; y_i++)
      h[x_i][y_i] = lookup_height(ned_, origin_x + x1 + xd * x_i, origin_y + y1 + yd * y_i) * height_factor;

  glColor3f(1, 1, 1);
  for (x_i = 0; x_i <= NED_N; x_i++) {
    glBegin(GL_LINE_STRIP);
    for (y_i = 0; y_i <= NED_N; y_i++)
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);
    glEnd();
  }

  for (y_i = 0; y_i <= NED_N; y_i++) {
    glBegin(GL_LINE_STRIP);
    for (x_i = 0; x_i <= NED_N; x_i++)
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);
    glEnd();
  }
}

void Imagery::drawTexture3D(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, bool smooth) {
  double dx, dy;

  if (state == REQUESTED) {
    glBegin(GL_LINE_LOOP);
    glColor3f(0, 0, 1);
    glVertex2f(x1, y1);
    glVertex2f(x2, y1);
    glVertex2f(x2, y2);
    glVertex2f(x1, y2);
    glEnd();
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t.glTextureId());

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t.textureWidth());
    dy = 1. / (2 * t.textureHeight());
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);

  glTexCoord2f(dx, t.maxV() - dy);
  glVertex2f(x1, y1);
  glTexCoord2f(t.maxU() - dx, t.maxV() - dy);
  glVertex2f(x2, y1);
  glTexCoord2f(t.maxU() - dx, dy);
  glVertex2f(x2, y2);
  glTexCoord2f(dx, dy);
  glVertex2f(x1, y2);

  glEnd();

  glDisable(GL_TEXTURE_2D);
}

void Imagery::drawTextureNED3D(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double origin_x, double origin_y, bool smooth,
    shortmap_t* ned, double height_factor) {
  double dx, dy;
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;

  if (state == REQUESTED) {
    draw_wireframe_ned3D(x1, y1, x2, y2, origin_x, origin_y, ned, height_factor);
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t.glTextureId());

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t.textureWidth());
    dy = 1. / (2 * t.textureHeight());
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  xd = (x2 - x1) / NED_N;
  yd = (y2 - y1) / NED_N;

  for (x_i = 0; x_i <= NED_N; x_i++)
    for (y_i = 0; y_i <= NED_N; y_i++)
      h[x_i][y_i] = lookup_height(ned, origin_x + x1 + xd * x_i, origin_y + y1 + yd * y_i) * height_factor;

  glColor3f(1, 1, 1);
  for (x_i = 0; x_i < NED_N; x_i++) {
    glBegin(GL_QUAD_STRIP);
    for (y_i = 0; y_i <= NED_N; y_i++) {
      glTexCoord2f(dx + x_i / (double) NED_N * (t.maxU() - 2 * dx), t.maxV() - (dy + y_i / (double) NED_N * (t.maxV() - 2 * dy)));
      glVertex3f(x1 + xd * x_i, y1 + yd * y_i, h[x_i][y_i]);

      glTexCoord2f(dx + (x_i + 1) / (double) NED_N * (t.maxU() - 2 * dx), t.maxV() - (dy + y_i / (double) NED_N * (t.maxV() - 2 * dy)));
      glVertex3f(x1 + xd * (x_i + 1), y1 + yd * y_i, h[x_i + 1][y_i]);
    }
    glEnd();
  }

  glDisable(GL_TEXTURE_2D);
}

void Imagery::drawTexture3DFourPt(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4,
    bool smooth) {
  double dx, dy;

  if (state == REQUESTED) {
    glBegin(GL_LINE_LOOP);
    glColor3f(0, 0, 1);
    glVertex2f(x1, y1);
    glVertex2f(x2, y2);
    glVertex2f(x3, y3);
    glVertex2f(x4, y4);
    glEnd();
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t.glTextureId());

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1. / (2 * t.textureWidth());
    dy = 1. / (2 * t.textureHeight());
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  glColor3f(1, 1, 1);
  glBegin(GL_QUADS);

  glTexCoord2f(dx, t.maxV() - dy);
  glVertex2f(x1, y1);
  glTexCoord2f(t.maxU() - dx, t.maxV() - dy);
  glVertex2f(x2, y2);
  glTexCoord2f(t.maxU() - dx, dy);
  glVertex2f(x3, y3);
  glTexCoord2f(dx, dy);
  glVertex2f(x4, y4);

  glEnd();

  glDisable(GL_TEXTURE_2D);
}

void Imagery::drawTextureNED3DFourPt(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4,
    double origin_x, double origin_y, bool smooth, shortmap_p ned_, double height_factor) {
  double dx, dy;
  double h[NED_N + 1][NED_N + 1];
  double xd = (x2 - x1) / NED_N;
  double yd = (y2 - y1) / NED_N;
  int x_i, y_i;
  double xb, yb, xe, ye, xc, yc, x2_frac, x_frac, y_frac;

  if (state == REQUESTED) {
    draw_wireframe_ned3D(x1, y1, x2, y2, origin_x, origin_y, ned_, height_factor);
    return;
  }

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, t.glTextureId());

  if (smooth) {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    dx = 1 / (2 * t.textureWidth());
    dy = 1 / (2 * t.textureHeight());
  }
  else {
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    dx = 0;
    dy = 0;
  }

  xd = (x3 - x1) / NED_N;
  yd = (y3 - y1) / NED_N;

  for (x_i = 0; x_i <= NED_N; x_i++) {
    for (y_i = 0; y_i <= NED_N; y_i++) {
      h[x_i][y_i] = lookup_height(ned_, origin_x + x1 + xd * x_i, origin_y + y1 + yd * y_i) * height_factor;
    }
  }

  glColor3f(1, 1, 1);
  for (x_i = 0; x_i < NED_N; x_i++) {
    x_frac = x_i / (double) NED_N;
    x2_frac = (x_i + 1) / (double) NED_N;

    glBegin(GL_QUAD_STRIP);
    for (y_i = 0; y_i <= NED_N; y_i++) {
      y_frac = y_i / (double) NED_N;

      glTexCoord2f(dx + x_frac * (t.maxU() - 2 * dx), t.maxV() - (y_frac * (t.maxV() - 2 * dy)));

      xb = x1 + x_frac * (x2 - x1);
      yb = y1 + x_frac * (y2 - y1);
      xe = x4 + x_frac * (x3 - x4);
      ye = y4 + x_frac * (y3 - y4);
      xc = xb + y_frac * (xe - xb);
      yc = yb + y_frac * (ye - yb);

      glVertex3f(xc, yc, h[x_i][y_i]);

      glTexCoord2f(dx + x2_frac * (t.maxU() - 2 * dx), t.maxV() - (y_frac * (t.maxV() - 2 * dy)));

      xb = x1 + x2_frac * (x2 - x1);
      yb = y1 + x2_frac * (y2 - y1);
      xe = x4 + x2_frac * (x3 - x4);
      ye = y4 + x2_frac * (y3 - y4);
      xc = xb + y_frac * (xe - xb);
      yc = yb + y_frac * (ye - yb);

      glVertex3f(xc, yc, h[x_i + 1][y_i]);
    }
    glEnd();
  }

  glDisable(GL_TEXTURE_2D);
}

void Imagery::drawTerraTiles(int imagery_type, int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int terra_zone,
    int terra_res, double x_origin, double y_origin, int use_ned, shortmap_p ned, double height_factor, int fetch_only) {
  ImageState_t state;
  double x3, y3, x4, y4;
  TileId id;
  int r, c;

  id.type = imagery_type;
  id.res = terra_res;
  id.zone = terra_zone;
  id.zone_letter = ' ';
  for (r = min_tile_y; r <= max_tile_y; r++)
    for (c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      Texture* t = cache_.get(detected_subdir_, id, !fetch_only, state);
      if(!t) {continue;}

      if (!fetch_only) {
        dgc_terra_tile_bounds_utm(c, r, terra_res, &x3, &y3, &x4, &y4);
        if (use_ned) {
          drawTextureNED3D(*t, state, x3 - x_origin, y3 - y_origin, x4 - x_origin, y4 - y_origin, x_origin, y_origin, 1, ned, height_factor);
        }
        else {
          drawTexture3D(*t, state, x3 - x_origin, y3 - y_origin, x4 - x_origin, y4 - y_origin, 1);
        }
      }
    }
}

void Imagery::drawLaserTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, const std::string& utmzone, double image_resolution,
    double x_origin, double y_origin, int use_ned, shortmap_t* ned, double height_factor, int fetch_only)

{
  ImageState_t state;
  double x3, y3, x4, y4;
  char *zone_letter;
  TileId id;
  int r, c;

  id.type = LASER;
  id.res = (int) rint(image_resolution * 100);
  id.zone = strtoul(utmzone.c_str(), &zone_letter, 10);
  id.zone_letter = zone_letter[0];

  for (r = min_tile_y; r <= max_tile_y; r++)
    for (c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      Texture* t = cache_.get(detected_subdir_, id, !fetch_only, state);
      if(!t) {continue;}

      if (!fetch_only) {
        dgc_laser_tile_bounds_utm(c, r, image_resolution, &x3, &y3, &x4, &y4);
        if (use_ned) {
          drawTextureNED3D(*t, state, x3 - x_origin, y3 - y_origin, x4 - x_origin, y4 - y_origin, x_origin, y_origin, 1, ned, height_factor);
        }
        else {
          drawTexture3D(*t, state, x3 - x_origin, y3 - y_origin, x4 - x_origin, y4 - y_origin, 1);
        }
      }
    }
}

void Imagery::drawGSATTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int gmaps_zoom, double x_origin,
    double y_origin, int use_ned, shortmap_p ned_, double height_factor, int fetch_only) {
  double x1, y1, x2, y2, x3, y3, x4, y4;
  char utmzone1[10], utmzone2[10], utmzone3[10], utmzone4[10];
  ImageState_t state;
  TileId id;
  int r, c;

  id.type = GSAT;
  id.res = gmaps_zoom;
  id.zone = 0;
  id.zone_letter = ' ';
  for (r = min_tile_y; r <= max_tile_y; r++)
    for (c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      Texture* t = cache_.get(detected_subdir_, id, !fetch_only, state);
      if(!t) {continue;}

      if (!fetch_only) {
        dgc_gmaps_tile_bounds_utm(c, r, gmaps_zoom, &x1, &y1, utmzone1, &x2, &y2, utmzone2, &x3, &y3, utmzone3, &x4, &y4, utmzone4);
        if (strcmp(utmzone1, utmzone2) != 0 || strcmp(utmzone1, utmzone3) != 0 || strcmp(utmzone1, utmzone4) != 0) continue;
        if (use_ned) {
          drawTextureNED3DFourPt(*t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin, x3 - x_origin, y3 - y_origin,
                                 x4 - x_origin, y4 - y_origin, x_origin, y_origin, 1, ned_, height_factor);
        }
        else {
          drawTexture3DFourPt(*t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin, x3 - x_origin, y3 - y_origin,
                              x4 - x_origin, y4 - y_origin, 1);
        }
      }
    }
}

void Imagery::drawDarpaTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int darpa_resolution, double x_origin,
    double y_origin, int use_ned, shortmap_p ned_, double height_factor, int fetch_only) {
  double x1, y1, x2, y2, x3, y3, x4, y4;
  ImageState_t state;
  TileId id;
  int r, c;

  id.type = DARPA;
  id.res = darpa_resolution;
  id.zone = 0;
  id.zone_letter = ' ';
  for (r = min_tile_y; r <= max_tile_y; r++)
    for (c = min_tile_x; c <= max_tile_x; c++) {
      id.x = c;
      id.y = r;
      Texture* t = cache_.get(detected_subdir_, id, !fetch_only, state);
      if(!t) {continue;}

      if (!fetch_only) {
        dgc_darpa_tile_bounds_utm(c, r, darpa_resolution, &x1, &y1, &x2, &y2, &x3, &y3, &x4, &y4);
        if (use_ned) {
          drawTextureNED3DFourPt(*t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin, x3 - x_origin, y3 - y_origin,
                                 x4 - x_origin, y4 - y_origin, x_origin, y_origin, 1, ned_, height_factor);
        }
        else drawTexture3DFourPt(*t, state, x1 - x_origin, y1 - y_origin, x2 - x_origin, y2 - y_origin, x3 - x_origin, y3 - y_origin,
                                 x4 - x_origin, y4 - y_origin, 1);
      }
    }
}

void Imagery::imageryCore(double image_resolution, double x_origin, double y_origin, double x1, double y1, double x2, double y2,
    const std::string& utmzone, int use_ned, double height_factor, int fetch_only) {
  int gmaps_zoom, darpa_resolution, terra_res, terra_zone, max_tiles = 14;
  int min_tile_x, min_tile_y, max_tile_x, max_tile_y;
  double xc, yc;

  xc = (x1 + x2) / 2.0;
  yc = (y1 + y2) / 2.0;
  detectSubdirUtm(xc, yc, utmzone);

//  if (detected_subdir_ == NULL) return;

  switch (current_imagery_type_) {
    case NONE:
      return;
      break;
    case COLOR:
    case TOPO:
    case BW:
      if (cache_.lastGrayscale() && image_resolution < 1.0) image_resolution = 1.0;
      dgc_utm_to_terra_tile(x1, y1, utmzone, image_resolution, &terra_res, &min_tile_x, &min_tile_y, &terra_zone);
      dgc_utm_to_terra_tile(x2, y2, utmzone, image_resolution, &terra_res, &max_tile_x, &max_tile_y, &terra_zone);
      max_tiles = 14;
      break;
    case LASER:
      dgc_utm_to_laser_tile(x1, y1, image_resolution, &min_tile_x, &min_tile_y);
      dgc_utm_to_laser_tile(x2, y2, image_resolution, &max_tile_x, &max_tile_y);
      max_tiles = 5;
      break;
    case GSAT:
      dgc_utm_to_gmaps_tile(x1, y1, utmzone, image_resolution, &min_tile_x, &max_tile_y, &gmaps_zoom);
      dgc_utm_to_gmaps_tile(x2, y2, utmzone, image_resolution, &max_tile_x, &min_tile_y, &gmaps_zoom);
      max_tiles = 14;
      break;
    case DARPA:
      dgc_utm_to_darpa_tile(x1, y1, utmzone, image_resolution, &min_tile_x, &min_tile_y, &darpa_resolution);
      dgc_utm_to_darpa_tile(x2, y2, utmzone, image_resolution, &max_tile_x, &max_tile_y, &darpa_resolution);
      max_tiles = 14;
      break;
  }

  clampTileInterval(min_tile_x, max_tile_x, max_tiles);
  clampTileInterval(min_tile_y, max_tile_y, max_tiles);

  if (current_imagery_type_ == COLOR || current_imagery_type_ == TOPO || current_imagery_type_ == BW) {
    drawTerraTiles(current_imagery_type_, min_tile_x, min_tile_y, max_tile_x, max_tile_y, terra_zone, terra_res, x_origin, y_origin, use_ned, ned_,
      height_factor, fetch_only);
  }
  else if (current_imagery_type_ == LASER){
    drawLaserTiles(min_tile_x, min_tile_y, max_tile_x, max_tile_y, utmzone,
      image_resolution, x_origin, y_origin, use_ned, ned_, height_factor, fetch_only);
  }
  else if (current_imagery_type_ == GSAT) {
    drawGSATTiles(min_tile_x, min_tile_y, max_tile_x, max_tile_y, gmaps_zoom, x_origin,
      y_origin, use_ned, ned_, height_factor, fetch_only);
  }
  else if (current_imagery_type_ == DARPA) {
    drawDarpaTiles(min_tile_x, min_tile_y, max_tile_x, max_tile_y, darpa_resolution,
      x_origin, y_origin, use_ned, ned_, height_factor, fetch_only);
  }
}

inline void Imagery::imagery2DCore(double image_resolution, double window_width, double window_height, double zoom, double x_origin,
    double y_origin, double view_x_offset, double view_y_offset, const std::string& utmzone, int fetch_only) {
  double x1, y1, x2, y2;

  /* find window boundaries in UTM */
  x1 = x_origin + view_x_offset - window_width / 2.0 / zoom * 1.5;
  y1 = y_origin + view_y_offset - window_height / 2.0 / zoom * 1.5;
  x2 = x_origin + view_x_offset + window_width / 2.0 / zoom * 1.5;
  y2 = y_origin + view_y_offset + window_height / 2.0 / zoom * 1.5;

  imageryCore(image_resolution, x_origin, y_origin, x1, y1, x2, y2, utmzone, 0, 1.0, fetch_only);
}

inline void Imagery::imagery3DCore(double image_resolution, double x_origin, double y_origin, double view_x_offset, double view_y_offset,
    const std::string& utmzone, double radius, int use_ned, double height_factor, int fetch_only) {
  double x1, y1, x2, y2;

  /* find window boundaries in UTM */
  x1 = x_origin + view_x_offset - radius;
  y1 = y_origin + view_y_offset - radius;
  x2 = x_origin + view_x_offset + radius;
  y2 = y_origin + view_y_offset + radius;

  imageryCore(image_resolution, x_origin, y_origin, x1, y1, x2, y2, utmzone, use_ned, height_factor, fetch_only);
}

void Imagery::draw2D(double window_width, double window_height, double zoom, double x_origin, double y_origin,
    double view_x_offset, double view_y_offset, const std::string& utmzone, int prefetch) {
  double image_resolution = 1;
  double min_res = 1, max_res = 256;

  //  if(dgc_texture_cache_get_version() == 1)
  //    prefetch = 0;

  min_res = min_res_[current_imagery_type_];
  max_res = max_res_[current_imagery_type_];

  image_resolution = min_res;
  while (image_resolution * 3.0 / 2 < 1 / zoom)
    image_resolution *= 2;
  if (image_resolution > max_res) image_resolution = max_res;

  imagery2DCore(image_resolution, window_width, window_height, zoom, x_origin, y_origin, view_x_offset, view_y_offset, utmzone, 0);
  if (prefetch) {
    if (image_resolution * 2 <= max_res) imagery2DCore(image_resolution * 2, window_width, window_height, zoom * 2, x_origin, y_origin,
        view_x_offset, view_y_offset, utmzone, 1);
    if (image_resolution / 2 >= min_res) imagery2DCore(image_resolution / 2, window_width, window_height, zoom / 2, x_origin, y_origin,
        view_x_offset, view_y_offset, utmzone, 1);
  }
}

void Imagery::draw3D(double camera_distance, double camera_x_offset, double camera_y_offset, double x_origin, double y_origin,
    const std::string& utmzone, int flat, double height_factor, int prefetch) {
  double r, resc = 1, min_res = 1, max_res = 1, res;

  if (cache_.version() != 0) prefetch = 0;

  /* pick a viewing radius and resolution */
  r = camera_distance / 4000 * 3000;
  if (r < 100) r = 100;

  resc = camera_distance / 160.0 * 0.25;
  min_res = min_res_[current_imagery_type_];
  max_res = max_res_[current_imagery_type_];

  res = min_res;
  while (res * 2 < resc)
    res *= 2;
  if (res > max_res) res = max_res;

  glPushMatrix();
  if (flat) {
    glTranslatef(0, 0, -0.2);
    imagery3DCore(res, x_origin, y_origin, camera_x_offset, camera_y_offset, utmzone, r, 0, 1.0, 0);
  }
  else {
    glTranslatef(0, 0, -lookup_height(ned_, x_origin + camera_x_offset, y_origin + camera_y_offset) * height_factor - 1);
    imagery3DCore(res, x_origin, y_origin, camera_x_offset, camera_y_offset, utmzone, r, 1, height_factor, 0);
  }
  glPopMatrix();

  /* prefetch lower and higher resolution images, if requested */
  if (prefetch) {
    if (res / 2.0 >= min_res) imagery3DCore(res / 2, x_origin, y_origin, camera_x_offset, camera_y_offset, utmzone, r / 2, 0, 1.0, 1);
    if (res * 2 <= max_res) imagery3DCore(res * 2, x_origin, y_origin, camera_x_offset, camera_y_offset, utmzone, r * 2, 0, 1.0, 1);
  }
}

} // namespace vlr
