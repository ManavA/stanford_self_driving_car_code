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


#ifndef VLR_IMAGERY_H_
#define VLR_IMAGERY_H_

#include <bil.h>
#include <imagery_proj.h>
#include <textureCache.h>

namespace vlr {

class Imagery {

public:
  static const double COLOR_MIN_RES = 0.25, TOPO_MIN_RES = 2.0, LASER_MIN_RES = 0.15;
  static const double GSAT_MIN_RES = 0.30, DARPA_MIN_RES = 0.1, BW_MIN_RES = 1.0;

  static const double COLOR_MAX_RES = 256, TOPO_MAX_RES = 256, LASER_MAX_RES = 38.4;
  static const double GSAT_MAX_RES = 512, DARPA_MAX_RES = 512, BW_MAX_RES = 256;

  typedef enum {NONE=0, COLOR, TOPO, LASER, GSAT, DARPA, BW} Type_t;
  typedef enum {BOTTOM_LEFT, BOTTOM_RIGHT} RoseLocation_t;

  static const uint32_t num_types_ = 7; // public until gcc understands generalized constant expressions

private:
  std::vector<double> min_res_;
  std::vector<double> max_res_;

public:
  Imagery(const std::string& imagery_root = "");
  virtual ~Imagery();

  //  constexpr static uint32_t numTypes() {return num_types_;} or so ...
  static uint32_t numTypes() {return num_types_;}
  void setOffset(Type_t imagery_type, double x_offset, double y_offset);

  void setMinResolution(Type_t image_type, double min_resolution) {min_res_[image_type] = min_resolution;}
  void setMaxResolution(Type_t image_type, double max_resolution) {max_res_[image_type] = max_resolution;}

  float lookupHeight(shortmap_p ned, double utm_x, double utm_y);


  void cycleType();
  Type_t currentType() {return current_imagery_type_;}
  void setType(Type_t imagery_type) {current_imagery_type_ = imagery_type;}

  bool update() {return cache_.syncCache();}
  void stop() {cache_.stopCaching();}

  void draw2D(double window_width, double window_height, double zoom, double x_origin, double y_origin,
      double camera_x_offset, double camera_y_offset, const std::string& utmzone, int prefetch);

  void draw3D(double camera_distance, double camera_x_offset, double camera_y_offset, double x_origin, double y_origin,
      const std::string& utmzone, int flat, double height_factor, int prefetch);

  void drawCompassRose(double window_width, double window_height, double rotation, double zoom, RoseLocation_t location);

private:
  bool insideSubdir(const std::string& dir, double lat, double lon);
  bool detectSubdir(double lat, double lon);
  bool detectSubdirUtm(double utm_x, double utm_y, const std::string& utmzone);

  void drawTexture3D(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, bool smooth);

  void drawTexture3DFourPt(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4,
      bool smooth);

  void drawTextureNED3D(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double origin_x, double origin_y, bool smooth,
      shortmap_p ned_, double height_factor);

  void drawTextureNED3DFourPt(Texture& t, ImageState_t state, double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4,
      double origin_x, double origin_y, bool smooth, shortmap_p ned, double height_factor);

  void drawRoseTexture(double x, double y, double rotation, double scale, double zoom);

  void drawLaserTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, const std::string& utmzone, double image_resolution,
      double x_origin, double y_origin, int use_ned, shortmap_p ned, double height_factor, int fetch_only);

  void drawGSATTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int gmaps_zoom, double x_origin,
      double y_origin, int use_ned, shortmap_p ned, double height_factor, int fetch_only);

  void drawDarpaTiles(int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int darpa_resolution, double x_origin,
      double y_origin, int use_ned, shortmap_p ned, double height_factor, int fetch_only);

  void drawTerraTiles(int imagery_type, int min_tile_x, int min_tile_y, int max_tile_x, int max_tile_y, int terra_zone,
      int terra_res, double x_origin, double y_origin, int use_ned, shortmap_p ned_, double height_factor, int fetch_only);

    void imageryCore(double image_resolution, double x_origin, double y_origin, double x1, double y1, double x2, double y2,
      const std::string& utmzone, int use_ned, double height_factor, int fetch_only);

  void imagery2DCore(double image_resolution, double window_width, double window_height, double zoom, double x_origin,
      double y_origin, double view_x_offset, double view_y_offset, const std::string& utmzone, int fetch_only);

  inline void imagery3DCore(double image_resolution, double x_origin, double y_origin, double view_x_offset, double view_y_offset,
      const std::string& utmzone, double radius, int use_ned, double height_factor, int fetch_only);

    inline void clampTileInterval(int32_t& min, int32_t& max, int32_t w) {
    int32_t center = 0.5*(max + min);
    if (max - min + 1 > w) {
      min = center - 0.5*w;
      max = min + w - 1;
    }
  }

private:
  TextureCache cache_;

  std::string imagery_root_;
  std::string detected_subdir_;

  Type_t current_imagery_type_;
  std::vector<bool> has_imagery_type_;

  Texture rose_texture_;
  shortmap_t* ned_;




};

} // namespace vlr

#endif

