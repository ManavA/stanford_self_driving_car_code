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
#include <cmath>
#include <global.h>
#include <lltransform.h>
#include <imagery.h>
#include <imagery_proj.h>

//double log2(double x);
//double exp2(double x);

namespace vlr {

#define      GOOGLE_MINX        -1
#define      GOOGLE_MAXX        1 
#define      GOOGLE_MINY        -M_PI
#define      GOOGLE_MAXY        M_PI
#define      GOOGLE_OFF         17

#define      LASER_IMAGE_SIZE   500
#define      LASER_EXT          "png"
#define      FORCE_UTM          0

#define      TERRA_IMAGE_SIZE   200

static double imagery_x_offset[Imagery::num_types_] = {0, 0, 0, 0, 0, 0};
static double imagery_y_offset[Imagery::num_types_] = {0, 0, 0, 0, 0, 0};

/* google imagery */

void
dgc_imagery_set_offset(int imagery_type, double x_offset, double y_offset)
{
  imagery_x_offset[imagery_type] = x_offset;
  imagery_y_offset[imagery_type] = y_offset;
}

inline double gmaps_x_coord(double longitude) 
{
  return longitude / 180.0;
}

inline double gmaps_y_coord(double latitude)
{
  double rad = latitude * M_PI / 180;
  return -log(tan(rad) + 1.0 / cos(rad));
}

double gmaps_latitude(double y)
{
  double rad = 2 * atan(exp(y)) - M_PI / 2;
  return -rad * 180.0 / M_PI;
}

void dgc_ll_to_gmaps_tile(double lat, double lon, int gmaps_zoom,
			  int *gmaps_x, int *gmaps_y)
{
  double xFraction, yFraction;
  int numTiles;

  numTiles = (int)rint(exp2((double)gmaps_zoom));
  xFraction = (lon / 180.0 - GOOGLE_MINX) / (GOOGLE_MAXX - GOOGLE_MINX);
  yFraction = (gmaps_y_coord(lat) - GOOGLE_MINY) / (GOOGLE_MAXY - GOOGLE_MINY);
  *gmaps_x = (int)floor(xFraction * numTiles);
  *gmaps_y = (int)floor(yFraction * numTiles);
}

void dgc_llz_to_gmaps_tile(double lat, double lon, double image_resolution, 
			   int *gmaps_x, int *gmaps_y, int *gmaps_zoom)
{
  int steps = GOOGLE_OFF - (int)rint(log2(image_resolution));
  int numTiles = (int)rint(exp2((double)steps));

  double y = gmaps_y_coord(lat);
  double x = gmaps_x_coord(lon);
  double xFraction = (x - GOOGLE_MINX) / (GOOGLE_MAXX - GOOGLE_MINX);
  double yFraction = (y - GOOGLE_MINY) / (GOOGLE_MAXY - GOOGLE_MINY);

  *gmaps_x = (int) floor((xFraction * numTiles));
  *gmaps_y = (int) floor((yFraction * numTiles));
  *gmaps_zoom = steps;
}

void dgc_utm_to_gmaps_tile(double easting, double northing, const std::string& zone,
			   double image_resolution, int *gmaps_x, int *gmaps_y, 
			   int *gmaps_zoom)
{
  
  double lat, lon;
  
  utmToLatLong(easting, northing, zone, &lat, &lon);
  dgc_llz_to_gmaps_tile(lat, lon, image_resolution, gmaps_x, gmaps_y, 
			gmaps_zoom);
}

void dgc_gmaps_tile_filename(TileId id, char *filename)
{
  int x2, y2;
  char tempstr[200];

  sprintf(filename, "gsat/%d/", id.res);

  if(id.res >= 14) {
    x2 = id.x / 10000;
    y2 = id.y / 10000;
    sprintf(tempstr, "%02d%02d/", x2, y2);
    strcat(filename, tempstr);
    id.x %= 10000;
    id.y %= 10000;
  }

  x2 = id.x / 100;
  y2 = id.y / 100;
  id.x %= 100;
  id.y %= 100;
  sprintf(tempstr, "%02d%02d/", x2, y2);
  strcat(filename, tempstr);

  sprintf(tempstr, "%02d%02d.jpg", id.x, id.y);
  strcat(filename, tempstr);
}

void dgc_gmaps_tile_bounds(int gmaps_x, int gmaps_y, int gmaps_zoom, 
			   double *lat1, double *lon1, double *lat2, 
			   double *lon2)
{
  int numTiles = (int)rint(exp2((double)gmaps_zoom));

  *lon1 = 360 * ((double) gmaps_x / numTiles) - 180;
  *lon2 = 360 * (((double) gmaps_x + 1) / numTiles) - 180;
  *lat1 = gmaps_latitude(GOOGLE_MINY + (((double)gmaps_y + 1)/ numTiles) *
			 (GOOGLE_MAXY - GOOGLE_MINY));
  *lat2 = gmaps_latitude(GOOGLE_MINY + ((double)gmaps_y / numTiles) *
			 (GOOGLE_MAXY - GOOGLE_MINY));
}

void dgc_gmaps_tile_bounds_utm(int gmaps_x, int gmaps_y, int gmaps_zoom,
			       double *x1, double *y1, char* utmzone1,
			       double *x2, double *y2, char* utmzone2,
			       double *x3, double *y3, char* utmzone3,
			       double *x4, double *y4, char* utmzone4)
{
  int numTiles = (int)rint(exp2((double)gmaps_zoom));

  double long1 = 360 * ((double)gmaps_x / numTiles) - 180;
  double long2 = 360 * (((double)gmaps_x + 1) / numTiles) - 180;
  
  double lat1 = gmaps_latitude(GOOGLE_MINY + (((double)gmaps_y + 1)/ numTiles)
			       * (GOOGLE_MAXY - GOOGLE_MINY));
  double lat2 = gmaps_latitude(GOOGLE_MINY + ((double)gmaps_y / numTiles)
			       * (GOOGLE_MAXY - GOOGLE_MINY));
  
  latLongToUtm(lat1, long1, x1, y1, utmzone1);
  latLongToUtm(lat1, long2, x2, y2, utmzone2);
  latLongToUtm(lat2, long2, x3, y3, utmzone3);
  latLongToUtm(lat2, long1, x4, y4, utmzone4);

  *x1 += imagery_x_offset[Imagery::GSAT];
  *y1 += imagery_y_offset[Imagery::GSAT];
  *x2 += imagery_x_offset[Imagery::GSAT];
  *y2 += imagery_y_offset[Imagery::GSAT];
  *x3 += imagery_x_offset[Imagery::GSAT];
  *y3 += imagery_y_offset[Imagery::GSAT];
  *x4 += imagery_x_offset[Imagery::GSAT];
  *y4 += imagery_y_offset[Imagery::GSAT];
}

/* LASER imagery */

void dgc_utm_to_laser_tile(double utm_x, double utm_y, double resolution, 
			   int *laser_x, int *laser_y)
{
  *laser_x = (int)floor(utm_x / (double)LASER_IMAGE_SIZE / resolution);
  *laser_y = (int)floor(utm_y / (double)LASER_IMAGE_SIZE / resolution);
}

void dgc_laser_tile_bounds_utm(int laser_x, int laser_y, 
			       double image_resolution, double *x1, double *y1,
			       double *x2, double *y2)
{
  *x1 = laser_x * image_resolution * LASER_IMAGE_SIZE;
  *y1 = laser_y * image_resolution * LASER_IMAGE_SIZE;
  *x2 = (laser_x + 1) * image_resolution * LASER_IMAGE_SIZE;
  *y2 = (laser_y + 1) * image_resolution * LASER_IMAGE_SIZE;

  *x1 += imagery_x_offset[Imagery::LASER];
  *y1 += imagery_y_offset[Imagery::LASER];
  *x2 += imagery_x_offset[Imagery::LASER];
  *y2 += imagery_y_offset[Imagery::LASER];
}

void dgc_laser_tile_filename(TileId id, char *filename, int new_version)
{
  char utmzone[10];

  sprintf(utmzone, "%d%c", id.zone, id.zone_letter);
  if(new_version)
    sprintf(filename, "laser/%d/%d/lmap-%s-%d-%d-%06d-%06d.%s", 
	    id.res, id.x, FORCE_UTM ? "11S" : utmzone, 
	    id.res, LASER_IMAGE_SIZE, id.x, id.y, LASER_EXT);
  else
    sprintf(filename, "laser/lmap-%s-%d-%d-%06d-%06d.%s", 
	    FORCE_UTM ? "11S" : utmzone, id.res,
	    LASER_IMAGE_SIZE, id.x, id.y, LASER_EXT);
}

int get_terra_res(double image_resolution)
{
  if(image_resolution == 0.25)
    return 8;
  else if(image_resolution == 0.5)
    return 9;
  else if(image_resolution == 1)
    return 10;
  else if(image_resolution == 2)
    return 11;
  else if(image_resolution == 4)
    return 12;
  else if(image_resolution == 8)
    return 13;
  else if(image_resolution == 16)
    return 14;
  else if(image_resolution == 32)
    return 15;
  else if(image_resolution == 64)
    return 16;
  else if(image_resolution == 128)
    return 17;
  else if(image_resolution == 256)
    return 18;
  else if(image_resolution == 512)
    return 19;
  else
    dgc::dgc_die("Error: invalid resolution (%f)\n", image_resolution);
  return 8;
}

double terra_res_to_double(int terra_res)
{
  if(terra_res == 8)
    return 0.25;
  else if(terra_res == 9)
    return 0.5;
  else if(terra_res == 10)
    return 1.0;
  else if(terra_res == 11)
    return 2;
  else if(terra_res == 12)
    return 4;
  else if(terra_res == 13)
    return 8;
  else if(terra_res == 14)
    return 16;
  else if(terra_res == 15)
    return 32;
  else if(terra_res == 16)
    return 64;
  else if(terra_res == 17)
    return 128;
  else if(terra_res == 18)
    return 256;
  else if(terra_res == 19)
    return 512;
  else
    dgc::dgc_die("Error: invalid terra resolution (%d)\n", terra_res);
  return 0.25;
}

/* terraserver imagery */

void dgc_utm_to_terra_tile(double utm_x, double utm_y, const std::string& utmzone,
			   double image_resolution, int *terra_res_code, 
			   int *terra_x, int *terra_y, int *terra_zone) {
  int utm_multiplier;
  
  *terra_res_code = get_terra_res(image_resolution);
  utm_multiplier = (int)floor(image_resolution * TERRA_IMAGE_SIZE);
  *terra_zone = atoi(utmzone.substr(0, utmzone.size()-1).c_str());
  *terra_x = (int)floor(utm_x / (double)utm_multiplier);
  *terra_y = (int)floor(utm_y / (double)utm_multiplier);
}

void dgc_terra_tile_bounds_utm(int terra_x, int terra_y, int terra_res,
			       double *x1, double *y1, 
			       double *x2, double *y2)
{
  double image_resolution = terra_res_to_double(terra_res);
  int utm_multiplier;

  utm_multiplier = (int)floor(image_resolution * TERRA_IMAGE_SIZE);
  *x1 = terra_x * utm_multiplier;
  *y1 = terra_y * utm_multiplier;
  *x2 = *x1 + utm_multiplier;
  *y2 = *y1 + utm_multiplier;

  *x1 += imagery_x_offset[Imagery::COLOR];
  *y1 += imagery_y_offset[Imagery::COLOR];
  *x2 += imagery_x_offset[Imagery::COLOR];
  *y2 += imagery_y_offset[Imagery::COLOR];
}

void dgc_terra_color_tile_filename(TileId id, char *filename,
				   int new_version)
{
  double image_resolution;

  if(new_version)
    sprintf(filename, "usgs/4/%d/%d/usgs-%d-%d-%d-%d.jpg", 
	    id.res, id.x, id.res, id.x, id.y, id.zone);
  else {
    image_resolution = terra_res_to_double(id.res);
    if(image_resolution < 1.0)
      sprintf(filename, "%d/color/%d/usgs-%.2f-%d-%d-%d.jpg", 
	      TERRA_IMAGE_SIZE, id.x, image_resolution, id.x, id.y, id.zone);
    else
      sprintf(filename, "%d/color/%d/usgs-%d-%d-%d-%d.jpg", 
	      TERRA_IMAGE_SIZE, id.x, (int)rint(image_resolution), id.x,
	      id.y, id.zone);
  }
}

void dgc_terra_bw_tile_filename(TileId id, char *filename)
{
  sprintf(filename, "usgs/1/%d/%d/usgs-%d-%d-%d-%d.jpg", 
	  id.res, id.x, id.res, id.x, id.y, id.zone);
}

void dgc_terra_topo_tile_filename(TileId id, char *filename,
				  int new_version)
{
  double image_resolution;

  image_resolution = terra_res_to_double(id.res);
  if(new_version) {
    if(image_resolution == 4.0 || image_resolution == 16.0 || 
       image_resolution == 64.0 || image_resolution == 128.0 || 
       image_resolution == 256.0)
      sprintf(filename, "usgs/2/%d/%d/usgs-%d-%d-%d-%d.jpg", 
	      id.res, id.x, id.res, id.x, id.y, id.zone);
    else
      sprintf(filename, "usgs/2/%d/%d/usgs-%d-%d-%d-%d.gif", 
	      id.res, id.x, id.res, id.x, id.y, id.zone);
  }
  else {
    if(image_resolution < 1.0)
      sprintf(filename, "%d/topo/%d/usgs-%.2fd-%d-%d-%d.gif",
	      TERRA_IMAGE_SIZE, id.x, image_resolution, id.x, id.y, id.zone);
    else if(image_resolution == 4.0 || image_resolution == 16.0 || 
	    image_resolution == 64.0 || image_resolution == 128.0 || 
	    image_resolution == 256.0)
      sprintf(filename, "%d/topo/%d/usgs-%d-%d-%d-%d.jpg",
	      TERRA_IMAGE_SIZE, id.x, (int)rint(image_resolution), id.x,
	      id.y, id.zone);
    else
      sprintf(filename, "%d/topo/%d/usgs-%d-%d-%d-%d.gif",
	      TERRA_IMAGE_SIZE, id.x, (int)rint(image_resolution), id.x, 
	      id.y, id.zone);
  }
}

/* darpa provided imagery */

void utm2spcs(double utm_x, double utm_y, const std::string& utmzone,
	      double *spcs_x, double *spcs_y)
{
  double lat, lon;

  utmToLatLong(utm_x, utm_y, utmzone, &lat, &lon);
  latLongToSpcs(lat, lon, spcs_x, spcs_y);
}

void spcs2utm(double spcs_x, double spcs_y, double *utm_x,
	      double *utm_y, std::string& utmzone)
{
  double lat, lon;

  spcsToLatLong(spcs_x, spcs_y, &lat, &lon);
  latLongToUtm(lat, lon, utm_x, utm_y, utmzone);
}

void dgc_utm_to_darpa_tile(double x, double y, const std::string& utmzone,
			   double image_resolution, int *darpa_x, 
			   int *darpa_y, int *darpa_resolution)
{
  double spcs_x, spcs_y, min_res;

  
  *darpa_resolution = 1;
  min_res = dgc::dgc_surveyor_feet2meters(0.5);
  while(min_res * 2 < image_resolution) {
    *darpa_resolution *= 2;
    min_res *= 2;
  }

  utm2spcs(x, y, utmzone, &spcs_x, &spcs_y);

  *darpa_x = (int)floor(spcs_x / (dgc::dgc_surveyor_feet2meters(0.5) *
				  *darpa_resolution) / 256.0);
  *darpa_y = (int)floor(spcs_y / (dgc::dgc_surveyor_feet2meters(0.5) *
				  *darpa_resolution) / 256.0);
}

void dgc_darpa_tile_bounds_utm(int darpa_x, int darpa_y, int darpa_resolution,
			       double *x1, double *y1, double *x2, double *y2,
			       double *x3, double *y3, double *x4, double *y4)
{
  double spcs_x, spcs_y;
  std::string utmzone;

  spcs_x = darpa_x * 256 * dgc::dgc_surveyor_feet2meters(0.5) * darpa_resolution;
  spcs_y = darpa_y * 256 * dgc::dgc_surveyor_feet2meters(0.5) * darpa_resolution;
  spcs2utm(spcs_x, spcs_y, x1, y1, utmzone);
  *x1 += imagery_x_offset[Imagery::DARPA];
  *y1 += imagery_y_offset[Imagery::DARPA];

  spcs_x = (darpa_x + 1) * 256 * dgc::dgc_surveyor_feet2meters(0.5) *
    darpa_resolution;
  spcs_y = darpa_y * 256 * dgc::dgc_surveyor_feet2meters(0.5) *
    darpa_resolution;
  spcs2utm(spcs_x, spcs_y, x2, y2, utmzone);
  *x2 += imagery_x_offset[Imagery::DARPA];
  *y2 += imagery_y_offset[Imagery::DARPA];

  spcs_x = (darpa_x + 1) * 256 * dgc::dgc_surveyor_feet2meters(0.5) *
    darpa_resolution;
  spcs_y = (darpa_y + 1) * 256 * dgc::dgc_surveyor_feet2meters(0.5) *
    darpa_resolution;
  spcs2utm(spcs_x, spcs_y, x3, y3, utmzone);
  *x3 += imagery_x_offset[Imagery::DARPA];
  *y3 += imagery_y_offset[Imagery::DARPA];

  spcs_x = darpa_x * 256 * dgc::dgc_surveyor_feet2meters(0.5) * darpa_resolution;
  spcs_y = (darpa_y + 1) * 256 * dgc::dgc_surveyor_feet2meters(0.5) *
    darpa_resolution;
  spcs2utm(spcs_x, spcs_y, x4, y4, utmzone);
  *x4 += imagery_x_offset[Imagery::DARPA];
  *y4 += imagery_y_offset[Imagery::DARPA];
}

void dgc_darpa_tile_filename(TileId id, char *filename,
			     int new_version)
{
  if(new_version)
    sprintf(filename, "darpa/%d/%d/darpa-405-%d-%d-%d.jpg", 
	    id.res, id.x, id.res, id.x, id.y);
  else
    sprintf(filename, "darpa/%d/darpa-405-%d-%d-%d.jpg", 
	    id.x, id.res, id.x, id.y);
}

} // namespace vlr
