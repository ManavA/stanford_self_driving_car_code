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


#ifndef IMAGERY_PROJ_H_
#define IMAGERY_PROJ_H_

#include <imagery.h>
#include <imageryStructs.h>

namespace vlr {

void
dgc_ll_to_gmaps_tile(double lat, double lon, int gmaps_zoom,
		     int *gmaps_x, int *gmaps_y);

void
dgc_llz_to_gmaps_tile(double lat, double lon, double image_resolution, 
		      int *gmaps_x, int *gmaps_y, int *gmaps_zoom);

void
dgc_utm_to_gmaps_tile(double easting, double northing, const std::string& zone,
		      double image_resolution, int *gmaps_x, int *gmaps_y, 
		      int *gmaps_zoom);

void
dgc_gmaps_tile_filename(TileId id, char *filename);

void
dgc_gmaps_tile_bounds(int gmaps_x, int gmaps_y, int gmaps_zoom, 
		      double *lat1, double *lon1, double *lat2, 
		      double *lon2);

void
dgc_gmaps_tile_bounds_utm(int gmaps_x, int gmaps_y, int gmaps_zoom,
			  double *x1, double *y1, char* utmzone1,
			  double *x2, double *y2, char* utmzone2,
			  double *x3, double *y3, char* utmzone3,
			  double *x4, double *y4, char* utmzone4);

void
dgc_utm_to_laser_tile(double utm_x, double utm_y, double resolution, 
		      int *laser_x, int *laser_y);

void
dgc_laser_tile_bounds_utm(int laser_x, int laser_y, double image_resolution,
			  double *x1, double *y1, double *x2, double *y2);

void
dgc_laser_tile_filename(TileId id, char *filename, int new_version);

void
dgc_utm_to_terra_tile(double utm_x, double utm_y, const std::string& utmzone,
		      double image_resolution, int *terra_res_code, 
		      int *terra_x, int *terra_y, int *terra_zone);

void
dgc_terra_tile_bounds_utm(int terra_x, int terra_y, int terra_res,
			  double *x1, double *y1, double *x2, double *y2);

void
dgc_terra_bw_tile_filename(TileId id, char *filename);

void
dgc_terra_color_tile_filename(TileId id, char *filename, int new_version);

void
dgc_terra_topo_tile_filename(TileId id, char *filename, int new_version);

void
dgc_utm_to_darpa_tile(double x, double y, const std::string& utmzone,
		      double image_resolution, int *darpa_x, 
		      int *darpa_y, int *darpa_resolution);

void
dgc_darpa_tile_bounds_utm(int darpa_x, int darpa_y, int darpa_resolution,
			  double *x1, double *y1, double *x2, double *y2,
			  double *x3, double *y3, double *x4, double *y4);

void
dgc_darpa_tile_filename(TileId id, char *filename, int new_version);

} // namespace vlr

#endif
