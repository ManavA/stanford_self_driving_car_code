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


#ifndef LLTRANSFORM_H_
#define LLTRANSFORM_H_

#include <string>

namespace vlr {

typedef struct {
  double lat;
  double lon;
} coordinate_latlon_t;

typedef struct {
  double x;
  double y;
  char zone[4];
} coordinate_utm_t;

void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, std::string& UTMZone);
void latLongToUtm(double Lat, double Long, double* UTMEasting, double* UTMNorthing, char* UTMZone);
void utmToLatLong(double UTMEasting, double UTMNorthing, const std::string& UTMZone, double* Lat,  double* Long);
void utmToLatLong(double UTMEasting, double UTMNorthing, const char* UTMZone, double* Lat,  double* Long);

int spcsInit(int zone, int verbose, int use_nad27);
int spcsToLatLong(double easting, double northing, double *latitude, double *longitude);
int latLongToSpcs(double latitude, double longitude, double* easting, double* northing);

}

#endif
