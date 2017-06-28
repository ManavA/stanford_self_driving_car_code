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


#include <projects.h>
#include <string.h>
#include <global.h>

namespace vlr {

static int spcs_initialized = 0;
static PJ *Proj = NULL;

int spcsInit(int zone, int verbose, int use_nad27) {
  char *pargv[10];

  pargv[0] = (char*)calloc(100, 1);
  strcpy(pargv[0], "+units=m");
  pargv[1] = (char*)calloc(100, 1);
  if (use_nad27)
    sprintf(pargv[1], "+init=/usr/share/proj/nad27:%d", zone);
  else
    sprintf(pargv[1], "+init=/usr/share/proj/nad83:%d", zone);
  if (!(Proj = pj_init(2, pargv))) {
    printf("Error: could not initialize conversion library.\n");
    return -1;
  }
  if (verbose) pj_pr_list(Proj);
  spcs_initialized = 1;
  return 0;
}

int spcsToLatLong(double easting, double northing, double *latitude, double *longitude) {
  projUV data;

  if (!spcs_initialized) spcsInit(405, 0, 0);
  data.u = easting;
  data.v = northing;
  data = pj_inv(data, Proj);
  *longitude = dgc::dgc_r2d(data.u);
  *latitude = dgc::dgc_r2d(data.v);
  if (data.u == HUGE_VAL)
    return -1;
  else
    return 0;
}

int latLongToSpcs(double latitude, double longitude, double* easting, double* northing) {
  projUV data;

  if (!spcs_initialized) spcsInit(405, 0, 0);
  data.u = longitude * M_PI / 180.0;
  data.v = latitude * M_PI / 180.0;
  data = pj_fwd(data, Proj);
  *easting = data.u;
  *northing = data.v;
  if (data.u == HUGE_VAL)
    return -1;
  else
    return 0;
}

} // namespace vlr
