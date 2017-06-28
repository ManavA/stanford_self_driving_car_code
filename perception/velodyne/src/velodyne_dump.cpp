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


#include <param_interface.h>
#include <applanix_interface.h>
#include <passat_constants.h>
#include <transform.h>
#include <velodyne_shm_interface.h>
#include <velocore.h>

using namespace dgc;

#define MAX_NUM_SCANS	10000

int main(void)
{
  dgc_velodyne_config_p	  veloconfig = NULL;
  dgc_velodyne_scan_p	  scans = NULL;
  int			  num_scans = 0;
  int scanctr = 0;
  int n, b;

  VelodyneInterface *velo_interface = new VelodyneShmInterface;
  if(velo_interface->CreateClient() < 0)
    dgc_die("Error: could not connect to velodyne interface.\n");
  
  dgc_velodyne_get_config(&veloconfig);
  scans = (dgc_velodyne_scan_p)malloc(MAX_NUM_SCANS * 
				      sizeof(dgc_velodyne_scan_t));
  dgc_test_alloc(scans);

  while(1) {
    while(velo_interface->ScanDataWaiting()) {
      num_scans = velo_interface->ReadScans(scans, MAX_NUM_SCANS);
      if(num_scans > 0) {
	printf( "#SCAN %d %d\n",
		scanctr++, num_scans * VELO_BEAMS_IN_SCAN);
	for(n = 0; n < num_scans; n++) {
	  for(b = 0; b < VELO_BEAMS_IN_SCAN; b++) {
	    printf("%f %f %f %d\n",
		   scans[n].p[b].x * 0.01 + scans[n].robot.x,
		   scans[n].p[b].y * 0.01 + scans[n].robot.y,
		   scans[n].p[b].z * 0.01 + scans[n].robot.z,
		   scans[n].p[b].intensity);
	  }
	}
      }
    }
    usleep(500);
  }
  return 0;
}
