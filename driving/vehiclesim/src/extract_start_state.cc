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


#include <ipc_std_interface.h>
#include <param_interface.h>

using namespace dgc;

int main(int argc, char **argv)
{
  double vehicle_lat, vehicle_lon, vehicle_theta, vehicle_velocity;
  char *rndf_filename, *mdf_filename;
  int i, n;

  if(argc < 2) 
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s num-vehicles\n", argv[0]);

  n = atoi(argv[1]);

  for(i = 0; i < n; i++) {
    IpcInterface *ipc = new IpcStandardInterface;
    ParamInterface *pint = new ParamInterface(ipc);

    /* connect to central */
    ipc->Connect("setup", "localhost", 1381 + i);
    
    if(i == 0) {
      pint->GetString("rndf", "rndf_file", &rndf_filename, NULL);
      pint->GetString("rndf", "mdf_file", &mdf_filename, NULL);
      printf("%s\n", rndf_filename);
      printf("%s\n", mdf_filename);
      printf("%d\n", n);
    }

    /* set simulation parameters */
    pint->GetDouble("sim", "vehicle_start_latitude", &vehicle_lat, NULL);
    pint->GetDouble("sim", "vehicle_start_longitude", &vehicle_lon, NULL);
    pint->GetDouble("sim", "vehicle_start_theta", &vehicle_theta, NULL);
    pint->GetDouble("sim", "vehicle_start_velocity", &vehicle_velocity, NULL);

    printf("%.6f %.6f %f %f\n", vehicle_lat, vehicle_lon,
	   vehicle_theta, vehicle_velocity);
        
    delete pint;
    delete ipc;
  }
  return 0;
}
