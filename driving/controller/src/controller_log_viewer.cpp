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


#include <fstream>
#include <iostream>
#include <logio.h>
#include <controller/ControllerTarget.h>
#include <driving_common/CanStatus.h>
#include <applanix/ApplanixPose.h>

using namespace std;
using namespace dgc;

int main(int argc, const char* argv[])
{
  dgc_FILE *logfile;
  char *line = NULL;
  controller::ControllerTarget target;
  driving_common::CanStatus can;
  applanix::ApplanixPose applanix;
  LineBuffer line_buffer;
  double base_time = 0.0, time = 0.0, steering_angle = 0.0, velocity = 0.0;
  ofstream fout("out.dat");

  if (argc < 2) {
    cout << "Usage: ./controller_log_viewer <log file>" << endl;
    return 1;
  }
  
  logfile = dgc_fopen(argv[1], "r");
  if(logfile == NULL)
    dgc_die("Error: could not open file %s for reading.\n", argv[1]);

  /* read logfile */
  
  do {
    line = line_buffer.ReadLine(logfile);
    if (line != NULL) {

      if (strncmp(line, "CONT_TARGET", 11) == 0) {
	StringToControllerTarget(dgc_next_word(line), &target);
	fout << time << " " << target.target_velocity << " " << velocity << " "
	     << target.target_steering_angle << " " << steering_angle << " "
	     << " " << target.cross_track_error << " " << target.heading_error
	     << endl;
      } else if (strncmp(line, "CAN4", 4) == 0) {
	StringV4ToCanStatus(dgc_next_word(line), &can);
	steering_angle = can.steering_angle * (M_PI /3.14);
	cout << "got can message " << can.steering_angle << endl;
      } else if (strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	StringV2ToApplanixPose(dgc_next_word(line), &applanix);
	if (base_time == 0) {
	  base_time = applanix.timestamp;
	  time = 0.0;
	} else {
	  time = applanix.timestamp - base_time;
	}
	velocity = applanix.speed;
	cout << "got applanix message " << applanix.timestamp << " "
	     << applanix.speed << endl;
      }
							     
    }
  } while (line != NULL);
    

  dgc_fclose(logfile);

  return 0;
}

