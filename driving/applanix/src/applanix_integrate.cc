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


#include <global.h>

#include <applanix/ApplanixPose.h>
#include <applanix/ApplanixRMS.h>
#include <applanix/ApplanixDMI.h>
#include <applanix/ApplanixGPS.h>

using namespace dgc;
using namespace vlr;

dgc_FILE *in = NULL, *out = NULL, *pp = NULL;
char *output_name = NULL;

#define       BUFFER_SIZE      (500000)

char log_buffer[BUFFER_SIZE];
char pp_buffer[BUFFER_SIZE];

const double display_refresh = .25;

ApplanixPose pose;
ApplanixRms rms;
double pp_time, pp_latitude, pp_longitude, pp_altitude;
double pp_roll, pp_pitch, pp_heading;
double pp_v_north, pp_v_east, pp_v_up, pp_rms_north, pp_rms_east;
double pp_rms_up, pp_rms_roll, pp_rms_pitch, pp_rms_heading;
int got_pose = 0;

static void shutdown_module(int x)
{
  if(x == SIGINT) {
    fprintf(stderr, "\nCaught SIGINT.  Closing files.\n");
    dgc_fclose(in);
    dgc_fclose(out);
    dgc_fclose(pp);
    exit(1);
  }
}

void populate_output_name(const char *input_name)
{
  if(strlen(input_name) < 7 || 
     strcmp(".log.gz", input_name + strlen(input_name) - 7) != 0) {
    fprintf(stderr,
            "Error: Input file does not have the correct extension.\n");
    exit(-1);
  }
  
  if(output_name != NULL)
    free(output_name);
  
  output_name = (char *)malloc((strlen(input_name) + 
                                strlen("_POSTPROCESSED") + 1) * sizeof(char));
  if(output_name == NULL) {
    fprintf(stderr, "Error: Out of memory.\n");
    exit(-1);
  }
  strcpy(output_name, input_name);
  strcpy(output_name + strlen(input_name) - 7, "_POSTPROCESSED.log.gz");
  return;
}

static int getline(dgc_FILE *fp, char *buf, int buf_size)
{
  int t, index = 0;
  
  if(buf_size <= 0) {
    fprintf(stderr, "Error: Buffer size is <= 0.  Huh?\n");
    exit(-1);
  }
  buf[0] = '\0';
  
  while((t = dgc_fgetc(fp)) != EOF) {
    if(index >= buf_size - 1) {
      fprintf(stderr, "Error: Line buffer too small.\n");
      exit(-1);
    }
        
    buf[index++] = t;
    
    if(t == '\n')
      break;
  }
  buf[index] = '\0';
  
  if(t == EOF)
    return EOF;
  return 0;
}

void populate_pp_pose_from_string(char *string)
{
  sscanf(string, "\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\"%lf\"\n",
         &pp_time, &pp_latitude, &pp_longitude, &pp_altitude, &pp_roll, 
         &pp_pitch, &pp_heading, &pp_v_north, &pp_v_east,
         &pp_v_up, &pp_rms_north, &pp_rms_east, &pp_rms_up, &pp_rms_roll, 
         &pp_rms_pitch, &pp_rms_heading);
}

void find_and_replace(void)
{
  /* Find the first pose message. */
  while(1) {
    int eof_code;
    eof_code = getline(in, log_buffer, BUFFER_SIZE);
    
    if(strncmp(log_buffer, "APPLANIX_POSE_V1 ", 
               strlen("APPLANIX_POSE_V1 ")) == 0) {
      StringV1ToApplanixPose(log_buffer + 
			     strlen("APPLANIX_POSE_V1 "),
			     &pose);
      break;
    }
    else if(strncmp(log_buffer, "APPLANIX_POSE_V2 ", 
		    strlen("APPLANIX_POSE_V2 ")) == 0) {
      StringV2ToApplanixPose(log_buffer + 
			     strlen("APPLANIX_POSE_V2 "),
			     &pose);
      break;
    }
    
    if(eof_code == EOF) {
      fprintf(stderr, "Error: Log file contains no pose messages.\n");
      return;
    }
  }
  
  while(1) {
    if(fabs(pp_time - pose.hardware_timestamp) < .001)
      break;

    if(getline(pp, pp_buffer, BUFFER_SIZE) == EOF) {
      fprintf(stderr, "Error: Post-processed file timestamps"
              " don't overlap with log file.\n");
      exit(-1);
    }
    
    populate_pp_pose_from_string(pp_buffer);                
  }
  
  /* Now rewrite the log file. */        
  double hardware_time_start, real_time_start, last_display = 0;
  
  hardware_time_start = pose.hardware_timestamp;
  real_time_start = dgc_get_time();
  
  dgc_fseek(in, 0L, SEEK_SET);
  
  dgc_fprintf(out,
             "# The Applanix data in this file has been post-processed.\n");
        
  int break_next = 0;
  while(!break_next) {
    break_next = getline(in, log_buffer, BUFFER_SIZE);
    
    if(strncmp(log_buffer, "APPLANIX_POSE_", strlen("APPLANIX_POSE_")) == 0) {
      char *s = NULL;
      double logger_timestamp;

      if(strncmp(log_buffer, "APPLANIX_POSE_V1 ", 
		 strlen("APPLANIX_POSE_V1 ")) == 0) {
	s = StringV1ToApplanixPose(dgc_next_word(log_buffer), &pose);
      }
      else if(strncmp(log_buffer, "APPLANIX_POSE_V2 ", 
		      strlen("APPLANIX_POSE_V2 ")) == 0) {
	s = StringV2ToApplanixPose(dgc_next_word(log_buffer), &pose);
      }
      else
	dgc_die("Error: unknown applanix pose message type.\n");

      logger_timestamp = atof(s);
      
      if(fabs(pp_time - pose.hardware_timestamp) > .001) {
        fprintf(stderr, "\nWarning: Post processing dropped a timestamp."
                "  Will be dropped from log file.\n");
        continue;
      }

      pose.latitude = pp_latitude;
      pose.longitude = pp_longitude;
      pose.altitude = pp_altitude;
      
      pose.v_north = pp_v_north;
      pose.v_east = pp_v_east;
      pose.v_up = pp_v_up;
      
      pose.roll = dgc_d2r(pp_roll);
      pose.pitch = dgc_d2r(-1. * pp_pitch);
      pose.yaw = vlr::normalizeAngle(dgc_d2r(-1. * pp_heading) + dgc_d2r(90.));
      
      pose.speed = sqrt(pose.v_north * pose.v_north + 
                        pose.v_east * pose.v_east);
      pose.track = vlr::normalizeAngle(atan2(pose.v_north, pose.v_east));
      
      pose.postprocess_code = 1;
      
      ApplanixPoseWrite(&pose, logger_timestamp, out);
      
      if(got_pose == 0)
        got_pose = 1;

      double this_display;
      if((this_display = dgc_get_time()) - last_display > display_refresh) {
        int hours, minutes, seconds;
        
        hours = (int)floor((pose.hardware_timestamp - 
			    hardware_time_start) / 60. / 60.);
        minutes = (int)floor((pose.hardware_timestamp - 
			      hardware_time_start) / 60. -
			     (hours * 60.));
        seconds = (int)floor((pose.hardware_timestamp - hardware_time_start) - 
			     (hours * 60. * 60.) - (minutes * 60.));
        
        fprintf(stderr, "\rProcessed %02d Hrs %02d Min %02d "
		"Sec    %.3fx    %c    ", 
		hours, minutes, seconds,
		(pose.hardware_timestamp - hardware_time_start) / 
		(dgc_get_time() - real_time_start), dgc_ascii_rotor());
        last_display = this_display;
      }
      
      if(getline(pp, pp_buffer, BUFFER_SIZE) == EOF) {
        fprintf(stderr, "Error: Post-processed file is too short.\n");
        exit(-1);
      }
      populate_pp_pose_from_string(pp_buffer);
    }
    else if(strncmp(log_buffer, "APPLANIX_RMS_V1 ", 
                    strlen("APPLANIX_RMS_V1 ")) == 0) {
      if(got_pose == 0)        {
        fprintf(stderr, "Warning: RMS message comes before pose message."
                "  Will not be post-processed.\n");
        dgc_fprintf(out, log_buffer);
      }
      else {
        char *logger_timestamp_as_string;
        logger_timestamp_as_string = 
	  StringToApplanixRms(log_buffer + 
			      strlen("APPLANIX_RMS_V1 "), &rms);
        
        double logger_timestamp;
        logger_timestamp = atof(logger_timestamp_as_string);
        
        rms.rms_north = pp_rms_north;
        rms.rms_east = pp_rms_east;
        rms.rms_up = pp_rms_up;
        
        rms.rms_roll = dgc_d2r(pp_rms_roll);
        rms.rms_pitch = dgc_d2r(pp_rms_pitch);
        rms.rms_yaw = dgc_d2r(pp_rms_heading);
        
        rms.postprocess_code = 1;
        
        ApplanixRmsWrite(&rms, logger_timestamp, out);
      }
    }
    else
      dgc_fprintf(out, log_buffer);
  }
  fprintf(stderr, "\n");
}

int main(int argc, char **argv)
{
  if(argc < 3) { 
    fprintf(stderr, "Error: Usage: %s input.applanix_pp input1.log.gz"
            " [input2.log.gz] ...\n", argv[0]);
    return -1;
  }
  
  fprintf(stderr, "Opening %s for reading.\n", argv[1]);
  pp = dgc_fopen(argv[1], "r");
  if(pp == NULL) {
    fprintf(stderr, "Error: Cannot open %s for reading.\n", argv[1]);
    return -2;
  }
  /* Cut the first line of the post-processed file. (Header information.) */
  getline(pp, pp_buffer, BUFFER_SIZE);        
  
  int i;
  for(i = 2; i < argc; i++) {
    fprintf(stderr, "Opening %s for reading.\n", argv[i]);
    in = dgc_fopen(argv[i], "r");
    if(in == NULL) {
      fprintf(stderr, "Error: Cannot open %s for reading.\n", argv[i]);
      return -2;
    }
    
    populate_output_name(argv[i]);
    fprintf(stderr, "Opening %s for writing.\n", output_name);
    out = dgc_fopen(output_name, "w");
    if(out == NULL) {
      fprintf(stderr, "Error: Cannot open %s for writing.\n", output_name);
      return -3;
    }
    
    signal(SIGINT, shutdown_module);
    
    find_and_replace();
    
    dgc_fclose(in);
    fprintf(stderr, "Closed %s.\n", argv[i]);
    
    dgc_fclose(out);
    fprintf(stderr, "Closed %s.\n", output_name);
  }
  
  dgc_fclose(pp);
  fprintf(stderr, "Closed %s.\n", argv[1]);
  return 0;
}
