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


#include <logio.h>
#include <lltransform.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <vector>

using namespace dgc;

typedef struct {
  double x, y, yaw;
  double timestamp;
} pose_t;

void log_to_poselist(char *filename, std::vector <pose_t> &pose)
{
  ApplanixPose applanix_pose;
  LineBuffer *line_buffer = NULL;
  char *line = NULL, *s, utmzone[10];
  dgc_FILE *fp;
  double utm_x, utm_y;
  pose_t p;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);

  line_buffer = new LineBuffer();
  do {
    /* read a complete line */
    //    do {
    line = line_buffer->ReadLine(fp);
    //    } while(line == NULL && !line_buffer->eof);
    
    if(line != NULL) {
      //      fprintf(stderr, "line = *%s*\n", line);
      
      if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
	s = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude,
		    &utm_x, &utm_y, utmzone);
	p.x = 1 * utm_x + 0 * applanix_pose.smooth_x;
	p.y = 1 * utm_y + 0 * applanix_pose.smooth_y;
	p.yaw = applanix_pose.yaw;
	p.timestamp = applanix_pose.timestamp;
	pose.push_back(p);
      }
    }
  } while(line != NULL);
  dgc_fclose(fp);
}

void localized_log_to_poselist(char *filename, std::vector <pose_t> &pose)
{
  ApplanixPose applanix_pose;
  LocalizePose localize_pose;
  LineBuffer *line_buffer = NULL;
  int received_localize = 0;
  char *line = NULL, *s, utmzone[10];
  dgc_FILE *fp;
  double utm_x, utm_y;
  pose_t p;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);

  line_buffer = new LineBuffer();
  do {
    /* read a complete line */
    line = line_buffer->ReadLine(fp);
    if(line != NULL) {
      if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0 && received_localize) {
	s = StringV2ToApplanixPose(dgc_next_word(line), &applanix_pose);
	vlr::latLongToUtm(applanix_pose.latitude, applanix_pose.longitude,
		    &utm_x, &utm_y, utmzone);
	p.x = utm_x - localize_pose.x_offset;
	p.y = utm_y - localize_pose.y_offset;
	//p.x = applanix_pose.smooth_x + localize_pose.x_offset;
	//p.y = applanix_pose.smooth_y + localize_pose.y_offset;
	p.yaw = applanix_pose.yaw;
	p.timestamp = applanix_pose.timestamp;
	pose.push_back(p);
      }
      else if(strncmp(line, "LOCALIZE_POSE2", 14) == 0) {
	s = StringV2ToLocalizePose(dgc_next_word(line), &localize_pose);
	received_localize = 1;
      }
    }
  } while(line != NULL);
  dgc_fclose(fp);
}

void locfile_to_poselist(char *filename, std::vector <pose_t> &pose)
{
  LineBuffer *line_buffer = NULL;
  char *line = NULL;
  dgc_FILE *fp;
  pose_t p;
  double tmp1, tmp2, dx, dy, dlat, dlon;
  double sum_lat = 0, sum_lon = 0;
  int count = 0;

  fp = dgc_fopen(filename, "r");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for reading.\n", filename);

  line_buffer = new LineBuffer();
  do {
    /* read a complete line */
    line = line_buffer->ReadLine(fp);

    if(line != NULL) {
      if(strncmp(line, "ts", 2) != 0) {
	sscanf(line, "%lf %lf %lf %lf %lf %lf %lf %lf %lf\n", &p.timestamp, &p.x, &p.y, &tmp1, &tmp2, &dx, &dy, &dlat, &dlon);
	p.yaw = 0;
	pose.push_back(p);


	sum_lat += dlat * dlat;
	sum_lon += dlon * dlon;
	count++;
      }
    }
  } while(line != NULL);
  dgc_fclose(fp); 
  printf("RMS lateral correction by localizer: %.3f\n", pow(sum_lat/count, .5));
  printf("RMS longitudinal correction by localizer: %.3f\n", pow(sum_lon/count, .5));
}

void test_accuracy(std::vector <pose_t> &ref_pose, 
		   std::vector <pose_t> &loc_pose,
		   char *outfilename)
{
  double sum = 0, lat_sum = 0, lon_sum = 0, dx, dy, yaw;
  double lat_err, lon_err;
  int i, j, count = 0;
  FILE *fp;

  fp = fopen(outfilename, "w");
  if(fp == NULL)
    dgc_die("Error: could not open file %s for writing.\n", outfilename);

  for(i = 0; i < (int)loc_pose.size(); i++) {
    for(j = 0; j < (int)ref_pose.size(); j++) {
      if(fabs(loc_pose[i].timestamp - ref_pose[j].timestamp) < 0.001) {
	sum += dgc_square(hypot(loc_pose[i].x - ref_pose[j].x,
				loc_pose[i].y - ref_pose[j].y));

	dx = loc_pose[i].x - ref_pose[j].x;
	dy = loc_pose[i].y - ref_pose[j].y;
	yaw = ref_pose[j].yaw;

	lon_err = dx * cos(yaw) + dy * sin(yaw);
	lat_err = -dx * sin(yaw) + dy * cos(yaw);

	lon_sum += dgc_square(dx * cos(yaw) + dy * sin(yaw));
	lat_sum += dgc_square(-dx * sin(yaw) + dy * cos(yaw));

	fprintf(fp, "%f %f %f %f %f\n", 
		loc_pose[i].timestamp - ref_pose[0].timestamp,
		loc_pose[i].x - ref_pose[j].x,
		loc_pose[i].y - ref_pose[j].y, lat_err, lon_err);


	count++;
	break;
      }
    }
  }

  fclose(fp);

  fprintf(stderr, "RMS error : %f\n", sqrt(sum / count));
  fprintf(stderr, "RMS lat error : %f\n", sqrt(lat_sum / count));
  fprintf(stderr, "RMS lon error : %f\n", sqrt(lon_sum / count));
}

int main(int argc, char **argv)
{
  std::vector <pose_t> slam_pose, loc_pose;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
	    "Usage: %s logfilename locfilename\n", argv[0]);
  
  /* the first argument is the reference logfile. */
  log_to_poselist(argv[1], slam_pose);

  /* if the second file ends in .txt, treat it as text output */
  if(strcmp(argv[2] + strlen(argv[2]) - 4, ".txt") == 0)
    locfile_to_poselist(argv[2], loc_pose);
  else
    localized_log_to_poselist(argv[2], loc_pose);

  fprintf(stderr, "Found %d poses in reference log.\n", (int)slam_pose.size());
  fprintf(stderr, "Found %d poses in test log.\n", (int)loc_pose.size());

  test_accuracy(slam_pose, loc_pose, "loctest.txt");
  
  return 0;
}
