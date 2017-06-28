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
#include <grid.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <velo_support.h>

#define MODE_RANGE 1
#define MODE_ANGLE 2

using namespace dgc;

IpcInterface *ipc = NULL;


int mode = MODE_RANGE;

/* parameters */
char *cal_filename = NULL;
dgc_transform_t velodyne_offset;

FILE *FCAL = NULL;

int num_spins = 0;
double first_timestamp = 0;
int ignore_seconds = 20;
int num_seconds = 0;
int done = 0;
int polepoints = 0;

int count[64];
double range_sum[64];
double dist_sum[64];
double height_sum[64];
double angle_sum[64];
double min_pole_dist[64];
double sum_pole_count[64];
double sum_pole_theta[64];
double sum_pole_avgtheta[64];
double sum_pole_offset[64];
double count_pole_offset[64];
double avg_pole_dist = 0.0;
dgc_velodyne_config_p config_copy;

int timecheck(double t) {
	if(t > 6 && t < 16)
		return 1;
	if(t > 50)
		return 1;
	return 0;
}

void init() {
	int i;
	for(i = 0; i < 64; i++) {
		count[i] = 0;
		range_sum[i] = 0;
		dist_sum[i] = 0;
		angle_sum[i] = 0;
		sum_pole_avgtheta[i] = 0;
		sum_pole_offset[i] = 0;
		count_pole_offset[i] = 0;
	}
	config_copy = (dgc_velodyne_config_p) malloc(sizeof(dgc_velodyne_config_t));
}

void calibrate() {
	//return;
	printf("Calibrating laser...\n");
	printf("%d spins were seen\n", num_spins);
	fprintf(FCAL, "RANGE_MULTIPLIER %.3f\n", config_copy->range_multiplier);
	int i;
	double drs[64];
	double das[64];
	double dhs[64];
	if(mode == MODE_RANGE) {
		for(i = 0; i < 64; i++) {
			int beam = config_copy->beam_order[i];
			double v_ang = config_copy->vert_angle[beam];
			if(count_pole_offset[i] > 0) {
				double avg_delta = sum_pole_offset[i] / count_pole_offset[i];
				double range_delta = -avg_delta / cos(v_ang);
				double avg_theta = sum_pole_avgtheta[i] / count_pole_offset[i];
				printf("Ring %d: average pole offset: %.2f cm --> dr = %.2f cm    dha = %.2f deg\n", i, 100*avg_delta, 100*range_delta, avg_theta*180/3.14159);
				drs[i] = 1*range_delta;
				dhs[i] = avg_theta;
			} else {
				drs[i] = 0;
				dhs[i] = 0;
			}
			das[i] = 0;
		}
	} else if(mode == MODE_ANGLE) {
		double total_h = 0.0;
		for(i = 0; i < 32; i++) {
			//if(i == 5) continue;
			double r = range_sum[i] / count[i];
			double h = height_sum[i] / count[i];
			total_h += h;
			int beam = config_copy->beam_order[i];
			int ring = config_copy->inv_beam_order[beam];	
			double v_ang = config_copy->vert_angle[beam];
			printf("Ring %d = %d, VA: %.3f  Beam %d: avg range: %.2f    avg height: %.2f\n", i, ring, v_ang, beam, r, h);
		}
		double avg_h = total_h / 32;
		printf("Average height: %.3f\n", avg_h);
		for(i = 0; i < 64; i++) {
			double h = height_sum[i] / count[i];
			double dh = h - avg_h;
			int beam = config_copy->beam_order[i];
			double v_ang = config_copy->vert_angle[beam];
			//double a_avg = angle_sum[i] / count[i];
			double r = range_sum[i] / count[i];
			double dr = 0, da = 0;
			if(i < -1) { // don't do this
				dr = -dh / sin(v_ang);
				printf("Ring %d = Beam %d--> dh: %.3f  dr: %.3f\n", i, beam, dh, dr);
				da = 0;		
			} else {
				if(i < 48) {
					da = -atan(dh / (.01 * r));
					//da = a_avg - v_ang;	
					printf("Ring %d: old angle: %f    new angle: %f    delta angle: %.4f\n", i, v_ang, v_ang + da, da);
					//printf("   check difference: %f\n", check - v_ang);
				} else {
					da = 0;
				}
				dr = 0;
			}
			drs[i] = dr;
			das[i] = da;
			dhs[i] = 0;
		}
	}
	for(i = 0; i < 64; i++) {
		int beam = i;
		int ring = config_copy->inv_beam_order[beam];
		double dr = drs[ring];
		double da = das[ring];
		double dh = dhs[ring];
		if(ring == 5 || ring == 36) {
			//dr = da = 0;
		}
		double v_angle = dgc_r2d(config_copy->vert_angle[beam] + 1*da);
		double h_angle = dgc_r2d(config_copy->rot_angle[beam] + 1*dh);
		double range = config_copy->range_offset[beam] + dr;
		printf("Beam %d: offset %.2f --> %.2f\n", i, config_copy->range_offset[beam], range);
		int enabled = config_copy->laser_enabled[beam];
		double v_off = config_copy->v_offset[beam];
		double h_off = config_copy->h_offset[beam];
		/* if(ring < 32)
			v_off = .15;
		double h_off = -.04 + .08 * (i % 2); */
		fprintf(FCAL, "%d\t%2.8f\t%2.8f\t%1.2f\t%1.2f\t%1.2f\t%d\n", beam, h_angle, v_angle, 1*range, v_off, h_off, enabled);
	}	
	printf("polepoint: %d\n", polepoints);
}

void my_spin_func(dgc_velodyne_spin *spin, 
			 dgc_velodyne_config_p config,
			 ApplanixPose *applanix_pose) {
  if(done) return;
  static int first = 1; 
  static int spin_polepoints = 0;
  int i, j, beam_num, ring_num;
  double p_x, p_y, p_z, utm_x, utm_y;
  char utmzone[10];
  static int found_offset = 0;
  static double utm_offset_x = 0, utm_offset_y = 0, utm_offset_z = 0;

  if(first) {
	memcpy(config_copy, config, sizeof(dgc_velodyne_config_t));
	first = 0;
  }
  if(spin->num_scans <= 0) {
    fprintf(stderr, "Warning: spin has zero scans.  Shouldn't happen\n");
	done = 1;
    return;
  }

  if(spin->num_scans < 20) {
     printf("STRANGE: not enough scans: %d. Exiting.\n", spin->num_scans);
     done = 1;
	return;
  }

  if(num_spins % 100 == 0)
  	printf("Spin! %d   Scans: %d  Time: %fs\n", num_spins, spin->num_scans, applanix_pose->timestamp - first_timestamp);

  num_spins++;

  vlr::latLongToUtm(applanix_pose->latitude, applanix_pose->longitude, 
	      &utm_x, &utm_y, utmzone);

  static double next_t = 0.0;
  if(!first_timestamp) {
	first_timestamp = applanix_pose->timestamp; 
  } else {
	if(!timecheck(applanix_pose->timestamp - first_timestamp)) {
		//return;
	}
	double elapsed = applanix_pose->timestamp - first_timestamp;
	if(elapsed < ignore_seconds) {
		printf("ignoring t = %f\n", elapsed);
		return;
	} else if(elapsed > num_seconds) {
		printf("Done with %d seconds! Gone from %f to %f.\n", num_seconds, first_timestamp, applanix_pose->timestamp);
		done = 1;
		return;
	}
	if(elapsed > next_t) {
		printf("elapsed: %.3f seconds\n", elapsed);
		next_t += 1;
	}
  }

  if(!found_offset) {
    utm_offset_x = utm_x - spin->scans[0].robot.x;
    utm_offset_y = utm_y - spin->scans[0].robot.y;
    utm_offset_z = applanix_pose->altitude - spin->scans[0].robot.z;
    found_offset = 1;
  }

  if(fabs(applanix_pose->speed) < dgc_mph2ms(.5))
    return;

  spin_polepoints = 0;
  avg_pole_dist = 0;
  float pole_theta_sum = 0.0;
  for(i = 0; i < 64; i++) {
	sum_pole_count[i] = 0;
	sum_pole_theta[i] = 0.0;
	min_pole_dist[i] = 1000000.0;
  }

  for(i = 0; i < spin->num_scans; i++) {
    for(j = 0; j < VELO_BEAMS_IN_SCAN; j++) {
      /* skip bad readings */
      if(spin->scans[i].p[j].range < 0.01)
	continue;
      
      beam_num = j + spin->scans[i].block * 32;
      ring_num = config->inv_beam_order[beam_num];

      /* project beam */
      p_x = spin->scans[i].p[j].x * 0.01 + spin->scans[i].robot.x + 
	utm_offset_x;
      p_y = spin->scans[i].p[j].y * 0.01 + spin->scans[i].robot.y + 
	utm_offset_y;
      p_z = spin->scans[i].p[j].z * 0.01 + spin->scans[i].robot.z + 
	utm_offset_z;

      double z = spin->scans[i].p[j].z * .01;
      double x = spin->scans[i].p[j].x * .01;
      double y = spin->scans[i].p[j].y * .01;
      double r = spin->scans[i].p[j].range * .01;
      double dist = pow(x * x + y * y, .5);

      /* if(p_z < applanix_pose->altitude - 3)
	continue; */

      float pole_heading = (atan2(y, x) - applanix_pose->yaw) * 180 / 3.14159;
      if(spin->scans[i].p[j].z * .01 > -.8 && fabs(pole_heading) < 6 && r < 40) { // pole points
	polepoints++;
	if(polepoints % 1000 == 0) {
		double dt = applanix_pose->timestamp - first_timestamp;
		printf("polepoints: %d at t=%.3f\n", polepoints, dt);
	}
	double dist = pow(x*x + y*y, .5);
	if(dist < min_pole_dist[ring_num]) {
		min_pole_dist[ring_num] = dist;
	}
	avg_pole_dist += dist;
	pole_theta_sum += atan2(y, x) - applanix_pose->yaw;
	sum_pole_theta[ring_num] += atan2(y, x) - applanix_pose->yaw;
	sum_pole_count[ring_num]++;
	spin_polepoints++;
      }
      if(z < -1.2 && z > -2.5 && fabs(pole_heading) > 8 && r < 50) {
	count[ring_num]++;
	range_sum[ring_num] += spin->scans[i].p[j].range;
	height_sum[ring_num] += z;
	dist_sum[ring_num] += dist;
	//double range = spin->scans[i].p[j].range * .01;
	//double s = sin(z/range);
	//double angle = asin(z / range);
	//printf("vert_angle: %f   real_angle: %f   da:   %f\n", config->vert_angle[beam_num], angle, config->vert_angle[beam_num]-angle);
	//printf("   v_sin: %f     real_sin: %f\n", config->sin_vert_angle[beam_num], s);
	angle_sum[ring_num] += asin(100 * z / spin->scans[i].p[j].range);
	//height_sum[ring_num] += p_z - applanix_pose->altitude;
      }
    }
  }
	// pole point stuff
  if(spin_polepoints > 0) {
	double avg_dist = avg_pole_dist / spin_polepoints;
	double avg_theta = pole_theta_sum / spin_polepoints;
	printf("%d polepoints\n", spin_polepoints);
	printf("   Average pole distance for scan %d: %.2f meters\n", num_spins, avg_dist);
	printf("   Average pole bearing for scan %d: %.2f degrees\n", num_spins, avg_theta * 180 / 3.14159);
	double sum_dist = 0.0;
	int count_dist = 0;
	for(i = 0; i < 64; i++) {
		if(min_pole_dist[i] > 0 && min_pole_dist[i] < 100) {
			count_dist++;
			sum_dist += min_pole_dist[i];
		}
	}
	avg_dist = sum_dist / count_dist;
	printf("  New average pole distance: %.2f meters with %d beams\n", avg_dist, count_dist);
  	for(i = 0; i < 64; i++) {
		// :)
		if(min_pole_dist[i] > 0 && min_pole_dist[i] < 100) { // means this beam saw the pole this scan
			double delta_dist = min_pole_dist[i] - avg_dist;
			sum_pole_offset[i] += delta_dist;
			sum_pole_avgtheta[i] += sum_pole_theta[i] / sum_pole_count[i] - avg_theta;
			count_pole_offset[i]++;
		}
	}
  }
}

void 
read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = { 
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset, 0, NULL}
 };
  pint->InstallParams(argc, argv, params, sizeof(params) / 
                           sizeof(params[0]));
}

int main(int argc, char **argv) {
  if(argc < 5)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s log-file.log.gz velodyne-file.vlf num-seconds cal-output-file.cal\n", argv[0]);

  /* connect to IPC server, get parameters, and disconnect */
  ipc = new IpcStandardInterface;
  if (ipc->Connect("map_velodyne") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  dgc::ParamInterface  *pint;
  pint = new dgc::ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  ipc->Disconnect();


  num_seconds = atoi(argv[3]);
  char *cal_file = argv[4];
  FCAL = fopen(cal_file, "w");
  if(!FCAL) {
	fprintf(stderr, "Error opening file %s for writing\n", cal_file);
  }
  init();
  vlf_projector(argv[2], argv[1], cal_filename, NULL, velodyne_offset, my_spin_func);
  calibrate();
  fclose(FCAL);
  return 0;
}
