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
#include <ipc_std_interface.h>
#include <lltransform.h>
#include <perception_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define FAKE_CAR_IPC_TIMEOUT_MS  0.05

using namespace vlr;
using namespace dgc;
using namespace std;

IpcInterface* ipc = NULL;
bool received_applanix_pose_=false;
bool received_localize_pose_=false;
dgc::ApplanixPose applanix_pose_;
dgc::LocalizePose localize_pose_;
dgc_pose_t pose_;
pthread_mutex_t pose_mutex_;

double deg2rad (double angle_deg) {
	double angle_rad = angle_deg*M_PI/180;
	return angle_rad;
}

double rad2deg (double angle_rad) {
	double angle_deg = angle_rad/M_PI*180;
	return angle_deg;
}

void updatePose() {

  if (!received_applanix_pose_) {
    return;
  } // localize pose alone doesn't help..and anyway..shouldn't occur

  pthread_mutex_lock(&pose_mutex_);

  if (received_localize_pose_) {
    pose_.x = applanix_pose_.smooth_x + localize_pose_.x_offset;
    pose_.y = applanix_pose_.smooth_y + localize_pose_.y_offset;
  }

  pose_.z = applanix_pose_.altitude; // no offset_z defined in localize pose so far
  pose_.yaw = applanix_pose_.yaw;
  pose_.pitch = applanix_pose_.pitch;
  pose_.roll = applanix_pose_.roll;


  pthread_mutex_unlock(&pose_mutex_);
}

void applanixHandler() {
  received_applanix_pose_ = true;
  updatePose();
}

void localizePoseHandler() {
  received_localize_pose_ = true;
  updatePose();
}

int main(int argc, char *argv[]) {
	
	uint32_t num_cars = 1;
	uint32_t id_vehicle = 0;

    if (argc == 2) {
		id_vehicle = atoi(argv[1]);
    }
    
    // connect to the IPC server, register messages
    ipc = new IpcStandardInterface;
    if (ipc->Connect(argv[0]) < 0) dgc_fatal_error("Could not connect to IPC network.");
    
    // poses
    ipc->Subscribe(ApplanixPoseID, &applanix_pose_, applanixHandler, DGC_SUBSCRIBE_LATEST, &pose_mutex_);
    ipc->Subscribe(LocalizePoseID, &localize_pose_, localizePoseHandler, DGC_SUBSCRIBE_LATEST, &pose_mutex_);

    //register messages
    int err = ipc->DefineMessage(vlr::PerceptionObstaclesID);
    TestIpcExit(err, "Could not define", vlr::PerceptionObstaclesID);
    
    //send message
    vlr::PerceptionObstacles* cars = new vlr::PerceptionObstacles();
    memset(cars, 0, sizeof(cars));
    cars->dynamic_obstacle = new vlr::PerceptionDynamicObstacle[num_cars];
    memset(cars->dynamic_obstacle, 0, num_cars * sizeof(PerceptionDynamicObstacle));
    cars->num_dynamic_obstacles = num_cars;
    
    uint32_t nr_timespans = 0;
    double lat0 = 0;
    double lon0 = 0;
    double *velocities = NULL;
    double *directions = NULL;
    double *timespan = NULL;
    
    // VAIL intersection
    if (id_vehicle==1) {
		
	    nr_timespans = 12;
		
		lat0 = 37.431419;
		
		lon0 = -122.18396;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 3;
		velocities[1] = 2;
		velocities[2] = 3;
		velocities[3] = 3;
		velocities[4] = 2;
		velocities[5] = 0;
		velocities[6] = 2;
		velocities[7] = 2;
		velocities[8] = 2;
		velocities[9] = 2;
		velocities[10] = 2;
		velocities[11] = 3;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(240);
		directions[1] = deg2rad(235);
		directions[2] = deg2rad(265);
		directions[3] = deg2rad(255);
		directions[4] = deg2rad(265);
		directions[5] = deg2rad(260);
		directions[6] = deg2rad(260);
		directions[7] = deg2rad(285);
		directions[8] = deg2rad(310);
		directions[9] = deg2rad(330);
		directions[10] = deg2rad(350);
		directions[11] = deg2rad(355);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 6;
		timespan[1] = 10;
		timespan[2] = 5;
		timespan[3] = 3.5;
		timespan[4] = 6;
		timespan[5] = 3;
		timespan[6] = 8;
		timespan[7] = 2;
		timespan[8] = 2;
		timespan[9] = 1;
		timespan[10] = 3;
		timespan[11] = 8;
	    
	}
	
	// Other intersection 1
	else if (id_vehicle==2) {
		
	    nr_timespans = 8;
		
		lat0 = 37.4307620015140;
		
		lon0 = -122.1787469994489;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 3;
		velocities[1] = 1;
		velocities[2] = 0;
		velocities[3] = 2;
		velocities[4] = 2;
		velocities[5] = 2;
		velocities[6] = 2;
		velocities[7] = 3;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(10);
		directions[1] = deg2rad(10);
		directions[2] = deg2rad(10);
		directions[3] = deg2rad(8);
		directions[4] = deg2rad(30);
		directions[5] = deg2rad(50);
		directions[6] = deg2rad(70);
		directions[7] = deg2rad(90);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 7;
		timespan[1] = 2;
		timespan[2] = 3;
		timespan[3] = 7;
		timespan[4] = 0.5;
		timespan[5] = 0.5;
		timespan[6] = 1;
		timespan[7] = 8;
	}
	
	// Other intersection 1
	else if (id_vehicle==3) {
		
	    nr_timespans = 10;
		
		lat0 = 37.4300370020586;
		
		lon0 = -122.1783080000964;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 3;
		velocities[1] = 3;
		velocities[2] = 3;
		velocities[3] = 2;
		velocities[4] = 3;
		velocities[5] = 0;
		velocities[6] = 3;
		velocities[7] = 2;
		velocities[8] = 2;
		velocities[9] = 3;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(90);
		directions[1] = deg2rad(60);
		directions[2] = deg2rad(50);
		directions[3] = deg2rad(80);
		directions[4] = deg2rad(90);
		directions[5] = deg2rad(90);
		directions[6] = deg2rad(90);
		directions[7] = deg2rad(50);
		directions[8] = deg2rad(30);
		directions[9] = deg2rad(0);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 10;
		timespan[1] = 1;
		timespan[2] = 1;
		timespan[3] = 1;
		timespan[4] = 12;
		timespan[5] = 3;
		timespan[6] = 4;
		timespan[7] = 1;
		timespan[8] = 1;
		timespan[9] = 8;
	}
	
	// Other intersection 1
	else if (id_vehicle==4) {
		
	    nr_timespans = 7;
		
		lat0 = 37.4310480009330;
		
		lon0 = -122.1776999994465;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 4;
		velocities[1] = 3;
		velocities[2] = 2;
		velocities[3] = 2;
		velocities[4] = 2;
		velocities[5] = 0;
		velocities[6] = 3;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(180);
		directions[1] = deg2rad(180);
		directions[2] = deg2rad(210);
		directions[3] = deg2rad(240);
		directions[4] = deg2rad(270);
		directions[5] = deg2rad(270);
		directions[6] = deg2rad(270);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 10;
		timespan[1] = 4;
		timespan[2] = 2;
		timespan[3] = 1;
		timespan[4] = 6;
		timespan[5] = 3;
		timespan[6] = 10;
	}
	
	// Other intersection 1, for debugging only
	else if (id_vehicle==5) {
		
	    nr_timespans = 1;
		
		lat0 = 37.43075;
		
		lon0 = -122.1783;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 0;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(180);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 10000000;
	}
	
	// VAIL intersection
	else if (id_vehicle==6) {
		
	    nr_timespans = 12;
		
		lat0 = 37.431419;
		
		lon0 = -122.18396;
		
		velocities = (double*)malloc(nr_timespans * sizeof(double));
		velocities[0] = 3;
		velocities[1] = 2;
		velocities[2] = 3;
		velocities[3] = 3;
		velocities[4] = 2;
		velocities[5] = 0;
		velocities[6] = 2;
		velocities[7] = 2;
		velocities[8] = 2;
		velocities[9] = 2;
		velocities[10] = 2;
		velocities[11] = 3;
		
		directions = (double*)malloc(nr_timespans * sizeof(double));
		directions[0] = deg2rad(240);
		directions[1] = deg2rad(240);
		directions[2] = deg2rad(250);
		directions[3] = deg2rad(255);
		directions[4] = deg2rad(265);
		directions[5] = deg2rad(270);
		directions[6] = deg2rad(270);
		directions[7] = deg2rad(285);
		directions[8] = deg2rad(310);
		directions[9] = deg2rad(330);
		directions[10] = deg2rad(350);
		directions[11] = deg2rad(355);
		
		timespan = (double*)malloc(nr_timespans * sizeof(double));
		timespan[0] = 6;
		timespan[1] = 10;
		timespan[2] = 5;
		timespan[3] = 3.5;
		timespan[4] = 6;
		timespan[5] = 3;
		timespan[6] = 8;
		timespan[7] = 2;
		timespan[8] = 2;
		timespan[9] = 1;
		timespan[10] = 3;
		timespan[11] = 8;
	    
	}
	
	else {
		printf("Error: No trajectories defined for this vehicle id"); fflush(stdout);
	}
	
	double x0, y0;
	char dummy_zone[4];
	double x_old, y_old;
	double t0 = Time::current();
	double t_old = t0;
	uint32_t current_timespan = 0;
	double t_last_timespan_change = t0;
	srand (time(NULL));
  
    pthread_mutex_init(&pose_mutex_, NULL);
	latLongToUtm(lat0, lon0, &x0, &y0, dummy_zone);
	x_old = x0;
	y_old = y0;
  
	double x_step, y_step;
	double x_cur, y_cur;
	double t;
	double delta_t;
	
    double rand_velocity, rand_direction;
    double orig_velocity, orig_direction;
	
	printf("%d, current velocity=%f, current direction=%f\n", current_timespan, velocities[current_timespan], rad2deg(directions[current_timespan])); fflush(stdout);
    while (ipc->Sleep(FAKE_CAR_IPC_TIMEOUT_MS) >=0) {
 		if(!received_applanix_pose_ || !received_localize_pose_) {continue;}
 		for (uint32_t it_car = 0; it_car < num_cars; it_car++) {
			
			orig_velocity = velocities[current_timespan];
			orig_direction = directions[current_timespan];
	
			rand_velocity = (rand() % 601)/1000.0 - 0.3;
			rand_direction = (rand() % 201)/1000.0 - 0.1;
						
			PerceptionDynamicObstacle& car_ipc = cars->dynamic_obstacle[it_car];
			car_ipc.id = it_car;
			car_ipc.obstacleType = OBSTACLE_CAR;
			car_ipc.obstacleTypeThisFrame = OBSTACLE_CAR;
			car_ipc.confidence = 1;
			car_ipc.direction = orig_direction + rand_direction;
			car_ipc.width = 2;
			car_ipc.length = 5;
			car_ipc.velocity = orig_velocity + rand_velocity;
			
			x_step = car_ipc.velocity * cos(car_ipc.direction);
			y_step = car_ipc.velocity * sin(car_ipc.direction);
			t = Time::current();
			delta_t = t - t_old;
			x_cur = x_old + delta_t * x_step;
			y_cur = y_old + delta_t * y_step;
			car_ipc.x = x_old + delta_t * x_step - pose_.x + applanix_pose_.smooth_x;
			car_ipc.y = y_old + delta_t * y_step - pose_.y + applanix_pose_.smooth_y;
			car_ipc.turn_signal = 0;
			
			//printf("Vehicle %d, car_ipc.x=%f, car_ipc.y=%f \n", it_car_ipc, car_ipc.x, car_ipc.y);
		  
			//err = ipc->Publish(vlr::PerceptionObstaclesID, cars);
			//TestIpcExit(err, "Could not publish", vlr::PerceptionObstaclesID);
			
			t_old = t;
			x_old = x_cur;
			y_old = y_cur;
			
			if (t-t_last_timespan_change>timespan[current_timespan]) {
				if (current_timespan<nr_timespans-1) {
					current_timespan++;
				}
				else {
					current_timespan = 0;
					x_old = x0;
					y_old = y0;
				}
				t_last_timespan_change = t;
				printf("%d, current velocity=%f, current direction=%f\n", current_timespan, velocities[current_timespan], rad2deg(directions[current_timespan])); fflush(stdout);
			}
		}
		
		cars->timestamp = Time::current();
		err = ipc->Publish(vlr::PerceptionObstaclesID, cars);
		TestIpcExit(err, "Could not publish", vlr::PerceptionObstaclesID);
			
	}

	IPC_disconnect();
	delete ipc;
	return 0;
	
}

