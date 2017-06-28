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
#include <carlist.h>

#define LABELED_CAR_IPC_TIMEOUT_MS  0.05

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

//double deg2rad (double angle_deg) {
	//double angle_rad = angle_deg*M_PI/180;
	//return angle_rad;
//}

//double rad2deg (double angle_rad) {
	//double angle_deg = angle_rad/M_PI*180;
	//return angle_deg;
//}

void updatePose() {

  if (!received_applanix_pose_) {
    return;
  } // localize pose alone doesn't help..and anyway..shouldn't occur

  pthread_mutex_lock(&pose_mutex_);

  //if (received_localize_pose_) {
    //pose_.x = applanix_pose_.smooth_x + localize_pose_.x_offset;
    //pose_.y = applanix_pose_.smooth_y + localize_pose_.y_offset;
  //}

  //pose_.z = applanix_pose_.altitude; // no offset_z defined in localize pose so far
  //pose_.yaw = applanix_pose_.yaw;
  //pose_.pitch = applanix_pose_.pitch;
  //pose_.roll = applanix_pose_.roll;


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
    
    // connect to the IPC server, register messages
    ipc = new IpcStandardInterface;
    if (ipc->Connect(argv[0]) < 0) dgc_fatal_error("Could not connect to IPC network.");
    
    // poses
    ipc->Subscribe(ApplanixPoseID, &applanix_pose_, applanixHandler, DGC_SUBSCRIBE_LATEST, &pose_mutex_);
    ipc->Subscribe(LocalizePoseID, &localize_pose_, localizePoseHandler, DGC_SUBSCRIBE_LATEST, &pose_mutex_);

    //register messages
    int err = ipc->DefineMessage(vlr::PerceptionObstaclesID);
    TestIpcExit(err, "Could not define", vlr::PerceptionObstaclesID);
    
    
    
    
    
    // Get list of labeled cars
    
    char* cars_file = NULL;
	//if (argc == 2) {
		cars_file = argv[1];
	//}
	//else {
		//printf("Error, you need to give label file as argument");
	//}
	
	
    char* fn = dgc_expand_filename(cars_file);
    if (fn) {
		cars_file = fn;
	}
      
    car_list_t carlist;
    carlist.load_labels(cars_file);
    int num_cars = 0;
    int first_car;
    
    // If vehicle 0 is bad, don't consider it
    car_t* car = &carlist.car[0];
    double x, y, th, vel;
    bool extrapolated;
    //applanix_pose_.timestamp replaced by Time::current()
    if (car->estimate_pose(applanix_pose_.timestamp, &x, &y, &th, &extrapolated) && car->estimate_velocity(applanix_pose_.timestamp, &vel)) {
		num_cars = carlist.num_cars();
		first_car = 0;
	}
	else {
		num_cars = carlist.num_cars() - 1;
		first_car = 1;
	}
    
    printf("Number of labeled cars=%d \n", num_cars);
    
    //send message
    vlr::PerceptionObstacles* cars = new vlr::PerceptionObstacles();
    memset(cars, 0, sizeof(cars));
    cars->dynamic_obstacle = new vlr::PerceptionDynamicObstacle[num_cars];
    memset(cars->dynamic_obstacle, 0, num_cars * sizeof(PerceptionDynamicObstacle));
    cars->num_dynamic_obstacles = num_cars;
    
    while (ipc->Sleep(LABELED_CAR_IPC_TIMEOUT_MS) >=0) {
		
 		if(!received_applanix_pose_ || !received_localize_pose_) {continue;}
      
		for (int i = first_car; i < num_cars+first_car; i++) {
			
	      car_t* car = &carlist.car[i];
	      
	      //printf("i=%d\n", i); fflush(stdout);
	
	      double x, y, th, vel;
	      TurnSignalState signal;
	      bool extrapolated;
	      //bool result = car->estimate_pose(applanix_pose_.timestamp, &x, &y, &th, &extrapolated);
	      //printf("result=%d\n", result); fflush(stdout);
	      //printf("applanix timestamp=%d\n", applanix_pose_.timestamp); fflush(stdout);
	      if (car->estimate_signal(applanix_pose_.timestamp, &signal) && car->estimate_pose(applanix_pose_.timestamp, &x, &y, &th, &extrapolated) && car->estimate_velocity(applanix_pose_.timestamp, &vel)) {
			  
			  PerceptionDynamicObstacle& car_ipc = cars->dynamic_obstacle[i-first_car];
			  car_ipc.id = i-first_car;
			  car_ipc.obstacleType = OBSTACLE_CAR;
			  car_ipc.obstacleTypeThisFrame = OBSTACLE_CAR;
			  car_ipc.confidence = 1;
			  car_ipc.direction = th;
			  car_ipc.width = 2;
			  car_ipc.length = 5;
			  car_ipc.velocity = vel;
			  car_ipc.turn_signal = signal;
			  car_ipc.x = x;// - pose_.x + applanix_pose_.smooth_x;
			  car_ipc.y = y;// - pose_.y + applanix_pose_.smooth_y;
			  
			  printf("Vehicle %d, x=%f, y=%f, vel=%f signal=%d \n", car_ipc.id, car_ipc.x, car_ipc.y, car_ipc.velocity, car_ipc.turn_signal);
			  
			  //err = ipc->Publish(vlr::PerceptionObstaclesID, cars);
			  //TestIpcExit(err, "Could not publish", vlr::PerceptionObstaclesID);
			  
	      }
	      else
	      {
			  //if (!car->estimate_signal(applanix_pose_.timestamp, &signal)) {
				  //printf("id %d, pb signal\n", i-first_car);
			  //}
			  //if (!car->estimate_pose(applanix_pose_.timestamp, &x, &y, &th, &extrapolated)) {
				  //printf("id %d, pb pose\n", i-first_car);
			  //}
			  //if (!car->estimate_velocity(applanix_pose_.timestamp, &vel)) {
				  //printf("id %d, pb vel\n", i-first_car);
			  //}
			  PerceptionDynamicObstacle& car_ipc = cars->dynamic_obstacle[i-first_car];
			  car_ipc.velocity = -100.0;
		  }
	
	    }
	    cars->timestamp = applanix_pose_.timestamp;//Time::current();//
	    err = ipc->Publish(vlr::PerceptionObstaclesID, cars);
	    TestIpcExit(err, "Could not publish", vlr::PerceptionObstaclesID);
	    
	}
    
    
    

	IPC_disconnect();
	delete ipc;
	return 0;
	
}

