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


#include <time.h>
#include <stdio.h>

#include <stdint.h>
#include <iostream>
#include <vector>

#include <lltransform.h>

#include <circle_demo.h>

using namespace std;

namespace vlr {

CircleDemo::CircleDemo(const std::string& rndf_name, const std::string& mdf_name,
                       const double start_lat, const double start_lon, const double base_r) : BaseDemo(rndf_name, mdf_name, start_lat, start_lon, -M_PI / 2.),
                       fake_tracker_(NULL), base_r_(base_r),
                       num_fast_cars_(4), num_slow_cars_(3), num_trucks_(5) {
   lane_width_ = 4;
   num_lanes_ = 3;


   c_x_ = start_x_ + base_r;
   c_y_ = start_y_;

   if(!createMultiCircleRNDF()) {
     throw(VLRException("Could not create RNDF file " + rndf_name_ + " for circle demo"));
   }

   std::vector<uint32_t> checkpoints;
   checkpoints.push_back(2);
   checkpoints.push_back(2);
   checkpoints.push_back(2);

   if(!createMultiCircleMDF(checkpoints)) {
     throw(VLRException("Could not create MDF file " + mdf_name_ + " for circle demo"));
   }

  generateTraffic(num_fast_cars_, num_slow_cars_, num_trucks_);

  fake_tracker_ = new FakeObstacleTracker(car_states_);
}


CircleDemo::~CircleDemo() {

}

std::string currentDateString() {
  time_t t = time(NULL);
  struct tm* tmp = localtime(&t);

  if (!tmp) {
      std::cout << "Conversion to local time failed.\n";
      return std::string("");
  }

  char buf[200];
  if (strftime(buf, sizeof(buf), "%F", tmp) == 0) {
    std::cout << "Conversion of local time to string failed.\n";
    return std::string("");
  }

  return std::string(buf);
}

bool CircleDemo::createMultiCircleRNDF() {

  FILE* rndf = NULL;

  printf("Output file is %s\n", rndf_name_.c_str());

  rndf = fopen(rndf_name_.c_str(), "w");
  if (!rndf) {
    printf("Could not open output file.\n");
    return false;
  }

  printf("Center coordinates: %.16lf, %.16lf; (base) radius: %f\n", c_x_, c_y_, base_r_);

  fprintf(rndf, "RNDF_name %s\n", rndf_name_.c_str());
  fprintf(rndf, "num_segments 1\n");
  fprintf(rndf, "num_zones 0\n");
  fprintf(rndf, "num_intersections 0\n");
  fprintf(rndf, "format_version  1.1\n");
  fprintf(rndf, "creation_date %s\n", currentDateString().c_str());
  fprintf(rndf, "segment 1\n");
  fprintf(rndf, "num_lanes %u\n", num_lanes_);
  fprintf(rndf, "num_crosswalks  0\n");

  double r = base_r_ - lane_width_; // Assuming 3 lanes with base_r_ for middle lane
  uint32_t num_checkpoints = 0;
  for (uint32_t l = 0; l < num_lanes_; l++) {
    uint32_t num_points = uint32_t(2 * M_PI * r)/10;
    double phi_step = 2 * M_PI / (num_points);
    fprintf(rndf, "lane  1.%u\n", l + 1);
    fprintf(rndf, "num_waypoints %u\n", num_points);
    fprintf(rndf, "lane_width  %.16lf\n", lane_width_ * meters2feet_);
    fprintf(rndf, "exit  1.%u.%u 1.%u.1\n", l + 1, num_points, l + 1);
    if (l == 1) {
//      fprintf(rndf, "stop  1.%u.%u\n", l + 1, num_points);
    }
    num_checkpoints++;
    fprintf(rndf, "checkpoint 1.%u.%u %u\n", l + 1, num_points / 2, num_checkpoints);

    double phi = 0;
    for (uint32_t i = 0; i < num_points; i++, phi += phi_step) {
      double x = c_x_ + r * cos(phi);
      double y = c_y_ + r * sin(phi);
      double lat, lon;
      utmToLatLong(x, y, utm_zone_, &lat, &lon);
      fprintf(rndf, "1.%u.%u %.16lf %.16lf\n", l + 1, i + 1, lat, lon);
    }

    fprintf(rndf, "end_lane\n");
    r += lane_width_;
  }

  fprintf(rndf, "end_segment\n");
  fprintf(rndf, "end_file\n");
  fclose(rndf);

  return true;
}

bool CircleDemo::createMultiCircleMDF(std::vector<uint32_t>& checkpoints) {

  FILE* mdf = NULL;

  printf("Output file is %s\n", mdf_name_.c_str());

  mdf = fopen(mdf_name_.c_str(), "w");
  if (!mdf) {
    printf("Could not open output file.\n");
    return false;
  }

  printf("Center coordinates: %.16lf, %.16lf; (base) radius: %f\n", c_x_, c_y_, base_r_);

  fprintf(mdf, "MDF_name %s\n", mdf_name_.c_str());
  fprintf(mdf, "RNDF %s\n", rndf_name_.c_str());
  fprintf(mdf, "format_version  1.0\n");
  fprintf(mdf, "creation_date %s\n", currentDateString().c_str());
  fprintf(mdf, "checkpoints\n");
  fprintf(mdf, "num_checkpoints %u\n", (uint32_t)checkpoints.size());
  for (uint32_t i = 0; i < checkpoints.size(); i++) {
    fprintf(mdf, "%u\n", checkpoints[i]);
  }
  fprintf(mdf, "end_checkpoints\n");
  fprintf(mdf, "speed_limits\n");
  fprintf(mdf, "num_speed_limits %u\n", 1);
  fprintf(mdf, "1\t0\t100\n");
  fprintf(mdf, "end_speed_limits\n");
  fprintf(mdf, "end_file\n");
  fclose(mdf);

  return true;
}

// Generate fake obstacles for 3 lane scenario
void CircleDemo::generateTraffic(uint32_t num_fast_cars_, uint32_t num_slow_cars_, uint32_t num_trucks_) {

  CircleDemoCarState car;

    // Fast cars
	for (uint32_t i=0; i <num_fast_cars_; i++) {
		car.phi = i*2*M_PI / num_fast_cars_ - M_PI*0.0;
		car.r   = base_r_ + lane_width_;
		car.v   = 5.0;
		car.x   = c_x_;
		car.y   = c_y_;
    car.width = 3;
    car.length = 5.5;
    car.ref_offset = car.length/2; //1.0;
		car_states_.push_back(car);
	}

    // Slow cars
	for (uint32_t i=0; i < num_slow_cars_; i++) {
		car.phi = i*2*M_PI / num_slow_cars_;
		car.r   = base_r_;
		car.v   = -1.5;
		car.x   = c_x_;
		car.y   = c_y_;
    car.width = 3;
    car.length = 5.1;
    car.ref_offset =  car.length/2;//1.0;
		car_states_.push_back(car);
	}

    // Trucks
	for (uint32_t i=0; i < num_trucks_; i++) {
		car.phi = i*2*M_PI / num_trucks_;
		car.r   = base_r_ - lane_width_;
		car.v   = 1.0;
		car.x   = c_x_;
		car.y   = c_y_;
    car.width = 3.5;
    car.length = 7.0;
    car.ref_offset =  car.length/2;// 1.0;
		car_states_.push_back(car);
	}
}

} // namespace vlr
