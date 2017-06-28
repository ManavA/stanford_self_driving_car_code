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


#include <ros/ros.h>
#include <driving_common/CanStatus.h>
#include <usbfind.h>
#include <passatcore.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "PassatTest");

  vlr::PassatCore* passat_core = new vlr::PassatCore;
  ros::Rate r(20);

  uint32_t num_tests = 20;

  double throttle_fraction = 0.0;
  double brake_pressure = 50.0;

  std::cout << "Test 1/2: " << num_tests << " iterations.\n";

  for(uint32_t i = 0; i < num_tests; i++) {
    std::cout << i << ". Sending engine command: throttle fraction = " << throttle_fraction << " brake pressure = " << brake_pressure << ".\n";
    if(!ros::ok()) {break;}
    passat_core->readStatus();
    passat_core->sendEngineCommand(throttle_fraction, brake_pressure);

    ros::spinOnce();
    r.sleep();
  }

  std::cout << "Test 2/2: " << num_tests << " iterations.\n";

  throttle_fraction = 0.1;
  brake_pressure = 0.0;

  for(uint32_t i = 0; i < num_tests; i++) {
    std::cout << i << ". Sending engine command: throttle fraction = " << throttle_fraction << " brake pressure = " << brake_pressure << ".\n";
    if(!ros::ok()) {break;}
    passat_core->readStatus();
    passat_core->sendEngineCommand(throttle_fraction, brake_pressure);

    ros::spinOnce();
    r.sleep();
  }

    // resetting throttle and brake
  throttle_fraction = 0.0;
  brake_pressure = 0.0;
  passat_core->sendEngineCommand(throttle_fraction, brake_pressure);

  delete passat_core;
  return 0;
}
