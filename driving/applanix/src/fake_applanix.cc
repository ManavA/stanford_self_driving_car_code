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
#include <applanix/ApplanixPose.h>

ros::Publisher applanix_pub;
applanix::ApplanixPose pose;

void publishPoseMessage() {
  pose.header.seq += 1;
  pose.header.stamp = ros::Time::now(); //ros::Time(vlr::Time::current());
  applanix_pub.publish(pose);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "applanix");
  ros::NodeHandle nh("/driving");
  applanix_pub = nh.advertise<applanix::ApplanixPose>("ApplanixPose", 1000);

  switch(argc) {
    case 1:
      pose.latitude = 37.430198390;
      pose.longitude = -122.182457738;
      pose.altitude = 3.201104;
      pose.yaw = -1.846357;
      break;

    case 3:
      pose.latitude = atof(argv[1]);
      pose.longitude = atof(argv[2]);
      pose.altitude = 0;
      pose.yaw = 0;
      break;
    case 5:
      pose.latitude = atof(argv[1]);
      pose.latitude = atof(argv[1]);
      pose.longitude = atof(argv[2]);
      pose.altitude = atof(argv[3]);
      pose.yaw = atof(argv[4]);
      break;

    default:
      printf("Usage:\nfake_applanix [lat, lon [, alt, heading]]\n");
      exit(0);
  }
 
  ros::Rate r(200); // 200 Hz

  while (ros::ok()) {
    publishPoseMessage();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
