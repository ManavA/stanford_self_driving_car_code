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
#include <ibeo_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_interface.h>
#include "lasersim.h"

using namespace vlr;

int received_applanix_pose = 0;
ApplanixPose applanix_pose;

int received_localize_pose = 0;
LocalizePose localize_pose;

double laser_max_range, laser_x_offset, laser_y_offset, laser_yaw_offset;
char *obstacle_map_filename;

IpcInterface *ipc = NULL;

void publish_ibeo(void)
{
  static int first = 1;
  static IbeoLaser ibeo;
  double ibeo_x, ibeo_y;

  if(!received_applanix_pose || !received_localize_pose)
    return;

  if(first) {
    ibeo.point = 
      (IbeoLaserPoint *)calloc(10000, sizeof(IbeoLaserPoint));
    dgc_test_alloc(ibeo.point);
    strcpy(ibeo.host, dgc_hostname());
    first = 0;
  }

  ibeo.start_angle = dgc_d2r(-120.0);
  ibeo.end_angle = dgc_d2r(120.0);

  ibeo_x = applanix_pose.smooth_x + localize_pose.x_offset +
    laser_x_offset * cos(applanix_pose.yaw) +
    laser_y_offset * cos(applanix_pose.yaw + M_PI / 2.0);
  ibeo_y = applanix_pose.smooth_y + localize_pose.y_offset +
    laser_x_offset * sin(applanix_pose.yaw) +
    laser_y_offset * sin(applanix_pose.yaw + M_PI / 2.0);

  generate_ibeo_laser_scan(&ibeo, ibeo_x, ibeo_y, applanix_pose.yaw + 
			   dgc_d2r(laser_yaw_offset), laser_max_range);
  ibeo.timestamp = dgc_get_time();
  int err = ipc->Publish(IbeoLaser2ID, &ibeo);
  TestIpcExit(err, "Could not publish", IbeoLaser2ID);
}

void applanix_pose_handler(void)
{
  received_applanix_pose = 1;
}

void localize_pose_handler(void)
{
  received_localize_pose = 1;
}

void laser_timer(void)
{
  publish_ibeo();
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
  Param params[] = {
    {"sim", "obstacle_map", DGC_PARAM_FILENAME, &obstacle_map_filename, 0, NULL},
    {"sim", "laser_max_range", DGC_PARAM_DOUBLE, &laser_max_range, 0, NULL},
    {"sim", "laser_x_offset", DGC_PARAM_DOUBLE, &laser_x_offset, 0, NULL},
    {"sim", "laser_y_offset", DGC_PARAM_DOUBLE, &laser_y_offset, 0, NULL},
    {"sim", "laser_yaw_offset", DGC_PARAM_DOUBLE, &laser_yaw_offset, 0, NULL},
  };
  pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));
}

int main(int argc, char **argv)
{
  ParamInterface *pint;

  ipc = new IpcStandardInterface;
  pint = new ParamInterface(ipc);
  if (ipc->Connect(argv[0]) < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  read_parameters(pint, argc, argv);

  load_obstacle_map(obstacle_map_filename);

  int err = ipc->DefineMessage(IbeoLaser2ID);
  TestIpcExit(err, "Could not define", IbeoLaser2ID);

  ipc->Subscribe(ApplanixPoseID, &applanix_pose, &applanix_pose_handler,
		 DGC_SUBSCRIBE_ALL);
  ipc->Subscribe(LocalizePoseID, &localize_pose, &localize_pose_handler);

  ipc->AddTimer(1.0 / 12.5, laser_timer);
  ipc->Dispatch();
  return 0;
}
