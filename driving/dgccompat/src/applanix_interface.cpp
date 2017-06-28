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


#include <applanix_interface.h>
#include <logio.h>

namespace dgc {

// Note: if you change the following lines
// make sure to change them in applanix_parse.c as well

void ApplanixCalculateSmoothedPose(ApplanixPose *pose) {
  static double old_hw_timestamp = 0;

  if (old_hw_timestamp != 0) {
    double dt = pose->hardware_timestamp - old_hw_timestamp;
    if (dt < -600000) {
      dt = 0.005;
      fprintf(stderr, "\nAPPLANIX: GPS week rollover handled \n");
    }
    if (dt <= 0) fprintf(stderr, "\nAPPLANIX: Hardware TS difference = %.6f (<=0) \n", dt);
    else {
      if (dt >= 0.1) fprintf(stderr, "\nAPPLANIX: WARNING: Large hardware TS jump = %.6f (>=0.1) \n", dt);
      pose->smooth_x += pose->v_east * dt;
      pose->smooth_y += pose->v_north * dt;
      pose->smooth_z += pose->v_up * dt;
    }
  }
  else {
    pose->smooth_x = 0;
    pose->smooth_y = 0;
    pose->smooth_z = 0;
  }
  old_hw_timestamp = pose->hardware_timestamp;
}

char *StringV1ToApplanixPose(char *string, ApplanixPose *pose) {
  char *pos = string;
  char *pos_tmp;
  double dbl_val, int_val;

  pose->latitude = READ_DOUBLE(&pos);
  pose->longitude = READ_DOUBLE(&pos);
  pose->altitude = READ_DOUBLE(&pos);

  pose->v_north = READ_FLOAT(&pos);
  pose->v_east = READ_FLOAT(&pos);
  pose->v_up = READ_FLOAT(&pos);

  pose->speed = READ_FLOAT(&pos);
  pose->track = READ_FLOAT(&pos);

  pose->roll = READ_DOUBLE(&pos);
  pose->pitch = READ_DOUBLE(&pos);
  pose->yaw = READ_DOUBLE(&pos);

  pose->ar_roll = READ_DOUBLE(&pos);
  pose->ar_pitch = READ_DOUBLE(&pos);
  pose->ar_yaw = READ_DOUBLE(&pos);

  pose->a_x = READ_DOUBLE(&pos);
  pose->a_y = READ_DOUBLE(&pos);
  pose->a_z = READ_DOUBLE(&pos);

  pose->wander = READ_DOUBLE(&pos);

  pose->ID = READ_UINT(&pos);
  pos_tmp = pos;
  dbl_val = READ_DOUBLE(&pos_tmp);
  pos_tmp = pos;
  int_val = READ_INT(&pos_tmp);

  pose->postprocess_code = READ_INT(&pos);
  pose->hardware_timestamp = READ_DOUBLE(&pos);
  pose->hardware_time_mode = READ_INT(&pos);
  pose->timestamp = READ_DOUBLE(&pos);
  READ_HOST(pose->host, &pos);

  ApplanixCalculateSmoothedPose(pose);
  return pos;
}

char *StringV2ToApplanixPose(char *string, ApplanixPose *pose) {
  char *pos = string;
  char *pos_tmp;
  double dbl_val, int_val;

  pose->smooth_x = READ_DOUBLE(&pos);
  pose->smooth_y = READ_DOUBLE(&pos);
  pose->smooth_z = READ_DOUBLE(&pos);

  pose->latitude = READ_DOUBLE(&pos);
  pose->longitude = READ_DOUBLE(&pos);
  pose->altitude = READ_DOUBLE(&pos);

  pose->v_north = READ_FLOAT(&pos);
  pose->v_east = READ_FLOAT(&pos);
  pose->v_up = READ_FLOAT(&pos);

  pose->speed = READ_FLOAT(&pos);
  pose->track = READ_FLOAT(&pos);

  pose->roll = READ_DOUBLE(&pos);
  pose->pitch = READ_DOUBLE(&pos);
  pose->yaw = READ_DOUBLE(&pos);

  pose->ar_roll = READ_DOUBLE(&pos);
  pose->ar_pitch = READ_DOUBLE(&pos);
  pose->ar_yaw = READ_DOUBLE(&pos);

  pose->a_x = READ_DOUBLE(&pos);
  pose->a_y = READ_DOUBLE(&pos);
  pose->a_z = READ_DOUBLE(&pos);

  pose->wander = READ_DOUBLE(&pos);

  pose->ID = READ_UINT(&pos);
  pos_tmp = pos;
  dbl_val = READ_DOUBLE(&pos_tmp);
  pos_tmp = pos;
  int_val = READ_INT(&pos_tmp);

  pose->postprocess_code = READ_INT(&pos);
  pose->hardware_timestamp = READ_DOUBLE(&pos);
  pose->hardware_time_mode = READ_INT(&pos);

  pose->timestamp = READ_DOUBLE(&pos);
  READ_HOST(pose->host, &pos);
  return pos;
}

char *StringToApplanixRms(char *string, ApplanixRms *rms) {
  char *pos = string;

  rms->rms_north = READ_FLOAT(&pos);
  rms->rms_east = READ_FLOAT(&pos);
  rms->rms_up = READ_FLOAT(&pos);

  rms->rms_v_north = READ_FLOAT(&pos);
  rms->rms_v_east = READ_FLOAT(&pos);
  rms->rms_v_up = READ_FLOAT(&pos);

  rms->rms_roll = READ_FLOAT(&pos);
  rms->rms_pitch = READ_FLOAT(&pos);
  rms->rms_yaw = READ_FLOAT(&pos);

  rms->semi_major = READ_FLOAT(&pos);
  rms->semi_minor = READ_FLOAT(&pos);
  rms->orientation = READ_FLOAT(&pos);

  rms->ID = READ_UINT(&pos);
  rms->postprocess_code = READ_INT(&pos);
  rms->hardware_timestamp = READ_DOUBLE(&pos);
  rms->hardware_time_mode = READ_INT(&pos);
  rms->timestamp = READ_DOUBLE(&pos);
  READ_HOST(rms->host, &pos);
  return pos;
}

char *StringToApplanixGps(char *string, ApplanixGps *gps) {
  char *pos = string;

  gps->primary_sats = READ_INT(&pos);
  gps->primary_ID = READ_UINT(&pos);
  gps->primary_timestamp = READ_DOUBLE(&pos);

  gps->secondary_sats = READ_INT(&pos);
  gps->secondary_ID = READ_UINT(&pos);
  gps->secondary_timestamp = READ_DOUBLE(&pos);

  gps->gams_solution_code = READ_INT(&pos);
  gps->gams_ID = READ_UINT(&pos);
  gps->gams_timestamp = READ_DOUBLE(&pos);

  gps->timestamp = READ_DOUBLE(&pos);
  READ_HOST(gps->host, &pos);
  return pos;
}

char *StringToApplanixDmi(char *string, ApplanixDmi *dmi) {
  char *pos = string;

  dmi->signed_odometer = READ_DOUBLE(&pos);
  dmi->unsigned_odometer = READ_DOUBLE(&pos);

  dmi->ID = READ_UINT(&pos);
  dmi->hardware_timestamp = READ_DOUBLE(&pos);
  dmi->hardware_time_mode = READ_INT(&pos);
  dmi->timestamp = READ_DOUBLE(&pos);
  READ_HOST(dmi->host, &pos);
  return pos;
}

void ApplanixAddLogReaderCallbacks(LogReaderCallbackList* callbacks) {
  callbacks->AddCallback("APPLANIX_POSE_V1", ApplanixPoseID, (LogConverterFunc) StringV1ToApplanixPose, sizeof(ApplanixPose), 0);
  callbacks->AddCallback("APPLANIX_POSE_V2", ApplanixPoseID, (LogConverterFunc) StringV2ToApplanixPose, sizeof(ApplanixPose), 0);
  callbacks->AddCallback("APPLANIX_RMS_V1", ApplanixRmsID, (LogConverterFunc) StringToApplanixRms, sizeof(ApplanixRms), 0);
  callbacks->AddCallback("APPLANIX_GPS_V1", ApplanixGpsID, (LogConverterFunc) StringToApplanixGps, sizeof(ApplanixGps), 0);
  callbacks->AddCallback("APPLANIX_DMI_V1", ApplanixDmiID, (LogConverterFunc) StringToApplanixDmi, sizeof(ApplanixDmi), 0);
}

//void ApplanixPoseWrite(ApplanixPose *pose, double logger_timestamp, vlr::cio::FILE* outfile) {
//  vlr::cio::fprintf(outfile, "APPLANIX_POSE_V2 %.9f %.9f %f %.9f %.9f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %u %d %f %d %f %s %f\n", pose->smooth_x,
//      pose->smooth_y, pose->smooth_z, pose->latitude, pose->longitude, pose->altitude, pose->v_north, pose->v_east, pose->v_up, pose->speed, pose->track,
//      pose->roll, pose->pitch, pose->yaw, pose->ar_roll, pose->ar_pitch, pose->ar_yaw, pose->a_x, pose->a_y, pose->a_z, pose->wander, pose->ID,
//      pose->postprocess_code, pose->hardware_timestamp, pose->hardware_time_mode, pose->timestamp, pose->host, logger_timestamp);
//}

//void ApplanixRmsWrite(ApplanixRms *rms, double logger_timestamp, vlr::cio::FILE* outfile) {
//  vlr::cio::fprintf(outfile, "APPLANIX_RMS_V1 %f %f %f %f %f %f %f %f %f %f %f %f %u %d %f %d %f %s %f\n", rms->rms_north, rms->rms_east, rms->rms_up,
//      rms->rms_v_north, rms->rms_v_east, rms->rms_v_up, rms->rms_roll, rms->rms_pitch, rms->rms_yaw, rms->semi_major, rms->semi_minor, rms->orientation,
//      rms->ID, rms->postprocess_code, rms->hardware_timestamp, rms->hardware_time_mode, rms->timestamp, rms->host, logger_timestamp);
//}
//
//void ApplanixGpsWrite(ApplanixGps *gps, double logger_timestamp, vlr::cio::FILE* outfile) {
//  vlr::cio::fprintf(outfile, "APPLANIX_GPS_V1 %d %u %f %d %u %f %d %u %f %f %s %f\n", gps->primary_sats, gps->primary_ID, gps->primary_timestamp,
//      gps->secondary_sats, gps->secondary_ID, gps->secondary_timestamp, gps->gams_solution_code, gps->gams_ID, gps->gams_timestamp, gps->timestamp, gps->host,
//      logger_timestamp);
//}
//
//void ApplanixDmiWrite(ApplanixDmi *dmi, double logger_timestamp, vlr::cio::FILE* outfile) {
//  vlr::cio::fprintf(outfile, "APPLANIX_DMI_V1 %f %f %u %f %d %f %s %f\n", dmi->signed_odometer, dmi->unsigned_odometer, dmi->ID, dmi->hardware_timestamp,
//      dmi->hardware_time_mode, dmi->timestamp, dmi->host, logger_timestamp);
//}
//
//void ApplanixAddLogWriterCallbacks(IpcInterface *ipc, double start_time, vlr::cio::FILE *logfile, dgc_subscribe_t subscribe_how) {
//  ipc->AddLogHandler(ApplanixPoseID, NULL, sizeof(ApplanixPose), (dgc_log_handler_t) ApplanixPoseWrite, start_time, logfile, subscribe_how);
//  ipc->AddLogHandler(ApplanixRmsID, NULL, sizeof(ApplanixRms), (dgc_log_handler_t) ApplanixRmsWrite, start_time, logfile, subscribe_how);
//  ipc->AddLogHandler(ApplanixGpsID, NULL, sizeof(ApplanixGps), (dgc_log_handler_t) ApplanixGpsWrite, start_time, logfile, subscribe_how);
//  ipc->AddLogHandler(ApplanixDmiID, NULL, sizeof(ApplanixDmi), (dgc_log_handler_t) ApplanixDmiWrite, start_time, logfile, subscribe_how);
//}

}
