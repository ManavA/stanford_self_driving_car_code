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


//#include <logio.h>
#include <global.h>
#include <velodyne/Block.h>
#include <velo_support.h>
#include <velodyne.h>
#include <lineBuffer.h>

namespace velodyne {

Spin::Spin() {
  blocks_.resize(MAX_NUM_VELOSCANS);
}

Spin::Spin(const Spin& copy) {
//  blocks_ = copy.blocks_;
}

void Spin::copy(const Spin& copy) {
  blocks_ = copy.blocks_;
}

Spin::~Spin() {
}

void Spin::save(const std::string& filename, double applanix_lat, double applanix_lon, double applanix_altitude) {
  FILE *fp;

  fp = fopen(filename.c_str(), "w");
  if (fp == NULL) {
    throw VLRException("could not open file " + filename + std::string(" for writing."));
  }

  size_t num_scans = blocks_.size();
  int dummy = fwrite(&num_scans, sizeof(size_t), 1, fp);
  dummy = fwrite(&blocks_[0], sizeof(velodyne::Block), blocks_.size(), fp);
  dummy = fwrite(&applanix_lat, sizeof(double), 1, fp);
  dummy = fwrite(&applanix_lon, sizeof(double), 1, fp);
  dummy = fwrite(&applanix_altitude, sizeof(double), 1, fp);
  fclose(fp);
}

void Spin::load(const std::string& filename, double* applanix_lat, double* applanix_lon, double* applanix_altitude) {

  FILE* fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    throw VLRException("could not open file " + filename + std::string(" for reading."));
  }

  size_t num_scans;
  int dummy = fread(&num_scans, sizeof(size_t), 1, fp);
  blocks_.resize(num_scans);
  dummy = fread(&blocks_[0], sizeof(velodyne::Block), num_scans, fp);
  dummy = fread(applanix_lat, sizeof(double), 1, fp);
  dummy = fread(applanix_lon, sizeof(double), 1, fp);
  dummy = fread(applanix_altitude, sizeof(double), 1, fp);
  fclose(fp);
}

void Spin::load(Velodyne::VelodyneFile& vlf, Velodyne& velodyne, dgc_velodyne_index* vindex, int which, double* applanix_lat,
    double* applanix_lon, double* applanix_alt) {

  blocks_.clear();
  vlr::cio::fseek(vlf.fp, vindex->spin[which].file_offset, SEEK_SET);

  Velodyne::UDPPacket pkt;
  bool first = true;
  int last_encoder = 0, current_pose = 0;
  driving_common::Pose pose;

  while (1) {
    Velodyne::readPacket(&vlf, &pkt);

    int start = 0;
    if (first) {
      start = vindex->spin[which].spin_start_i;
      first = false;
    }

    for (int32_t i = start; i < velodyne::Packet::NUM_BLOCKS; i++) {
      int encoder = (pkt.scan[i].encoder + velodyne.config().spin_start) % VELO_NUM_TICKS;

      if (encoder < last_encoder) {
        *applanix_lat = vindex->spin[which].pose[0].latitude;
        *applanix_lon = vindex->spin[which].pose[0].longitude;
        *applanix_alt = vindex->spin[which].pose[0].altitude;
        return;
      }

      if (current_pose < vindex->spin[which].num_poses && blocks_.size() == (size_t)vindex->spin[which].pose[current_pose].scan_num) {
        pose.x = vindex->spin[which].pose[current_pose].smooth_x;
        pose.y = vindex->spin[which].pose[current_pose].smooth_y;
        pose.z = vindex->spin[which].pose[current_pose].smooth_z;
        pose.roll = vindex->spin[which].pose[current_pose].roll;
        pose.pitch = vindex->spin[which].pose[current_pose].pitch;
        pose.yaw = vindex->spin[which].pose[current_pose].yaw;
        current_pose++;
      }

      blocks_.resize(blocks_.size()+1);
      velodyne.projectMeasurement(pkt.scan[i], blocks_[blocks_.size()-1], pose);
      last_encoder = encoder;
    }
  }
}

#define READ_FLOAT(pos) strtof(*(pos), (pos))
#define READ_DOUBLE(pos) strtod(*(pos), (pos))
#define READ_UINT(pos) strtoul(*(pos), (pos), 10)
#define READ_INT(pos) strtol(*(pos), (pos), 10)

char* StringV2ToApplanixPose(char* string, applanix::ApplanixPose* pose) {
  char *pos = string;
  char *pos_tmp;
  double dbl_val, int_val;

  pose->smooth_x = READ_DOUBLE(&pos);
  pose->smooth_y = READ_DOUBLE(&pos);
  pose->smooth_z = READ_DOUBLE(&pos);

  pose->latitude = READ_DOUBLE(&pos);
  pose->longitude = READ_DOUBLE(&pos);
  pose->altitude = READ_DOUBLE(&pos);

  pose->vel_north = READ_FLOAT(&pos);
  pose->vel_east = READ_FLOAT(&pos);
  pose->vel_up = READ_FLOAT(&pos);

  pose->speed = READ_FLOAT(&pos);
  pose->track = READ_FLOAT(&pos);

  pose->roll = READ_DOUBLE(&pos);
  pose->pitch = READ_DOUBLE(&pos);
  pose->yaw = READ_DOUBLE(&pos);

  pose->rate_roll = READ_DOUBLE(&pos);
  pose->rate_pitch = READ_DOUBLE(&pos);
  pose->rate_yaw = READ_DOUBLE(&pos);

  pose->accel_x = READ_DOUBLE(&pos);
  pose->accel_y = READ_DOUBLE(&pos);
  pose->accel_z = READ_DOUBLE(&pos);

  pose->wander = READ_DOUBLE(&pos);

  pose->id = READ_UINT(&pos);
  pos_tmp = pos;
  dbl_val = READ_DOUBLE(&pos_tmp);
  pos_tmp = pos;
  int_val = READ_INT(&pos_tmp);

  pose->postprocess_code = READ_INT(&pos);
  pose->hardware_timestamp = READ_DOUBLE(&pos);
  pose->hardware_time_mode = READ_INT(&pos);

  pose->timestamp = READ_DOUBLE(&pos);
  return pos;
}

int read_applanix_message(vlr::cio::FILE* log_fp, driving_common::LineBuffer* line_buffer, applanix::ApplanixPose* applanix_pose) {
  char *line = NULL, *s = NULL;

  do {
    line = line_buffer->readLine(log_fp);
    if (line == NULL) return -1;
    if (strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
      s = StringV2ToApplanixPose(dgc::dgc_next_word(line), applanix_pose);
    }
  } while (s == NULL);
  return 0;
}

//int read_applanix_message(cio::FILE *log_fp, LineBuffer *line_buffer, ApplanixPose *applanix_pose, LocalizePose *localize_pose) {
//  char *line = NULL, *s = NULL;
//
//  do {
//    line = line_buffer->ReadLine(log_fp);
//    if (line == NULL) return -1;
//    if (strncmp(line, "APPLANIX_POSE_V2", 16) == 0) s = StringV2ToApplanixPose(dgc_next_word(line), applanix_pose);
//    else if (strncmp(line, "LOCALIZE_POSE2", 14) == 0) StringV2ToLocalizePose(dgc_next_word(line), localize_pose);
//  } while (s == NULL);
//  return 0;
//}
//
void vlf_projector(const std::string& velo_log_filename, const std::string& ipc_log_filename,
                   const std::string& cal_filename, const std::string& int_filename, dgc::dgc_transform_t velodyne_offset, spin_function f) {

  applanix::ApplanixPose applanix_pose, next_applanix_pose;
  driving_common::Pose pose;
  int i, err, encoder, last_encoder = 0;
  bool incomplete = false;
  vlr::cio::FILE *log_fp = NULL;
  Spin spin;

    // prepare for velodyne
  Velodyne::VelodyneFile* velodyne_file = Velodyne::openFile(velo_log_filename);
  if (velodyne_file == NULL) {
    throw VLRException("Could not open velodyne file " + velo_log_filename);
  }

  Config config;
  if (!config.readCalibration(cal_filename)) {
    throw VLRException("Could not read calibration file " + cal_filename);
  }

  if (!config.readIntensity(int_filename)) {
    throw VLRException("Could not read intensity calibration file " + int_filename);
  }

  config.integrateOffset(velodyne_offset);

    // prepare IPC logfile
  log_fp = vlr::cio::fopen(ipc_log_filename.c_str(), "r");
  if (!log_fp) {
    throw VLRException("Could not open file " + ipc_log_filename + "for reading.");
  }

  driving_common::LineBuffer* line_buffer = new driving_common::LineBuffer;

  /* read two applanix messages */
  err = read_applanix_message(log_fp, line_buffer, &applanix_pose);
  if (err < 0) return;
  err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose);
  if (err < 0) return;

  Velodyne::UDPPacket pkt;
  while (1) {
      // read a velodyne packet
    Velodyne::readPacket(velodyne_file, &pkt);

    /* remember if we got a packet before the 1st pose */
    if (pkt.timestamp < applanix_pose.timestamp) incomplete = true;
    /* read applanix messages until the velodyne data falls
     between the timestamps */
    else while (next_applanix_pose.timestamp < pkt.timestamp) {
      applanix_pose = next_applanix_pose;
      err = read_applanix_message(log_fp, line_buffer, &next_applanix_pose);
      if (err < 0) return;
    }

    /* project the scan according to the current pose */
    for (i = 0; i < velodyne::Packet::NUM_BLOCKS; i++) {
      encoder = (pkt.scan[i].encoder + config.spin_start) % VELO_NUM_TICKS;
      if (encoder < last_encoder) {
        if (!incomplete) f(&spin, &config, &applanix_pose);
        //spin.numBlocks() = 0;
        spin.blocks_.clear();
        incomplete = false;
      }

      if (spin.numBlocks() < MAX_NUM_VELOSCANS) {
        pose.x = applanix_pose.smooth_x;
        pose.y = applanix_pose.smooth_y;
        pose.z = applanix_pose.smooth_z;
        pose.roll = applanix_pose.roll;
        pose.pitch = applanix_pose.pitch;
        pose.yaw = applanix_pose.yaw;
        spin.blocks_.resize(spin.blocks_.size()+1);
        Velodyne::projectMeasurement(config, pkt.scan[i], spin.blocks_[spin.numBlocks()-1], pose);
        spin.blocks_[spin.numBlocks()-1].timestamp = pkt.timestamp;
 //       spin.numBlocks()++;
      }
      last_encoder = encoder;
    }
  }
}

//dgc_velodyne_index::dgc_velodyne_index() {
//  num_spins = 0;
//  spin = NULL;
//}
//
//dgc_velodyne_index::~dgc_velodyne_index() {
//  int i;
//
//  for (i = 0; i < num_spins; i++)
//    free(spin[i].pose);
//  free(spin);
//}
//
//int dgc_velodyne_index::load(char *filename) {
//  int n, max_spins = 0;
//  off64_t offset;
//  cio::FILE *fp;
//
//  fp = cio::fopen(filename, "r");
//  if (fp == NULL) {
//    fprintf(stderr, "Error: could not open file %s for reading.\n", filename);
//    return -1;
//  }
//
//  do {
//    n = cio::fread(&offset, sizeof(off64_t), 1, fp);
//    if (n == 1) {
//      if (num_spins == max_spins) {
//        max_spins += 10000;
//        spin = (dgc_velodyne_index_entry *) realloc(spin, max_spins * sizeof(dgc_velodyne_index_entry));
//        dgc_test_alloc(spin);
//      }
//
//      spin[num_spins].file_offset = offset;
//      cio::fread(&spin[num_spins].spin_start_i, sizeof(int), 1, fp);
//      cio::fread(&spin[num_spins].num_scans, sizeof(int), 1, fp);
//      cio::fread(&spin[num_spins].num_poses, sizeof(int), 1, fp);
//
//      spin[num_spins].pose = (dgc_velodyne_index_pose *) calloc(spin[num_spins].num_poses, sizeof(dgc_velodyne_index_pose));
//      dgc_test_alloc(spin[num_spins].pose);
//
//      cio::fread(spin[num_spins].pose, sizeof(dgc_velodyne_index_pose), spin[num_spins].num_poses, fp);
//
//      num_spins++;
//    }
//  } while (n == 1);
//  cio::fclose(fp);
//  return 0;
//}
//
//int dgc_velodyne_index::save(char *filename) {
//  int i, percent, last_percent = -1;
//  cio::FILE *fp;
//
//  fp = cio::fopen(filename, "w");
//  if (fp == NULL) {
//    fprintf(stderr, "Error: could not open file %s for writing.\n", filename);
//    return -1;
//  }
//
//  for (i = 0; i < num_spins; i++) {
//    cio::fwrite(&spin[i].file_offset, sizeof(off64_t), 1, fp);
//    cio::fwrite(&spin[i].spin_start_i, sizeof(int), 1, fp);
//    cio::fwrite(&spin[i].num_scans, sizeof(int), 1, fp);
//    cio::fwrite(&spin[i].num_poses, sizeof(int), 1, fp);
//    cio::fwrite(spin[i].pose, sizeof(dgc_velodyne_index_pose), spin[i].num_poses, fp);
//    percent = (int) rint(i / (double) num_spins * 100.0);
//    if (percent != last_percent) {
//      fprintf(stderr, "\rSaving %s (%d%%)    ", filename, percent);
//      last_percent = percent;
//    }
//  }
//  fprintf(stderr, "\rSaving %s (100%%)    \n", filename);
//  cio::fclose(fp);
//  return 0;
//}

} // namespace velodyne
