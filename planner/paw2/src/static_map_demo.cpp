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

#include <global.h>
#include <opencv2/core/core.hpp>
#include <highgui.h>

#include <lltransform.h>
#include <obstacle_types.h>
#include "static_map_demo.h"

using namespace std;

namespace drc = driving_common;

namespace vlr {

StaticMapDemo::StaticMapDemo(const std::string& rndf_name, const std::string& mdf_name, const std::string& map_name,
        const double start_lat, const double start_lon, const double start_yaw) :
    BaseDemo(rndf_name, mdf_name, start_lat, start_lon, start_yaw), static_obstacle_map_resolution_(.1), new_map_read_(false) {

    readObstacleMap(map_name);
}


StaticMapDemo::~StaticMapDemo() {
}

void StaticMapDemo::readObstacleMap(const std::string& map_name) {

    printf("Trying to load map ##############\n");

    cv::Mat image = cv::imread(map_name, CV_LOAD_IMAGE_GRAYSCALE);

    printf("MAP LOADED ##############\n");

    static_obstacle_map_size_x_ = image.cols;
    static_obstacle_map_size_y_ = image.rows;
    map_data_ = (uint8_t*) image.data;

    uint32_t numpix=0;
    double sum=0;
    for(int32_t i=0; i<static_obstacle_map_size_x_*static_obstacle_map_size_y_; i++) {
      if(map_data_[i]) {numpix++;}
      sum+=map_data_[i];
    }
    printf("Found %u pixel, from averaging: %u\n", numpix, (uint32_t)sum/255);
//          uint32_t img_id=0;
//          ilGenImages(1, &img_id);
//          ilBindImage(img_id);
//       char buf[100];
//        uint8_t* tdata = new uint8_t[static_obstacle_map_size_x_*static_obstacle_map_size_y_];
//        memcpy(tdata, map_data_, static_obstacle_map_size_x_*static_obstacle_map_size_y_);
//        static uint32_t frame_num=0;
//        sprintf(buf, "staticmap%04d.png", frame_num);
//        printf("%s\n", buf);
//        ilTexImage(static_obstacle_map_size_x_, static_obstacle_map_size_y_, 1, 1, IL_LUMINANCE, IL_UNSIGNED_BYTE, tdata);
//        ilSave(IL_PNG, buf);
//        frame_num++;
//        ilBindImage( img_id_);

    obstacle_msg_.static_point.resize(static_obstacle_map_size_x_ * static_obstacle_map_size_y_);
    obstacle_msg_.timestamp = drc::Time::current();
    new_map_read_ = true;

}

void StaticMapDemo::updateObstaclePredictions(double /*t*/) {
    //    chsm_planner_->updateStaticObstacleMapSize(static_obstacle_map_size_x_*static_obstacle_map_resolution_, static_obstacle_map_size_y_*static_obstacle_map_resolution_, static_obstacle_map_resolution_);

    if (new_map_read_) {
        double cx, cy;
        //       latLongToUtm(map_center_lat, map_center_lon, &cx, &cy, map_zone);
        cx = cy = 0;
        obstacle_msg_.static_point.clear();
        perception::StaticObstaclePoint point;
        for (int32_t yi = 0; yi < (int32_t) static_obstacle_map_size_y_; yi++) {
            for (int32_t xi = 0; xi < (int32_t) static_obstacle_map_size_x_; xi++) {
                if (map_data_[yi * static_obstacle_map_size_x_ + xi]) {
                    point.x = static_obstacle_map_resolution_ * (static_obstacle_map_size_x_ / 2 - xi) + cx;
                    point.y = static_obstacle_map_resolution_ * (static_obstacle_map_size_y_ / 2 - yi) + cy;
                    point.z_min = 0;
                    point.z_max = 5;
                    point.type = OBSTACLE_UNKNOWN; // PERCEPTION_MAP_OBSTACLE_FREE; //OBSTACLE_UNKNOWN; // now this is *** :-(
                    obstacle_msg_.static_point.push_back(point);
                }
            }
        }
        new_map_read_ = false;
    }
    obstacle_msg_.timestamp = current_timestamp_;  // update timestamp even if map stays the same..otherwise we loose the map in the pose queue
}

} // namespace vlr
