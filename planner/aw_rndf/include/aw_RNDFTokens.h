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


#ifndef AW_RNDFTOKENS_H
#define AW_RNDFTOKENS_H

namespace vlr {

// General
#define RNDF_DELIMITER                          "\012 \015 \t \011"

  // Misc
#define SRNDF_HEADER                            "SRNDF"
#define SRNDF_IDSTRING                          "id_string"
#define SRNDF_LIBVERSION                        "rndf_lib_version"

// Road Network
#define RNDF_ROADNETWORK_NAME                   "RNDF_name"
#define RNDF_ROADNETWORK_NUM_SEGMENTS           "num_segments"
#define RNDF_ROADNETWORK_NUM_ZONES              "num_zones"
#define RNDF_ROADNETWORK_NUM_INTERSECTIONS      "num_intersections"
#define RNDF_ROADNETWORK_FORMAT_VERSION         "format_version"
#define RNDF_ROADNETWORK_CREATION_DATE          "creation_date"
#define RNDF_ROADNETWORK_END_FILE               "end_file"

// Segments
#define RNDF_SEGMENT_BEGIN                      "segment"
#define RNDF_SEGMENT_END                        "end_segment"
#define RNDF_SEGMENT_NAME                       "segment_name"
#define RNDF_SEGMENT_NUM_LANES                  "num_lanes"
#define RNDF_SEGMENT_SPEED_LIMIT                "speed_limit"
#define RNDF_SEGMENT_NUM_CROSSWALKS             "num_crosswalks"
#define RNDF_SEGMENT_OFFROAD                    "offroad"

// Lanes
#define RNDF_LANE_BEGIN                         "lane"
#define RNDF_LANE_END                           "end_lane"
#define RNDF_LANE_NUM_WAYPOINTS                 "num_waypoints"
#define RNDF_LANE_WIDTH                         "lane_width"
#define RNDF_LANE_TYPE                          "lane_type"
#define RNDF_LANE_LEFT_BOUNDARY                 "left_boundary"
#define RNDF_LANE_RIGHT_BOUNDARY                "right_boundary"
#define RNDF_LANE_SPEED_LIMIT                   "speed_limit" // not used yet...

// Lane boundary
#define RNDF_LANE_BOUNDARYTYPE_SOLIDWHITE       "solid_white"
#define RNDF_LANE_BOUNDARYTYPE_BROKENWHITE      "broken_white"
#define RNDF_LANE_BOUNDARYTYPE_SOLIDYELLOW      "solid_yellow"
#define RNDF_LANE_BOUNDARYTYPE_DOUBLEYELLOW     "double_yellow"

// Lane type
#define RNDF_LANE_TYPE_CARLANE                  "car_lane"
#define RNDF_LANE_TYPE_BIKELANE                 "bike_lane"

// Checkpoints
#define RNDF_CHECKPOINT                         "checkpoint"
// Exits
#define RNDF_EXIT                               "exit"
// Stops
#define RNDF_STOP                               "stop"
// Crosswalks
#define RNDF_CROSS                              "cross"
// Traffic lights
#define RNDF_LIGHT                              "light"

// Zones
#define RNDF_ZONE_BEGIN                         "zone"
#define RNDF_ZONE_END                           "end_zone"
#define RNDF_ZONE_NUM_SPOTS                     "num_spots"
#define RNDF_ZONE_NAME                          "zone_name"
#define RNDF_ZONE_OFFROAD                       "offroad"

// Perimeter
#define RNDF_PERIMETER_BEGIN                     "perimeter"
#define RNDF_PERIMETER_END                       "end_perimeter"
#define RNDF_PERIMETER_NUM_PERIMETERPOINTS       "num_perimeterpoints"

// Spots
#define RNDF_SPOT_BEGIN                          "spot"
#define RNDF_SPOT_END                            "end_spot"
#define RNDF_SPOT_WIDTH                          "spot_width"

// Intersections
#define RNDF_INTERSECTION_BEGIN                  "intersection"
#define RNDF_INTERSECTION_END                    "end_intersection"
#define RNDF_INTERSECTION_NUM_LIGHTS             "num_trafficlights"

// Traffic lights
#define RNDF_TRAFFIC_LIGHT_BEGIN                 "trafficlight"
#define RNDF_TRAFFIC_LIGHT_END                   "end_trafficlight"
#define RNDF_TRAFFIC_LIGHT_GROUP_ID              "group_id"
#define RNDF_TRAFFIC_LIGHT_POSITION              "position"
#define RNDF_TRAFFIC_LIGHT_ORIENTATION           "orientation"

// Crosswalks
#define RNDF_CROSSWALK_BEGIN                    "crosswalk"
#define RNDF_CROSSWALK_END                      "end_crosswalk"
#define RNDF_CROSSWALK_WIDTH                    "crosswalk_width"
#define RNDF_CROSSWALK_P1                       "crosswalk_p1"
#define RNDF_CROSSWALK_P2                       "crosswalk_p2"
#define RNDF_CROSSWALK_TYPE_STOP                "stop"
#define RNDF_CROSSWALK_TYPE_INCOMING            "incoming"

} // namespace vlr

#endif
