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


#ifndef PAW2GLVIEW_H_
#define PAW2GLVIEW_H_

#include <boost/signals.hpp>
#include <tf/tf.h>

//#include <camera_interface.h>
#include <vehiclemodels.h>
#include <passatmodel.h>
#include <imagery.h>
//#include <rndfgl.h>

#include <vlrImage.h>
#include <glWidget.h>

#include <paw2.h>

namespace vlr {

class Paw2Gui;

class Paw2GlView : public vlr::GLWidget {
     Q_OBJECT

private:
  vlr::Image<float>* map;
  unsigned int frame_count_;

public:
	Paw2GlView(QWidget *parent = NULL);
    ~Paw2GlView();

  void signalGuiReady(Paw2Gui* gui);

public slots:

protected:
     void initializeGL();
     void paintGL();
//     void resizeGL(int width, int height);

private:
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
  void keyPressEvent(QKeyEvent* event);

//	void drawTrajectory(double center_x, double center_y);
//
//	bool displayRings();
//	bool drawCircle(double cx, double cy, double cz, double r);
//	bool displayVehicle(int draw_vehicle);
//	bool displayVehicleSignal();
//	bool displayFlags(int offsetMode, double& offsetX, double& offsetY, double& yaw, double& pitch, double& roll);
//	bool displayRadar1(double& smoothXPos, double& smoothYPos, double& smoothZos);
//	bool displayCameraMap(double& originX, double& originY);
//	bool displaySimulatorData(double& originX, double& originY);
//	bool displayHelp();
//
//
//	void draw_left_turn_signal_3D(float x, float y, float z, float r, float t);
//	void draw_right_turn_signal_3D(float x, float y, float z, float r, float t);
//	void draw_left_turn_signal(float x, float y, float r);
//	void draw_right_turn_signal(float x, float y, float r);
//	void draw_pedals(float x, float y, float w, float h, double throttle, double brake);
//	void draw_steering_wheel(float x, float y, float r, float steering_angle);
//	inline void draw_nline_flag(double x, double y, double w, double h, int num_lines, char **line);
//	inline void draw_vehicle_cage(double x, double y, double theta, double w, double l, int id, double v, double range, int draw_flag);
//	inline void draw_vehicle_halo(double x, double y, double theta, double w, double l, int id, double v, double range, int draw_flag);
//	inline void draw_observed_car(double x, double y, double theta, double w, double l, int id, double v, double range, int draw_flag);
//	void draw_cube(float x, float y, float z1, float z2, float w);
	//	FontRenderer fr;

  void drawObstacles(double center_x, double center_y);
  void drawObstaclePredictions(driving_common::GlobalPose& pose);
  void drawPoseHistory(double center_x, double center_y);
  void drawTrajectories(driving_common::GlobalPose& pose);
  void drawBestTrajectory(driving_common::GlobalPose& pose);
  void drawCenterline(std::vector<CurvePoint>& center_line, double center_x, double center_y);
  void drawCompleteCenterLine(const std::vector<CurvePoint>& mission_line_bez, const std::vector<CurvePoint>& mission_line, double center_x, double center_y);
  void drawTopology(double center_x, double center_y);
  void drawPoseAccuracy(dgc::dgc_pose_t robot_pose_accuracy);
  void drawDistanceCond(const double x, const double y, const double r);
  void drawEgoDistances(const double center_x, const double center_y, const double yaw, Topology* t);
  void drawDestinations(const double center_x, const double center_y);
  void drawVehicle(const driving_common::GlobalPose& pose);
  void updateObstacleMapTexture(const uint8_t* map_data, GLuint& texture, double& last_timestamp);
  void drawObstacleMap(GLuint texture,  double smooth_x, double smooth_y, const driving_common::GlobalPose& latest_pose, float r, float g, float b, bool invert);
  void updateRoadMapTexture(const uint8_t* map_data, double timestamp);
  void drawRoadMap(const driving_common::GlobalPose& pose, const driving_common::GlobalPose& latest_pose, float r, float g, float b);
  void drawVehDistances(const double center_x, const double center_y, Topology* t);
  void drawIntersectionMergingPoint(const double center_x, const double center_y, Topology* t);
  void drawLaneChangeMergingPoints(const double center_x, const double center_y, Topology* t);
  void drawSensorData(const driving_common::GlobalPose& pose);

private:
  Paw2Gui* gui_;

  GLfloat light_ambient_[4];
  GLfloat light_diffuse_[4];
  GLfloat light_specular_[4];
  GLfloat light_position_[4];

  static const float cmap_rb1_red_[256], cmap_rb2_red_[256];
  static const float cmap_rb1_green_[256], cmap_rb2_green_[256];
  static const float cmap_rb1_blue_[256], cmap_rb2_blue_[256];

  ChsmPlanner::Parameters planner_params_;

    // imagery
  //dgc_imagery_p imagery_;

  passatwagonmodel_t* car_;
  VehiclePorscheModel* porsche_;
  VehicleHummerModel* hummer_;
  VehicleEliseModel* elise_;

  float max_velocity_, curvature_slowdown_factor_;
  float urgency_;

  GLuint configuration_space_texture_, obstacle_map_texture_, road_map_texture_;

  GLUquadric* sphere_;
  bool create_rndf_display_list_;

  GLint rndf_display_list_;
  double rndf_dl_origin_x_, rndf_dl_origin_y_;

//  rndf::rndf_display_list rndf_display_list2_; // TODO: should go to RoadNetworkGL
  double last_raw_map_timestamp_, last_configuration_space_timestamp_, last_roadmap_timestamp_;
  double* smoothed_mission_points_buf_;  // buffer for OpenGL Bezier visualization
  uint32_t smoothed_mission_points_buf_size_;

  GLint pedestrian_display_list_;
  GLint bicycle_display_list_;

  Imagery* imagery_;

  pthread_mutex_t gui_ready_mutex_;
  pthread_cond_t gui_ready_cv_;
};

} // namespace vlr

#endif // PAW2GLVIEW_H_


