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


#ifndef PAW2GUI_H_
#define PAW2GUI_H_

#include <iostream>
#include <iomanip>
#include <string>

#include <boost/signals.hpp>
#include <ros/ros.h>
#include <tf/tf.h>

#include <QtGui/QtGui>

#include <paw2.h>
#include <paw2App.h>

//namespace vlr {
//class LocalizeHistMod;
//class ReadLaserMapMod;
//class CreateLaserMapMod;
//};

#include <paw2GlView.h>
#include <ui_paw2_qtgui.h>

namespace vlr {
class Paw2Can;
class Paw2ApplanixPose;
class Paw2ApplanixRMS;
class Paw2ApplanixDMI;
class Paw2ApplanixGPS;
class Paw2Velodyne;
class Paw2Camera;
class Paw2InternalData;

typedef enum {
  LOCALIZATION_METHOD_HISTOGRAM
} LocalizationMethod;

typedef enum {
  LOCALIZATION_OFFSET_NONE, LOCALIZATION_OFFSET_LASER, LOCALIZATION_OFFSET_CAMERA
} LocalizationOffsetMode;

class Paw2Gui : public QMainWindow {
  Q_OBJECT

public:
	Paw2Gui(Paw2App& app, AWRoadPlanner& awp, ros::NodeHandle& nh, QWidget *parent = NULL);
    ~Paw2Gui();

  void registerMessageHandlers();  // this function is not put in the constructor: sometimes we don't want to register all of this
  void requestPlannerGlRedraw() {ui.plannerGlView->requestRedraw();}
  void requestCameraGlRedraw() {ui.cameraGlView->requestRedraw();}
  inline driving_common::GlobalPose pose(double timestamp) {return awp_.chsm_planner_->pose(timestamp);}
  inline double getGLCamDistance() {return ui.plannerGlView->getCamDistance();}
  inline double getGLCamPan() {return ui.plannerGlView->getCamPan();}
  inline double getGLCamTilt() {return ui.plannerGlView->getCamTilt();}

//  inline bool showVehicle() {return showVehicle_;}
//	inline bool showFlea2Camera1() {return showFlea2Camera1_;}
////	inline bool showLDLRS1() {return showLDLRS1_;}
////	inline bool showLDLRS2() {return showLDLRS2_;}
//	inline bool showRadar() {return showRadar_;}
//	inline bool showEdges() {return showEdges_;}
//	inline bool showFlags() {return showFlags_;}
//	inline bool showRings() {return showRings_;}
//	inline bool showHelp() {return showHelp_;}
//	inline bool showPoints() {return showPoints_;}
//  inline bool showLocalization() {return showLocalization_;}
//  inline bool showImagery() {return showImagery_;}
//	inline bool showRNDF() {return showRNDF_;}

  LocalizationOffsetMode localizationOffsetMode();
  LocalizationMethod localizationMethod();
  std::string& videoFileName() {return video_file_name_;}
  bool recordVideoState() {return record_video_;}

  bool event(QEvent* ev);


private:
  void updateStatus();
  void updateMissionDisplay();
  void updateMissionVelocity();
  void updateMissionCurvature();

private slots:
  void on_action_Exit_activated();
  void on_action_Open_Log_activated();
  void on_action_Open_Camera_Log_activated();
  void on_action_Open_Velodyne_Log_activated();
	void on_action_Save_Video_As_activated();

	void on_action_Reset_activated();
	void on_action_Rewind_activated();
	void on_action_Play_activated();
	void on_action_Pause_activated();
	void on_action_Forward_activated();
	void on_action_Stop_activated();

	void on_action_Set_Laser_Map_Folder_activated();
  void on_action_Create_Laser_Map_activated();

	//	void on_laneWidth_valueChanged(double laneWidth);
//	void on_leftBoundary_currentIndexChanged(int index);
//	void on_rightBoundary_currentIndexChanged(int index);
//
//	void on_wpLat_valueChanged(double wpLat);
//	void on_wpLon_valueChanged(double wpLon);

    // planner parameter tab
public slots:  // TODO: Make GlView friend class
  inline void on_plannerDisplayRNDF_stateChanged(int state) {show_rndf_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayImagery_stateChanged(int state) {show_imagery_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayVehicle_stateChanged(int state) {show_vehicle_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayCenterLine_stateChanged(int state) {show_center_line_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayCompleteCenterLine_stateChanged(int state) {show_complete_center_line_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayTrajectories_stateChanged(int state) {show_trajectories_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayBestTrajectory_stateChanged(int state) {show_best_trajectory_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayCompleteGraph_stateChanged(int state) {show_complete_graph_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayMissionGraph_stateChanged(int state) {show_mission_graph_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayCompleteMissionGraph_stateChanged(int state) {show_complete_mission_graph_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayMatchedEdge_stateChanged(int state) {show_matched_edge_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayDynamicObstacles_stateChanged(int state) {show_dynamic_objects_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayDynamicObstaclePredictions_stateChanged(int state) {show_obstacle_predictions_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayDynamicObstacleDestinations_stateChanged(int state) {show_obstacle_destinations_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayRoadMap_stateChanged(int state) {show_road_map_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayStaticObstacleMap_stateChanged(int state) {show_obstacle_map_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayConfigurationSpace_stateChanged(int state) {show_configuration_space_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayEgoDistances_stateChanged(int state) {show_ego_distances_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayVehicleDistances_stateChanged(int state) {show_veh_distances_ = (state!=0);  ui.plannerGlView->requestRedraw();}
  inline void on_plannerDisplayIntersectionMergePoint_stateChanged(int state) {show_intersection_merging_point_ = (state!=0);  ui.plannerGlView->requestRedraw();}

    // view parameter tab
  void on_showVelodyne_stateChanged(int state);
  void on_velodyneColorMode_currentIndexChanged(int index);
  void on_velodyneDisplayStyle_currentIndexChanged(int index);
  void on_velodynePointSize_valueChanged(int value);
  void on_velodyneShowUpperBlock_stateChanged(int state);
  void on_velodyneShowLowerBlock_stateChanged(int state);

//  void on_showVehicle_stateChanged(int state);
//	void on_showFlea2Camera1_stateChanged(int state);
////	void on_showLDLRS1_stateChanged(int state);
////	void on_showLDLRS2_stateChanged(int state);
//	void on_showRadar_stateChanged(int state);
//	void on_showFlags_stateChanged(int state);
//	void on_showRings_stateChanged(int state);
//	void on_showHelp_stateChanged(int state);
//  void on_showLocalization_stateChanged(int state);
//  void on_showImagery_stateChanged(int state);
//  void on_showLaserMap_stateChanged(int state);
//	void on_showRNDF_stateChanged(int state);
//	void on_showEdges_stateChanged(int state);
//	void on_showPoints_stateChanged(int state);

	void on_camHeight_valueChanged(double value);
	void on_camYawSlider_valueChanged(int value);
	void on_camYaw_valueChanged(double value);
	void on_camPitchSlider_valueChanged(int value);
	void on_camPitch_valueChanged(double value);
	void on_camRollSlider_valueChanged(int value);
	void on_camRoll_valueChanged(double value);

	void on_locMethod_currentIndexChanged(int index);

private:
	bool showVehicle_;
	bool showLeftSick_;
	bool showRightSick_;
	bool showFlea2Camera1_;
	bool showRiegl1_;
	bool showIbeo1_;
	bool showIbeo2_;
	bool showLDLRS1_;
	bool showLDLRS2_;
	bool showRadar_;
	bool showEdges_;
	bool showFlags_;
	bool showRings_;
	bool showHelp_;
	bool showPoints_;
  bool showLocalization_;
  bool showImagery_;
  bool showLaserMap_;
	bool showRNDF_;

public:
	bool show_topology_;
	bool show_rndf_;
	bool show_vehicle_;
	bool show_pose_history_;
	bool show_curvepoints_;
	bool show_center_line_;
	bool show_complete_center_line_;
	bool show_trajectories_;
	bool show_best_trajectory_;
	bool show_complete_graph_;
	bool show_mission_graph_;
	bool show_matched_edge_;
	bool show_yellow_dot_;
	bool show_dynamic_objects_;
	bool show_complete_mission_graph_;
	bool show_obstacle_destinations_;
	bool show_imagery_;
  bool show_road_map_;
  bool show_obstacle_map_;
	bool show_configuration_space_;
	bool show_radius_;
	bool show_pose_accuracy_;
	bool show_ego_distances_;
	bool show_veh_distances_;
	bool show_intersection_merging_point_;
	bool show_lanchange_merging_points_;
	bool show_obstacle_predictions_;

	bool show_velodyne_;
  bool show_camera_lf_;
  bool show_camera_rf_;

private:
	std::string video_file_name_;
	bool record_video_;

public:	//TODO: Only until threading is running...
    Ui::PAW2 ui;
    Paw2App& app_;
    std::string imagery_folder_;
    AWRoadPlanner& awp_;
    Paw2Velodyne* velodyne_;
    Paw2Camera* camera_lf_, *camera_rf_;
    Paw2InternalData* internal_data_;

private:
    Paw2Can* can_status_;
    Paw2ApplanixPose* applanix_pose_;
    Paw2ApplanixRMS* applanix_rms_;
    Paw2ApplanixDMI* applanix_dmi_;
    Paw2ApplanixGPS* applanix_gps_;
};

} // namespace vlr

#endif // PAW2GUI_H_
