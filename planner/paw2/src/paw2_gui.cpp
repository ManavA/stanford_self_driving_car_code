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


#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>

#include <velodyne.h>
#include <glWidget.h>

#include <paw2.h>
#include <paw2_gui.h>
#include <graphics.h>
#include <paw2Velodyne.h>
#include <paw2Camera.h>
#include <paw2InternalData.h>
#include <drawSensorData.h>

namespace drc = driving_common;

namespace vlr {

Paw2Gui::Paw2Gui(Paw2App& app, AWRoadPlanner& awp, ros::NodeHandle& nh, QWidget* parent) :
                                        QMainWindow(parent), app_(app), awp_(awp),
                                        velodyne_(NULL), camera_lf_(NULL), camera_rf_(NULL) {
ui.setupUi(this);

//showVehicle_ = ui.showVehicle->checkState();
////showLeftSick_ = ui.showLeftSick->checkState();
////showRightSick_ = ui.showRightSick->checkState();
//showFlea2Camera1_ = ui.showFlea2Camera1->checkState();
//showRadar_ = ui.showRadar->checkState();
////showEdges_ = ui.showEdges->checkState();
////showFlags_ = ui.showFlags->checkState();
////showRings_ = ui.showRings->checkState();
////showHelp_ = ui.showHelp->checkState();
////showPoints_ = ui.showPoints->checkState();
//showLocalization_ = ui.showLocalization->checkState();
//showImagery_ = ui.showImagery->checkState();
//showLaserMap_ = ui.showLaserMap->checkState();
//showRNDF_ = ui.showRNDF->checkState();


show_topology_ = true;
show_rndf_ = ui.plannerDisplayRNDF->checkState();
show_imagery_ = ui.plannerDisplayImagery->checkState();
show_vehicle_ = ui.plannerDisplayVehicle->checkState();
show_pose_history_ = false;//ui.plannerDisplayPoseHistory->checkState();
show_center_line_ = ui.plannerDisplayCenterLine->checkState();
show_complete_center_line_ = ui.plannerDisplayCompleteCenterLine->checkState();
show_trajectories_ = ui.plannerDisplayTrajectories->checkState();
show_best_trajectory_ = ui.plannerDisplayBestTrajectory->checkState();
show_complete_graph_ = ui.plannerDisplayCompleteGraph->checkState();
show_mission_graph_ = ui.plannerDisplayMissionGraph->checkState();
show_complete_mission_graph_ = ui.plannerDisplayCompleteMissionGraph->checkState();
show_matched_edge_ = ui.plannerDisplayMatchedEdge->checkState();
show_yellow_dot_ = false;
show_dynamic_objects_ = ui.plannerDisplayDynamicObstacles->checkState();
show_obstacle_predictions_ = ui.plannerDisplayDynamicObstaclePredictions->checkState();
show_obstacle_destinations_ = ui.plannerDisplayDynamicObstacleDestinations->checkState();
show_road_map_ = ui.plannerDisplayRoadMap->checkState();
show_obstacle_map_ = ui.plannerDisplayStaticObstacleMap->checkState();
show_configuration_space_ = ui.plannerDisplayConfigurationSpace->checkState();
show_radius_ = false;
show_pose_accuracy_ = false;
show_ego_distances_ = ui.plannerDisplayEgoDistances->checkState();
show_veh_distances_ = ui.plannerDisplayVehicleDistances->checkState();
show_intersection_merging_point_ = ui.plannerDisplayIntersectionMergePoint->checkState();
show_lanchange_merging_points_ = false;

can_status_ = new Paw2Can(nh, *this);
applanix_pose_ = new Paw2ApplanixPose(nh, *this);
applanix_rms_ = new Paw2ApplanixRMS(nh, *this);
applanix_dmi_ = new Paw2ApplanixDMI(nh, *this);
applanix_gps_ = new Paw2ApplanixGPS(nh, *this);

velodyne_ = new Paw2Velodyne(*this);
show_velodyne_ = ui.showVelodyne->checkState();
velodyne_->updateData(show_velodyne_);  // if we don't see the data, we don't need to bother reading them
velodyne_->pointSize(uint32_t(ui.velodynePointSize->value()));
velodyne_->displayStyle(Paw2Velodyne::DisplayStyle_t(ui.velodyneDisplayStyle->currentIndex()));
velodyne_->colorMode(Paw2Velodyne::ColorMode_t(ui.velodyneColorMode->currentIndex()));

show_camera_lf_=true;
int32_t camera_lf_id_ = 0;
camera_lf_ = new Paw2Camera(camera_lf_id_, *this);

show_camera_rf_=false;
int32_t camera_rf_id_ = 1;
camera_rf_ = new Paw2Camera(camera_rf_id_, *this);

internal_data_ = new Paw2InternalData(*this);
internal_data_->initializeGL();

nh.getParam("/driving_common/imagery_root", imagery_folder_);

ui.plannerGlView->signalGuiReady(this);
}

Paw2Gui::~Paw2Gui() {
delete can_status_;
delete velodyne_;
delete applanix_pose_;
delete applanix_rms_;
delete applanix_dmi_;
delete applanix_gps_;
}

void Paw2Gui::on_action_Exit_activated() {
//  extern int sendIPCMessage;
//  sendIPCMessage=CBO_SEND_IPC_STOP_COMMAND;
  std::cout << "Exiting..." << std::endl << std::flush;
  awp_.quitPlanner();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for opening an IPC log file (instead of connecting to IPC network)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Open_Log_activated()
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open"), NULL, tr("Log Files (*.tgz *.tar.gz)"));

if (fileName.isEmpty()) {return;}
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for opening a binary camera log file (instead of loading from shared memory)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Open_Camera_Log_activated(void)
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open"), NULL, tr("Camera Log Files (*.blf)"));

if (fileName.isEmpty()) {return;}
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for opening a binary camera log file (instead of loading from shared memory)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Open_Velodyne_Log_activated(void)
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open"), NULL, tr("Velodyne Log Files (*.vlf)"));

if (fileName.isEmpty()) {return;}
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for saving OpenGL display as video
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Save_Video_As_activated(void)
{
QString file_name = QFileDialog::getSaveFileName(this, tr("Save As ..."), NULL, tr("All Files (*.*)"));
video_file_name_=file_name.toStdString();
record_video_=true;
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for opening a binary camera log file (instead of loading from shared memory)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Set_Laser_Map_Folder_activated(void) {
}

//-------------------------------------------------------------------------------------------
/**
 \brief create a laser map
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Create_Laser_Map_activated() {
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for play button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Reset_activated(void)
{
printf("%s\n", __FUNCTION__);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for play button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Rewind_activated(void)
{
printf("%s\n", __FUNCTION__);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for play button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Play_activated() {
  awp_.chsm_planner_->resetEmergencyStopState();
  awp_.sendEStopRun();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for play button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Pause_activated(void)
{
  awp_.sendEStopPause();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for play button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Forward_activated(void)
{
printf("%s\n", __FUNCTION__);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for stop button
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void Paw2Gui::on_action_Stop_activated(void)
{
printf("%s\n", __FUNCTION__);
}


//void Paw2Gui::on_showVehicle_stateChanged(int state)        {   showVehicle_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showFlea2Camera1_stateChanged(int state)   {   showFlea2Camera1_ = (state!=0);   ui.glWindow->requestRedraw();   }
////void Paw2Gui::on_showLDLRS1_stateChanged(int state)       {   showLDLRS1_ = (state!=0);   ui.glWindow->requestRedraw();   }
////void Paw2Gui::on_showLDLRS2_stateChanged(int state)       {   showLDLRS2_ = (state!=0);   ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showRadar_stateChanged(int state)        {   showRadar_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showFlags_stateChanged(int state)        {   showFlags_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showHelp_stateChanged(int state)       {   showHelp_ = (state!=0);   ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showRings_stateChanged(int state)        {   showRings_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showLocalization_stateChanged(int state)       {   showLocalization_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showImagery_stateChanged(int state)       {   showImagery_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showLaserMap_stateChanged(int state)       {   showLaserMap_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showEdges_stateChanged(int state)        {   showEdges_ = (state!=0);  ui.glWindow->requestRedraw();   }
//void Paw2Gui::on_showPoints_stateChanged(int state)       {   showPoints_ = (state!=0);   ui.glWindow->requestRedraw();   }


void Paw2Gui::on_showVelodyne_stateChanged(int state) {
  show_velodyne_ = (state!=0);
  velodyne_->updateData(show_velodyne_);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_velodyneColorMode_currentIndexChanged(int index) {
  velodyne_->colorMode(Paw2Velodyne::ColorMode_t(index));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_velodyneDisplayStyle_currentIndexChanged(int index) {
  velodyne_->displayStyle(Paw2Velodyne::DisplayStyle_t(index));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_velodynePointSize_valueChanged(int value) {
  velodyne_->pointSize(uint32_t(value));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_velodyneShowUpperBlock_stateChanged(int state) {
  velodyne_->showUpperBlock(state !=0);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_velodyneShowLowerBlock_stateChanged(int state) {
  velodyne_->showLowerBlock(state !=0);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camHeight_valueChanged(double) {
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camYawSlider_valueChanged(int value) {
  ui.camYaw->setValue((double) value / 100);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camYaw_valueChanged(double value) {
  ui.camYawSlider->setValue(rint(value * 100));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camPitchSlider_valueChanged(int value) {
  ui.camPitch->setValue((double) value / 100);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camPitch_valueChanged(double value) {
  ui.camPitchSlider->setValue(rint(value * 100));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camRollSlider_valueChanged(int value) {
  ui.camRoll->setValue((double) value / 100);
  requestPlannerGlRedraw();
}

void Paw2Gui::on_camRoll_valueChanged(double value) {
  ui.camRollSlider->setValue(rint(value * 100));
  requestPlannerGlRedraw();
}

void Paw2Gui::on_locMethod_currentIndexChanged(int index) {
  printf("%s: %i\n", __FUNCTION__, index);
  requestPlannerGlRedraw();
}


LocalizationOffsetMode Paw2Gui::localizationOffsetMode() {
  switch(ui.locOffsetMode->currentIndex()) {
    case 0:
      return LOCALIZATION_OFFSET_NONE;

    case 1:
      return LOCALIZATION_OFFSET_CAMERA;

    case 2:
      return LOCALIZATION_OFFSET_LASER;

    default:
      std::cout << "Localization offset mode unknown => using default\n";
      return LOCALIZATION_OFFSET_NONE;
  }
}

LocalizationMethod Paw2Gui::localizationMethod() {
  switch(ui.locMethod->currentIndex()) {
    case 0:
      return LOCALIZATION_METHOD_HISTOGRAM;

    default:
      std::cout << "Localization method unknown => using default\n";
      return LOCALIZATION_METHOD_HISTOGRAM;
  }
}

bool Paw2Gui::event(QEvent* ev) {
  if(ev->type() == app_.statusUpdateEventType()) {
    updateStatus();
    return true;
  }
  return QWidget::event(ev);//return false;
}

void Paw2Gui::updateStatus() {
  updateMissionDisplay();
  updateMissionVelocity();
  updateMissionCurvature();

//  Lock lock(internal_data_->mutex());
  drc::GlobalPose latest_pose;
  double latest_timestamp;
  try {
    awp_.chsm_planner_->latestPose(latest_pose, latest_timestamp);
  }
  catch(...) {
    return;
  }
  pthread_mutex_lock(&awp_.chsm_planner_->center_line_mutex_);
  internal_data_->draw(awp_.chsm_planner_->center_line_, awp_.chsm_planner_->obstacleMapWidth(), awp_.chsm_planner_->obstacleMapHeight(),
                       awp_.chsm_planner_->obstacleMapResolution(), latest_pose, latest_timestamp);
  pthread_mutex_unlock(&awp_.chsm_planner_->center_line_mutex_);
  if (internal_data_->roadMap()) {
    awp_.chsm_planner_->setRoadMap(internal_data_->roadMap()->data());
  }
}

void Paw2Gui::updateMissionVelocity() {
  if(!awp_.chsm_planner_->topology_->isMissionPlanned()) {
    return;
  }
  pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
  vlr::Image<float> velocity(awp_.chsm_planner_->smoothed_mission_points_.size(), 1);
  for(uint32_t i=0; i<awp_.chsm_planner_->smoothed_mission_points_.size(); i++) {
    velocity[i] = awp_.chsm_planner_->smoothed_mission_points_[i].v;
  }
  ui.missionVelocityGlView->updateImage(velocity);
  pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
}

void Paw2Gui::updateMissionCurvature() {
  if(!awp_.chsm_planner_->topology_->isMissionPlanned()) {
    return;
  }
  pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
  vlr::Image<float> curvature(awp_.chsm_planner_->smoothed_mission_points_.size(), 1);
  for(uint32_t i=0; i<awp_.chsm_planner_->smoothed_mission_points_.size(); i++) {
    curvature[i]=awp_.chsm_planner_->smoothed_mission_points_[i].kappa;
  }
  ui.missionCurvatureGlView->updateImage(curvature);
  pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
}

void Paw2Gui::updateMissionDisplay() {
  if(!awp_.chsm_planner_->topology_->isMissionPlanned()) {
    ui.missionEventList->clear();
    return;
  }

  pthread_mutex_lock(&awp_.chsm_planner_->mission_mutex_);
  static bool first_time=true;
  if(first_time) {
    for( Route::RouteEdgeList::const_iterator it = awp_.chsm_planner_->topology_->route.route.begin();
         it != awp_.chsm_planner_->topology_->route.route.end(); it++ ) {
      const RoutePlanner::AnnotatedRouteEdge::AnnotationList& annotation_list = (*it)->annotations();
      if(annotation_list.empty()) {continue;}
      RoutePlanner::AnnotatedRouteEdge::AnnotationList::const_iterator ait = annotation_list.begin(), ait_end = annotation_list.end();
      std::stringstream s;
      for(;ait != ait_end; ait++) {
        if((*ait)->maneuver() == UC_MANEUVER_TRAVEL) {continue;}
        if(!s.str().empty()) {s << " - ";}
//        if(!(*ait)) {
//          std::cout << "Zero pointer in annotation list :-(\n";
//          continue;
//        }
        s << (*ait)->maneuverToString((*ait)->maneuver());
        if((*ait)->maneuver() == UC_MANEUVER_CHECKPOINT) {
          if((*it)->isFromCheckpointEdge()) {
            s << (*it)->edge()->fromVertex()->checkpointId();
          }
          if((*it)->isToCheckpointEdge()) {
            s << (*it)->edge()->toVertex()->checkpointId();
          }
        }
      }
      if(!s.str().empty()) {ui.missionEventList->addItem(s.str().c_str());}
    }
//    awp_.chsm_planner_->topology_->mission_graph_map;
    first_time=false;
  }
  pthread_mutex_unlock(&awp_.chsm_planner_->mission_mutex_);
}

void Paw2Gui::registerMessageHandlers() {
  try {
    can_status_->subscribe();
    applanix_pose_->subscribe();
    applanix_rms_->subscribe();
    applanix_dmi_->subscribe();
    applanix_gps_->subscribe();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }

  try {
    velodyne_->subscribe();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }
  try {
    camera_lf_->subscribe();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }
  try {
    camera_rf_->subscribe();
  }
  catch(vlr::Ex<>& e) {
    std::cout << e.what() << std::endl;
  }
}

} // namespace vlr
