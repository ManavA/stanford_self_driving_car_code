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

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <QtGui/QFileDialog>

#include <global.h>
#include <lltransform.h>

#include <glwidget.h>
#include <rndf_edit_gui.h>
#include <REFindElement.h>

using namespace Eigen;

namespace vlr {

RNDFEditGUI::RNDFEditGUI(std::string& rndf_filename, std::string& imagery_folder, int imageryZoomLevel,
							int imageryGridSizeX, int imageryGridSizeY, QWidget* parent) :
					QMainWindow(parent), rndf_filename_(rndf_filename), imagery_folder_(imagery_folder),
					rn_(NULL), rn_search_(NULL), last_rn_(NULL), last_rn_search_(NULL),
					current_element(RNDF_ELEMENT_SEGMENT),
					last_mouse_x(0), last_mouse_y(0), last_utm_x(0), last_utm_y(0),
					last_move_utm_x(0), last_move_utm_y(0), last_theta(0), gotFromPoint(false), show_imagery_(true)

{
ui.setupUi(this);

centerWindow();

// TODO: Set in designer or move to a place where gui is fully initialized
QList<int> list;
list << ui.consoleSplitter->height() << 0;//ui.paramSplitter->height();
//printf("%i, %i, %i, %i\n", list[0], list[1], ui.consoleSplitter->height(), ui.paramSplitter->height());
ui.consoleSplitter->setSizes(list);
ui.leftBoundary->addItem("unknown", rndf::Lane::UnknownBoundary);
ui.leftBoundary->addItem("none", rndf::Lane::NoBoundary);
ui.leftBoundary->addItem("solid white", rndf::Lane::SolidWhite);
ui.leftBoundary->addItem("broken white", rndf::Lane::BrokenWhite);
ui.leftBoundary->addItem("solid yellow", rndf::Lane::SolidYellow);
ui.leftBoundary->addItem("double yellow", rndf::Lane::DoubleYellow);

ui.rightBoundary->addItem("unknown", rndf::Lane::UnknownBoundary);
ui.rightBoundary->addItem("none", rndf::Lane::NoBoundary);
ui.rightBoundary->addItem("solid white", rndf::Lane::SolidWhite);
ui.rightBoundary->addItem("broken white", rndf::Lane::BrokenWhite);
ui.rightBoundary->addItem("solid yellow", rndf::Lane::SolidYellow);
ui.rightBoundary->addItem("double yellow", rndf::Lane::DoubleYellow);

QActionGroup* ag  = new QActionGroup(ui.toolBar);

ag->addAction(ui.action_Segment);
ag->addAction(ui.action_Lane);
ag->addAction(ui.action_WayPoint);
ag->addAction(ui.action_Exit);
ag->addAction(ui.action_Zone);
ag->addAction(ui.action_Perimeter);
ag->addAction(ui.action_PerimeterPoint);
ag->addAction(ui.action_Spot);
ag->addAction(ui.action_SpotPoint);
ag->addAction(ui.action_Traffic_Light);
ag->addAction(ui.action_Crosswalk);

double ref_lat = 49.02259578774029;
double ref_lon = 8.431273010875101;

latLongToUtm(ref_lat, ref_lon, &rndf_center.x, &rndf_center.y, (char*)&rndf_center.zone);

re_wp_ = new REWayPoint(&ui, rn_);
re_lane_ = new RELane(&ui, rn_, *re_wp_);
re_segment_ = new RESegment(&ui, rn_, *re_lane_);
re_pp_ = new REPerimeterPoint(&ui, rn_);
re_perimeter_ = new REPerimeter(&ui, rn_, *re_pp_);
re_sp_ = new RESpotPoint(&ui, rn_);
re_spot_ = new RESpot(&ui, rn_, *re_sp_);
re_zone_ = new REZone(&ui, rn_, *re_spot_, *re_perimeter_);
re_tl_ = new RETrafficLight(&ui, rn_);
re_crosswalk_ = new RECrosswalk(&ui, rn_);

if(!loadRNDF(rndf_filename_)) {
	throw("not implemented yet: deactivate GUI if RNDF not available");
	}

std::cout <<"reference: "<<rndf_center.x<<", "<<rndf_center.y<<"\n";

selectElements(rndf_center.x, rndf_center.y);
}

RNDFEditGUI::~RNDFEditGUI() {
delete re_wp_;
delete re_lane_;
delete re_segment_;
delete re_pp_;
delete re_perimeter_;
delete re_sp_;
delete re_spot_;
delete re_zone_;
delete re_tl_;
delete re_crosswalk_;
}

void RNDFEditGUI::centerWindow() {
  QDesktopWidget* desktop = QApplication::desktop();
  QSize window_size = size();

  int x = (desktop->width() - window_size.width()) / 2;
  int y = (desktop->height() - window_size.height()) / 2;
  y -= 50;

  move (x, y);
}

void RNDFEditGUI::on_action_Lane_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_LANE;
    selectElements(re_wp_->current());
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}

void RNDFEditGUI::on_action_WayPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_WAYPOINT;
    selectElements(re_wp_->current());
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}

void RNDFEditGUI::on_action_Exit_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_EXIT;
    if (re_wp_->current()) {
      selectElements(re_wp_->current());
      if (re_wp_->current()->parentLane()) {
        ui.paramTab->setCurrentIndex(TAB_STREET);
      }
      else {
        ui.paramTab->setCurrentIndex(TAB_ZONE);
      }
    }
    else {
      selectElements(re_pp_->current());
      ui.paramTab->setCurrentIndex(TAB_ZONE);
    }
  }

  gotFromPoint = false;
}

void RNDFEditGUI::on_action_Segment_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SEGMENT;
    selectElements(re_wp_->current());
    ui.paramTab->setCurrentIndex(TAB_STREET);
  }
}

void RNDFEditGUI::on_action_Traffic_Light_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_TRAFFIC_LIGHT;
    ui.paramTab->setCurrentIndex(TAB_TRAFFIC_LIGHT);
  }
}

void RNDFEditGUI::on_action_Crosswalk_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_CROSSWALK;
    ui.paramTab->setCurrentIndex(TAB_CROSSWALK);
  }
}

void RNDFEditGUI::on_action_Zone_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_ZONE;
    selectElements(re_pp_->current());
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

void RNDFEditGUI::on_action_Perimeter_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_PERIMETER;
    selectElements(re_pp_->current());
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

void RNDFEditGUI::on_action_PerimeterPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_PERIMETERPOINT;
    selectElements(re_pp_->current());
    ui.paramTab->setCurrentIndex(TAB_ZONE);
  }
}

void RNDFEditGUI::on_action_Spot_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SPOT;
    ui.paramTab->setCurrentIndex(TAB_SPOT);
  }
}

void RNDFEditGUI::on_action_SpotPoint_toggled(bool checked) {
  if (checked) {
    current_element = RNDF_ELEMENT_SPOTPOINT;
    ui.paramTab->setCurrentIndex(TAB_SPOT);
  }
}

void RNDFEditGUI::on_actionFindWay_Point_activated() {
  FindDialog dlg;
  dlg.exec();
  QString text = dlg.searchText();

  if (dlg.closeButtonPressed()) {
    return;
  }

  if (text.isEmpty()) {
    return;
  }

  rndf::TWayPointMap::const_iterator wpit = rn_->wayPoints().find(text.toStdString());

  if (wpit == rn_->wayPoints().end()) {
    std::cout << "Couldn't find way point " << text.toStdString() << "\n";
    return;
  }

  CameraPose pose = ui.glWindow->cameraPose();
  ui.glWindow->setInitialCameraPos(pose.pan, pose.tilt, pose.distance, (*wpit).second->utmX()-rndf_center.x, (*wpit).second->utmY()-rndf_center.y, pose.z_offset);
  ui.glWindow->requestRedraw();
}

void RNDFEditGUI::addElement(double utm_x, double utm_y, const std::string& utm_zone) {
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT:
      std::cout << "WayPoint)" << std::endl;
      re_wp_->create(re_lane_->current(), utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_LANE:
      std::cout << "Lane)" << std::endl;
      re_lane_->create(re_segment_->current(), utm_x, utm_y, utm_zone, 0, 10, 3);
      break;

    case RNDF_ELEMENT_SEGMENT:
      std::cout << "Segment)" << std::endl;
      re_segment_->create(utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_PERIMETERPOINT:
      std::cout << "Perimeter Point)" << std::endl;
      re_pp_->create(re_perimeter_->current(), utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_PERIMETER:
      std::cout << "Perimeter)" << std::endl;
      re_perimeter_->create(re_zone_->current(), utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_ZONE:
      std::cout << "Zone)" << std::endl;
      re_zone_->create(utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_SPOTPOINT:
      std::cout << "Spot Point)" << std::endl;
      re_sp_->create(re_spot_->current(), utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_SPOT:
      std::cout << "Spot)" << std::endl;
      re_spot_->create(re_zone_->current(), utm_x, utm_y, utm_zone);
      break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      std::cout << "Traffic Light)" << std::endl;
      re_tl_->create(utm_x, utm_y, utm_zone, 0);
      break;

    case RNDF_ELEMENT_CROSSWALK:
      std::cout << "Crosswalk)" << std::endl;
      re_crosswalk_->create(utm_x, utm_y, utm_zone, 0, 8, 2);
      break;

    default:
      break;
  }
}

void RNDFEditGUI::removeElement() {
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT: {
      std::cout << "(delete WayPoint)" << std::endl;

      if (!re_wp_->current() || !re_wp_->current()->parentLane()) {break;}

      rndf::Lane* parent_lane = re_wp_->current()->parentLane();
      if (parent_lane->numWayPoints() <= 2) {break;}

      int index = re_wp_->current()->index();
      rn_->delWayPoint(re_wp_->current());
      re_wp_->select(parent_lane->wayPoint(index));

      if (!re_wp_->current()) {
        re_wp_->select(parent_lane->wayPoint(index - 1));
      }

      selectElements(re_wp_->current());
      break;
    }
    case RNDF_ELEMENT_LANE: {
      if (!re_lane_->current() || !re_lane_->current()->segment()) {break;}
      std::cout << "(delete Lane)" << std::endl;
      rndf::Segment* parent_segment = re_lane_->current()->segment();
      if (parent_segment->numLanes() <= 1) {break;}
      rn_->delLane(re_lane_->current());
      re_lane_->select(*parent_segment->getLanes().begin());

      re_wp_->select(re_lane_->current()->wayPoint(0));

      selectElements(re_wp_->current());
      break;
    }
    case RNDF_ELEMENT_SEGMENT: {
      if (!re_segment_->current()) {break;}
      std::cout << "(delete Segment)" << std::endl;
      rn_->delSegment(re_segment_->current());
      re_segment_->deselect();
      re_lane_->deselect(); // TODO: what about the others?!?
      re_wp_->deselect();
      break;
    }
    case RNDF_ELEMENT_PERIMETERPOINT: {
      if (!re_pp_->current()) {break;}
      rndf::Perimeter* p = (rndf::Perimeter*) re_pp_->current()->perimeter();
      if (p->numPerimeterPoints() <= 3) {break;}
      std::cout << "(delete Perimeter Point)" << std::endl;
      rn_->delPerimeterPoint(re_pp_->current());
      re_pp_->deselect();
      re_spot_->deselect();
      re_sp_->deselect();
      break;
    }
    case RNDF_ELEMENT_PERIMETER: {
      if (!re_perimeter_->current()) {break;}
      rndf::Zone* z = re_perimeter_->current()->zone();
      if (!z) {break;}
      if (z->numPerimeters() <= 1) {break;}
      std::cout << "(delete Perimeter)" << std::endl;
      rn_->delPerimeter(re_perimeter_->current());
      re_perimeter_->deselect();
      re_pp_->deselect();
      re_spot_->deselect();
      re_sp_->deselect();
      break;
    }

    case RNDF_ELEMENT_ZONE: {
      if (!re_zone_->current()) {break;}
      std::cout << "(delete Zone)" << std::endl;
      rn_->delZone(re_zone_->current());
      re_zone_->deselect();
      re_perimeter_->deselect();
      re_pp_->deselect();
      re_spot_->deselect();
      re_sp_->deselect();
      break;
    }

    case RNDF_ELEMENT_SPOTPOINT: {
      //        if (!re_sp_->current()) {break;}
      //        rndf::spot* s=re_sp_->current()->parentSpot();
      //        if(s->numSpotPoints()<=2) {break;}
      //        std::cout << "(delete Spot Point)"<< std::endl;
      //        rn_->delSpotPoint(re_sp_->current());
      //        re_sp_->deselect();
      break;
    }

    case RNDF_ELEMENT_SPOT: {
      if (!re_spot_->current()) {break;}
      rndf::Zone* z = re_spot_->current()->zone();
      if (!z) {break;}
      std::cout << "(delete Spot)" << std::endl;
      rn_->delSpot(re_spot_->current());
      re_spot_->deselect();
      re_sp_->deselect();
      break;
    }

    case RNDF_ELEMENT_TRAFFIC_LIGHT: {
      if (!re_tl_->current()) {break;}
      std::cout << "(delete Traffic Light)" << std::endl;
      rn_->delTrafficLight(re_tl_->current());
      re_tl_->deselect();
      break;
    }

    case RNDF_ELEMENT_CROSSWALK: {
      if (!re_crosswalk_->current()) {break;}
      std::cout << "(delete Crosswalk)" << std::endl;
      rn_->delCrosswalk(re_crosswalk_->current());
      re_crosswalk_->deselect();
      break;
    }

    default: {
      break;
    }
  }
}

void RNDFEditGUI::copyElement() {
  switch(current_element) {
    case RNDF_ELEMENT_WAYPOINT:
      re_wp_->copy(re_wp_->current(), re_lane_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_LANE:
      re_lane_->copy(re_lane_->current(), re_segment_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_SEGMENT:
      re_segment_->copy(re_segment_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_PERIMETERPOINT:
      re_pp_->copy(re_pp_->current(), re_perimeter_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_PERIMETER:
      re_perimeter_->copy(re_perimeter_->current(), re_zone_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_ZONE:
      re_zone_->copy(re_zone_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_SPOTPOINT:
      re_sp_->copy(re_sp_->current(), re_spot_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_SPOT:
      re_spot_->copy(re_spot_->current(), re_zone_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      re_tl_->copy(re_tl_->current(), 10, 10);
      break;

    case RNDF_ELEMENT_CROSSWALK:
      re_crosswalk_->copy(re_crosswalk_->current(), 10, 10);
      break;

    default:
      break;
    }
}

void RNDFEditGUI::moveElement(double utm_x, double utm_y, const std::string& utm_zone) {
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT:
      re_wp_->move(re_wp_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_LANE:
      re_lane_->move(re_lane_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_SEGMENT:
      re_segment_->move(re_segment_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_ZONE:
      re_zone_->move(re_zone_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_PERIMETER:
      re_perimeter_->move(re_perimeter_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_PERIMETERPOINT:
      re_pp_->move(re_pp_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_SPOT:
      re_spot_->move(re_spot_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_SPOTPOINT:
      re_sp_->move(re_sp_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      re_tl_->move(re_tl_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    case RNDF_ELEMENT_CROSSWALK:
      re_crosswalk_->move(re_crosswalk_->current(), utm_x - last_move_utm_x, utm_y - last_move_utm_y);
      break;

    default:
      return;
  }
}

bool RNDFEditGUI::currentElementPosition(double& utm_x, double& utm_y, std::string& utm_zone) {

  double lat, lon;
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT:
      if(!re_wp_->current()) {return false;}
      utm_x = re_wp_->current()->utmX();
      utm_y = re_wp_->current()->utmY();
      utm_zone = re_wp_->current()->utmZone();
      return true;

    case RNDF_ELEMENT_LANE:
      if(!re_lane_->current()) {return false;}
      re_lane_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;

    case RNDF_ELEMENT_SEGMENT:
      if(!re_segment_->current()) {return false;}
      re_segment_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;

    case RNDF_ELEMENT_ZONE:
      if(!re_zone_->current()) {return false;}
      re_zone_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;

    case RNDF_ELEMENT_PERIMETER:
      if(!re_perimeter_->current()) {return false;}
      re_perimeter_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;

    case RNDF_ELEMENT_PERIMETERPOINT:
      if(!re_pp_->current()) {return false;}
      utm_x = re_pp_->current()->utmX();
      utm_y = re_pp_->current()->utmY();
      utm_zone = re_pp_->current()->utmZone();
      return true;

    case RNDF_ELEMENT_SPOT:
      if(!re_spot_->current()) {return false;}
      re_spot_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;

    case RNDF_ELEMENT_SPOTPOINT:
      if(!re_sp_->current()) {return false;}
      utm_x = re_sp_->current()->utmX();
      utm_y = re_sp_->current()->utmY();
      utm_zone = re_sp_->current()->utmZone();
      return true;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      if(!re_tl_->current()) {return false;}
      utm_x = re_tl_->current()->utmX();
      utm_y = re_tl_->current()->utmY();
      utm_zone = re_tl_->current()->utmZone();
      return true;

    case RNDF_ELEMENT_CROSSWALK:
      if(!re_crosswalk_->current()) {return false;}
      re_crosswalk_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
      return true;
  }

  return false;
}

void RNDFEditGUI::rotateElement(double utm_x, double utm_y, const std::string& utm_zone) {
  double lat, lon, theta = 0.;
  double center_x, center_y;
  std::string elem_utm_zone;

  switch (current_element) {
    case RNDF_ELEMENT_LANE:
      if (!re_lane_->current()) {return;}
      re_lane_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_lane_->rotate(re_lane_->current(), center_x, center_y, theta - last_theta);
      break;

    case RNDF_ELEMENT_SEGMENT:
      if (!re_segment_->current()) {return;}
      re_segment_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_segment_->rotate(re_segment_->current(), center_x, center_y, theta - last_theta);
      break;

    case RNDF_ELEMENT_ZONE:
      if (!re_zone_->current()) {return;}
      re_zone_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_zone_->rotate(re_zone_->current(), center_x, center_y, theta - last_theta);
      break;

    case RNDF_ELEMENT_PERIMETER:
      if (!re_perimeter_->current()) {return;}
      re_perimeter_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_perimeter_->rotate(re_perimeter_->current(), center_x, center_y, theta - last_theta);
      break;

    case RNDF_ELEMENT_SPOT:
      if (!re_spot_->current()) {return;}
      re_spot_->current()->centerLatLon(lat, lon);
      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_spot_->rotate(re_spot_->current(), center_x, center_y, theta - last_theta);
      break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      if (!re_tl_->current()) {return;}
      latLongToUtm(re_tl_->current()->lat(), re_tl_->current()->lon(), &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_tl_->rotate(re_tl_->current(), theta - last_theta);
      break;

    case RNDF_ELEMENT_CROSSWALK:
      if (!re_crosswalk_->current()) {return;}
      center_x = 0.5*(re_crosswalk_->current()->utmX1() + re_crosswalk_->current()->utmX2());
      center_y = 0.5*(re_crosswalk_->current()->utmY1() + re_crosswalk_->current()->utmY2());
//      latLongToUtm(lat, lon, &center_x, &center_y, elem_utm_zone);
      theta = atan2(utm_y - center_y, utm_x - center_x);
      re_crosswalk_->rotate(re_crosswalk_->current(), center_x, center_y, theta - last_theta);
      break;

    default:
      return;
  }

  last_theta = theta;
}

void RNDFEditGUI::selectElements(rndf::WayPoint* waypoint) {

  if (!waypoint) {return;}

  deselectAllElements();

  // check if way points belongs to Lane or spot
  if (waypoint->parentLane()) {
    re_wp_->select(waypoint);
    re_lane_->select(waypoint->parentLane());
    re_segment_->select(re_lane_->current()->segment());
    updateSmoothedLaneStack();
  }
  else {
    re_sp_->select(waypoint);
    re_spot_->select(re_sp_->current()->parentSpot());
    if (re_spot_->current()) {
      re_zone_->select(re_spot_->current()->zone());
    }
  }

  updateGUI();
}

void RNDFEditGUI::selectElements(rndf::PerimeterPoint* pp) {
  if (!pp) {
    std::cout << "Invalid Perimeter point.\n";
    return;
  }

  deselectAllElements();

  re_pp_->select(pp);

  re_perimeter_->select((rndf::Perimeter*) re_pp_->current()->perimeter());
  if (re_perimeter_->current()) {
    re_zone_->select(re_perimeter_->current()->zone());
  }

  updateGUI();
}

void RNDFEditGUI::selectElements(rndf::TrafficLight* tl) {

  if (!tl) {
    std::cout << "Invalid traffic light.\n";
    return;
  }

  deselectAllElements();

  re_tl_->select(tl);
  updateGUI();
}

void RNDFEditGUI::selectElements(rndf::Crosswalk* cw) {

  if (!cw) {
    std::cout << "Invalid crosswalk.\n";
    return;
  }

  deselectAllElements();

  re_crosswalk_->select(cw);
  updateGUI();
}


void RNDFEditGUI::deselectAllElements() {
  re_wp_->deselect();
  re_lane_->deselect();
  re_segment_->deselect();
  re_sp_->deselect();
  re_spot_->deselect();
  re_pp_->deselect();
  re_perimeter_->deselect();
  re_zone_->deselect();
  re_tl_->deselect();
  re_crosswalk_->deselect();
}

void RNDFEditGUI::selectElements(double utm_x, double utm_y) {
  switch (current_element) {
    case RNDF_ELEMENT_WAYPOINT:
    case RNDF_ELEMENT_LANE:
    case RNDF_ELEMENT_SEGMENT:
    case RNDF_ELEMENT_SPOTPOINT:
    case RNDF_ELEMENT_SPOT:
      selectElements(rn_search_->closest_waypoint(utm_x, utm_y));
      break;

    case RNDF_ELEMENT_PERIMETERPOINT:
    case RNDF_ELEMENT_PERIMETER:
    case RNDF_ELEMENT_ZONE:
      selectElements(rn_search_->closest_perimeterpoint(utm_x, utm_y));
      break;

    case RNDF_ELEMENT_EXIT: {
      rndf::PerimeterPoint* p = NULL;
      rndf::WayPoint* w = NULL;

      p = rn_search_->closest_perimeterpoint(utm_x, utm_y);
      w = rn_search_->closest_waypoint(utm_x, utm_y);

      if (!p && !w) {
        return;
      }

      if (!p) {
        selectElements(w);
      }
      else if (!w) {
        selectElements(p);
      }
      else {
        printf("ERROR: Perimeter and waypoint selected at the same time (should never happen)\n");
        double dx1, dy1, dx2, dy2;

        dx1 = w->utmX() - utm_x;
        dy1 = w->utmY() - utm_y;
        dx2 = p->utmX() - utm_x;
        dy2 = p->utmY() - utm_y;

        (dx1 * dx1 + dy1 * dy1 <= dx2 * dx2 + dy2 * dy2 ? selectElements(w) : selectElements(p));
      }
    }
    break;

    case RNDF_ELEMENT_TRAFFIC_LIGHT:
      selectElements(rn_search_->closestTrafficLight(utm_x, utm_y));
      break;

    case RNDF_ELEMENT_CROSSWALK:
      selectElements(rn_search_->closestCrosswalk(utm_x, utm_y));
      break;
  }
}

void RNDFEditGUI::updateSmoothedLaneStack() {

    if(!re_lane_->current()) {return;}
    smoothed_lane_stack_.clear();

//    rndf::TExitMap::const_iterator entry_it=re_lane_->current()->entries().begin(), entry_it_end=re_lane_->current()->entries().end();
//    rndf::TExitMap::const_iterator exit_it=re_lane_->current()->exits().begin(), exit_it_end=re_lane_->current()->exits().end();
//    CurvePoint cp;
//    memset(&cp, 0, sizeof(cp));
//
//    double sample_dist = 7;
//    for(; entry_it != entry_it_end; entry_it++) {
//        for(; exit_it != exit_it_end; exit_it++) {
//
//            std::vector<CurvePoint> raw_lane;
//            sampleRawLaneLine(*(*entry_it).second, *(*exit_it).second, raw_lane);
//            std::vector<CurvePoint> extended_lane;
//            std::vector<CurvePoint>::const_iterator original_lane_begin, original_lane_end;
//            extendRawLaneLine(raw_lane, *(*entry_it).second, *(*exit_it).second, sample_dist, extended_lane, original_lane_begin, original_lane_end);
//            std::vector<bool> ignore_points;
//            std::vector<CurvePoint> sampled_lane;
//            ignore_points.resize(raw_lane.size());
//            smoother_.sampleLinearEquidist(raw_lane, ignore_points, sample_dist, sampled_lane);
//
//            if(sampled_lane.size() < 2) {
//              throw VLRException("Lane must contain at least 2 waypoints; resampling failed.");
//            }
//
//            std::vector<CurvePoint> bezier_lane;
//            smoother_.cubicBezierFromClothoid(sampled_lane, bezier_lane);
//
//            std::vector<CurvePoint> temp_points;
//            smoother_.sampleCubicBezierEquidist(bezier_lane, 1, temp_points);
//
//            std::vector<CurvePoint> smoothed_lane;
//            double smoothing_range = 8;
//            smoother_.clothoideSpline(temp_points, temp_points[0].theta, temp_points[0].kappa, temp_points[0].s, smoothing_range, smoothed_lane);
//
//            if(!clothoid_smoothing_) {smoothed_lane = temp_points;}
//            smoothed_lane_stack_.push_back(smoothed_lane);
//        }
//    }
//
//    for(size_t i=0; i<curvature_plots_.size(); i++) {
//      curvature_plots_[i]->detach();
//      delete curvature_plots_[i];
//    }
//
//    for(size_t i=0; i<plot_data_.size(); i++) {
//      delete plot_data_[i];
//    }
//
//    curvature_plots_.clear();
//    plot_data_.clear();
////    std::vector<std::vector<CurvePoint> >::const_iterator lsit = smoothed_lane_stack_.begin(), lsit_end = smoothed_lane_stack_.end();
////    for(; lsit != lsit_end; lsit++) {
//    std::stringstream title_s;
//    for(size_t i=0; i<smoothed_lane_stack_.size(); i++) {
//      plot_data_.push_back(new PlotData(smoothed_lane_stack_[i]));
//      title_s.str("");
//      title_s << "Curvature " << i;
//      curvature_plots_.push_back(new QwtPlotCurve(title_s.str().c_str()));
//      curvature_plots_[i]->setData(*plot_data_[i]);
//      curvature_plots_[i]->attach(ui.curvaturePlot);
//    }
//
////    for(size_t i=0; i<smoothed_lane_stack_.size(); i++) {
//////      if(i<plot_data_.size()) {}
////    }
////
////    std::vector<double> s_plot, kappa_plot;
////    std::vector<CurvePoint>::const_iterator cpit = smoothed_lane.begin(), cpit_end = smoothed_lane.end();
////    for(; cpit != cpit_end; cpit++) {
////      s_plot.push_back(cpit->s);
////      kappa_plot.push_back(cpit->kappa);
////    }
////    QwtPlotCurve* curve = new QwtPlotCurve("Curvature");
////    curve->setData(&s_plot[0], &kappa_plot[0], smoothed_lane.size());
////    curve->attach(ui.curvaturePlot);
//    ui.curvaturePlot->replot();

}

void RNDFEditGUI::sampleRawLaneLine(const rndf::Exit& entry, const rndf::Exit& exit, std::vector<CurvePoint>& raw_line) {
    CurvePoint cp;
    for(uint32_t i=0; i<re_lane_->current()->numWayPoints(); i++) {
        cp.x = re_lane_->current()->wayPoints()[i]->x();
        cp.y = re_lane_->current()->wayPoints()[i]->y();
        raw_line.push_back(cp);
    }
}

void RNDFEditGUI::extendRawLaneLine(const std::vector<CurvePoint>& raw_line, rndf::Exit& entry, rndf::Exit& exit,
                                    int32_t num_samples, std::vector<CurvePoint>& extended_line,
                                    std::vector<CurvePoint>::const_iterator& original_start,
                                    std::vector<CurvePoint>::const_iterator& original_end) {
  rndf::WayPoint* wp  = entry.getExitFromLane();
  rndf::PerimeterPoint* pp = entry.getExitFromPerimeter();

  if (wp) {
    CurvePoint cp;
    int32_t c=0;
    while(wp && c < num_samples) {
      cp.x = wp->x();
      cp.y = wp->y();
      extended_line.push_back(cp);
      rndf::Lane* l = wp->parentLane();
      if(l) {
        uint32_t index = l->wayPointIndex(wp);
        wp = NULL;
        if(index>0) {
          index--;
          wp = l->wayPoint(index);
        }
        else {
//          const rndf::TExitMap entries = l->entries();
        }
      }
    }
  }
  else if (pp) {
    CurvePoint cp;
    cp.x = pp->x();
    cp.y = pp->y();
    extended_line.push_back(cp);
  }

  if(extended_line.empty()) {
    original_start = extended_line.begin();
  }
  else {
    original_start = extended_line.begin() + extended_line.size()-1;
  }
}

void RNDFEditGUI::smoothLane(rndf::Exit& entry, rndf::Exit& exit) {
//smoother.sampleLinearEquidist(mission_points_, getParameters().sample_dist_m, &sampled_mission_points_);
//
//if(sampled_mission_points_.size() < 2) {
//throw(Exception("Mission must contain at least 2 waypoints; resampling failed."));
//}
//
//last_start_idx_ = 0;
//last_start_it_ = sampled_mission_points_.begin();
//
//double dx = sampled_mission_points_[1].x - sampled_mission_points_[0].x;
//double dy = sampled_mission_points_[1].y - sampled_mission_points_[0].y;
//double theta0 = atan2(dy, dx);
//double kappa0=0;
//double s0=0;
//pthread_mutex_unlock(&mission_mutex_);
//
//estimateKappa(sampled_mission_points_[0], sampled_mission_points_[1], sampled_mission_points_[2], kappa0);
////  printf("start kappa: %f\n", kappa0);
//smoother.clothoideSpline(sampled_mission_points_, theta0, kappa0, s0, getParameters().smoothing_range, &smoothed_mission_points_);

}

void RNDFEditGUI::optimizeVirtualPoints() {
  rndf::TExitMap::const_iterator entry_it=re_lane_->current()->entries().begin(), entry_it_end=re_lane_->current()->entries().end();
  rndf::TExitMap::const_iterator exit_it=re_lane_->current()->exits().begin(), exit_it_end=re_lane_->current()->exits().end();
  CurvePoint cp;
  memset(&cp, 0, sizeof(cp));

//  double sample_dist = 7;
//  for(; entry_it != entry_it_end; entry_it++) {
//      for(; exit_it != exit_it_end; exit_it++) {

  std::vector<CurvePoint> raw_lane, temp_points;
  sampleRawLaneLine(*(*entry_it).second, *(*exit_it).second, raw_lane);
  temp_points = raw_lane;

    // forward optimization
  std::vector<CurvePoint>::iterator cpit = temp_points.begin(), cpit_end =temp_points.end();
  for(size_t i=0; cpit != cpit_end; cpit++, i++) {
    if(re_lane_->current()->wayPoints()[i]->isVirtual()) {
      optimizeVirtualPoint(temp_points, i);
    }
  }

    // backward optimization
  std::vector<CurvePoint>::reverse_iterator rcpit = temp_points.rbegin(), rcpit_end =temp_points.rend();
  for(size_t i=re_lane_->current()->numWayPoints()-1; rcpit != rcpit_end; rcpit++, i--) {
    if(re_lane_->current()->wayPoints()[i]->isVirtual()) {
      optimizeVirtualPoint(temp_points, i);
    }
  }

    // copy optimized curve points back to current lane
  cpit = temp_points.begin();
  for(size_t i=0; cpit != cpit_end; cpit++, i++) {
    rndf::WayPoint* wp = re_lane_->current()->wayPoints()[i];
    if(wp->isVirtual()) {
      wp->setUtm((*cpit).x, (*cpit).y, wp->utmZone());
    }
  }
}

void RNDFEditGUI::optimizeVirtualPoint(std::vector<CurvePoint>& lane_points, size_t idx) {
  size_t idx0 = (idx == 0 ? 0 : idx-1);
  size_t idx1 = (idx == lane_points.size()-1 ? lane_points.size()-1 : idx+1);
  ParametrizedLine<double, 2> tangent(Vector2d(lane_points[idx0].x, lane_points[idx0].y),
                   Vector2d(lane_points[idx1].x, lane_points[idx1].y)-Vector2d(lane_points[idx0].x, lane_points[idx0].y));

}

  // raw_center_line should be sampled and extended
double RNDFEditGUI::squaredCurvatureSum(std::vector<CurvePoint>& raw_center_line) {

//  std::vector<CurvePoint> sampled_lane=raw_center_line;
//
//  std::vector<CurvePoint> bezier_lane;
//  smoother_.cubicBezierFromClothoid(sampled_lane, bezier_lane);
//
//  std::vector<CurvePoint> temp_points;
//  smoother_.sampleCubicBezierEquidist(bezier_lane, 1, temp_points);
  double sum=0;
//  for(std::vector<CurvePoint>::const_iterator cpit=temp_points.begin(); cpit != temp_points.end(); cpit++) {
//    sum+= (*cpit).kappa*(*cpit).kappa;
//  }

  return sum;
}

void RNDFEditGUI::addExit() {
  rndf::WayPoint *w1 = NULL, *w2 = NULL;
  rndf::PerimeterPoint* p1 = NULL, *p2 = NULL;
  double utm_x, utm_y;
  if (re_wp_->current()) {
    w1 = re_wp_->current();
  }
  else if (re_pp_->current()) {
    p1 = re_pp_->current();
  }
  else {
    return;
  }

  utm_x = last_utm_x;
  utm_y = last_utm_y;

  w2 = rn_search_->closest_waypoint(utm_x, utm_y);
  p2 = rn_search_->closest_perimeterpoint(utm_x, utm_y);

  if (!w2 && !p2) {
    return;
  }

  // get closest point (Perimeter point or way point)
  // and set pointer to other one to zero
  if (w2 && p2) {
    double dx1, dy1, dx2, dy2;

    dx1 = w2->utmX() - utm_x;
    dy1 = w2->utmY() - utm_y;
    dx2 = p2->utmX() - utm_x;
    dy2 = p2->utmY() - utm_y;

    if (dx1 * dx1 + dy1 * dy1 <= dx2 * dx2 + dy2 * dy2) {
      p2 = NULL;
    }
    else {
      w2 = NULL;
    }
  }

  if (w1 && w2) // exit from way point to way point
  {
    if (w1 != w2 && w1->parentLane() != w2->parentLane()) {
      rn_->addExit(w1, w2);
    }
  }
  else if (w1 && p2) // exit from way point to Perimeter point
  {
    rn_->addExit(w1, p2);
  }
  else if (p1 && w2) // exit from Perimeter point to way point
  {
    rn_->addExit(p1, w2);
  }
  else if (p1 && p2) // exit from Perimeter point to Perimeter point
  {
    if (p1 != p2 && p1->perimeter() != p2->perimeter()) {
      rn_->addExit(p1, p2);
    }
  }
}

void RNDFEditGUI::on_action_Open_Trajectory_activated()
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open Trajectory Dump"), NULL, tr("Trajectory Files (*.txt)"));

if (fileName.isEmpty()) {return;}

std::ifstream trajectory_file;

trajectory_file.open(fileName.toAscii().constData());

if ( !trajectory_file.good() )
	{
	std::cout << "something wrong with trajectory file \""<< fileName.toAscii().constData() << "\"\n";
	trajectory.clear();
	return;
	}

bool first = true;

trajectory.clear();

while (trajectory_file.good() )
	{
	double x, y;
	trajectory_file >> x >> y;
	
	if ( !first )
		{
		// skip points that differ hardly from the preceding ones
		double dx = trajectory.back().x - x;
		double dy = trajectory.back().y - y;

		if (dx*dx + dy*dy < 0.1*0.1)
			continue;
		}
	first = false;

	trajectory.push_back(xy(x, y ) );
	}

ui.glWindow->requestRedraw();
}

double RNDFEditGUI::calcPointDeviation(xy& p, xy& l, xy& r)
{
double rx = r.x - l.x;
double ry = r.y - l.y;
double len = sqrt(rx*rx+ry*ry);

if(len != 0.0)
	{
	rx/=len; ry/=len;
	double nx = ry;
	double ny = -rx;
	return fabs(nx * (p.x - l.x) + ny * (p.y - l.y));
	}
else
	{
	std::cout<< "unexpected double point in trajectory.\n";
	return 0;
	//throw("unexpected double point in trajectory.");
	}
}

void RNDFEditGUI::on_action_ReverseTrajectory_activated(void)
{
std::cout << __FUNCTION__ << "\n";
std::vector<xy> tvec;

std::vector<xy>::const_reverse_iterator rtit, rtit_end;

for(rtit=trajectory.rbegin(), rtit_end=trajectory.rend(); rtit!=rtit_end; ++rtit)
	{
	tvec.push_back(*rtit);
	}

tvec.swap(trajectory);
}

void RNDFEditGUI::on_action_Trajectory2Lane_activated(void)
{
std::cout << __FUNCTION__ << "\n";
//
//if(trajectory.size()<2)
//	{
//	std::cout << "trajectory does not contain enough points (minimum are 2)\n";
//	return;
//	}
//
//	// add segment
//rndf::Segment* s = rn_->addSegment();
//
//if(!s)
//	{
//	std::cout << "failed to add segment to road network\n";
//	return;
//	}
//
//	// add Lane
//rndf::Lane* l = rn_->addLane(s);
//
//if(!l)
//	{
//	std::cout << "failed to add Lane to road network\n";
//	rn_->delSegment(s);
//	return;
//	}
//
//l->setLaneWidth(ui.laneWidth->value());
//
//char* utm_zone = rndf_center.zone;
//
//std::set<distElement*, distElemCompare> des;
//
//distElement* first = new distElement;
//distElement* last = new distElement;
//
//first->index=0;
//last->index=trajectory.size()-1;
//
//des.insert(first);
//des.insert(last);
//
//if(trajectory.size()>2)
//	{
//	distElement* left=0, *de=0, *right=0;
//
//	left = new distElement;
//	de = new distElement;
//
//	left->left=0;
//	left->distance=0;
//	left->right=de;
//	left->index=0;
//	de->left=left;
//
//	for (unsigned int index = 1; index < trajectory.size()-1; ++index)
//		{
//		right = new distElement;
//		right->left=de;
//		right->index=index+1;
//		de->right=right;
//		de->index=index;
//		if(left && right)
//			{de->distance=calcPointDeviation(trajectory[de->index], trajectory[left->index], trajectory[right->index]);}
//		else
//			{de->distance=0;}
//
//		des.insert(de);
//
//		left=de;
//		de=right;
//		}
//	}
//
//
//std::cout << "original trajectory points: " << des.size() << "\n";
//
//double distThresh=0.007;
//
//std::set<distElement*, distElemCompare>::const_iterator deit, deit_end;
//
//for(deit=des.begin(), deit_end=des.end(); deit != deit_end; ++deit)
//	{
//	std::cout << "distance: "<<(*deit)->distance<<"\n";
//	}
//
//
//const distElement* de=*(des.begin());
//
//while(de->distance < distThresh)
//	{
//	des.erase(des.begin());
//
//	if(de->left)
//		{
//		distElement* lde=de->left;	// to make code more readable...
//		des.erase(lde);
//		lde->right=de->right;
//		if(de->right) {de->right->left=lde;}
//
//		if(lde->left && lde->right)
//			{lde->distance=calcPointDeviation(trajectory[lde->index], trajectory[lde->left->index], trajectory[lde->right->index]);}
//		else
//			{lde->distance=DBL_MAX;}
//
//		des.insert(lde);
//		}
//
//	if(de->right)
//		{
//		distElement* rde=de->right;	// to make code more readable...
//		des.erase(rde);
//		rde->left=de->left;
//
//		if(de->left) {de->left->right=rde;}
//
//		if(rde->left && rde->right)
//			{rde->distance=calcPointDeviation(trajectory[rde->index], trajectory[rde->left->index], trajectory[rde->right->index]);}
//		else
//			{rde->distance=DBL_MAX;}
//
//		des.insert(rde);
//		}
//
//	delete de;
//
//	if (des.begin()==des.end()) {break;}
//
////	std::cout << "distance: "<<de->distance<<"\n";
//	de=*(des.begin());
//	}
//
//std::set<distElement*, distElemCompare>::const_iterator seit, seit_end;
//std::set<distElement*, distElemIdxCompare> idxset;
//
//	// reindex remaining distElements by index (original point order)
//for(seit=des.begin(), seit_end=des.end(); seit != seit_end; ++seit)
//	{
//	idxset.insert(*seit);
//	}
//
//std::set<distElement*, distElemCompare>::const_iterator idxsit, idxsit_end;
//std::vector<xy> ergtrj;
//for(idxsit=idxset.begin(), idxsit_end=idxset.end(); idxsit != idxsit_end; ++idxsit)
//	{
////	std::cout << "inserting point "<< (*idxsit)->index<<"\n";
//	ergtrj.push_back(trajectory[(*idxsit)->index]);
//	delete (*idxsit);
//	}
//
//std::cout << "new trajectory points: " << ergtrj.size() << "\n";
////trajectory.clear();
//ergtrj.swap(trajectory);
//
//double lat, lon;
//
//for (unsigned int index = 0; index < trajectory.size(); ++index)
//	{
//
//	utmToLatLong(trajectory[index].x, trajectory[index].y, utm_zone, &lat, &lon);
//	rn_->addWayPoint(l, lat, lon, index);
//	}
//
//ui.glWindow->requestRedraw();
}

bool RNDFEditGUI::loadRNDF(std::string fileName)
{
std::cout << "filename: " << fileName <<"\n";

if (rn_) {delete rn_;}
if (rn_search_) {delete rn_search_;}

rn_ = new rndf::RoadNetwork("A Road Network", false);
std::cout << "Loading RNDF"<< std::endl;

if (rn_->loadRNDF(fileName.c_str()))
	{std::cout << "Road network loaded successfully\n";}
else
	{std::cout << "Error loading road network : "<< rn_->status() << std::endl;}

rn_search_ = new rndf::RoadNetworkSearch(rn_);

coordinate_latlon_t rndf_center_latlon = rn_->center();
latLongToUtm(rndf_center_latlon.lat, rndf_center_latlon.lon, &rndf_center.x, &rndf_center.y, rndf_center.zone);

//std::cout <<"refll: "<<rndf_center_latlon.lat<<", "<<rndf_center_latlon.lon<<"\n";
//std::cout <<"reference: "<<rndf_center.x<<", "<<rndf_center.y<<"\n";
//imagery_lat = rndf_center_latlon.lat;
//imagery_lon = rndf_center_latlon.lon;

re_segment_->deselect();
re_lane_->deselect();
re_wp_->deselect();

//rndf::TWayPointMap::const_iterator wpit=rn_->WayPoints().begin();
//if(wpit != rn_->WayPoints().end()) {
//  rndf_center.x = (*wpit).second->x();
//  rndf_center.y = (*wpit).second->y();
//}
re_wp_->setRoadNetwork(rn_);
re_lane_->setRoadNetwork(rn_);
re_segment_->setRoadNetwork(rn_);
re_pp_->setRoadNetwork(rn_);
re_perimeter_->setRoadNetwork(rn_);
re_sp_->setRoadNetwork(rn_);
re_spot_->setRoadNetwork(rn_);
re_zone_->setRoadNetwork(rn_);
re_tl_->setRoadNetwork(rn_);
re_crosswalk_->setRoadNetwork(rn_);

ui.glWindow->requestRedraw();

return true;
}

void RNDFEditGUI::updateGUI() {
  re_wp_->updateGUI();
  re_lane_->updateGUI();
  updateSmoothedLaneStack();
  re_segment_->updateGUI();
  re_sp_->updateGUI();
  re_spot_->updateGUI();
  re_pp_->updateGUI();
  re_perimeter_->updateGUI();
  re_zone_->updateGUI();
  re_tl_->updateGUI();
  re_crosswalk_->updateGUI();
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for loading a road network definition file (RNDF)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Open_RNDF_activated(void)
{
std::cout << __FUNCTION__ << "\n";
QString fileName = QFileDialog::getOpenFileName(this, tr("Open RNDF"), NULL, tr("RNDF Files (*.txt *.rndf)"));

if (fileName.isEmpty()) {return;}

loadRNDF(fileName.toStdString());

rndf_filename_=fileName.toStdString();
}

void RNDFEditGUI::on_action_Undo_activated() {

  if(!last_rn_ || !last_rn_search_) {
    std::cout << "Undo buffer is empty\n";
    return;
  }

  rndf::RoadNetwork* tmp_rn = rn_;
  rndf::RoadNetworkSearch* tmp_rn_search = rn_search_;

  rn_ = last_rn_;
  rn_search_ = last_rn_search_;

  last_rn_ = tmp_rn;
  last_rn_search_ = tmp_rn_search;

  std::cout << "Toggled to undo buffer"<< std::endl;
  deselectAllElements();
  ui.glWindow->requestRedraw();
}


//-------------------------------------------------------------------------------------------
/**
 \brief callback for saving a road network definition file (RNDF)
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Save_RNDF_activated()
{
printf("%s\n", __FUNCTION__);
rn_->saveRNDF(rndf_filename_);
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for saving a road network definition file (RNDF) with a new name
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_action_Save_RNDF_As_activated()
{
QString fileName = QFileDialog::getSaveFileName(this, tr("Save RNDF As ..."), NULL, tr("RNDF Files (*.txt *.rndf)"));

rn_->saveRNDF(fileName.toAscii().constData());
}

//-------------------------------------------------------------------------------------------
/**
 \brief callback for toggling imagery
 \param -
 \return -
 \ingroup
 *///-----------------------------------------------------------------------------------------
void RNDFEditGUI::on_showImagery_stateChanged(int state) {
show_imagery_ = state != 0;
}

RNDFEditGUI::PlotData::PlotData(std::vector<CurvePoint>& cps) : cps_(cps) {update();}
RNDFEditGUI::PlotData::~PlotData() {

}

void RNDFEditGUI::PlotData::update() {
    std::vector<CurvePoint>::const_iterator cpit = cps_.begin(), cpit_end = cps_.end();
    double min_x = DBL_MAX, max_x = -DBL_MAX, min_y = DBL_MAX, max_y = -DBL_MAX;
    for (; cpit != cpit_end; cpit++) {
      if(cpit->s < min_x) {min_x = cpit->s;}
      else if(cpit->s > max_x) {max_x = cpit->s;}
      if(cpit->kappa < min_y) {min_y = cpit->kappa;}
      else if(cpit->kappa > max_y) {max_y = cpit->kappa;}
    }
    bounding_box_.setRect(min_x, min_y, max_x-min_x, max_y-min_y);
  }

RNDFEditGUI::PlotData* RNDFEditGUI::PlotData::copy() const {return new PlotData(cps_);}
size_t RNDFEditGUI::PlotData::size() const {return cps_.size();}
double RNDFEditGUI::PlotData::x(size_t i) const {return cps_[i].s;}
double RNDFEditGUI::PlotData::y(size_t i) const {return cps_[i].kappa;}
QwtDoubleRect RNDFEditGUI::PlotData::boundingRect() const {return bounding_box_;}

} // namespace vlr
