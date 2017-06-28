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


#include <QtGui/QtGui>
#include <rndf_edit_gui.h>
#include <RELane.h>

namespace vlr {

RELane::RELane(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn, REWayPoint& re_wp) : REElement<rndf::Lane>(ui, rn), re_wp_(re_wp) {

  connect(ui_->laneWidth, SIGNAL( valueChanged(double) ), this, SLOT(on_laneWidth_valueChanged(double)));
  connect(ui_->leftBoundary, SIGNAL( currentIndexChanged(int) ), this, SLOT(on_leftBoundary_currentIndexChanged(int)));
  connect(ui_->rightBoundary, SIGNAL( currentIndexChanged(int) ), this, SLOT(on_rightBoundary_currentIndexChanged(int)));
};

RELane::~RELane() {

};

rndf::Lane* RELane::create(rndf::Segment* s, double utm_x, double utm_y, const std::string& utm_zone, double theta, double length, double width) {

  if (!s) {return NULL;}

  rndf::Lane* lane = rn_->addLane(s);
  lane->setLaneWidth(width);

  re_wp_.create(lane, utm_x + cos(theta - M_PI) * length * 0.5, utm_y + sin(theta - M_PI) * length * 0.5, utm_zone);
  re_wp_.create(lane, utm_x + cos(theta) * length * 0.5, utm_y + sin(theta) * length * 0.5, utm_zone);

  // add waypoints
  //	for (int wp = 0; wp < 2; ++wp) {
  //		re_wp_.create(lane, utm_x + cos(wp)*length - length*0.5, utm_y);
  //	}
  elem_ = lane;
  return lane;
}

rndf::Lane* RELane::copy(rndf::Lane* source_lane, rndf::Segment* dest_segment, double delta_x, double delta_y) {
  if (!dest_segment) {
    std::cout << "select a segment before copying" << std::endl;
    return NULL;
  }

  if (!source_lane) {
    std::cout << "select a Lane before copying" << std::endl;
    return NULL;
  }

  // add the Lane
  std::string strLaneName = dest_segment->nextLaneStr();

  rndf::Lane* dest_lane = rn_->addLane(dest_segment, strLaneName);
  dest_lane->setLaneWidth(source_lane->laneWidth());
  dest_lane->setLeftBoundaryType(source_lane->leftBoundaryType());
  dest_lane->setRightBoundaryType(source_lane->rightBoundaryType());

  rndf::TWayPointVec::const_iterator wpit =  source_lane->wayPoints().begin(), wpit_end = source_lane->wayPoints().end();
  for (; wpit != wpit_end; ++wpit) {
    re_wp_.copy(*wpit, dest_lane, delta_x, delta_y);
  }

  elem_ = dest_lane;

  return dest_lane;
}

void RELane::move(rndf::Lane* l, double delta_x, double delta_y) {
  if (!l) {return;}

  rndf::TWayPointVec::const_iterator wpit =  l->wayPoints().begin(), wpit_end = l->wayPoints().end();
  for (; wpit != wpit_end; ++wpit) {
    re_wp_.move(*wpit, delta_x, delta_y);
  }
}

void RELane::rotate(rndf::Lane* l, double center_x, double center_y, double theta) {
  if (!l) {return;}


  dgc::dgc_transform_t t;
  dgc::dgc_transform_identity(t);
  dgc::dgc_transform_rotate_z(t, theta);

  rndf::TWayPointVec::const_iterator wpit =  l->wayPoints().begin(), wpit_end = l->wayPoints().end();
  for (; wpit != wpit_end; wpit++) {
    rndf::WayPoint* wp = *wpit;
    double x = wp->utmX() - center_x;
    double y = wp->utmY() - center_y;
    double z = 0;
//    const double z = 0;
    dgc::dgc_transform_point(&x, &y, &z, t);
    wp->setUtm(x + center_x, y + center_y, wp->utmZone());
  }
}

void RELane::updateGUI() {
  const QString baseTitle("Current Lane: ");

  if (!elem_) {
    ui_->laneBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->laneBox->setTitle(baseTitle + elem_->name().c_str());

  ui_->laneWidth->setValue(elem_->laneWidth());
  ui_->leftBoundary->setCurrentIndex((int) elem_->leftBoundaryType());
  ui_->rightBoundary->setCurrentIndex((int) elem_->rightBoundaryType());
}

void RELane::on_laneWidth_valueChanged(double laneWidth) {
  if (!elem_) {return;}
  elem_->setLaneWidth(laneWidth);
  ui_->glWindow->requestRedraw();
}

void RELane::on_leftBoundary_currentIndexChanged(int index) {
  if (!elem_) {return;}
  elem_->setLeftBoundaryType(rndf::Lane::eBoundaryTypes(ui_->leftBoundary->itemData(index).toInt()));
  ui_->glWindow->requestRedraw();
}

void RELane::on_rightBoundary_currentIndexChanged(int index) {
  if (!elem_) {return;}
  elem_->setRightBoundaryType(rndf::Lane::eBoundaryTypes(ui_->rightBoundary->itemData(index).toInt()));
  ui_->glWindow->requestRedraw();
}

} // namespace vlr
