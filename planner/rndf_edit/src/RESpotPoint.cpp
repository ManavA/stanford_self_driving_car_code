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


#include <aw_CGAL.h>
#include <rndf_edit_gui.h>
#include <RESpotPoint.h>

using namespace CGAL_Geometry;

namespace vlr {

RESpotPoint::RESpotPoint(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) : REElement<rndf::WayPoint>(ui, rn) {

  connect(ui_->spLat, SIGNAL(editFinished()), this, SLOT(on_spLat_editFinished()));
  connect(ui_->spLon, SIGNAL(editFinished()), this, SLOT(on_spLon_editFinished()));
  connect(ui_->spUtmX, SIGNAL(editFinished()), this, SLOT(on_spUtmX_editFinished()));
  connect(ui_->spUtmY, SIGNAL(editFinished()), this, SLOT(on_spUtmY_editFinished()));
  connect(ui_->spCheckPoint, SIGNAL(stateChanged(int)), this, SLOT(on_spCheckPoint_stateChanged(int)));
 }

RESpotPoint::~RESpotPoint() {

}

rndf::WayPoint* RESpotPoint::create(rndf::Spot* s, double utm_x, double utm_y, const std::string& utm_zone) {

  if(!s) {return NULL;}

  if (s->numSpotPoints() >= 2) {
    return NULL;
  }

  double lat, lon;
  utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
  elem_ = rn_->addWayPoint(s, lat, lon);

  return elem_;
}

rndf::WayPoint* RESpotPoint::copy(rndf::WayPoint* source_spotpoint, rndf::Spot* dest_spot, double delta_x, double delta_y) {

  if (!dest_spot) {
    std::cout << "select a Spot before copying" << std::endl;
    return NULL;
  }

  if (dest_spot->numSpotPoints() >= 2) {
    return NULL;
  }

  if (!source_spotpoint) {
    std::cout << "select a Spot point before copying" << std::endl;
    return NULL;
  }

  double lat, lon;
  utmToLatLong(source_spotpoint->utmX() + delta_x, source_spotpoint->utmY() + delta_y, source_spotpoint->utmZone(), &lat, &lon);
  elem_ = rn_->addWayPoint(dest_spot, dest_spot->nextSpotPointStr(), lat, lon);
  return elem_;
}

void RESpotPoint::move(rndf::WayPoint* sp, double delta_x, double delta_y) {
  if(!sp) {return;}

  sp->setUtm(sp->utmX() + delta_x, sp->utmY() + delta_y, sp->utmZone());
}

void RESpotPoint::updateGUI() {
  const QString baseTitle("Current Spot Point: ");

  if (!elem_) {
    ui_->spotPointBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->spotPointBox->setTitle(baseTitle + elem_->name().c_str());

  ui_->spLat->setValue(elem_->lat());
  ui_->spLon->setValue(elem_->lon());
  ui_->spUtmX->setValue(elem_->utmX());
  ui_->spUtmY->setValue(elem_->utmY());

  Qt::CheckState state;

  state = (elem_->checkPoint() != NULL ? Qt::Checked : Qt::Unchecked);
  ui_->spCheckPoint->setCheckState(state);
}

void RESpotPoint::on_spLat_editFinished() {

  if (!elem_) {return;}

  if (ui_->spLat->value() == elem_->lat()) {
    return;
  }

  elem_->setLatLon(ui_->spLat->value(), elem_->lon());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RESpotPoint::on_spLon_editFinished() {

  if (!elem_) {return;}

  if (ui_->spLon->value() == elem_->lon()) {
    return;
  }

  elem_->setLatLon(elem_->lat(), ui_->spLon->value());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RESpotPoint::on_spUtmX_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->spUtmX->value(), elem_->x())) {
    return;
  }

  elem_->setUtm(ui_->spUtmX->value(), elem_->utmY(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RESpotPoint::on_spUtmY_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->spUtmY->value(), elem_->y())) {
    return;
  }

  elem_->setUtm(elem_->utmX(), ui_->spUtmY->value(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RESpotPoint::on_spCheckPoint_stateChanged(int state) {

  if (!elem_) {return;}

  bool currentState = elem_->checkPoint() != NULL;

  if (state && !currentState) {
    rn_->addCheckPoint(elem_);
  }
  else if (!state && currentState) {
    rndf::CheckPoint* cp = elem_->checkPoint();
    if (!cp) {
      return;
    }
    rn_->delCheckPoint(cp);
  }

  ui_->glWindow->requestRedraw();
}

} // namespace vlr
