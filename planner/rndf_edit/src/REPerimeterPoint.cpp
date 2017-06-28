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


#include <string>
#include <aw_CGAL.h>
#include <rndf_edit_gui.h>
#include <REPerimeterPoint.h>

using namespace CGAL_Geometry;
using namespace vlr;

namespace vlr {

REPerimeterPoint::REPerimeterPoint(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) :
  REElement<rndf::PerimeterPoint> (ui, rn) {

  connect(ui_->ppLat, SIGNAL(editFinished()), this, SLOT(on_ppLat_editFinished()));
  connect(ui_->ppLon, SIGNAL(editFinished()), this, SLOT(on_ppLon_editFinished()));
  connect(ui_->ppUtmX, SIGNAL(editFinished()), this, SLOT(on_ppUtmX_editFinished()));
  connect(ui_->ppUtmY, SIGNAL(editFinished()), this, SLOT(on_ppUtmY_editFinished()));
//  connect(ui_->ppCheckPoint, SIGNAL(stateChanged(int)), this, SLOT(on_ppCheckPoint_stateChanged(int)));
//  connect(ui_->ppStopPoint, SIGNAL(stateChanged(int)), this, SLOT(on_ppStopPoint_stateChanged(int)));
}

REPerimeterPoint::~REPerimeterPoint() {

}

rndf::PerimeterPoint* REPerimeterPoint::create(rndf::Perimeter* p, double utm_x, double utm_y, const string& utm_zone) {
  if (!p) {return NULL;}

  unsigned int index;

  if (elem_) {
    if (elem_->perimeter() == p) {
      if (elem_->index() < p->numPerimeterPoints() - 1) {
        rndf::PerimeterPoint* p1 = p->perimeterPoint(elem_->index());
        rndf::PerimeterPoint* p2 = p->perimeterPoint(elem_->index() + 1);
        Line_2 lin(Point_2(p1->utmX(), p1->utmY()), Point_2(p2->utmX(), p2->utmY()));
        Point_2 p = lin.projection(Point_2(utm_x, utm_y));
        utm_x = p.x();
        utm_y = p.y();
      }
    }

    index = elem_->index() + 1;
  }
  else {
    index = p->numPerimeterPoints();
  }

  double lat, lon;
  utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
  elem_ = rn_->addPerimeterPoint(p, lat, lon, index);

  return elem_;
}

rndf::PerimeterPoint* REPerimeterPoint::copy(rndf::PerimeterPoint* source_pp, rndf::Perimeter* dest_perimeter, double delta_x, double delta_y) {

  if (!dest_perimeter) {
    std::cout << "select a Perimeter before copying" << std::endl;
    return NULL;
  }

  if (!source_pp) {
    std::cout << "select a Perimeterpoint before copying" << std::endl;
    return NULL;
  }

  double lat, lon;
  utmToLatLong(source_pp->utmX() + delta_x, source_pp->utmY() + delta_y, source_pp->utmZone(), &lat, &lon);
  elem_ = rn_->addPerimeterPoint(dest_perimeter, dest_perimeter->nextPerimeterPointStr(), lat, lon);
  return elem_;
}

void REPerimeterPoint::move(rndf::PerimeterPoint* pp, double delta_x, double delta_y) {
  if(!pp) {return;}

  pp->setUtm(pp->utmX() + delta_x, pp->utmY() + delta_y, pp->utmZone());
}

void REPerimeterPoint::updateGUI() {
  const QString baseTitle("Current Perimeter Point: ");

  if (!elem_) {
    ui_->perimeterPointBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->perimeterPointBox->setTitle(baseTitle + elem_->name().c_str());

  ui_->ppLat->setValue(elem_->lat());
  ui_->ppLon->setValue(elem_->lon());
  ui_->ppUtmX->setValue(elem_->utmX());
  ui_->ppUtmY->setValue(elem_->utmY());
}

void REPerimeterPoint::on_ppLat_editFinished() {

  if (!elem_) {return;}

  if (ui_->ppLat->value() == elem_->lat()) {
    return;
  }

  elem_->setLatLon(ui_->ppLat->value(), elem_->lon());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REPerimeterPoint::on_ppLon_editFinished() {

  if (!elem_) {return;}

  if (ui_->ppLon->value() == elem_->lon()) {
    return;
  }

  elem_->setLatLon(elem_->lat(), ui_->ppLon->value());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REPerimeterPoint::on_ppUtmX_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->ppUtmX->value(), elem_->x())) {
    return;
  }

  elem_->setUtm(ui_->ppUtmX->value(), elem_->utmY(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REPerimeterPoint::on_ppUtmY_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->ppUtmY->value(), elem_->y())) {
    return;
  }

  elem_->setUtm(elem_->utmX(), ui_->ppUtmY->value(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

//void REPerimeterPoint::on_ppCheckPoint_stateChanged(int state) {
//
//  if (!elem_) {return;}
//
//  bool currentState = elem_->checkPoint() != NULL;
//
//  if (state && !currentState) {
//    rn_->addCheckPoint(elem_);
//  }
//  else if (!state && currentState) {
//    rndf::CheckPoint* cp = elem_->checkPoint();
//    if (!cp) {
//      return;
//    }
//    rn_->delCheckPoint(cp);
//  }
//
//  ui_->glWindow->requestRedraw();
//}
//
//void REPerimeterPoint::on_ppStopPoint_stateChanged(int state) {
//
//  if (!elem_) {return;}
//  if (!elem_->parentLane()) {return;}   // no stop sign on parking spots
//
//  bool currentState = elem_->stop() != 0;
//
//  if (state && !currentState) {
//    rn_->addStop(elem_->name());
//  }
//  else if (!state && currentState) {
//    rndf::Stop* sp = elem_->stop();
//    if (!sp) {
//      return;
//    }
//    rn_->delStop(sp);
//  }
//
//  ui_->glWindow->requestRedraw();
//}

} // namespace vlr
