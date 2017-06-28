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
#include <REWayPoint.h>

using namespace CGAL_Geometry;
using namespace vlr;

namespace vlr {

REWayPoint::REWayPoint(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) : REElement<rndf::WayPoint>(ui, rn) {

  connect(ui_->wpLat, SIGNAL(editFinished()), this, SLOT(on_wpLat_editFinished()));
  connect(ui_->wpLon, SIGNAL(editFinished()), this, SLOT(on_wpLon_editFinished()));
  connect(ui_->wpUtmX, SIGNAL(editFinished()), this, SLOT(on_wpUtmX_editFinished()));
  connect(ui_->wpUtmY, SIGNAL(editFinished()), this, SLOT(on_wpUtmY_editFinished()));
  connect(ui_->wpCheckPoint, SIGNAL(stateChanged(int)), this, SLOT(on_wpCheckPoint_stateChanged(int)));
  connect(ui_->wpStopPoint, SIGNAL(stateChanged(int)), this, SLOT(on_wpStopPoint_stateChanged(int)));
  connect(ui_->wpVirtualPoint, SIGNAL(stateChanged(int)), this, SLOT(on_wpVirtualPoint_stateChanged(int)));
 }

REWayPoint::~REWayPoint() {

}

rndf::WayPoint* REWayPoint::create(rndf::Lane* l, double utm_x, double utm_y, const std::string& utm_zone) {
  if (!l) {return NULL;}

  uint32_t index;

  if (elem_) {
    if (elem_->parentLane() == l) {
      if (elem_->index() < l->numWayPoints() - 1) {
        rndf::WayPoint* w1 = l->wayPoint(elem_->index());
        rndf::WayPoint* w2 = l->wayPoint(elem_->index() + 1);
        Line_2 lin(Point_2(w1->utmX(), w1->utmY()), Point_2(w2->utmX(), w2->utmY()));
        Point_2 p = lin.projection(Point_2(utm_x, utm_y));
        utm_x = p.x();
        utm_y = p.y();
      }
    }

    index = elem_->index() + 1;
  }
  else {
    index = l->numWayPoints();
  }

  double lat, lon;
  utmToLatLong(utm_x, utm_y, utm_zone, &lat, &lon);
  elem_ = rn_->addWayPoint(l, lat, lon, index, false);
  return elem_;
}

rndf::WayPoint* REWayPoint::copy(rndf::WayPoint* source_waypoint, rndf::Lane* dest_lane, double delta_x, double delta_y) {

  if (!dest_lane) {
    std::cout << "select a lane before copying" << std::endl;
    return NULL;
  }

  if (!source_waypoint) {
    std::cout << "select a way point before copying" << std::endl;
    return NULL;
  }

  double lat, lon;
  utmToLatLong(source_waypoint->utmX() + delta_x, source_waypoint->utmY() + delta_y, source_waypoint->utmZone(), &lat, &lon);
  elem_ = rn_->addWayPoint(dest_lane, dest_lane->nextWayPointStr(), lat, lon, false);
  return elem_;
}

void REWayPoint::move(rndf::WayPoint* wp, double delta_x, double delta_y) {
  if (!wp) {return;}

  rndf::Lane* l = wp->parentLane();
  if(l) {
    if(wp->index() > 0) {
      rndf::WayPoint* w = l->wayPoint(wp->index()-1);
      double dx = w->x() - wp->x();
      double dy = w->y() - wp->y();
      if(dx*dx+dy*dy < min_wp_dist_squared_) {return;}

    }
    if(wp->index() < l->numWayPoints() - 1) {
      rndf::WayPoint* w = l->wayPoint(wp->index()+1);
      double dx = w->x() - wp->x();
      double dy = w->y() - wp->y();
      if(dx*dx+dy*dy < min_wp_dist_squared_) {return;}
    }
  }

  wp->setUtm(wp->utmX() + delta_x, wp->utmY() + delta_y, wp->utmZone());

//  updateGUI();
}

void REWayPoint::updateGUI() {
  const QString baseTitle("Current Way Point: ");

  if (!elem_) {
    ui_->wayPointBox->setTitle(baseTitle + "-");
    return;
  }

  //std::cout << "baseTitle: " << baseTitle.toStdString() <<", wp: " << elem_->name() << std::endl;
  ui_->wayPointBox->setTitle(baseTitle + elem_->name().c_str());

  ui_->wpLat->setValue(elem_->lat());
  ui_->wpLon->setValue(elem_->lon());
  ui_->wpUtmX->setValue(elem_->utmX());
  ui_->wpUtmY->setValue(elem_->utmY());

  Qt::CheckState state;

  state = (elem_->checkPoint() != NULL ? Qt::Checked : Qt::Unchecked);
  ui_->wpCheckPoint->setCheckState(state);

  state = (elem_->stop() != NULL ? Qt::Checked : Qt::Unchecked);
  ui_->wpStopPoint->setCheckState(state);

  state = (elem_->isVirtual() ? Qt::Checked : Qt::Unchecked);

  ui_->wpVirtualPoint->setCheckState(state);
}

void REWayPoint::on_wpLat_editFinished() {

  if (!elem_) {return;}

  if (ui_->wpLat->value() == elem_->lat()) {
    return;
  }

  elem_->setLatLon(ui_->wpLat->value(), elem_->lon());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REWayPoint::on_wpLon_editFinished() {

  if (!elem_) {return;}

  if (ui_->wpLon->value() == elem_->lon()) {
    return;
  }

  elem_->setLatLon(elem_->lat(), ui_->wpLon->value());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REWayPoint::on_wpUtmX_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->wpUtmX->value(), elem_->x())) {
    return;
  }

  elem_->setUtm(ui_->wpUtmX->value(), elem_->utmY(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REWayPoint::on_wpUtmY_editFinished() {

  if (!elem_) {return;}

  if (utmDiffZero(ui_->wpUtmY->value(), elem_->y())) {
    return;
  }

  elem_->setUtm(elem_->utmX(), ui_->wpUtmY->value(), elem_->utmZone());
  updateGUI();
  ui_->glWindow->requestRedraw();
}

void REWayPoint::on_wpCheckPoint_stateChanged(int state) {

  if (!elem_) {return;}

  elem_->setVirtual(false);

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

void REWayPoint::on_wpStopPoint_stateChanged(int state) {

  if (!elem_) {return;}
  if (!elem_->parentLane()) {return;}   // no stop sign on parking spots

  elem_->setVirtual(false);

  bool currentState = elem_->stop() != 0;

  if (state && !currentState) {
    rn_->addStop(elem_->name());
  }
  else if (!state && currentState) {
    rndf::Stop* sp = elem_->stop();
    if (!sp) {
      return;
    }
    rn_->delStop(sp);
  }

  ui_->glWindow->requestRedraw();
}

void REWayPoint::on_wpVirtualPoint_stateChanged(int state) {

  if (!elem_) {return;}
  if (!elem_->parentLane()) {return;}   // no virtual points on parking spots

  bool currentState = elem_->isVirtual();

   if (state && !currentState) {
     rndf::CheckPoint* cp = elem_->checkPoint();
     if (cp) {
       rn_->delCheckPoint(cp);
     }
     rndf::Stop* sp = elem_->stop();
     if (sp) {
       rn_->delStop(sp);
     }
     elem_->setVirtual(true);
   }
   else if (!state && currentState) {
     elem_->setVirtual(false);
   }

  ui_->glWindow->requestRedraw();
}

} // namespace vlr
