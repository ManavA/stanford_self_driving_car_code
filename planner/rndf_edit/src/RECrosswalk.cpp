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
#include <RECrosswalk.h>

namespace vlr {

RECrosswalk::RECrosswalk(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn) : REElement<rndf::Crosswalk>(ui, rn) {
  connect(ui_->cwLat, SIGNAL( editFinished() ), this, SLOT(on_cwLat_editFinished()));
  connect(ui_->cwLon, SIGNAL( editFinished() ), this, SLOT(on_cwLon_editFinished()));
  connect(ui_->cwUtmX, SIGNAL( editFinished() ), this, SLOT(on_cwUtmX_editFinished()));
  connect(ui_->cwUtmY, SIGNAL( editFinished() ), this, SLOT(on_cwUtmY_editFinished()));
  connect(ui_->cwOrientation, SIGNAL( editFinished() ), this, SLOT(on_cwOrientation_editFinished()));
  connect(ui_->cwWidth, SIGNAL( editFinished() ), this, SLOT(on_cwWidth_editFinished()));
  connect(ui_->cwLinkedWayPoints, SIGNAL( itemChanged(QListWidgetItem*) ), this, SLOT(on_cwLinkedWayPoints_itemChanged(QListWidgetItem*)));
}

RECrosswalk::~RECrosswalk() {

}

rndf::Crosswalk* RECrosswalk::create(double utm_x, double utm_y, const std::string& utm_zone, double yaw, double length, double width) {

  elem_ = rn_->addCrosswalk();
  elem_->setUtm1(utm_x, utm_y - 0.5*length, utm_zone);
  elem_->setUtm2(utm_x, utm_y + 0.5*length, utm_zone);
  elem_->width(width);
  rotate(elem_, utm_x, utm_y, yaw);

  return elem_;
}

rndf::Crosswalk* RECrosswalk::copy(rndf::Crosswalk* source_cw, double delta_x, double delta_y) {
  if (!source_cw) {
    std::cout << "Cannot copy - no crosswalk selected" << std::endl;
    return NULL;
  }

  elem_ = rn_->addCrosswalk();
  elem_->setUtm1(source_cw->utmX1() + delta_x, source_cw->utmY1() + delta_y, source_cw->utmZone());
  elem_->setUtm2(source_cw->utmX2() + delta_x, source_cw->utmY2() + delta_y, source_cw->utmZone());
  elem_->width(source_cw->width());

  return elem_;
}

void RECrosswalk::move(rndf::Crosswalk* cw, double delta_x, double delta_y) {
  if (!cw) {return;}

  cw->setUtm1(cw->utmX1() + delta_x, cw->utmY1() + delta_y, cw->utmZone());
  cw->setUtm2(cw->utmX2() + delta_x, cw->utmY2() + delta_y, cw->utmZone());
}


void RECrosswalk::rotate(rndf::Crosswalk* cw, double yaw) {
  if (!cw) {return;}

  double center_x = 0.5*(elem_->utmX1() + elem_->utmX2());
  double center_y = 0.5*(elem_->utmY1() + elem_->utmY2());
  rotate(cw, center_x, center_y, yaw);
}

void RECrosswalk::rotate(rndf::Crosswalk* cw, double center_x, double center_y, double yaw) {
  if (!cw) {return;}

  dgc::dgc_transform_t t;
  dgc::dgc_transform_identity(t);
  dgc::dgc_transform_rotate_z(t, yaw);

  double dx1 = cw->utmX1() - center_x;
  double dy1 = cw->utmY1() - center_y;
  double dz1 = 0;
  dgc::dgc_transform_point(&dx1, &dy1, &dz1, t);
  cw->setUtm1(dx1 + center_x, dy1 + center_y, cw->utmZone());

  double dx2 = cw->utmX2() - center_x;
  double dy2 = cw->utmY2() - center_y;
  double dz2 = 0;
  dgc::dgc_transform_point(&dx2, &dy2, &dz2, t);
  cw->setUtm2(dx2 + center_x, dy2 + center_y, cw->utmZone());
}

void RECrosswalk::updateGUI() {

  const QString baseTitle("Current Crosswalk: ");

  if (!elem_) {
    ui_->crosswalkBox->setTitle(baseTitle + "-");
    return;
  }

  ui_->crosswalkBox->setTitle(baseTitle + elem_->name().c_str());

  double lat, lon;
  elem_->centerLatLon(lat, lon);
  ui_->cwLat->setValue(lat);
  ui_->cwLon->setValue(lon);

  double utm_x, utm_y;
  std::string utm_zone;
  latLongToUtm(lat, lon, &utm_x, &utm_y, utm_zone);
  ui_->cwUtmX->setValue(utm_x);
  ui_->cwUtmY->setValue(utm_y);

  ui_->cwOrientation->setValue(elem_->orientation());
  ui_->cwLinkedWayPoints->clear();

  std::map<std::string, rndf::WayPoint*>::const_iterator wpit = elem_->linkedWayPoints().begin(), wpit_end = elem_->linkedWayPoints().end();
  for(; wpit != wpit_end; wpit++) {
    ui_->cwLinkedWayPoints->addItem((*wpit).second->name().c_str());
    int num_items = ui_->cwLinkedWayPoints->count();
    QListWidgetItem* item = ui_->cwLinkedWayPoints->item(num_items-1);
    item->setFlags(item->flags() | Qt::ItemIsEditable);
  }
    // add empty item to allow adding new way points
  REElement<rndf::Crosswalk>::addEmptyLine(new_way_point_txt,  *ui_->cwLinkedWayPoints);
}

void RECrosswalk::on_cwLat_editFinished() {

  if(!elem_) {return;}

  double utm_x, utm_y;
  std::string utm_zone;
  latLongToUtm(ui_->cwLat->value(), ui_->cwLon->value(), &utm_x, &utm_y, utm_zone);

  double center_x = 0.5*(elem_->utmX1() + elem_->utmX2());
  double center_y = 0.5*(elem_->utmY1() + elem_->utmY2());

  double dx  = utm_x - center_x;
  double dy  = utm_y - center_y;
  elem_->setUtm1(elem_->utmX1()+dx, elem_->utmY1() + dy, utm_zone);
  elem_->setUtm2(elem_->utmX2()+dx, elem_->utmY2() + dy, utm_zone);

  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwLon_editFinished() {

  if(!elem_) {return;}

  double utm_x, utm_y;
  std::string utm_zone;
  latLongToUtm(ui_->cwLat->value(), ui_->cwLon->value(), &utm_x, &utm_y, utm_zone);

  double center_x = 0.5*(elem_->utmX1() + elem_->utmX2());
  double center_y = 0.5*(elem_->utmY1() + elem_->utmY2());

  double dx  = utm_x - center_x;
  double dy  = utm_y - center_y;
  elem_->setUtm1(elem_->utmX1()+dx, elem_->utmY1() + dy, utm_zone);
  elem_->setUtm2(elem_->utmX2()+dx, elem_->utmY2() + dy, utm_zone);

  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwUtmX_editFinished() {

  if(!elem_) {return;}

  double center_x = 0.5*(elem_->utmX1() + elem_->utmX2());
  if(utmDiffZero(ui_->cwUtmX->value(), center_x)) {return;}
  double dx  = ui_->cwUtmX->value() - center_x;
  elem_->setUtm1(elem_->utmX1()+dx, elem_->utmY1(), elem_->utmZone());
  elem_->setUtm2(elem_->utmX2()+dx, elem_->utmY2(), elem_->utmZone());

  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwUtmY_editFinished() {

  if(!elem_) {return;}

  double center_y = 0.5*(elem_->utmY1() + elem_->utmY2());
  if(utmDiffZero(ui_->cwUtmY->value(), center_y)) {return;}
  double dy  = ui_->cwUtmY->value() - center_y;
  elem_->setUtm1(elem_->utmX1(), elem_->utmY1()+dy, elem_->utmZone());
  elem_->setUtm2(elem_->utmX2(), elem_->utmY2()+dy, elem_->utmZone());

  updateGUI();
  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwOrientation_editFinished() {

  if(!elem_) {return;}
  double yaw = ui_->cwOrientation->value();
  if(yaw == elem_->orientation()) {return;}

  rotate(elem_, yaw);

  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwWidth_editFinished() {
  if (!elem_) {return;}
  elem_->width(ui_->cwWidth->value());
  ui_->glWindow->requestRedraw();
}

void RECrosswalk::on_cwLinkedWayPoints_itemChanged(QListWidgetItem* item) {

  if (!elem_) {return;}
  if(item->text().toStdString() == new_way_point_txt) {return;}

  rndf::TWayPointMap::const_iterator wpit = rn_->wayPoints().find(item->text().toStdString());
  if (wpit != rn_->wayPoints().end()) {
    item->setForeground(QBrush(QColor(0, 0, 0)));
    elem_->addWayPoint((*wpit).second);
  }
  else {
//    ui_->cwLinkedWayPoints->removeItemWidget(item);
    delete ui_->cwLinkedWayPoints->takeItem(ui_->cwLinkedWayPoints->row(item));
  }

  addEmptyLine(new_way_point_txt, *ui_->cwLinkedWayPoints);

  ui_->glWindow->requestRedraw();
}

const std::string RECrosswalk::new_way_point_txt="<add way point>";

} // namespace vlr
