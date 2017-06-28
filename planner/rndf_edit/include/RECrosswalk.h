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


#ifndef RE_CROSSWALK_H_
#define RE_CROSSWALK_H_

#include <REElement.h>

namespace vlr {

class RECrosswalk : public QObject, public REElement<rndf::Crosswalk> {
  Q_OBJECT
public:
  RECrosswalk(Ui::RNDFEdit* ui, rndf::RoadNetwork* rn);
  virtual ~RECrosswalk();

  rndf::Crosswalk* create(double utm_x, double utm_y, const std::string& utm_zone, double yaw, double length, double width);
  rndf::Crosswalk* copy(rndf::Crosswalk* source_cw, double delta_x, double delta_y);
  void move(rndf::Crosswalk* cw, double delta_x, double delta_y);
  void rotate(rndf::Crosswalk* cw, double yaw);
  void rotate(rndf::Crosswalk* cw, double center_x, double center_y, double yaw);
  void updateGUI();

private:
  static const std::string new_way_point_txt;

private slots:
void on_cwLat_editFinished();
void on_cwLon_editFinished();
void on_cwUtmX_editFinished();
void on_cwUtmY_editFinished();
void on_cwOrientation_editFinished();
void on_cwWidth_editFinished();
void on_cwLinkedWayPoints_itemChanged(QListWidgetItem* item);
};

} // namespace vlr

#endif
