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


#include <global.h>
#include <gl_support.h>

#include <aw_roadNetwork.h>
#include <aw_roadNetworkGL.h>
#include <aw_trafficLightGL.h>
#include <trafficlightsign.h>
#include <stopsign.h>
#include <yieldsign.h>
#include <mergesign.h>
#include <crosswalksign.h>

using namespace std;
using namespace CGAL_Geometry;
using namespace dgc;
//
//typedef	Cartesian<double>	CS;
//typedef CS::Point_2			Point_2;
//typedef CS::Vector_2		Vector_2;
//typedef CS::Segment_2		Segment_2;
//typedef CS::Line_2			Line_2;

namespace vlr {
namespace rndf {

RoadNetworkGL::RoadNetworkGL() : display_list_genererated_(false) {

  memset(&display_list_, 0, sizeof(display_list_));

    // load sign textures
  traffic_light_.loadFromBytes(traffic_light_data, 256, true);
  traffic_light_sign_.loadFromBytes(traffic_light_data, 256, true);
  yield_sign_.loadFromBytes(yield_sign_data, 256, true);
  merge_sign_.loadFromBytes(merge_sign_data, 256, true);
  stop_sign1_.loadFromBytes(stop_sign_data, 256, true);
  stop_sign2_.loadFromBytes(stop_sign_data, 256, true);
  crosswalk_stop_sign_.loadFromBytes(crosswalk_stop_sign_data, 256, true);
  crosswalk_incoming_sign_.loadFromBytes(crosswalk_incoming_sign_data, 256, true);

//  dgc_gl_load_texture_from_bytes(TRAFFICLIGHT_SIZE, trafficlight_data, 256, true);
//  dgc_gl_load_texture_from_bytes(sizeof(trafficlight_sign_data), trafficlight_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(YIELD_SIGN_SIZE, yield_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(MERGE_SIGN_SIZE, merge_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(STOP_SIGN_SIZE, stop_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(STOP_SIGN_SIZE, stop_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(CROSSWALK_STOP_SIGN_SIZE, crosswalk_stop_sign_data, 256, FLIP_IMAGE);
//  dgc_gl_load_texture_from_bytes(CROSSWALK_INCOMING_SIGN_SIZE, crosswalk_incoming_sign_data, 256, FLIP_IMAGE);
}

RoadNetworkGL::~RoadNetworkGL() {

}

void RoadNetworkGL::draw_lane_background(double x, double y, double theta, double w, double l) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(2);

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(theta), 0, 0, 1);

  /* draw Lane outline */
  glBegin(GL_POLYGON);
  glVertex2f(l / 2, w / 2);
  glVertex2f(l / 2, -w / 2);
  glVertex2f(-l / 2, -w / 2);
  glVertex2f(-l / 2, w / 2);
  glEnd();

  glPopMatrix();
  glLineWidth(1);

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void RoadNetworkGL::draw_lane_background(Point_2 p1, Point_2 p2, Vector_2 dir1, Vector_2 dir2, double w1, double w2) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_LINE_SMOOTH);

  glLineWidth(2);

  //    glPushMatrix();
  //    glTranslatef(x, y, 0);
  //    glRotatef(kogmo_radians_to_degrees(atan2(dir1.y(),dir1.y())), 0, 0, 1);

  Vector_2 dir = p2 - p1;
  Vector_2 dir1p = (dir1 + dir).perpendicular(CGAL::COUNTERCLOCKWISE);
  dir1p = dir1p * w1 * 0.5 / sqrt(dir1p.squared_length());
  Vector_2 dir2p = (dir2 + dir).perpendicular(CGAL::COUNTERCLOCKWISE);
  dir2p = dir2p * w2 * 0.5 / sqrt(dir2p.squared_length());

  Point_2 p1l = p1 + dir1p;
  Point_2 p1r = p1 - dir1p;
  Point_2 p2l = p2 + dir2p;
  Point_2 p2r = p2 - dir2p;

  /* draw Lane outline */
  glBegin(GL_POLYGON);
  glVertex2f(p1l.x(), p1l.y());
  glVertex2f(p2l.x(), p2l.y());
  glVertex2f(p2r.x(), p2r.y());
  glVertex2f(p1r.x(), p1r.y());
  glEnd();

  //    glPopMatrix();
  glLineWidth(1);

  glDisable(GL_LINE_SMOOTH);
  glDisable(GL_BLEND);
}

void RoadNetworkGL::draw_checkpoint(double x, double y, double r, int num, double blend) {
  char str[100];

  sprintf(str, "%d", num);
  glColor4f(0, 1.0, 0.5, blend);
  draw_circle(x, y, r, 0);
  render_stroke_text_centered_2D(x - 1.5, y - 1, GLUT_STROKE_ROMAN, 1.0, str);
}

void RoadNetworkGL::draw_stoppoint(double x, double y, double r, double blend) {
  glColor4f(0.8, 0., 0., blend);
  draw_circle(x, y, r, 0);
}

void RoadNetworkGL::draw_traffic_light(double x, double y, double z, double orientation, double blend) {
  glPushMatrix();

  glEnable(GL_DEPTH_TEST);
  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(orientation), 0., 0., 1.);
  glTranslatef(.1, 0., 0.);

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_NORMALIZE);

  draw_light_base();

  glPopMatrix();
}

void RoadNetworkGL::draw_lane_boundary(Lane *l, double origin_x, double origin_y, int left, double blend) {
  double r, theta, dx, dy;
  TLaneMap::const_iterator itLanes, itLanes_end;
  TWayPointVec::const_iterator itWayPoints, itWayPoints_end, itWayPoints_next;
  WayPoint *w1, *w2;

  //	if((left && l->leftBoundaryType() == Lane::UnknownBoundary) ||
  //		(!left && l->rightBoundaryType() == Lane::UnknownBoundary))
  //		return;

  // set lane width
  if (l->laneWidth() == 0) r = 0.5;
  else r = l->laneWidth() / 2.0;

  // set lane boundaries color
  if ((left && (l->leftBoundaryType() == Lane::BrokenWhite || l->leftBoundaryType() == Lane::SolidWhite)) || (!left && (l->rightBoundaryType()
      == Lane::BrokenWhite || l->rightBoundaryType() == Lane::SolidWhite))) {
    glColor4f(1, 1, 1, blend);
  }
  else if ((left && (l->leftBoundaryType() == Lane::DoubleYellow)) || (!left && (l->rightBoundaryType() == Lane::DoubleYellow))) {
    glColor4f(1, 1, 0, blend);
  }
  else {
    glColor4f(0.5, 0.5, 0.5, blend);
  }

  // draw Lane
  glLineWidth(2);
  for (itWayPoints = l->wayPoints().begin(), itWayPoints_end = l->wayPoints().end(); itWayPoints != itWayPoints_end; ++itWayPoints) {
    w1 = *itWayPoints;
    itWayPoints_next = itWayPoints;
    itWayPoints_next++;
    if (itWayPoints_next == itWayPoints_end) {
      continue;
    }
    w2 = *itWayPoints_next;
    theta = atan2(w2->utmY() - w1->utmY(), w2->utmX() - w1->utmX());
    dx = r * cos(theta + M_PI_2);
    dy = r * sin(theta + M_PI_2);
    if (!left) {
      dx = -dx;
      dy = -dy;
    }

    // draw boundary
    if ((left && l->leftBoundaryType() == Lane::BrokenWhite) || (!left && l->rightBoundaryType() == Lane::BrokenWhite)) {
      draw_dashed_line(w1->utmX() - origin_x + dx, w1->utmY() - origin_y + dy, w2->utmX() - origin_x + dx, w2->utmY() - origin_y + dy, 1.0);
    }
    else if ((left && l->leftBoundaryType() == Lane::DoubleYellow) || (!left && l->rightBoundaryType() == Lane::DoubleYellow)) {
      draw_line(w1->utmX() - origin_x + dx, w1->utmY() - origin_y + dy, w2->utmX() - origin_x + dx, w2->utmY() - origin_y + dy);
      draw_line(w1->utmX() - origin_x + dx * 0.9, w1->utmY() - origin_y + dy * 0.9, w2->utmX() - origin_x + dx * 0.9, w2->utmY() - origin_y + dy * 0.9);
    }
    else {
      draw_line(w1->utmX() - origin_x + dx, w1->utmY() - origin_y + dy, w2->utmX() - origin_x + dx, w2->utmY() - origin_y + dy);
    }
  }
}

void RoadNetworkGL::draw_lane_connections(const Lane& l, double center_x, double center_y, double blend) {
  for (TLaneSegmentVec::const_iterator ls_it = l.laneSegments().begin(); ls_it != l.laneSegments().end(); ++ls_it) {
    LaneSegment& ls = **ls_it;

    glColor4f(0.1, 0.5, 0.0, blend);
    if (!ls.fromWayPoint() || !ls.toWayPoint()) return;
    for (TLaneSegmentSet::iterator ls2_it = ls.leftLaneSegments().begin(); ls2_it != ls.leftLaneSegments().end(); ++ls2_it) {
      LaneSegment* ls2 = *ls2_it;
      if (!ls2->fromWayPoint() || !ls2->toWayPoint()) return;
      Point_2 p1s = Point_2(ls.fromWayPoint()->utmX(), ls.fromWayPoint()->utmY());
      Point_2 p1e = Point_2(ls.toWayPoint()->utmX(), ls.toWayPoint()->utmY());
      Point_2 p2s = Point_2(ls2->fromWayPoint()->utmX(), ls2->fromWayPoint()->utmY());
      Point_2 p2e = Point_2(ls2->toWayPoint()->utmX(), ls2->toWayPoint()->utmY());

      // Punkte so normieren das Verbindungslinien senkrecht stehen
      Line_2 l1(p1s, p1e);
      Line_2 l2(p2s, p2e);
      Segment_2 s1(p1s, p1e);
      Segment_2 s2(p2s, p2e);
      Point_2 pp1s = l1.projection(p2s);
      Point_2 pp1e = l1.projection(p2e);
      Point_2 pp2s = l2.projection(p1s);
      Point_2 pp2e = l2.projection(p1e);
      if (squared_distance(s1, pp1s) > 0.1) p2s = pp2s;
      if (squared_distance(s1, pp1e) > 0.1) p2e = pp2e;
      if (squared_distance(s2, pp2s) > 0.1) p1s = pp1s;
      if (squared_distance(s2, pp2e) > 0.1) p1e = pp1e;

      // Verbindungslininen berechnen und zeichnen
      for (uint32_t i = 0; i < 2; ++i) {
        Point_2 p1 = p1s + (p1e - p1s) * (0.6 * i + 0.2);
        Point_2 p2 = p2s + (p2e - p2s) * (0.6 * i + 0.2);
        p1 = p1 + (p2 - p1) * 0.1;
        p2 = p2 + (p1 - p2) * 0.1;
        draw_circle(p1.x() - center_x, p1.y() - center_y, 0.25, true);
        draw_arrow(p1.x() - center_x, p1.y() - center_y, p2.x() - center_x, p2.y() - center_y, 0.25, 1.0);
        draw_circle(p2.x() - center_x, p2.y() - center_y, 0.25, true);
        draw_arrow(p2.x() - center_x, p2.y() - center_y, p1.x() - center_x, p1.y() - center_y, 0.25, 1.0);
      }
    }

    glColor4f(0.8, 0.3, 0.0, blend);
    for (TLaneSegmentSet::iterator ls2_it = ls.oncomingLaneSegments().begin(); ls2_it != ls.oncomingLaneSegments().end(); ++ls2_it) {
      LaneSegment* ls2 = *ls2_it;
      if (!ls2->fromWayPoint() || !ls2->toWayPoint()) return;
      Point_2 p1s = Point_2(ls.fromWayPoint()->utmX(), ls.fromWayPoint()->utmY());
      Point_2 p1e = Point_2(ls.toWayPoint()->utmX(), ls.toWayPoint()->utmY());
      Point_2 p2s = Point_2(ls2->fromWayPoint()->utmX(), ls2->fromWayPoint()->utmY());
      Point_2 p2e = Point_2(ls2->toWayPoint()->utmX(), ls2->toWayPoint()->utmY());

      Point_2 pp1 = p1s + (p1e - p1s) * 0.7;
      Point_2 pp2 = p2s + (p2e - p2s) * 0.3;
      draw_arrow(pp1.x() - center_x, pp1.y() - center_y, pp2.x() - center_x, pp2.y() - center_y, 0.25, 1.0);
      continue;

      // Punkte so normieren das Verbindungslinien senkrecht stehen
      Line_2 l1(p1s, p1e);
      Line_2 l2(p2s, p2e);
      Segment_2 s1(p1s, p1e);
      Segment_2 s2(p2s, p2e);
      Point_2 pp1s = l1.projection(p2s);
      Point_2 pp1e = l1.projection(p2e);
      Point_2 pp2s = l2.projection(p1s);
      Point_2 pp2e = l2.projection(p1e);
      if (squared_distance(s1, pp1s) > 0.1) p2s = pp2e;
      if (squared_distance(s1, pp1e) > 0.1) p2e = pp2s;
      if (squared_distance(s2, pp2s) > 0.1) p1s = pp1e;
      if (squared_distance(s2, pp2e) > 0.1) p1e = pp1s;

      // Verbindungslininen berechnen und zeichnen
      Point_2 p1 = p1s + (p1e - p1s) * 0.8;
      Point_2 p2 = p2s + (p2e - p2s) * 0.2;
      p1 = p1 + (p2 - p1) * 0.1;
      p2 = p2 + (p1 - p2) * 0.1;
      glColor4f(0.8, 0.3, 0.0, blend);
      draw_circle(p1.x() - center_x, p1.y() - center_y, 0.25, true);
      draw_arrow(p1.x() - center_x, p1.y() - center_y, p2.x() - center_x, p2.y() - center_y, 0.25, 1.0);
    }
  }
}

void RoadNetworkGL::draw_intersection(const Intersection& i, double center_x, double center_y, double blend) {
  const TLaneSegmentSet& laneSegs = i.laneSegments();
  double icenter_x = 0.;
  double icenter_y = 0.;
  double iradius = 0.;
  double lensum = 0.;
  uint32_t pcount = 0;
  glColor4f(0.8, 0.8, 0.0, blend);
  for (TLaneSegmentSet::const_iterator it = laneSegs.begin(); it != laneSegs.end(); ++it) {
    const LaneSegment* l = *it;
    if (!l->fromWayPoint() || !l->toWayPoint()) return;

    Point_2 ps = Point_2(l->fromWayPoint()->utmX(), l->fromWayPoint()->utmY());
    Point_2 pe = Point_2(l->toWayPoint()->utmX(), l->toWayPoint()->utmY());

    Vector_2 d = pe - ps;
    double len = std::sqrt(d.squared_length());
    d = d / len;
    double w = l->lane()->laneWidth() / 2 - 0.5;

    icenter_x += (ps.x() + pe.x()) * len;
    icenter_y += (ps.y() + pe.y()) * len;
    iradius += len;
    lensum += 2. * len;
    pcount += 2;

    Point_2 psl = ps + d * 0.5 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 psr = ps + d * 0.5 + d.perpendicular(CGAL::RIGHT_TURN) * w;
    Point_2 pel = pe - d * 0.5 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 per = pe - d * 0.5 + d.perpendicular(CGAL::RIGHT_TURN) * w;

    draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
    draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
    draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
    draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
  }

  // Kreis um Intersection malen
  draw_circle(icenter_x / lensum - center_x, icenter_y / lensum - center_y, iradius / pcount * 2.0, 0);
}

void RoadNetworkGL::draw_kturns(const LaneSegment* l, double center_x, double center_y, double blend) {
  //	if ( ! l->isKTurnEdge() ) return;

  glColor4f(0.4, 0.8, 1.0, blend);
  if (!l->fromWayPoint() || !l->toWayPoint()) return;

  Point_2 ps = Point_2(l->fromWayPoint()->utmX(), l->fromWayPoint()->utmY());
  Point_2 pe = Point_2(l->toWayPoint()->utmX(), l->toWayPoint()->utmY());

  Vector_2 d = pe - ps;
  double len = std::sqrt(d.squared_length());
  d = d / len;
  double w = l->lane()->laneWidth() / 2 - 0.6;

  Point_2 psl = ps + d * 0.6 + d.perpendicular(CGAL::LEFT_TURN) * w;
  Point_2 psr = ps + d * 0.6 + d.perpendicular(CGAL::RIGHT_TURN) * w;
  Point_2 pel = pe - d * 0.6 + d.perpendicular(CGAL::LEFT_TURN) * w;
  Point_2 per = pe - d * 0.6 + d.perpendicular(CGAL::RIGHT_TURN) * w;

  draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
  draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
  draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
  draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
}

void RoadNetworkGL::draw_TypesLaneSegment(const Lane& lane1, double center_x, double center_y, double blend) {
  const TLaneSegmentVec& laneSegs = lane1.laneSegments();
  for (TLaneSegmentVec::const_iterator it = laneSegs.begin(); it != laneSegs.end(); ++it) {
    const LaneSegment* l = *it;
    if (!l->fromWayPoint() || !l->toWayPoint()) return;

    // Farbe abhängig vom Typ setzen
    if (l->isPriorityLane()) glColor4f(0.0, 0.6, 0.0, blend);
    else if (l->isStopLane()) glColor4f(0.6, 0.2, 0.0, blend);
    else continue;

    Point_2 ps = Point_2(l->fromWayPoint()->utmX(), l->fromWayPoint()->utmY());
    Point_2 pe = Point_2(l->toWayPoint()->utmX(), l->toWayPoint()->utmY());

    Vector_2 d = pe - ps;
    double len = std::sqrt(d.squared_length());
    d = d / len;
    //		d = Vector_2(0., 0.);
    const double shrink = 0.3;
    double w = l->lane()->laneWidth() / 2 - shrink;

    // Randpunkte berechnen
    Point_2 psl = ps + d * shrink * 2 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 psr = ps + d * shrink * 2 + d.perpendicular(CGAL::RIGHT_TURN) * w;
    Point_2 pel = pe - d * shrink * 2 + d.perpendicular(CGAL::LEFT_TURN) * w;
    Point_2 per = pe - d * shrink * 2 + d.perpendicular(CGAL::RIGHT_TURN) * w;

    // Umrandung malen
    //		draw_line(psl.x() - center_x, psl.y() - center_y, psr.x() - center_x, psr.y() - center_y);
    //		draw_line(pel.x() - center_x, pel.y() - center_y, per.x() - center_x, per.y() - center_y);
    draw_line(psl.x() - center_x, psl.y() - center_y, pel.x() - center_x, pel.y() - center_y);
    draw_line(psr.x() - center_x, psr.y() - center_y, per.x() - center_x, per.y() - center_y);
  }
}

void RoadNetworkGL::get_exitpoint_params(rndf::Exit* e, Point_2& point_from, Point_2& point_to, Vector_2& dir_from, Vector_2& dir_to, double& width) {
  WayPoint* w1, *w2;
  PerimeterPoint* p1, *p2;

  switch (e->exitType()) {
  case rndf::Exit::LaneToLane: {
    w1 = e->getExitFromLane();
    w2 = e->getExitToLane();
    assert(w1);
    assert(w2);
    point_from = Point_2(w1->utmX(), w1->utmY());
    point_to = Point_2(w2->utmX(), w2->utmY());
    rndf::Lane* l1 = w1->parentLane();
    rndf::Lane* l2 = w2->parentLane();
    assert(l1);
    assert(l2);
    // Width berechnen
    width = (l1->laneWidth() + l2->laneWidth()) * 0.5;
    // Thetas berechnen
    const TWayPointVec& wps1 = l1->wayPoints();
    const TWayPointVec& wps2 = l2->wayPoints();
    assert(wps1.size() >= 2);
    assert(wps1.size() >= 2);
    WayPoint* wp1 = wps1[wps1.size() - 2];
    WayPoint* wp2 = wps1[wps1.size() - 1];
    WayPoint* wp3 = wps2[0];
    WayPoint* wp4 = wps2[1];
    Point_2 p1 = Point_2(wp1->utmX(), wp1->utmY());
    Point_2 p2 = Point_2(wp2->utmX(), wp2->utmY());
    Point_2 p3 = Point_2(wp3->utmX(), wp3->utmY());
    Point_2 p4 = Point_2(wp4->utmX(), wp4->utmY());
    dir_from = p2 - p1;
    dir_to = p4 - p3;
  }
    break;
  case rndf::Exit::LaneToPerimeter:
    w1 = e->getExitFromLane();
    p2 = e->getExitToPerimeter();
    assert(w1);
    assert(p2);
    point_from = Point_2(w1->utmX(), w1->utmY());
    point_to = Point_2(p2->utmX(), p2->utmY());
    width = w1->parentLane()->laneWidth();
    break;
  case rndf::Exit::PerimeterToLane:
    p1 = e->getExitFromPerimeter();
    w2 = e->getExitToLane();
    assert(p1);
    assert(w2);
    point_from = Point_2(p1->utmX(), p1->utmY());
    point_to = Point_2(w2->utmX(), w2->utmY());
    width = w2->parentLane()->laneWidth();
    break;
  case rndf::Exit::PerimeterToPerimeter:
    p1 = e->getExitFromPerimeter();
    p2 = e->getExitToPerimeter();
    assert(p1);
    assert(p2);
    point_from = Point_2(p1->utmX(), p1->utmY());
    point_to = Point_2(p2->utmX(), p2->utmY());
    width = 3.;
    break;
  default:
    break;
  }

}

// calculates intermediate points to draw exits as round lanes (no bezier but fast ;)
vector<Point_2> RoadNetworkGL::calc_intermediate_points(const Point_2 point_from, const Point_2 point_to, const Vector_2 dir_from, const Vector_2 dir_to) {
  vector<Point_2> res;

  // Zwischenpunkte berechnen um die Kurve abzurunden
  double d1 = sqrt(squared_distance(Line_2(point_to, dir_to), point_from));
  double d2 = sqrt(squared_distance(Line_2(point_from, dir_from), point_to));
  double norm_fak1 = 0.4;
  Point_2 p1 = point_from + dir_from * d1 * norm_fak1 / sqrt(dir_from.squared_length());
  Point_2 p2 = point_to - dir_to * d2 * norm_fak1 / sqrt(dir_to.squared_length());
  Point_2 p_center = p1 + (p2 - p1) * 0.5;
  Point_2 pp1 = Line_2(point_from, p_center).projection(p1);
  Point_2 pp2 = Line_2(point_to, p_center).projection(p2);
  double norm_fak2 = 0.5;
  Point_2 pr1 = pp1 + (p1 - pp1) * norm_fak2;
  Point_2 pr2 = pp2 + (p2 - pp2) * norm_fak2;

  res.push_back(point_from);
  res.push_back(pr1);
  res.push_back(p_center);
  res.push_back(pr2);
  res.push_back(point_to);

  return res;
}

void RoadNetworkGL::draw_traffic_light_2D(double x, double y, double z, double blend) {
  glPushMatrix();

  glTranslatef(x, y, z);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, traffic_light_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(traffic_light_.maxU(), traffic_light_.maxV(), 1);

  glColor4f(1, 1, 1, blend);

  glBegin(GL_POLYGON);
  glTexCoord2f(0., 0.);
  glVertex2f(0., 0.);
  glTexCoord2f(1., 0.);
  glVertex2f(1., 0.);
  glTexCoord2f(1., 1.);
  glVertex2f(1., 2.);
  glTexCoord2f(0., 1.);
  glVertex2f(0., 2.);
  glEnd();

  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}
void RoadNetworkGL::draw_light_state_2D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& /*z*/,
    const double& /*orientation*/, const double& blend) {
  glPushMatrix();
  if (rd && yl && gr) glColor4f(1., 0., 0., blend);
  else if (!(rd || yl || gr)) glColor4f(.3, .3, .3, blend);
  else glColor4f((float) (rd || yl), (float) (gr || yl), 0., blend);

  draw_diamond(x, y, 1.);
  glPopMatrix();
}

void RoadNetworkGL::draw_light_state_3D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& z,
    const double& orientation, const double& /*blend*/) {
  glPushMatrix();
  glTranslatef(x, y, z);
  glRotatef(dgc_r2d(orientation), 0., 0., 1.);
  glTranslatef(.1, 0., 0.);
  draw_red_light(rd);
  draw_yellow_light(yl);
  draw_green_light(gr);
  glPopMatrix();

}
void RoadNetworkGL::draw_light_state(const bool& threeD, const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& z,
    const double& orientation, const double& blend) {
  draw_light_state_2D(rd, yl, gr, x, y, z, orientation, blend);

  if (threeD) {
    draw_light_state_3D(rd, yl, gr, x, y, z, orientation, blend);
  }
}

void RoadNetworkGL::draw_stop_sign_3D(double x, double y, double z, double r, double heading, double blend) {
  int i;
  double angle, cangle, sangle;

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
  if (z != 0) glRotatef(90, 1, 0, 0);

  /* draw the stop sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, stop_sign1_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(stop_sign1_.maxU(), stop_sign1_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 8; i++) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 0.93), 0.5 * (1 + sangle / 0.93));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for (i = 7; i >= 0; i--) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void RoadNetworkGL::draw_stop_sign_2D(double x, double y, double r, double heading, double blend) {
  int i;
  double angle, cangle, sangle;

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, stop_sign2_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(stop_sign2_.maxU(), stop_sign2_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 8; i++) {
    angle = dgc_d2r(45.0 / 2.0 + i * 45.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 0.93), 0.5 * (1 + sangle / 0.93));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void RoadNetworkGL::draw_crosswalk_sign_3D(double x, double y, double z, double r, double heading, double blend, bool stop_waypoint) {
  int i;
  double angle, cangle, sangle;

  Texture* crosswalk_sign;
  if (stop_waypoint) {
    crosswalk_sign = &crosswalk_stop_sign_;
  }
  else {
    crosswalk_sign = &crosswalk_incoming_sign_;
  }

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);
  if (z != 0) glRotatef(90, 1, 0, 0);

  /* draw the crosswalk sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, crosswalk_sign->glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(crosswalk_sign->maxU(), crosswalk_sign->maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 4; i++) {
    angle = dgc_d2r(+i * 90.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / .99), 0.5 * (1 + sangle / .99));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for (i = 3; i >= 0; i--) {
    angle = dgc_d2r(+i * 90.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void RoadNetworkGL::draw_crosswalk_sign_2D(double x, double y, double r, double heading, double blend, bool stop_waypoint) {
  int i;
  double angle, cangle, sangle;

  Texture* crosswalk_sign;
  if (stop_waypoint) {
    crosswalk_sign = &crosswalk_stop_sign_;
  }
  else {
    crosswalk_sign = &crosswalk_incoming_sign_;
  }

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc::dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the crosswalk sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, crosswalk_sign->glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(crosswalk_sign->maxU(), crosswalk_sign->maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 4; i++) {
    angle = dgc_d2r(+i * 90.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / .99), 0.5 * (1 + sangle / .99));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void RoadNetworkGL::draw_trafficlight_sign_3D(double x, double y, double z, double r, double heading, double blend) {
  int i;
  double angle, cangle, sangle;

  glPushMatrix();
  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, -r);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc::dgc_r2d(heading) - 90.0, 0, 0, 1);
  if (z != 0) glRotatef(90, 1, 0, 0);

  /* draw the trafficlight sign */
  glEnable(GL_CULL_FACE);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, traffic_light_sign_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(traffic_light_sign_.maxU(), traffic_light_sign_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 4; i++) {
    angle = dgc_d2r(+i * 90.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 1.02), 0.5 * (1 + sangle / 1.02));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glBegin(GL_POLYGON);
  glColor4f(0.5, 0.5, 0.5, blend);
  for (i = 3; i >= 0; i--) {
    angle = dgc_d2r(+i * 90.0);
    glVertex2f(r * cos(angle), r * sin(angle));
  }
  glEnd();
  glDisable(GL_CULL_FACE);

  glPopMatrix();
}

void RoadNetworkGL::draw_trafficlight_sign_2D(double x, double y, double r, double heading, double blend) {
  int i;
  double angle, cangle, sangle;

  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the trafficlight sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, traffic_light_sign_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(traffic_light_sign_.maxU(), traffic_light_sign_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);
  for (i = 0; i < 4; i++) {
    angle = dgc_d2r(+i * 90.0);
    cangle = cos(angle);
    sangle = sin(angle);
    glTexCoord2f(0.5 * (1 + cangle / 1.02), 0.5 * (1 + sangle / 1.02));
    glVertex2f(r * cangle, r * sangle);
  }
  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void RoadNetworkGL::draw_yield_sign_2D(double x, double y, double r, double heading, double blend) {
  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc::dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, yield_sign_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(yield_sign_.maxU(), yield_sign_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);

  /* bottom of triangle */
  glTexCoord2f(0.5 - 0.025, 0);
  glVertex2f(-0.05 * r, -r * 0.85);

  glTexCoord2f(0.5 + 0.025, 0);
  glVertex2f(0.05 * r, -r * 0.85);

  /* top right */
  glTexCoord2f(1, 1 - 0.14 / 2.0);
  glVertex2f(r, r * 0.85 - 0.14 * r * 0.85);

  glTexCoord2f(1 - 0.01 / 2.0, 1 - 0.07 / 2.0);
  glVertex2f(r - 0.01 * r, r * 0.85 - 0.07 * r * 0.85);

  glTexCoord2f(1 - 0.08 / 2.0, 1);
  glVertex2f(r - 0.08 * r, r * 0.85);

  /* top left */
  glTexCoord2f(0 + 0.08 / 2.0, 1);
  glVertex2f(-r + 0.08 * r, r * 0.85);

  glTexCoord2f(0.01 / 2.0, 1 - 0.07 / 2.0);
  glVertex2f(-r + 0.01 * r, r * 0.85 - 0.07 * r * 0.85);

  glTexCoord2f(0, 1 - 0.14 / 2.0);
  glVertex2f(-r, r * 0.85 - 0.14 * r * 0.85);

  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void RoadNetworkGL::draw_merge_sign_2D(double x, double y, double r, double heading, double blend) {
  glPushMatrix();
  glTranslatef(x, y, 0);
  glRotatef(dgc_r2d(heading) - 90.0, 0, 0, 1);

  /* draw the stop sign */

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, merge_sign_.glTextureId());

  /* scale the texture */
  glMatrixMode(GL_TEXTURE);
  glPushMatrix();
  glScalef(merge_sign_.maxU(), merge_sign_.maxV(), 1);

  glColor4f(1, 1, 1, blend);
  glBegin(GL_POLYGON);

  glTexCoord2f(0, 0.5);
  glVertex2f(-r, 0);

  glTexCoord2f(0.5, 1);
  glVertex2f(0, r);

  glTexCoord2f(1, 0.5);
  glVertex2f(r, 0);

  glTexCoord2f(0.5, 0);
  glVertex2f(0, -r);

  glEnd();
  glDisable(GL_TEXTURE_2D);

  /* undo the scaling */
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
}

void RoadNetworkGL::draw_checkpoint(double x, double y, double r, const std::string& name, double blend) {
  glColor4f(1, 0.5, 0, blend);
  draw_circle(x, y, r, false);
  render_stroke_text_centered_2D(x - 3, y - 3, GLUT_STROKE_ROMAN, 1.0, name.c_str());
}

void RoadNetworkGL::draw_boundary(Lane* l, double origin_x, double origin_y, int left, double blend) {
  double r, theta, dx, dy;

  //  if((left && Lane->leftBoundaryType() == unknown) ||
  //     (!left && Lane->rightBoundaryType() == unknown))
  //    return;

  if (l->laneWidth() == 0) r = 0.5;
  else r = l->laneWidth() / 2.0;

  if ((left && (l->leftBoundaryType() == Lane::BrokenWhite || l->leftBoundaryType() == Lane::SolidWhite)) || (!left && (l->rightBoundaryType()
      == Lane::BrokenWhite || l->rightBoundaryType() == Lane::SolidWhite))) glColor4f(1, 1, 1, blend);
  else if ((left && (l->leftBoundaryType() == Lane::DoubleYellow)) || (!left && (l->rightBoundaryType() == Lane::DoubleYellow))) glColor4f(1, 1, 0, blend);
  else glColor4f(0.5, 0.5, 0.5, blend);

  for (uint32_t i = 0; i < l->wayPoints().size() - 1; i++) {
    theta = atan2(l->wayPoints()[i + 1]->utmY() - l->wayPoints()[i]->utmY(), l->wayPoints()[i + 1]->utmX() - l->wayPoints()[i]->utmX());
    dx = r * cos(theta + M_PI_2);
    dy = r * sin(theta + M_PI_2);
    if (!left) {
      dx = -dx;
      dy = -dy;
    }

    if ((left && l->leftBoundaryType() == Lane::BrokenWhite) || (!left && l->rightBoundaryType() == Lane::BrokenWhite)) draw_dashed_line(
        l->wayPoints()[i]->utmX() - origin_x + dx, l->wayPoints()[i]->utmY() - origin_y + dy, l->wayPoints()[i + 1]->utmX() - origin_x + dx,
        l->wayPoints()[i + 1]->utmY() - origin_y + dy, 1.0);
    else if ((left && l->leftBoundaryType() == Lane::DoubleYellow) || (!left && l->rightBoundaryType() == Lane::DoubleYellow)) {
      draw_line(l->wayPoints()[i]->utmX() - origin_x + dx, l->wayPoints()[i]->utmY() - origin_y + dy, l->wayPoints()[i + 1]->utmX() - origin_x + dx,
          l->wayPoints()[i + 1]->utmY() - origin_y + dy);
      draw_line(l->wayPoints()[i]->utmX() - origin_x + dx * 0.9, l->wayPoints()[i]->utmY() - origin_y + dy * 0.9, l->wayPoints()[i + 1]->utmX() - origin_x
          + dx * 0.9, l->wayPoints()[i + 1]->utmY() - origin_y + dy * 0.9);
    }
    else draw_line(l->wayPoints()[i]->utmX() - origin_x + dx, l->wayPoints()[i]->utmY() - origin_y + dy, l->wayPoints()[i + 1]->utmX() - origin_x + dx,
        l->wayPoints()[i + 1]->utmY() - origin_y + dy);
  }
}

void RoadNetworkGL::draw_crosswalk(Crosswalk *crosswalk, double origin_x, double origin_y, double blend) {

  glColor4f(0., .6, 0., blend);
  draw_line(crosswalk->utmX1() - origin_x, crosswalk->utmY1() - origin_y, crosswalk->utmX2() - origin_x, crosswalk->utmY2() - origin_y);

  draw_diamond(crosswalk->utmX1() - origin_x, crosswalk->utmY1() - origin_y, .6);
  draw_diamond(crosswalk->utmX2() - origin_x, crosswalk->utmY2() - origin_y, .6);

  double r, theta, dx, dy;

  if (crosswalk->width() == 0) r = 0.5;
  else r = crosswalk->width() / 2.0;

  glColor4f(1., 1., 1., blend);

  theta = atan2(crosswalk->utmY2() - crosswalk->utmY1(), crosswalk->utmX2() - crosswalk->utmX1());
  dx = r * cos(theta + M_PI_2);
  dy = r * sin(theta + M_PI_2);

  draw_line(crosswalk->utmX1() - origin_x + dx, crosswalk->utmY1() - origin_y + dy, crosswalk->utmX2() - origin_x + dx, crosswalk->utmY2() - origin_y + dy);

  dx = -dx;
  dy = -dy;

  draw_line(crosswalk->utmX1() - origin_x + dx, crosswalk->utmY1() - origin_y + dy, crosswalk->utmX2() - origin_x + dx, crosswalk->utmY2() - origin_y + dy);

}

void RoadNetworkGL::draw_road_boundaries(const RoadNetwork& rn, double origin_x, double origin_y, double blend) {

  glColor4f(1, 1, 1, blend);
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end = lanes.end();
  for (; lit != lit_end; lit++) {
    draw_boundary((*lit).second, origin_x, origin_y, 1, blend);
    draw_boundary((*lit).second, origin_x, origin_y, 0, blend);
  }
}

void RoadNetworkGL::draw_rndf_numbers(const RoadNetwork& rn, double origin_x, double origin_y) {

  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
    WayPoint* w = (*wpit).second;
    render_stroke_text_centered_2D(w->utmX() - origin_x + 3, w->utmY() - origin_y + 3, GLUT_STROKE_ROMAN, 1.0, w->name().c_str());
  }

  const TPerimeterPointMap& perpoints = rn.perimeterPoints();
  TPerimeterPointMap::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
  for (; ppit != ppit_end; ppit++) {
    PerimeterPoint* pp = (*ppit).second;
    render_stroke_text_centered_2D(pp->utmX() - origin_x + 2, pp->utmY() - origin_y + 2, GLUT_STROKE_ROMAN, 1.0, pp->name().c_str());
  }
}

void RoadNetworkGL::draw_signs(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend) {
  double angle;

  // draw the stop signs
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end = lanes.end();
  for (; lit != lit_end; lit++) {
    const TWayPointVec& waypoints = (*lit).second->wayPoints();
    TWayPointVec::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    for (uint32_t i = 0; i < waypoints.size(); i++) {
      WayPoint* w = waypoints[i];
      if (w->stop()) {
        glPushMatrix();
        glTranslatef(0, 0, 0.03);
        if (i > 0) {
          angle = atan2(w->utmY() - waypoints[i - 1]->utmY(), w->utmX() - waypoints[i - 1]->utmX());
        }
        else {
          angle = 0;
        }
        if (threeD) {
          draw_stop_sign_3D(w->utmX() - origin_x, w->utmY() - origin_y, 2.0, 0.5, angle, blend);
        }
        else {
          draw_stop_sign_2D(w->utmX() - origin_x, w->utmY() - origin_y, 1.2, angle, blend);
        }
        glPopMatrix();
      }
    }
  }
}

void RoadNetworkGL::draw_rndf_lights(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend, bool draw_state) {
  double angle;

  // draw the traffic lights
  const TTrafficLightMap& lights = rn.trafficLights();
  TTrafficLightMap::const_iterator tlit = lights.begin(), tlit_end = lights.end();
  for (; tlit != tlit_end; tlit++) {
    rndf::TrafficLight* tl = (*tlit).second;
    glPushMatrix();
    if (draw_state) {
      draw_light_state(threeD, true, true, true, tl->utmX() - origin_x, tl->utmY() - origin_y, tl->z(), tl->orientation(), blend);
    }
    if (threeD) {
      draw_traffic_light_3D(tl->utmX() - origin_x, tl->utmY() - origin_y, tl->z(), tl->orientation(), blend);
    }
    else {
      draw_traffic_light_2D(tl->utmX() - origin_x, tl->utmY() - origin_y, 0., blend);
    }
    glPopMatrix();

    // draw the traffic light signs and links
    const TWayPointMap& waypoints = tl->linkedWayPoints();
    TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    for (; wpit != wpit_end; wpit++) {
      WayPoint* w = (*wpit).second;

      glPushMatrix();
      glColor4f(0., .62, .13, blend);
      draw_arrow(w->utmX() - origin_x, w->utmY() - origin_y, tl->utmX() - origin_x, tl->utmY() - origin_y, .4, 1.2);
      glPopMatrix();

      glPushMatrix();
      glTranslatef(0, 0, 0.03);
      Lane* l = w->parentLane();
      uint32_t idx = l->wayPointIndex(w);
      if (idx > 1) {
        WayPoint* w_prev = l->wayPointFromId(idx - 1);
        angle = atan2(w->utmY() - w_prev->utmY(), w->utmX() - w_prev->utmX());
      }
      else {
        angle = 0;
      }
      if (threeD) draw_trafficlight_sign_3D(w->utmX() - origin_x, w->utmY() - origin_y, 2.0, 0.5, angle, blend);
      else draw_trafficlight_sign_2D(w->utmX() - origin_x, w->utmY() - origin_y, 1.5, angle, blend);
      glPopMatrix();

    }
  }

  //  for (i = 0; i < rndf->num_segments(); i++)
  //  {
  //    // draw the traffic light signs and links
  //    for (j = 0; j < rndf->segment(i)->num_lanes(); j++)
  //    {
  //      for (k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
  //        w = rndf->segment(i)->Lane(j)->waypoint(k);
  //        if (w->trafficlight()) {
  //          for (std::vector<rndf_trafficlight*>::iterator it =
  //              w->trafficlight_.begin(); it != w->trafficlight_.end(); ++it) {
  //
  //            glPushMatrix();
  //            glColor4f ( 0., .62, .13, blend);
  //            draw_arrow( w->utmX() - origin_x,
  //                        w->utmY() - origin_y,
  //                        (**it).utmX() - origin_x,
  //                        (**it).utmY() - origin_y,
  //                        .4, 1.2);
  //            glPopMatrix();
  //          }
  //
  //          glPushMatrix();
  //            glTranslatef(0, 0, 0.03);
  //            if (w->prev() != NULL)
  //              angle = atan2(w->utmY() - w->prev()->utmY(), w->utmX()
  //                  - w->prev()->utmX());
  //            else
  //              angle = 0;
  //            if (threeD)
  //              draw_trafficlight_sign_3D(w->utmX() - origin_x, w->utmY()
  //                  - origin_y, 2.0, 0.5, angle, blend);
  //            else
  //              draw_trafficlight_sign_2D(w->utmX() - origin_x, w->utmY()
  //                  - origin_y, 1.5, angle, blend);
  //            glPopMatrix();
  //        }
  //      }
  //    }
  //  }
}

void RoadNetworkGL::draw_rndf_crosswalks(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend) {
  double angle;

  //  TWayPointMap drawn_points;
  const TCrosswalkMap& crosswalks = rn.crosswalks();
  TCrosswalkMap::const_iterator cwit = crosswalks.begin(), cwit_end = crosswalks.end();
  for (; cwit != cwit_end; cwit++) {
    Crosswalk* cw = (*cwit).second;
    draw_crosswalk(cw, origin_x, origin_y, blend);
    const TWayPointMap& waypoints = cw->linkedWayPoints();
    TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    for (; wpit != wpit_end; wpit++) {
      WayPoint* w = (*wpit).second;
      //      if(drawn_points.find(w->name()) != drawn_points.end()) {continue;}
      //      drawn_points.insert(make_pair(w->name(), w));
      const TCrosswalkLinkMap& cwlinks = w->crosswalks();
      TCrosswalkLinkMap::const_iterator cwlit = cwlinks.find(cw->name());

      glPushMatrix();
      bool stop_wp = false;
      if ((*cwlit).second.type_ == stop_waypoint) {
        stop_wp = true;
        glColor4f(1., .8, 0, blend);
      }
      else {
        glColor4f(.8, .8, .8, blend);
      }
      draw_arrow(w->utmX() - origin_x, w->utmY() - origin_y, (cw->utmX1() + cw->utmX2()) / 2. - origin_x, (cw->utmY1() + cw->utmY2()) / 2. - origin_y,
          .4, 1.2);
      glPopMatrix();

      if ((!w->stop()) && (w->trafficLights().empty())) {
        glPushMatrix();
        glTranslatef(0, 0, 0.03);
        Lane* l = w->parentLane();
        uint32_t idx = l->wayPointIndex(w);
        if (idx > 1) {
          WayPoint* w_prev = l->wayPointFromId(idx - 1);
          angle = atan2(w->utmY() - w_prev->utmY(), w->utmX() - w_prev->utmX());
        }
        else {
          angle = 0;
        }
        if (threeD) {
          draw_crosswalk_sign_3D(w->utmX() - origin_x, w->utmY() - origin_y, 2.0, 0.5, angle, blend, stop_wp);
        }
        else {
          draw_crosswalk_sign_2D(w->utmX() - origin_x, w->utmY() - origin_y, 1.5, angle, blend, stop_wp);
        }
        glPopMatrix();
      }
    }
  }

  //
  //  for (i = 0; i < rndf->num_segments(); i++)
  //  {
  //    // draw the crosswalks
  //    for (j = 0; j < rndf->segment(i)->num_crosswalks(); j++)
  //    {
  //      c = rndf->segment(i)->crosswalk(j);
  //      draw_crosswalk(c, origin_x, origin_y, blend);
  //    }
  //
  //    // draw the crosswalk signs and links
  //    for (j = 0; j < rndf->segment(i)->num_lanes(); j++)
  //    {
  //      for (k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
  //        w = rndf->segment(i)->Lane(j)->waypoint(k);
  //        if (w->crosswalk()) {
  //          bool stop_wp = false;
  //          for (std::vector<rndf_crosswalk_link>::iterator it =
  //              w->crosswalk_.begin(); it != w->crosswalk_.end(); ++it) {
  //
  //            glPushMatrix();
  //            if (it->type_ == stop_waypoint) {
  //              stop_wp = true;
  //              glColor4f(1., .8, 0, blend);
  //            } else {
  //              glColor4f(.8, .8, .8, blend);
  //            }
  //            draw_arrow( w->utmX() - origin_x,
  //                        w->utmY() - origin_y,
  //                        (it->crosswalk_->utmX1() + it->crosswalk_->utmX2()) / 2. - origin_x,
  //                        (it->crosswalk_->utmY1() + it->crosswalk_->utmY2()) / 2. - origin_y,
  //                        .4, 1.2);
  //            glPopMatrix();
  //          }
  //          if(!w->stop())
  //          {
  //          glPushMatrix();
  //            glTranslatef(0, 0, 0.03);
  //            if (w->prev() != NULL)
  //              angle = atan2(w->utmY() - w->prev()->utmY(), w->utmX()
  //                  - w->prev()->utmX());
  //            else
  //              angle = 0;
  //            if (threeD)
  //              draw_crosswalk_sign_3D(w->utmX() - origin_x, w->utmY()
  //                  - origin_y, 2.0, 0.5, angle, blend, stop_wp);
  //            else
  //              draw_crosswalk_sign_2D(w->utmX() - origin_x, w->utmY()
  //                  - origin_y, 1.5, angle, blend, stop_wp);
  //            glPopMatrix();
  //          }
  //        }
  //      }
  //    }
  //  }
}

// lanechange links are hopefully gone soon...
void RoadNetworkGL::draw_lanechange_links(const RoadNetwork& /*rn*/, double /*origin_x*/, double /*origin_y*/, double /*blend*/) {

  //  int i, j, k, l;
  //  wayPoint *w;
  //
  //  for(i = 0; i < rndf->num_segments(); i++)
  //    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
  //      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
  //        w = rndf->segment(i)->Lane(j)->waypoint(k);
  //
  //  glColor4f(0, 0, 0, blend);
  //  for(l = 0; l < w->num_exits(lanechange); l++)
  //    draw_arrow(w->utmX() - origin_x,
  //         w->utmY() - origin_y,
  //         w->Exit(l, lanechange)->utmX() - origin_x,
  //         w->Exit(l, lanechange)->utmY() - origin_y, 0.25, 1.0);
  //  glColor4f(0, 1, 1, blend);
  //  for(l = 0; l < w->num_exits(uturn); l++)
  //    if(!w->exit_original(l, uturn))
  //      draw_arrow(w->utmX() - origin_x,
  //           w->utmY() - origin_y,
  //           w->Exit(l, uturn)->utmX() - origin_x,
  //           w->Exit(l, uturn)->utmY() - origin_y, 0.5, 2.0);
  //      }
}

void RoadNetworkGL::draw_rndf(const RoadNetwork& rn, int draw_numbers, int draw_stops, int threeD_signs, int draw_boundaries, int draw_lane_links,
    int draw_merges, double origin_x, double origin_y, double blend) {
  draw_rndf(rn, draw_numbers, draw_stops, threeD_signs, draw_boundaries, draw_lane_links, draw_merges, true, 1, origin_x, origin_y, blend);
}

void RoadNetworkGL::draw_rndf(const RoadNetwork& rn, int draw_numbers, int draw_stops, int threeD_signs, int draw_boundaries, int draw_lane_links,
    int draw_merges, int draw_crosswalks, int draw_lights, double origin_x, double origin_y, double blend) {
  double angle;

  glLineWidth(2);

  // draw the Lane centers
  const TLaneMap& lanes = rn.lanes();
  TLaneMap::const_iterator lit = lanes.begin(), lit_end = lanes.end();
  for (; lit != lit_end; lit++) {
    const TWayPointVec& waypoints = (*lit).second->wayPoints();
    if ((*lit).second->laneType() == Lane::bike_lane) {
      glColor4f(1, 0, 0, blend);
    }
    else {
      glColor4f(0, 0, 1, blend);
    }
    TWayPointVec::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
    glBegin(GL_LINE_STRIP);
    for (; wpit != wpit_end; wpit++) {
      glVertex2f((*wpit)->utmX() - origin_x, (*wpit)->utmY() - origin_y);
    }
    glEnd();
    glPushMatrix();
    glTranslatef(0, 0, 0.03);
    for (uint32_t k = 0; k < waypoints.size() - 1; k++) {
      WayPoint* w = waypoints[k], *w_next = waypoints[k + 1];
      angle = atan2(w_next->utmY() - w->utmY(), w_next->utmX() - w->utmX());
      RoadNetworkGL::drawArrowHead(w_next->utmX() - origin_x, w_next->utmY() - origin_y, angle);
    }
    glPopMatrix();
  }

  // draw road markings
  if (draw_boundaries) {
    draw_road_boundaries(rn, origin_x, origin_y, blend);
  }

  // draw the zones
  const TZoneMap& zones = rn.zones();
  TZoneMap::const_iterator zit = zones.begin(), zit_end = zones.end();
  for (; zit != zit_end; zit++) {
    Zone* z = (*zit).second;
    const TPerimeterMap& perimeters = z->perimeters();
    TPerimeterMap::const_iterator pit = perimeters.begin(), pit_end = perimeters.end();
    for (; pit != pit_end; pit++) {
      Perimeter* p = (*pit).second;
      const TPerimeterPointVec& perpoints = p->perimeterPoints();
      TPerimeterPointVec::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
      glColor4f(0, 1, 0, blend);
      glBegin(GL_LINE_LOOP);
      for (; ppit != ppit_end; ppit++) {
        glVertex2f((*ppit)->utmX() - origin_x, (*ppit)->utmY() - origin_y);
      }
    }
  }

  /* draw the waypoints in yellow */
  glPushMatrix();
  glColor4f(1, 1, 0, blend);
  glTranslatef(0, 0, -0.005);
  const TWayPointMap& waypoints = rn.wayPoints();
  TWayPointMap::const_iterator wpit = waypoints.begin(), wpit_end = waypoints.end();
  for (; wpit != wpit_end; wpit++) {
    WayPoint* w = (*wpit).second;
    draw_diamond(w->utmX() - origin_x, w->utmY() - origin_y, 1.0);
  }

  const TPerimeterPointMap& perpoints = rn.perimeterPoints();
  TPerimeterPointMap::const_iterator ppit = perpoints.begin(), ppit_end = perpoints.end();
  for (; ppit != ppit_end; ppit++) {
    PerimeterPoint* pp = (*ppit).second;
    draw_diamond(pp->utmX() - origin_x, pp->utmY() - origin_y, 1.0);
  }
  glPopMatrix();

  // draw the zone spots
  const TSpotMap& spots = rn.spots();
  TSpotMap::const_iterator sit = spots.begin(), sit_end = spots.end();
  glBegin(GL_LINES);
  glColor4f(0, 1, 0, blend);
  for (; sit != sit_end; sit++) {
    Spot* s = (*sit).second;
    WayPoint* w0 = s->wayPoints()[0], *w1 = s->wayPoints()[1];
    glVertex2f(w0->utmX() - origin_x, w0->utmY() - origin_y);
    glVertex2f(w1->utmX() - origin_x, w1->utmY() - origin_y);
  }
  glEnd();
  for (sit = spots.begin(); sit != sit_end; sit++) {
    Spot* s = (*sit).second;
    WayPoint* w0 = s->wayPoints()[0], *w1 = s->wayPoints()[1];
    draw_diamond(w0->utmX() - origin_x, w0->utmY() - origin_y, 1.0);
    draw_diamond(w1->utmX() - origin_x, w1->utmY() - origin_y, 1.0);
  }

  // draw exits
  const TExitMap& exits = rn.exits();
  TExitMap::const_iterator eit = exits.begin(), eit_end = exits.end();
  for (; eit != eit_end; eit++) {
    rndf::Exit* e = (*eit).second;
    double x0, y0, x1, y1;
    if (e->exitType() == rndf::Exit::LaneToLane || e->exitType() == rndf::Exit::LaneToPerimeter) {
      x0 = e->getExitFromLane()->utmX();
      y0 = e->getExitFromLane()->utmY();
    }
    else {
      x0 = e->getExitFromPerimeter()->utmX();
      y0 = e->getExitFromPerimeter()->utmY();
    }
    if (e->exitType() == rndf::Exit::LaneToLane || e->exitType() == rndf::Exit::PerimeterToLane) {
      x1 = e->getExitToLane()->utmX();
      y1 = e->getExitToLane()->utmY();
    }
    else {
      x1 = e->getExitToPerimeter()->utmX();
      y1 = e->getExitToPerimeter()->utmY();
    }

    glColor4f(1, 0.5, 0, blend);
    draw_arrow(x0 - origin_x, y0 - origin_y, x1 - origin_x, y1 - origin_y, 0.5, 2.0);
  }

  if (draw_lane_links) {
    draw_lanechange_links(rn, origin_x, origin_y, blend);
  }

  //  for (l = 0; l < w->num_exits(uturn); l++) {
  //    if (w->exit_original(l, uturn)) {
  //      glColor4f(0, 1, 0, blend);
  //      draw_arrow(w->utmX() - origin_x, w->utmY() - origin_y, w->Exit(l, uturn)->utmX() - origin_x,
  //          w->Exit(l, uturn)->utmY() - origin_y, 0.5, 2.0);
  //    }
  //  }

  if (draw_lights == 1) {
    draw_rndf_lights(rn, threeD_signs, origin_x, origin_y, blend, true);
  }
  if (draw_lights == 2) {
    draw_rndf_lights(rn, threeD_signs, origin_x, origin_y, blend, false);
  }

  if (draw_stops) {
    draw_signs(rn, threeD_signs, origin_x, origin_y, blend);
  }
  if (draw_crosswalks) {
    draw_rndf_crosswalks(rn, threeD_signs, origin_x, origin_y, blend);
  }

  if (draw_numbers) {
    glColor4f(1, 1, 1, blend);
    draw_rndf_numbers(rn, origin_x, origin_y);
  }

  /* draw the checkpoints */
  const TCheckPointMap& checkpoints = rn.checkPoints();
  TCheckPointMap::const_iterator cpit = checkpoints.begin(), cpit_end = checkpoints.end();
  for (; cpit != cpit_end; cpit++) {
    WayPoint* w = (*cpit).second->wayPoint();
    draw_checkpoint(w->utmX() - origin_x, w->utmY() - origin_y, 3.0, w->name(), blend);
  }
}

//void RoadNetworkGL::draw_intersection_links(const RoadNetwork& rn, double origin_x,
//           double origin_y, double blend)
//{
//  int i, j, k, l;
//  wayPoint *w;
//
//  glColor4f(1, 1, 1, blend);
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//
//  if(w->allway_stop())
//    draw_circle(w->utmX() - origin_x, w->utmY() - origin_y, 1.0);
//
//  for(l = 0; l < w->num_intersection_waypoints(); l++)
//    draw_arrow(w->utmX() - origin_x,
//         w->utmY() - origin_y,
//         w->intersection_waypoint(l)->utmX() - origin_x,
//         w->intersection_waypoint(l)->utmY() - origin_y,
//         0.25, 1.0);
//      }
//}
//
//void RoadNetworkGL::draw_yieldto_links(const RoadNetwork& rn, double origin_x, double origin_y,
//      double blend)
//{
//  int i, j, k, l, l2;
//  wayPoint *w, *w2 = NULL, *w3;
//  yieldto_data ydata;
//
//  glColor4f(0, 1, 1, blend);
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_original_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->original_waypoint(k);
//  if(k < rndf->segment(i)->Lane(j)->num_original_waypoints() - 1)
//    w2 = rndf->segment(i)->Lane(j)->original_waypoint(k + 1);
//  if(1||w->yield()) {
//    if(k < rndf->segment(i)->Lane(j)->num_original_waypoints() - 1)
//      for(l = 0; l < w->num_yieldtos(); l++) {
//        ydata = w->yieldto(l);
//        w3 = ydata.waypoint;
//
//        if(ydata.exit_num == 0)
//    draw_arrow((w->utmX() + w2->utmX()) * 0.5 - origin_x,
//         (w->utmY() + w2->utmY()) * 0.5 - origin_y,
//         w3->utmX() - origin_x, w3->utmY() - origin_y,
//         0.15, 0.3);
//        else
//    draw_arrow((w->utmX() + w2->utmX()) * 0.5 - origin_x,
//         (w->utmY() + w2->utmY()) * 0.5 - origin_y,
//         (w3->utmX() + w3->Exit(ydata.exit_num - 1)->utmX()) * 0.5 - origin_x,
//         (w3->utmY() + w3->Exit(ydata.exit_num - 1)->utmY()) * 0.5 - origin_y,
//         0.15, 0.3);
//
//      }
//  }
//      }
//
//  for(i = 0; i < rndf->num_segments(); i++)
//    for(j = 0; j < rndf->segment(i)->num_lanes(); j++)
//      for(k = 0; k < rndf->segment(i)->Lane(j)->num_waypoints(); k++) {
//        w = rndf->segment(i)->Lane(j)->waypoint(k);
//        for(l = 0; l < w->num_exits(); l++)
//    if(1||w->yield_exit(l))
//      for(l2 = 0; l2 < w->num_exit_yieldtos(l); l2++) {
//        ydata = w->exit_yieldto(l, l2);
//        w3 = ydata.waypoint;
//
//        if(ydata.exit_num == 0)
//    draw_arrow((w->utmX() + w->Exit(l)->utmX()) * 0.5 -
//         origin_x,
//         (w->utmY() + w->Exit(l)->utmY()) * 0.5 -
//         origin_y, w3->utmX() - origin_x,
//         w3->utmY() - origin_y, 0.15, 0.3);
//        else
//    draw_arrow((w->utmX() + w->Exit(l)->utmX()) * 0.5 - origin_x,
//         (w->utmY() + w->Exit(l)->utmY()) * 0.5 - origin_y,
//         (w3->utmX() + w3->Exit(ydata.exit_num - 1)->utmX()) * 0.5 - origin_x,
//         (w3->utmY() + w3->Exit(ydata.exit_num - 1)->utmY()) * 0.5 - origin_y,
//         0.15, 0.3);
//      }
//      }
//
//  for(i = 0; i < rndf->num_zones(); i++)
//    for(j = 0; j < rndf->zone(i)->num_perimeter_points(); j++) {
//      w = rndf->zone(i)->perimeter(j);
//      for(k = 0; k < w->num_exits(); k++)
//  if(1||w->yield_exit(k))
//    for(l2 = 0; l2 < w->num_exit_yieldtos(k); l2++) {
//      ydata = w->exit_yieldto(k, l2);
//      w3 = ydata.waypoint;
//
//      if(ydata.exit_num == 0)
//        draw_arrow((w->utmX() + w->Exit(k)->utmX()) * 0.5 - origin_x,
//       (w->utmY() + w->Exit(k)->utmY()) * 0.5 - origin_y,
//       w3->utmX() - origin_x, w3->utmY() - origin_y,
//       0.15, 0.3);
//      else
//        draw_arrow((w->utmX() + w->Exit(k)->utmX()) * 0.5 - origin_x,
//       (w->utmY() + w->Exit(k)->utmY()) * 0.5 - origin_y,
//       (w3->utmX() + w3->Exit(ydata.exit_num - 1)->utmX()) * 0.5 - origin_x,
//       (w3->utmY() + w3->Exit(ydata.exit_num - 1)->utmY()) * 0.5 - origin_y,
//       0.15, 0.3);
//
//    }
//    }
//
//}

void RoadNetworkGL::generate_rndf_display_list(const rndf::RoadNetwork& rn, double blend, bool dynamic) {

  display_list_.origin_x = rn.wayPoints().begin()->second->utmX();
  display_list_.origin_y = rn.wayPoints().begin()->second->utmY();

  display_list_.rndf_dl = glGenLists(1);
  glNewList(display_list_.rndf_dl, GL_COMPILE);
  draw_rndf(rn, false, false, true, false, false, false, true, dynamic ? 2 : 1, display_list_.origin_x, display_list_.origin_y, blend);
  glEndList();

  display_list_.numbers_dl = glGenLists(1);
  glNewList(display_list_.numbers_dl, GL_COMPILE);
  glColor4f(1, 1, 1, blend);
  draw_rndf_numbers(rn, display_list_.origin_x, display_list_.origin_y);
  glEndList();

  display_list_.threeD_stops_dl = glGenLists(1);
  glNewList(display_list_.threeD_stops_dl, GL_COMPILE);
  draw_signs(rn, 1, display_list_.origin_x, display_list_.origin_y, blend);
  glEndList();

  display_list_.flat_stops_dl = glGenLists(1);
  glNewList(display_list_.flat_stops_dl, GL_COMPILE);
  draw_signs(rn, 0, display_list_.origin_x, display_list_.origin_y, blend);
  glEndList();

  display_list_.boundaries_dl = glGenLists(1);
  glNewList(display_list_.boundaries_dl, GL_COMPILE);
  draw_road_boundaries(rn, display_list_.origin_x, display_list_.origin_y, blend);
  glEndList();

  display_list_.lanelinks_dl = glGenLists(1);
  glNewList(display_list_.lanelinks_dl, GL_COMPILE);
  draw_lanechange_links(rn, display_list_.origin_x, display_list_.origin_y, blend);
  glEndList();
}

void RoadNetworkGL::delete_rndf_display_list() {
  glDeleteLists(display_list_.rndf_dl, 1);
  glDeleteLists(display_list_.numbers_dl, 1);
  glDeleteLists(display_list_.threeD_stops_dl, 1);
  glDeleteLists(display_list_.flat_stops_dl, 1);
  glDeleteLists(display_list_.boundaries_dl, 1);
  glDeleteLists(display_list_.lanelinks_dl, 1);
}

void RoadNetworkGL::draw_rndf_display_list(bool draw_numbers, bool draw_stops, bool threeD_signs, bool draw_boundaries, bool draw_lane_links,
    double origin_x, double origin_y) {
  glPushMatrix();
  glTranslatef(display_list_.origin_x - origin_x, display_list_.origin_y - origin_y, 0);
  glCallList(display_list_.rndf_dl);
  if (draw_numbers) glCallList(display_list_.numbers_dl);
  if (draw_stops) {
    if (threeD_signs) glCallList(display_list_.threeD_stops_dl);
    else glCallList(display_list_.flat_stops_dl);
  }
  if (draw_boundaries) glCallList(display_list_.boundaries_dl);
  if (draw_lane_links) glCallList(display_list_.lanelinks_dl);
  glPopMatrix();
}

void RoadNetworkGL::draw_traffic_light_3D(double x, double y, double z, double orientation, double blend) {
  glPushMatrix();

  glTranslatef(x, y, z);

  /* draw stick */
  if (z != 0) {
    glColor4f(0, 0, 0, blend);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, -z);
    glEnd();
  }

  glRotatef(dgc_r2d(orientation), 0., 0., 1.);
  glTranslatef(.1, 0., 0.);

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_NORMALIZE);

  draw_light_base();

  glPopMatrix();
}

void RoadNetworkGL::draw_trafficlight_state(const lightState state, const double& x, const double& y, const double& z, const double& orientation) {
  static bool have_dl = false;

  if (!have_dl) {
    light_red_dl = glGenLists(1);
    glNewList(light_red_dl, GL_COMPILE);
    draw_light_state_3D(true, false, false, 0, 0, 0, 0., .8);
    glEndList();

    light_yellow_dl = glGenLists(1);
    glNewList(light_yellow_dl, GL_COMPILE);
    draw_light_state_3D(false, true, false, 0, 0, 0, 0., .8);
    glEndList();

    light_green_dl = glGenLists(1);
    glNewList(light_green_dl, GL_COMPILE);
    draw_light_state_3D(false, false, true, 0, 0, 0, 0., .8);
    glEndList();

    light_unknown_dl = glGenLists(1);
    glNewList(light_unknown_dl, GL_COMPILE);
    draw_light_state_3D(false, false, false, 0, 0, 0, 0., .8);
    glEndList();

    have_dl = true;
  }

  switch (state) {
  case LIGHT_STATE_RED:
    draw_light_state_2D(true, false, false, x, y, z, 0., .8);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_red_dl);
    glPopMatrix();
    break;
  case LIGHT_STATE_YELLOW:
    draw_light_state_2D(false, true, false, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_yellow_dl);
    break;
  case LIGHT_STATE_GREEN:
    draw_light_state_2D(false, false, true, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_green_dl);
    break;
  case LIGHT_STATE_UNKNOWN:
    draw_light_state_2D(false, false, false, x, y, z, 0., .8);
    glTranslatef(x, y, z);
    glRotatef(dgc_r2d(orientation), 0., 0., 1.);
    glCallList(light_unknown_dl);
    break;
  }

}

} // namespace rndf
} // namespace vlr

