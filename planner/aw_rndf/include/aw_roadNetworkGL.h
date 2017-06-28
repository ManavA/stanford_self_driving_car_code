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


#ifndef AW_ROADNETWORKGL_H_
#define AW_ROADNETWORKGL_H_

#include <istream>
#include <stdint.h>

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <textures.h>
#include <lltransform.h>

#include <aw_CGAL.h>

namespace vlr {

namespace rndf {

typedef struct {
  double origin_x, origin_y;
  GLint rndf_dl, numbers_dl, threeD_stops_dl, flat_stops_dl, boundaries_dl;
  GLint lanelinks_dl;
} rndf_display_list;

class RoadNetworkGL {
public:
    RoadNetworkGL();
//    RoadNetworkGL(const std::string& strName);
//    RoadNetworkGL(const RoadNetworkGL& other);
    virtual ~RoadNetworkGL(void);

//  RoadNetworkGL& operator=(const RoadNetworkGL& other);
//  RoadNetworkGL& copy(const RoadNetworkGL& other);
//  virtual RoadNetworkGL* clone() const;

  void draw(double center_x, double center_y, double blend, bool wp_labels) const;
  void drawExtras(double center_x, double center_y, double blend) const;

//private:
  void draw_stop_sign_poly(double x, double y, double r);

  void draw_stop_sign_3D(double x, double y, double z, double r, double heading, double blend);

  void draw_stop_sign_2D(double x, double y, double r, double heading, double blend);

  void draw_yield_sign_2D(double x, double y, double r, double heading,
        double blend);

  void draw_merge_sign_2D(double x, double y, double r, double heading,
        double blend);

  void draw_checkpoint(double x, double y, double r, const std::string& name, double blend);

  //for backwards compatibility
  void draw_rndf(const rndf::RoadNetwork& rn, int draw_numbers, int draw_stops,
           int threeD_signs, int draw_boundaries, int draw_lane_links,
           int draw_merges, double origin_x, double origin_y,
           double blend);

  void draw_rndf(const rndf::RoadNetwork& rn, int draw_numbers, int draw_stops,
           int threeD_signs, int draw_boundaries, int draw_lane_links,
           int draw_merges, int draw_crosswalks, int draw_lights, double origin_x, double origin_y,
           double blend);

  void draw_intersection_links(const rndf::RoadNetwork& rn, double origin_x,
             double origin_y, double blend);

  void draw_yieldto_links(const rndf::RoadNetwork& rn, double origin_x, double origin_y,
        double blend);

  void generate_rndf_display_list(const rndf::RoadNetwork& rn, double blend, bool dynamic);

  void delete_rndf_display_list();

  void draw_rndf_display_list(bool draw_numbers, bool draw_stops, bool threeD_signs, bool draw_boundaries, bool draw_lane_links,
                              double origin_x, double origin_y);

  void draw_traffic_light(double x, double y, double z, double orientation, double blend);

  void draw_trafficlight_state ( const rndf::lightState state, const double& x, const double& y, const double& z, const double& orientation);

  //! draws a checkpoint
  void draw_checkpoint(double x, double y, double r, int num, double blend);
  //! draws a stoppoint
  void draw_stoppoint(double x, double y, double r, double blend);

  //! draw the Lane boundary
  void draw_lane_boundary(Lane* l, double origin_x, double origin_y, int left, double blend);

  //! draw Lane connections
  void draw_lane_connections(const rndf::Lane& l1, double center_x, double center_y, double blend);

  void draw_lane_background(double x, double y, double theta, double w, double l);
  void draw_lane_background(CGAL_Geometry::Point_2 p1, CGAL_Geometry::Point_2 p2, CGAL_Geometry::Vector_2 dir1, CGAL_Geometry::Vector_2 dir2, double w1, double w2);

  void get_exitpoint_params(Exit* e, CGAL_Geometry::Point_2& point_from, CGAL_Geometry::Point_2& point_to, CGAL_Geometry::Vector_2& dir_from, CGAL_Geometry::Vector_2& dir_to, double& width);

  void draw_kturns(const LaneSegment* l, double center_x, double center_y, double blend);
  void draw_TypesLaneSegment(const Lane& lane1, double center_x, double center_y, double blend);

    //! draw intersections
  void draw_intersection(const Intersection& i, double center_x, double center_y, double blend);
  std::vector<CGAL_Geometry::Point_2> calc_intermediate_points(const CGAL_Geometry::Point_2 point_from, const CGAL_Geometry::Point_2 point_to, const CGAL_Geometry::Vector_2 dir_from,
      const CGAL_Geometry::Vector_2 dir_to);

  void draw_trafficlight_sign_3D(double x, double y, double z, double r, double heading, double blend);
  void draw_trafficlight_sign_2D(double x, double y, double r, double heading, double blend);
  void draw_traffic_light_2D(double x, double y, double z, double blend);
  void draw_traffic_light_3D(double x, double y, double z, double orientation, double blend);
  void draw_light_state_2D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& /*z*/,
                           const double& /*orientation*/, const double& blend);
  void draw_light_state_3D(const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& z, const double& orientation,
                           const double& /*blend*/);
  void draw_light_state(const bool& threeD, const bool& rd, const bool& yl, const bool& gr, const double& x, const double& y, const double& z,
                        const double& orientation, const double& blend);

  void draw_crosswalk_sign_3D(double x, double y, double z, double r, double heading, double blend, bool stop_waypoint = true);
  void draw_crosswalk_sign_2D(double x, double y, double r, double heading, double blend, bool stop_waypoint = true);
  void draw_crosswalk(Crosswalk *crosswalk, double origin_x, double origin_y, double blend);

  void draw_rndf_numbers(const RoadNetwork& rn, double origin_x, double origin_y);
  void draw_signs(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend);
  void draw_rndf_lights(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend, bool draw_state = false);
  void draw_rndf_crosswalks(const RoadNetwork& rn, int threeD, double origin_x, double origin_y, double blend);
  void draw_lanechange_links(const RoadNetwork& /*rn*/, double /*origin_x*/, double /*origin_y*/, double /*blend*/);
  void draw_boundary(Lane* l, double origin_x, double origin_y, int left, double blend);
  void draw_road_boundaries(const RoadNetwork& rn, double origin_x, double origin_y, double blend);

private:
  inline void drawArrowHead(double x, double y, double angle) {
    double ct, st, l = 2, l2 = 0.5;

    ct = cos(angle);
    st = sin(angle);
    glBegin(GL_POLYGON);
    glVertex2f(x, y);
    glVertex2f(x - l * ct + l2 * st, y - l * st - l2 * ct);
    glVertex2f(x - l * ct - l2 * st, y - l * st + l2 * ct);
    glEnd();
  }

private:
  Texture stop_sign1_, stop_sign2_;
  Texture yield_sign_;
  Texture merge_sign_;

  Texture traffic_light_sign_;
  Texture traffic_light_;
  Texture crosswalk_stop_sign_, crosswalk_incoming_sign_;

  GLint light_red_dl, light_yellow_dl, light_green_dl, light_unknown_dl;
  rndf_display_list display_list_;
  bool display_list_genererated_;

};

} // namespace rndf

} // namespace vlr

#endif
