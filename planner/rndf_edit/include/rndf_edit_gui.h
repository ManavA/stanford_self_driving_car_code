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


#ifndef RNDFEDITGUI_H
#define RNDFEDITGUI_H

#include <iostream>
#include <iomanip>
#include <string>

#include <QtGui/QtGui>
#include <qwt_plot_curve.h>

#include <aw_roadNetwork.h>
#include <aw_roadNetworkSearch.h>
#include <driving_common/Trajectory2D.h>
#include <poly_traj.h> // for CurvePoint
#include <curveSmoother.h>
#include <imagery.h>
#include <transform.h>

#include <ui_rndf_edit_qtgui.h>

#include <REWayPoint.h>
#include <RELane.h>
#include <RESegment.h>
#include <REZone.h>
#include <REPerimeter.h>
#include <REPerimeterPoint.h>
#include <RESpot.h>
#include <RESpotPoint.h>
#include <RETrafficLight.h>
#include <RECrosswalk.h>

namespace vlr {

enum RndfElements {
    RNDF_ELEMENT_SEGMENT = 0,
    RNDF_ELEMENT_LANE,
    RNDF_ELEMENT_WAYPOINT,
    RNDF_ELEMENT_EXIT,
    RNDF_ELEMENT_ZONE,
    RNDF_ELEMENT_PERIMETER,
    RNDF_ELEMENT_PERIMETERPOINT,
    RNDF_ELEMENT_SPOT,
    RNDF_ELEMENT_SPOTPOINT,
    RNDF_ELEMENT_TRAFFIC_LIGHT,
    RNDF_ELEMENT_CROSSWALK
};

class RNDFEditGUI: public QMainWindow {
Q_OBJECT

public:
    class xy {
    public:
        xy(double x_, double y_) :
            x(x_), y(y_) {
        }
        double x, y;
    };

private:
    class distElement {

    public:
        distElement() :
            index(-1), left(0), right(0), distance(0) {
        }

        uint32_t index;
        distElement* left, *right;
        double distance;
    };

    class distElemCompare {
    public:
        bool operator()(const distElement* x, const distElement* y) const {
            if (x->distance == y->distance) {
                return x->index < y->index;
            }
            return x->distance < y->distance;
        }
    };

    class distElemIdxCompare {
    public:
        bool operator()(const distElement* x, const distElement* y) const {
            return x->index < y->index;
        }
    };

public:
    RNDFEditGUI(std::string& rndf_filename_, std::string& imagery_folder, int imageryZoomLevel, int imageryGridSizeX,
            int imageryGridSizeY, QWidget *parent = 0);
    ~RNDFEditGUI();

    bool currentElementPosition(double& utm_x, double& utm_y, std::string& utm_zone);

    void addElement(double utm_x, double utm_y, const std::string& utm_zone);
    void removeElement();
    void copyElement();
    void moveElement(double utm_x, double utm_y, const std::string& utm_zone);
    void rotateElement(double utm_x, double utm_y, const std::string& utm_zone);
    void selectElements(vlr::rndf::WayPoint* waypoint);
    void selectElements(vlr::rndf::PerimeterPoint* Perimeterpoint);
    void selectElements(vlr::rndf::TrafficLight* traffic_light);
    void selectElements(vlr::rndf::Crosswalk* crosswalk);
    void selectElements(double utm_x, double utm_y);
    void deselectAllElements();

    void addExit();

    inline bool showImagery() const {return show_imagery_;}
    inline void showImagery(bool show) {show_imagery_ = show;} // will be replaced by qt control function
    void updateSmoothedLaneStack();

private slots:
    void on_action_Open_RNDF_activated();
    void on_action_Save_RNDF_activated();
    void on_action_Save_RNDF_As_activated();

    void on_action_Segment_toggled(bool checked);
    void on_action_Lane_toggled(bool checked);
    void on_action_WayPoint_toggled(bool checked);
    void on_action_Exit_toggled(bool checked);
    void on_action_Zone_toggled(bool checked);
    void on_action_Perimeter_toggled(bool checked);
    void on_action_PerimeterPoint_toggled(bool checked);
    void on_action_Spot_toggled(bool checked);
    void on_action_SpotPoint_toggled(bool checked);

    void on_action_Traffic_Light_toggled(bool checked);
    void on_action_Crosswalk_toggled(bool checked);

    void on_action_Open_Trajectory_activated();
    void on_action_Trajectory2Lane_activated();
    void on_action_ReverseTrajectory_activated();
    void on_showImagery_stateChanged(int state);

    void on_action_Undo_activated();
    void on_actionFindWay_Point_activated();

private:
    void centerWindow();
    bool loadRNDF(std::string fileName);

    void updateGUI();
    void sampleRawLaneLine(const rndf::Exit& entry, const rndf::Exit& exit, std::vector<CurvePoint>& raw_line);
    void extendRawLaneLine(const std::vector<CurvePoint>& raw_line, rndf::Exit& entry, rndf::Exit& exit,
                                        int32_t num_samples, std::vector<CurvePoint>& extended_line,
                                        std::vector<CurvePoint>::const_iterator& original_start,
                                        std::vector<CurvePoint>::const_iterator& original_end);

    void smoothLane(rndf::Exit& entry, rndf::Exit& exit);
    void optimizeVirtualPoints();
    void optimizeVirtualPoint(std::vector<CurvePoint>& lane_points, size_t idx);
    double squaredCurvatureSum(std::vector<CurvePoint>& raw_center_line);

    double calcPointDeviation(xy& p, xy& l, xy& r);

private:
    class PlotData : public QwtData {
     public:
      PlotData(std::vector<CurvePoint>& cps);
      virtual ~PlotData();

      void update();
      PlotData* copy() const;
      size_t size() const;
      double x(size_t i) const;
      double y(size_t i) const;
      QwtDoubleRect boundingRect() const;

     private:
      std::vector<CurvePoint>& cps_;
      QwtDoubleRect bounding_box_;
    };

    enum {
        TAB_STREET = 0, TAB_ZONE, TAB_SPOT, TAB_TRAFFIC_LIGHT, TAB_CROSSWALK
    };

    CurveSmoother smoother_;

public:
    Ui::RNDFEdit ui;

public:
    std::string rndf_filename_;
    std::string imagery_folder_;
    REWayPoint* re_wp_;
    RELane* re_lane_;
    RESegment* re_segment_;
    REPerimeterPoint* re_pp_;
    REPerimeter* re_perimeter_;
    RESpotPoint* re_sp_;
    RESpot* re_spot_;
    REZone*  re_zone_;
    RETrafficLight* re_tl_;
    RECrosswalk* re_crosswalk_;

    // rndf
    rndf::RoadNetwork* rn_;
    rndf::RoadNetworkSearch* rn_search_;

    // old rndf for undo
    rndf::RoadNetwork* last_rn_;
    rndf::RoadNetworkSearch* last_rn_search_;

    //double refLat, refLon;
    coordinate_utm_t rndf_center;

    std::vector<xy> trajectory;
    std::vector<std::vector<CurvePoint> > smoothed_lane_stack_;
    std::vector<PlotData*> plot_data_;
    std::vector<QwtPlotCurve*> curvature_plots_;

    int editMode;

    int current_element;
    int last_mouse_x, last_mouse_y;
    double last_utm_x, last_utm_y;
    double last_move_utm_x, last_move_utm_y;
    double last_theta;
    bool gotFromPoint;
    bool show_imagery_;
    bool clothoid_smoothing_;
};

} // namespace vlr

#endif // RNDFEDITGUI_H
