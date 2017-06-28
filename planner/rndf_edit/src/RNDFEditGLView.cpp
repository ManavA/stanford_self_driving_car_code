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
#include <aw_roadNetwork.h>
#include <lltransform.h>
#include <passat_constants.h>
#include <imagery.h>

#include "RNDFEditGLView.h"
#include "rndf_edit_gui.h"

namespace drc = driving_common;

namespace vlr {

extern RNDFEditGUI* gui;

RNDFEditGLView::RNDFEditGLView(QWidget* parent) :
  gridMapTexture(0), tex_type(GL_TEXTURE_RECTANGLE_ARB), imagery_(NULL) {
  //setFrameRate(30.0);
  setInitialCameraPos(180.0, 89.99, 200.0, 0, 0, 0);
  setCameraParams(0.01, 0.3, 0.001, 0.009, 60, 0.4, 200000);

  setMouseTracking(true);
}

RNDFEditGLView::~RNDFEditGLView()
{
//extern void Delete_FrameBufferObject(void);
//
//Delete_FrameBufferObject();
if(imagery_) {delete imagery_;}
}

void RNDFEditGLView::mousePressEvent(QMouseEvent* event) {
  double x2, y2, utm_x, utm_y;

  pickPoint(event->x(), event->y(), &x2, &y2);

  utm_x = x2 + gui->rndf_center.x;
  utm_y = y2 + gui->rndf_center.y;
  gui->last_utm_x = utm_x;
  gui->last_utm_y = utm_y;
  gui->last_move_utm_x = utm_x;
  gui->last_move_utm_y = utm_y;

  // update imagery
  //utmToLatLong(utm_x, utm_y, gui->rndf_center.zone, &gui->imagery_lat, &gui->imagery_lon);
  switch (event->modifiers()) {
    case Qt::ControlModifier: // selected object is affected
      if (event->buttons() & Qt::LeftButton) {
        if (gui->current_element == RNDF_ELEMENT_EXIT) {
          // if second point is clicked, connect both and add exit to rndf
          if (gui->gotFromPoint) {
            gui->addExit();
            gui->gotFromPoint = false;
          }
          else {
            gui->gotFromPoint = true;
            gui->selectElements(utm_x, utm_y);
          }
        }
        else {
          gui->selectElements(utm_x, utm_y);
        }
      }
      else if (event->buttons() & Qt::RightButton) {
        // go into rotate mode
        double lat, lon;
        double center_x, center_y;
        char utm_zone[4];

        switch (gui->current_element) {
          case RNDF_ELEMENT_LANE:
            if (!gui->re_lane_->current()) {
              return;
            }
            gui->re_lane_->current()->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_SEGMENT:
            if (!gui->re_segment_->current()) {
              return;
            }
            gui->re_segment_->current()->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_PERIMETER:
            if (!gui->re_perimeter_->current()) {
              return;
            }
            gui->re_perimeter_->current()->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_ZONE:
            if (!gui->re_zone_->current()) {
              return;
            }
            gui->re_zone_->current()->centerLatLon(lat, lon);
            break;

          case RNDF_ELEMENT_SPOT:
            if (!gui->re_spot_->current()) {
              return;
            }
            gui->re_spot_->current()->centerLatLon(lat, lon);
            break;

          default:
            return;
        }

        latLongToUtm(lat, lon, &center_x, &center_y, utm_zone);
        gui->last_theta = atan2(utm_y - center_y, utm_x - center_x);
      }

      requestRedraw();
      break;

    default: // let base class handle other cases
      GLWidget::mousePressEvent(event);
      break;
  }
}

void RNDFEditGLView::mouseReleaseEvent(QMouseEvent* event)
{
//		// go leave exit mode
//		gui3D_set_mode(GUI_MODE_3D);
}

void RNDFEditGLView::mouseMoveEvent(QMouseEvent *event) {
  double x2, y2;
  pickPoint(event->x(), event->y(), &x2, &y2);
  double utm_x = x2 + gui->rndf_center.x;
  double utm_y = y2 + gui->rndf_center.y;
  std::string utm_zone = gui->rndf_center.zone;
  gui->last_utm_x = utm_x;
  gui->last_utm_y = utm_y;

  switch (event->modifiers()) {
    case Qt::ControlModifier: // selected object is affected
        // update undo buffer
      if(gui->last_rn_) {delete gui->last_rn_;}
      if(gui->last_rn_search_) {delete gui->last_rn_search_;}
      gui->last_rn_ = new rndf::RoadNetwork(*gui->rn_);
      gui->last_rn_search_ = new rndf::RoadNetworkSearch(*gui->rn_search_, gui->last_rn_);
      if (event->buttons().testFlag(Qt::LeftButton)) {
        gui->moveElement(utm_x, utm_y, utm_zone);
      }
      else if (event->buttons() & Qt::RightButton) {
        gui->rotateElement(utm_x, utm_y, utm_zone);
     }

      gui->updateSmoothedLaneStack();
      requestRedraw();
      break;

    case Qt::NoModifier:
      gui->last_mouse_x = event->x();
      gui->last_mouse_y = event->y();
      if (gui->current_element == RNDF_ELEMENT_EXIT) {
        requestRedraw();
      }
      else {
        GLWidget::mouseMoveEvent(event);
      }

    default: // let base class handle other cases
      GLWidget::mouseMoveEvent(event);
      break;
  }

  gui->last_move_utm_x = utm_x;
  gui->last_move_utm_y = utm_y;
}

bool RNDFEditGLView::wayPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
    case 'C':
      std::cout << "(toggle Checkpoint)"<< std::endl;
      gui->ui.wpCheckPoint->toggle();
      return true;

    case 'S':
      std::cout << "(toggle Stoppoint)"<< std::endl;
      gui->ui.wpStopPoint->toggle();
      return true;
  }

  return false;
}

bool RNDFEditGLView::laneKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch (key) {
    case 'W':
      if (modifiers & Qt::ShiftModifier) {
        std::cout << "(increase lane width)" << std::endl;
        gui->ui.laneWidth->stepUp();
      }
      else {
        std::cout << "(decrease lane width)" << std::endl;
        gui->ui.laneWidth->stepDown();
      }
      return true;

    case 'S':
      gui->clothoid_smoothing_ = !gui->clothoid_smoothing_;
      return true;
  }

  return false;
}

bool RNDFEditGLView::segmentKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::perimeterPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::perimeterKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::exitKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
    case 27: // ESC
      gui->gotFromPoint=false;
      return true;

  }
  return false;
}

bool RNDFEditGLView::spotPointKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
    case 'C':
      std::cout << "(toggle Checkpoint)"<< std::endl;
      gui->ui.spCheckPoint->toggle();
      return true;
  }

  return false;
}

bool RNDFEditGLView::spotKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::zoneKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::trafficLightKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

bool RNDFEditGLView::crosswalkKeys(uint32_t key, Qt::KeyboardModifiers modifiers) {
  switch(key) {
  }
  return false;
}

// Parses keyboard commands
void RNDFEditGLView::keyPressEvent(QKeyEvent* event) {

  uint32_t key=(uint32_t)event->key();//(*(event->text().toAscii().constData()));

  double utm_x, utm_y;
  std::string utm_zone;
  const double move_step = 2;

  if(event->modifiers() & Qt::ControlModifier) {  // all element specific keys require control modifier
    switch (event->key()) {
      case Qt::Key_Up:
        if(!gui->currentElementPosition(gui->last_move_utm_x, gui->last_move_utm_y, utm_zone)) {return;}
        printf("up: %f, %f -> %f, %f\n", gui->last_move_utm_x, gui->last_move_utm_y, gui->last_move_utm_x, gui->last_move_utm_y - move_step);
        gui->moveElement(gui->last_move_utm_x, gui->last_move_utm_y - move_step, utm_zone);
        requestRedraw();
        return;

      case Qt::Key_Down:
        if(!gui->currentElementPosition(gui->last_move_utm_x, gui->last_move_utm_y, utm_zone)) {return;}
        printf("down: %f, %f -> %f, %f\n", gui->last_move_utm_x, gui->last_move_utm_y, gui->last_move_utm_x, gui->last_move_utm_y + move_step);
        gui->moveElement(gui->last_move_utm_x, gui->last_move_utm_y + move_step, utm_zone);
        requestRedraw();
        return;

      case Qt::Key_Left:
        if(!gui->currentElementPosition(gui->last_move_utm_x, gui->last_move_utm_y, utm_zone)) {return;}
        printf("left: %f, %f -> %f, %f\n", gui->last_move_utm_x, gui->last_move_utm_y, gui->last_move_utm_x - move_step, gui->last_move_utm_y);
        gui->moveElement(gui->last_move_utm_x - move_step, gui->last_move_utm_y, utm_zone);
        requestRedraw();
        return;

      case Qt::Key_Right:
        if(!gui->currentElementPosition(gui->last_move_utm_x, gui->last_move_utm_y, utm_zone)) {return;}
        printf("right: %f, %f -> %f, %f\n", gui->last_move_utm_x, gui->last_move_utm_y, gui->last_move_utm_x + move_step, gui->last_move_utm_y);
        gui->moveElement(gui->last_move_utm_x + move_step, gui->last_move_utm_y, utm_zone);
        requestRedraw();
        return;
    }

    switch(gui->current_element) {
      case RNDF_ELEMENT_WAYPOINT:
        if(wayPointKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_LANE:
        if(laneKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_SEGMENT:
        if(segmentKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_EXIT:
        if(exitKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_PERIMETERPOINT:
        if(perimeterPointKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_PERIMETER:
        if(perimeterKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_SPOTPOINT:
        if(spotPointKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_SPOT:
        if(spotKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_ZONE:
        if(zoneKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_TRAFFIC_LIGHT:
        if(trafficLightKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;

      case RNDF_ELEMENT_CROSSWALK:
        if(crosswalkKeys(key, event->modifiers())) {requestRedraw(); return;}
        break;
      }
  }

  std::cout << "keyboard: "<< key << " (#"<< (int) key <<") ";

  bool key_handled = true;
  if (event->modifiers() & Qt::ControlModifier) { // all element specific keys require control modifier
    switch (key) {
      case 'A': {
        double x2, y2;
        std::cout << "(add element ";

        pickPoint(gui->last_mouse_x, gui->last_mouse_y, &x2, &y2);

        utm_x = x2 + gui->rndf_center.x;
        utm_y = y2 + gui->rndf_center.y;
        std::string utm_zone = gui->rndf_center.zone;

        gui->addElement(utm_x, utm_y, utm_zone);
      }
      break;

      case 'D':
      case 8:
      case 127:
        gui->removeElement();
        break;

      case 'K':
        gui->copyElement();
        break;

//      case 'F': {
//        printf("-1- %s\n", __PRETTY_FUNCTION__);
//        gui->ui.actionFindWay_Point->activate(QAction::Trigger);
//      }
//      break;

      default:
        key_handled = false;
    }
  }

  if (!key_handled) {
    switch (key) {
      case 'I':
        if (!(event->modifiers() & Qt::ShiftModifier)) {
          gui->showImagery(!gui->showImagery());
          std::cout << (gui->showImagery() ? "imagery: ON" : "imagery: OFF") << std::endl;
        }
        else {
          imagery_->cycleType();
          if (imagery_->currentType() == Imagery::NONE) {
            imagery_->cycleType();
          }
          switch (imagery_->currentType()) {
            case Imagery::COLOR:
              std::cout << "imagery type: COLOR" << std::endl;
              break;
            case Imagery::TOPO:
              std::cout << "imagery type: TOPO" << std::endl;
              break;
            case Imagery::LASER:
              std::cout << "imagery type: LASER" << std::endl;
              break;
            case Imagery::GSAT:
              std::cout << "imagery type: GOOGLE" << std::endl;
              break;
            case Imagery::DARPA:
              std::cout << "imagery type: DARPA" << std::endl;
              break;
            case Imagery::BW:
              std::cout << "imagery type: BW" << std::endl;
              break;
            default:
              std::cout << "imagery type: UNKNOWN" << std::endl;
              break;
          }
        }
        break;

      default:
        std::cout << "(no command)" << std::endl;
        break;
    }
  }

requestRedraw();
}

void RNDFEditGLView::initializeGL()
{
float light_ambient[] = {0, 0, 0, 0};
float light_diffuse[] = {1, 1, 1, 1};
float light_specular[] = {1, 1, 1, 1};
float light_position[] = {0, 0, 100, 0};

glEnable(GL_DEPTH_TEST);
glShadeModel(GL_SMOOTH);
glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
glLightfv(GL_LIGHT0, GL_POSITION, light_position);
glEnable(GL_LIGHT0);
glDisable(GL_LIGHTING);
glEnable(GL_NORMALIZE);

//glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
//glClearDepth(1.0f);									// Depth Buffer Setup
//glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
//glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

// initialize texture for grid map
glGenTextures(1, &gridMapTexture);

glBindTexture(tex_type, gridMapTexture);

glTexParameteri(tex_type, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(tex_type, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

try {
  gui->rn_->createVisualization();
}
catch(vlr::Ex<>& e) {
  std::cout << "Failed to create rndf visualization :" << e.what() << std::endl;
}

try {
  imagery_ = new Imagery(gui->imagery_folder_);
}
catch(vlr::Ex<>& e) {
  std::cout << "Failed to create/load imagery :" << e.what() << std::endl;
}

imagery_->setType(Imagery::COLOR);

connect(&timer, SIGNAL(timeout()), this, SLOT(redraw(void)));
timer.start(DISPLAY_REFRESH_DELAY_MS);
}

void RNDFEditGLView::paintGL()
{
glClearColor(0, 0, 0, 0);
glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

activate3DMode();

glDisable(GL_DEPTH_TEST);

glPushMatrix();

  // draw imagery
bool show_flat_imagery = true;
double current_time = drc::Time::current();
static double last_time = 0;

  if (gui->show_imagery_) {
    if (current_time - last_time > 0.05) {
      try {
        imagery_->update();
      }
      catch(vlr::Ex<>& e) {
        std::cout << "Failed to update imagery :" << e.what() << std::endl;
      }

      last_time = current_time;
    }
    glPushMatrix();
    {
  //    glTranslatef(0, 0, -DGC_PASSAT_HEIGHT);
      try {
        imagery_->draw3D(camera_pose.distance, camera_pose.x_offset,
          camera_pose.y_offset, gui->rndf_center.x, gui->rndf_center.y, gui->rndf_center.zone, show_flat_imagery, 1.0, 1);
      }
      catch(vlr::Ex<>& e) {
        std::cout << "Failed to draw imagery :" << e.what() << std::endl;
      }
   }
    glPopMatrix();
  }

// draw road network
gui->rn_->draw(gui->rndf_center.x, gui->rndf_center.y, 1, true);

// draw selected waypoint
drawSelected(gui->rndf_center.x, gui->rndf_center.y);

// draw coordinate frame
draw_coordinate_frame(2.0);

drawTrajectory(gui->rndf_center.x, gui->rndf_center.y);

glPopMatrix();

refresh_required=false;
}

void RNDFEditGLView::drawGrid(double center_x, double center_y)
{
int grid_x, grid_y;

glLineWidth(0.5);
glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

glEnable(GL_LINE_SMOOTH);
glColor3f(0.4, 0.4, 0.4);
glBegin(GL_LINES);
for (grid_x = -100; grid_x < 100; grid_x++)
	{
	glVertex3f(grid_x - center_x, -100 - center_y, 0);
	glVertex3f(grid_x - center_x, 100 - center_y, 0);
	}
for (grid_y = -100; grid_y < 100; grid_y++)
	{
	glVertex3f(-100 - center_x, grid_y - center_y, 0);
	glVertex3f( 100 - center_x, grid_y - center_y, 0);
	}
glEnd();

glDisable(GL_LINE_SMOOTH);
glDisable(GL_BLEND);
}

void RNDFEditGLView::drawSelected(double center_x, double center_y) {
    rndf::WayPoint* w = NULL, *w_next = NULL;
    rndf::PerimeterPoint* p = NULL, *p_next = NULL;
    rndf::TWayPointVec::const_iterator wpit, wpit_end, wpit_next;
    rndf::TPerimeterPointVec::const_iterator ppit, ppit_end, ppit_next;
    int blend = 1;

    // draw the selected lane in yellow
    if (gui->re_lane_->current()) {
        glLineWidth(2);
        glColor4f(1, 0.5, 0, blend);
        for (wpit = gui->re_lane_->current()->wayPoints().begin(), wpit_end = gui->re_lane_->current()->wayPoints().end();
                wpit != wpit_end; ++wpit) {
            w = *wpit;
            wpit_next = wpit;
            wpit_next++;
            if (wpit_next != wpit_end) {
                w_next = *wpit_next;
                draw_arrow(w->utmX() - center_x, w->utmY() - center_y, w_next->utmX() - center_x, w_next->utmY() - center_y, 0.5, 2.0);
                // std::cout << "lane at " << w->utmX() << " " << w->utmY() << std::endl;
            }
        }

        drawSmoothedLaneStack(center_x, center_y);
    }

    // draw the selected waypoint in yellow
    if (gui->re_wp_->current()) {
        glPointSize(5.0);
        glColor4f(1, 0.5, 0, blend);
        glPushMatrix();
        glColor4f(1, 0.5, 0, blend);
        glTranslatef(0, 0, -0.005);
        draw_circle(gui->re_wp_->current()->utmX() - center_x, gui->re_wp_->current()->utmY() - center_y, 0.5, 1);
        render_stroke_text_centered_2D(gui->re_wp_->current()->utmX() - center_x - 1.5,
                gui->re_wp_->current()->utmY() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
                gui->re_wp_->current()->name().c_str());

        //	fr.drawString2D(gui->re_wp_->current()->name(), gui->re_wp_->current()->utmX() - center_x - 1.5, gui->re_wp_->current()->utmY() - center_y - 1);

        glPointSize(1.0);
        glPopMatrix();
    }

    // draw exit arrow
    double x1, y1, x2, y2, angle;
    if ((gui->current_element == RNDF_ELEMENT_EXIT) && gui->gotFromPoint) {
        if (gui->re_wp_->current() || gui->re_pp_->current()) {
            if (gui->re_wp_->current()) {
                x1 = gui->re_wp_->current()->utmX();
                y1 = gui->re_wp_->current()->utmY();
            }
            else {
                x1 = gui->re_pp_->current()->utmX();
                y1 = gui->re_pp_->current()->utmY();
            }

            x2 = gui->last_utm_x;
            y2 = gui->last_utm_y;
            glColor4f(1, 0, 1, blend);
            draw_dashed_line(x1 - center_x, y1 - center_y, x2 - center_x, y2 - center_y, 1.0);
            angle = atan2(y2 - y1, x2 - x1);
            draw_arrowhead_2D(x2 - center_x, y2 - center_y, angle);
        }
    }

    if (gui->re_perimeter_->current()) {
        glLineWidth(2);
        glColor4f(0, 0.5, 1, blend);
        for (ppit = gui->re_perimeter_->current()->perimeterPoints().begin(), ppit_end
                = gui->re_perimeter_->current()->perimeterPoints().end(); ppit != ppit_end; ++ppit) {
            p = *ppit;
            ppit_next = ppit;
            ppit_next++;
            if (ppit_next != ppit_end) {
                p_next = *ppit_next;
                draw_arrow(p->utmX() - center_x, p->utmY() - center_y, p_next->utmX() - center_x, p_next->utmY()
                        - center_y, 0.5, 2.0);
                //			 std::cout << "Perimeter at " << p->utmX() << " " << p->utmY() << std::endl;
            }
        }

    }

    if (gui->re_pp_->current()) {
        glPointSize(5.0);
        glColor4f(0, 0.5, 1, blend);
        glPushMatrix();
        glTranslatef(0, 0, -0.005);
        draw_circle(gui->re_pp_->current()->utmX() - center_x, gui->re_pp_->current()->utmY()
                - center_y, 0.5, 1);
        render_stroke_text_centered_2D(gui->re_pp_->current()->utmX() - center_x - 1.5,
                gui->re_pp_->current()->utmY() - center_y - 1, GLUT_STROKE_ROMAN, 0.5,
                (char*) gui->re_pp_->current()->name().c_str());
        glPointSize(1.0);
        glPopMatrix();
        //	 std::cout << "Perimeter point at " << p->utmX() << " " << p->utmY() << std::endl;
    }

}

void RNDFEditGLView::drawSmoothedLaneStack(double center_x, double center_y) {
if(!gui->re_lane_->current()) {return;}

if(gui->smoothed_lane_stack_.empty()) {return;}

std::vector<std::vector<CurvePoint> >::const_iterator stackit = gui->smoothed_lane_stack_.begin(),
                                                      stackit_end = gui->smoothed_lane_stack_.end();

    glDisable(GL_DEPTH_TEST);

    double stack_pos = 1;
    for (; stackit != stackit_end; stackit++) {
        const std::vector<CurvePoint>& center_line = (*stackit);
        if (center_line.empty()) {
            return;
        }

        // Draw line between center line points
        glLineWidth(3.0);
        glEnable(GL_LINE_SMOOTH);
        glColor3f(0.6, 0.0, 0.0);
        glBegin(GL_LINES);
        for(uint32_t i=0; i<center_line.size()-1;) {
            glVertex3f(center_line[i].x - center_x, center_line[i].y - center_y, stack_pos);
            i++;
            glVertex3f(center_line[i].x - center_x, center_line[i].y - center_y, stack_pos);
        }
        glEnd();
        glDisable(GL_LINE_SMOOTH);

//        // Draw cones to show orientation, color represents curvature
//        glPointSize(8.0);
//        clit = center_line.begin();
//        clit++;
//        clit++;
//        clit2 = clit;
//        clit2++;
//        clit2++;
//        //  clit_end = center_line.end();
//        clit_end = center_line.find(last_idx);
//        for (; clit2 != clit_end; clit++, clit2++) {
//            uint32_t col_index = std::min(255., 255 * std::abs((*clit).second.kappa / 0.2));
//            glColor3f(cmap_rb2_red_[col_index], cmap_rb2_green_[col_index], cmap_rb2_blue_[col_index]);
//            glPushMatrix();
//            glTranslatef((*clit).second.x - center_x, (*clit).second.y - center_y, 0.0);
//            glRotatef(90.0, 0., 1., 0.);
//            glRotatef(-dgc_r2d((*clit).second.theta), 1., 0., 0.);
//            glutSolidCone(0.4, 1.1, 6, 1);
//            glPopMatrix();
//        }
    stack_pos += 1;
    }

glEnable(GL_DEPTH_TEST);
}

void RNDFEditGLView::drawTrajectory(double center_x, double center_y)
{
if(gui->trajectory.size() == 0) {return;}

glLineWidth(4.0);

glEnable(GL_BLEND);
glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
glEnable(GL_LINE_SMOOTH);
glColor3f(0.6, 0.0, 0.0);
glBegin(GL_LINES);

for (unsigned int i=0; i<gui->trajectory.size() - 1; ++i )
	{
	//    std::cout << "paint\n" << std::flush;
	glVertex3f(gui->trajectory[i ].x - center_x, gui->trajectory[i ].y - center_y, 0);
	glVertex3f(gui->trajectory[i+1].x - center_x, gui->trajectory[i+1].y - center_y, 0);
	}
glEnd();

glDisable(GL_LINE_SMOOTH);
glDisable(GL_BLEND);
}

} // namespace vlr
