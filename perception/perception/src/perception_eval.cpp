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

#include <set>
#include <vector>
#include <sys/wait.h>
#include <perception_interface.h>
#include <ipc_std_interface.h>
#include <system_test.h>
#include <car_list.h>

#include <rtcGeometry3D.h>

#include "utils.h"

using namespace dgc;
using namespace dgc_test;
using namespace std;

class PerceptionTest;

IpcInterface *ipc = NULL;
PerceptionTest* test = NULL;
int verbose = 0;

class PerceptionTest : public dgc_test::SystemTest {
private:
  car_list_t carlist_;

  vector<float> fp_;
  vector<float> tp_;
  vector<float> fn_;
  vector<float> vel_error_;

  ApplanixPose* current_pose;

  int num_frames_;
  int num_obstacles_;
  int num_static_cells_;
  double cost_;

  void applanix_handler( ApplanixPose* pose) {
    current_pose = pose;
  }

  double distanceToBox(double x, double y, double theta, double h, double w, double* i_x, double* i_y) {
    dgc_transform_t t;
    dgc_transform_identity(t);
    dgc_transform_rotate_z(t, theta);
    dgc_transform_translate(t, x, y, 0.0);

    double t_x, t_y, t_z = 0.0;
    t_x = -h/2;
    t_y = -w/2;
    dgc_transform_point(&t_x, &t_y, &t_z, t);
    rtc::Vec3<double> p1(t_x, t_y, t_z);

    t_x = +h/2;
    t_y = -w/2;
    dgc_transform_point(&t_x, &t_y, &t_z, t);
    rtc::Vec3<double> p2(t_x, t_y, t_z);

    t_x = +h/2;
    t_y = +w/2;
    dgc_transform_point(&t_x, &t_y, &t_z, t);
    rtc::Vec3<double> p3(t_x, t_y, t_z);

    t_x = -h/2;
    t_y = +w/2;
    dgc_transform_point(&t_x, &t_y, &t_z, t);
    rtc::Vec3<double> p4(t_x, t_y, t_z);

//    t_x = current_pose->smooth_x;
//    t_y = current_pose->smooth_y;
//    dgc_transform_point(&t_x, &t_y, &t_z, t);
//    rtc::Vec3<double> p(t_x, t_y, t_z);
    rtc::Vec3<double> p(current_pose->smooth_x, current_pose->smooth_y, 0.0);
//    rtc::Vec3<double> p(*i_x, *i_y, 0.0);

    double d1, d2;
    rtc::Vec3<double> cp1, cp2;
    rtc::Geometry3D<double>::distancePointTriangle(p, p1, p2, p4, d1, cp1);
    rtc::Geometry3D<double>::distancePointTriangle(p, p2, p3, p4, d2, cp2);

    if (d1 < d2)
    {
      *i_x = cp1[0];
      *i_y = cp1[1];
      return d1;
    } else {
      *i_x = cp2[0];
      *i_y = cp2[1];
      return d2;
    }

    return std::min(d1,d2);
  }

  void obstacles_handler( PerceptionObstacles *obstacles )
  {
    num_frames_ ++;
    num_obstacles_ += obstacles->num_dynamic_obstacles;
    num_static_cells_ += obstacles->num_points;

    set<PerceptionDynamicObstacle*> matched;

    for (int i = 0, num_cars = carlist_.num_cars(); i < num_cars; i++) {
      car_t* car = &carlist_.car[i];

      double x, y, th, vel;
      bool extrapolated;
      float min_cost = 0.0;
      if (car->estimate_pose(obstacles->timestamp, &x, &y, &th, &extrapolated) && car->estimate_velocity(obstacles->timestamp, &vel)) {
        // find nearest detected obstacle
        float min_dist = 2.0;
        min_cost = 100.0;

        int match = -1;
        for (int j=0; j < obstacles->num_dynamic_obstacles; j++) {
          PerceptionDynamicObstacle* obstacle = &obstacles->dynamic_obstacle[j];

          double o_x, o_y, c_x, c_y;
          distanceToBox(obstacle->x, obstacle->y, obstacle->direction, obstacle->length, obstacle->width, &o_x, &o_y);
          distanceToBox(x, y, th, car->l, car->w, &c_x, &c_y);

          double dx = o_x - c_x;
          double dy = o_y - c_y;
          float d = sqrt(dx*dx + dy*dy);

          if (d < min_dist) {
            min_dist = d;
            match = j;
          }
        }

        if (match > -1) { // true positive
          PerceptionDynamicObstacle* obstacle = &obstacles->dynamic_obstacle[match];
          double dt = fabs(angle_diff(th, obstacle->direction));
          double dv = fabs(vel - obstacle->velocity);
          double dw = fabs(car->w - obstacle->width);
          double dl = fabs(car->l - obstacle->length);
          min_cost = min_dist + 0.3 * fabs(dt) + fabs(dv) + 0.1 * fabs(dw) + 0.1 * fabs(dl);
          matched.insert(obstacle);
          vel_error_.push_back(obstacle->velocity - vel);
        } else {  // false negative
          double dx = x - current_pose->smooth_x;
          double dy = y - current_pose->smooth_y;
          double dist = sqrt(dx*dx + dy*dy);
          fn_.push_back(dist);
        }
      }

      cost_ += min_cost;

      for (int j=0; j < obstacles->num_dynamic_obstacles; j++) {
        PerceptionDynamicObstacle* obstacle = &obstacles->dynamic_obstacle[j];
        double dx = obstacle->x - current_pose->smooth_x;
        double dy = obstacle->y - current_pose->smooth_y;
        double dist = sqrt(dx*dx + dy*dy);
        if (matched.find(obstacle) == matched.end()) { // false positive
          fp_.push_back(dist);
        } else { // true positive
          tp_.push_back(dist);
        }
      }
    }
  }

protected:
  virtual void addCallbacks() {
    int callback = ipc_->Subscribe(PerceptionObstaclesID, this, &PerceptionTest::obstacles_handler, DGC_SUBSCRIBE_ALL);
    addCallback(callback);

    callback = ipc_->Subscribe(ApplanixPoseID, this, &PerceptionTest::applanix_handler, DGC_SUBSCRIBE_LATEST);
    addCallback(callback);
  }

  virtual void launchPrograms() {
    test->launch("$RACE_ROOT/bin/fake_localize");
    test->launch("$RACE_ROOT/bin/perception");
  }

  virtual void setParams() {
    char *return_string;

    int err = param_->SetFilename("rndf", "rndf_file", "$RACE_ROOT/param/rndfs/stanford_rndf_with_lights20090814.txt", &return_string);
    if (err != 0) {
      fprintf(stderr, "Could not set param (%s)", param_->GetError());
      free(return_string);
    }
  }

public:
  PerceptionTest(IpcInterface* ipc) : SystemTest(ipc) {}
  virtual ~PerceptionTest() {}

  void prepareTest(char* cars_file) {
    fp_.clear();
    tp_.clear();
    fn_.clear();
    vel_error_.clear();

    current_pose = NULL;

    num_frames_ = 0;
    num_obstacles_ = 0;
    num_static_cells_ = 0;
    cost_ = 0;

    char* fn = dgc_expand_filename(cars_file);
    if (fn)
      cars_file = fn;

    carlist_.load_labels(cars_file);

    if (fn)
      free(fn);
  }

  virtual void printResults() {
    fprintf(stderr, "\n\n---------------------\n");
    fprintf(stderr, "observations: %d\n", num_frames_);

    fprintf(stderr, "true positives:  %d\n", tp_.size());
    fprintf(stderr, "false positives: %d\n", fp_.size());
    fprintf(stderr, "false negatives: %d\n", fn_.size());

//    fprintf(stderr, "false positive rate: %f\n", (float)fp_.size() / (tp_.size() + fp_.size() ))

    fprintf(stderr, "dynamic obstacles: %d\n", num_obstacles_);
    fprintf(stderr, "overall cost: %f\n", cost_);
    fprintf(stderr, "normalized cost: %f\n", cost_ / (double)num_frames_);
    fprintf(stderr, "obstacle cells: %d\n---------------------\n\n", num_static_cells_);
    fflush(stderr);
  }

  // TODO: save these out incrementally instead of just at the end
  virtual void saveDetailedResults() {
    FILE* fp = fopen("tp", "w");
    if (fp) {
      for (int i=0, c=tp_.size(); i < c; i++) {
        fprintf(fp, "%f\n", tp_[i]);
      }
      fclose(fp);
    }

    fp = fopen("fp", "w");
    if (fp) {
      for (int i=0, c=fp_.size(); i < c; i++) {
        fprintf(fp, "%f\n", fp_[i]);
      }
      fclose(fp);
    }

    fp = fopen("fn", "w");
    if (fp) {
      for (int i=0, c=fn_.size(); i < c; i++) {
        fprintf(fp, "%f\n", fn_[i]);
      }
      fclose(fp);
    }

    fp = fopen("vel", "w");
    if (fp) {
      for (int i=0, c=vel_error_.size(); i < c; i++) {
        fprintf(fp, "%f\n", vel_error_[i]);
      }
      fclose(fp);
    }

  }


  double getScore() { return cost_; }
};

void signal_handler(int x)
{
  if (x == SIGINT) {
    test->killAll();
    test->endTest();

    delete ipc;
    delete test;

    exit(1);
  }
}

int main(int argc, char **argv)
{
  /* Handle shutdown signal */
  signal(SIGINT, signal_handler);
  ipc = new IpcStandardInterface;

  test = new PerceptionTest(ipc);

  test->setVerbose(true);
  test->setUseXterm(true);

//  test->prepareTest("$DATA_ROOT/2009-11-18/I280N-11-18-2009_10-17-12.vlf.labels");
//  test->runLogTest("$DATA_ROOT/2009-11-18/I280N-11-18-2009_10-17-17.log.gz", NULL, 15.0, 1.0, false);
//  test->printResults();

  test->prepareTest("$DATA_ROOT/2009-10-03/Shortloop1-10-03-2009_15-52-28.vlf.labels");
  test->runLogTest("$DATA_ROOT/2009-10-03/Shortloop1-10-03-2009_15-52-28.log.gz", NULL, 600.0, 0.3, false);
  test->printResults();
  test->saveDetailedResults();

  delete test;
  delete ipc;

  return 0;
}
