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

#ifndef TRAFFIC_LIGHT_DETECTION_GRID_H
#define TRAFFIC_LIGHT_DETECTION_GRID_H

#include <applanix/ApplanixPose.h>
#include "traffic_light_transforms.h"

#ifndef round
#define round(x) (x<0?ceil((x)-0.5):floor((x)+0.5))
#endif

#define MAX_GRID_SIZE_TL 120

struct GridCell
{

	//grid cell center
	int u_center;
	int v_center;
	//cell corners
	int u1;
	int v1;
	int u2;
	int v2;
	int u3;
	int v3;
	int u4;
	int v4;
	//position in the grid
	int x;
	int y;
	//position of cell center with respect to the car (forward is positive)
	int robot_x;
	//data scores
	float r_score;
	float g_score;
	float y_score;
	//whether this grid is currently in the frame
	bool in_frame;

	GridCell()
	{
		u_center = -1;
		v_center = -1;
		u1 = -1;
		v1 = -1;
		u2 = -1;
		v2 = -1;
		u3 = -1;
		v3 = -1;
		u4 = -1;
		v4 = -1;
		x = -1;
		y = -1;
		robot_x = -1;
		r_score = 0.5;
		g_score = 0.5;
		y_score = 0.5;
		in_frame = false;
	}

};




//all the information about the structure of the detection grid
//one grid per light
class DetectionGrid
{

public:
	DetectionGrid(); //no one should call the default constructor

	DetectionGrid(double light_global_x, double light_global_y, double light_global_z_,
	              int width, int height, double spacing, TrafficLightTransform *tt,
					  bool NO_YELLOW);

	void init(double light_global_x, double light_global_y, double light_global_z_,
	              int width, int height, double spacing, TrafficLightTransform *tt,
					  bool NO_YELLOW);

	void UpdateDetectionGrid(LocalizePose *loc_pose, ApplanixPose *curr_pose,
	              double light_orientation, int cam_im_width, int cam_im_height, bool downsample);

	GridCell* getCell(int x, int y);

	//returns the best of r_score, g_score and y_score at cell x,y and the character of the score:
	float getBestScore(int x, int y, char *state);

	int getWidth() {return grid_width_;}
	int getHeight() {return grid_height_;}

	double getLightRobotX() {return light_robot_x_;}

  //returns the width and height of the image used for u,v calculations
  int getImageWidth() {return im_width_;}
  int getImageHeight() {return im_height_;}

  //returns the extreme bounds of the entire grid projected into image coordinates
  int getMinU() {return grid_min_u_;}
  int getMaxU() {return grid_max_u_;}
  int getMinV() {return grid_min_v_;}
  int getMaxV() {return grid_max_v_;}

	//returns the center of the grid (the light location prior)
  int getLightU() {return light_u_;}
  int getLightV() {return light_v_;}

	//To call camera-specific transform functions
	TrafficLightTransform *trans_;
private:
	GridCell grid_cells_[MAX_GRID_SIZE_TL][MAX_GRID_SIZE_TL];

	bool USE_YELLOW;

	//the center of the grid- the mapped light location
	double light_global_x_;
	double light_global_y_;
	double light_global_z_;

	//the x coordinate of the expected light location with respect to the car
	double light_robot_x_;

	//grid center with respect to the camera image
	int light_u_;
	int light_v_;

	//grid dimensions
	int grid_width_;
	int grid_height_;
	double grid_spacing_;

	//grid bounds and info with respect to the camera image
	int grid_max_u_;
	int grid_max_v_;
	int grid_min_u_;
	int grid_min_v_;

	//dimensions of the camera image the grid will be projected into
	int im_width_;
	int im_height_;

	//helper functions
	void WithinImageAdjustment(int *u, int *v);
	void GridBoundsCheck(int u, int v);
	void AdjustGridBounds(int *u1p, int *u2p, int *u3p, int *u4p, int *v1p, int *v2p, int *v3p, int *v4p);
};


#endif  // TRAFFIC_LIGHT_DETECTION_GRID_H
