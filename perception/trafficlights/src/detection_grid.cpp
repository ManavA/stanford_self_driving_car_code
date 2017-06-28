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

#include "detection_grid.h"
//#include <vec3.h>

using namespace Eigen; //for Vec3

DetectionGrid::DetectionGrid
(const double light_global_x, const double light_global_y, const double light_global_z,
 const int grid_width, const int grid_height, const double grid_spacing, TrafficLightTransform *tt,
 bool NO_YELLOW)
{
	init(light_global_x, light_global_y, light_global_x, grid_width, grid_height, grid_spacing,
			tt, NO_YELLOW);
}

void DetectionGrid::init
(const double light_global_x, const double light_global_y, const double light_global_z,
 const int grid_width, const int grid_height, const double grid_spacing, TrafficLightTransform *tt,
 bool NO_YELLOW)
{
	USE_YELLOW = !NO_YELLOW;
	trans_ = tt;

	light_global_x_ = light_global_x;
	light_global_y_ = light_global_y;
	light_global_z_ = light_global_z;

	if (grid_width > MAX_GRID_SIZE_TL || grid_height > MAX_GRID_SIZE_TL)
	{
		dgc_die("detection grid size exceeds maximum allowed grid size. change your #defines\n");
	}
	grid_width_   = grid_width;
	grid_height_  = grid_height;
	grid_spacing_ = grid_spacing;

	//initialize the following after receiving pose information:
	//center with respect to the camera image
	light_u_ = -1;
	light_v_ = -1;
	//grid bounds and with respect to the camera image
	grid_max_u_ = -1;
	grid_max_v_ = -1;
	grid_min_u_ = -1;
	grid_min_v_ = -1;
	//the dimensions of the camera image the grid will be projected into
	im_width_ = -1;
	im_height_ = -1;
}

//default constructor should not be called
//(though it is called automatically by stl map, so it needs to be public)
DetectionGrid::DetectionGrid()
{
	trans_ = NULL;
	light_global_x_ = -1;
	light_global_y_ = -1;
	light_global_z_ = -1;
	grid_width_   = -1;
	grid_height_  = -1;
	grid_spacing_ = -1;
	light_u_ = -1;
	light_v_ = -1;
	grid_max_u_ = -1;
	grid_max_v_ = -1;
	grid_min_u_ = -1;
	grid_min_v_ = -1;
	im_width_ = -1;
	im_height_ = -1;
}

GridCell* DetectionGrid::getCell(int x, int y)
{
	// TODO : fix all code that makes such calls.
	if (x < 0 || y < 0) return &grid_cells_[0][0];
	return &grid_cells_[y][x];

}

void DetectionGrid::WithinImageAdjustment(int *u, int *v)
{
	//make sure corners of grid cell fall within the image
	if (*u < 0) *u = 0;
	if (*v < 0) *v = 0;
	if (*u >= im_width_)  *u = im_width_  -1;
	if (*v >= im_height_) *v = im_height_ -1;
}

void DetectionGrid::GridBoundsCheck(int u, int v)
{
	if (u < grid_min_u_) grid_min_u_ = u;
	if (u > grid_max_u_) grid_max_u_ = u;
	if (v < grid_min_v_) grid_min_v_ = v;
	if (v > grid_max_v_) grid_max_v_ = v;
}

void DetectionGrid::AdjustGridBounds
(int *u1p, int *u2p, int *u3p, int *u4p, int *v1p, int *v2p, int *v3p, int *v4p)
{

	WithinImageAdjustment(u1p, v1p);
	WithinImageAdjustment(u2p, v2p);
	WithinImageAdjustment(u3p, v3p);
	WithinImageAdjustment(u4p, v4p);

	GridBoundsCheck(*u1p, *v1p);
	GridBoundsCheck(*u2p, *v2p);
	GridBoundsCheck(*u3p, *v3p);
	GridBoundsCheck(*u4p, *v4p);
}

float DetectionGrid::getBestScore(int x, int y, char *state)
{

	GridCell *curr_cell = getCell(x, y);
	float r_score = curr_cell->r_score;
	float g_score = curr_cell->g_score;
	float y_score = curr_cell->y_score;

	if (USE_YELLOW)
	{
		if (r_score > g_score && r_score > y_score)
		{
			*state = 'r';
			return r_score;
		}
		if (g_score > r_score && g_score > y_score)
		{
			*state = 'g';
			return g_score;
		}
		if (y_score > r_score && y_score > g_score)
		{
			*state = 'y';
			return y_score;
		}
	}
	else
	{
		//for VAIL demo:
		if (r_score > g_score)
		{
			*state = 'r';
			return r_score;
		}
		if (g_score > r_score)
		{
			*state = 'g';
			return g_score;
		}
	}

	*state = 'u';
	return 0;
}


void DetectionGrid::UpdateDetectionGrid
(LocalizePose *loc_pose, ApplanixPose *curr_pose, double light_orientation,
 const int cam_im_width, const int cam_im_height, bool downsample)
{
	im_width_ = cam_im_width;
	im_height_ = cam_im_height;

	int twice_width = -1;
	int twice_height = -1;

	double robotX = 0;

	if (downsample)
	{
		twice_width = im_width_*2;
		twice_height = im_height_*2;
		trans_->globalToUV(twice_width, twice_height, light_global_x_, light_global_y_, light_global_z_,  
				&light_u_, &light_v_, &robotX, *loc_pose, *curr_pose);
		light_u_ /=2;
		light_v_ /=2;
	}
	else
	{
		trans_->globalToUV(im_width_, im_height_, light_global_x_, light_global_y_, 
				light_global_z_,  &light_u_, &light_v_, &robotX, *loc_pose, *curr_pose);
	}

	//printf("from dg: light u %d light v %d\n", light_u_, light_v_);

	light_robot_x_ = robotX;
	//find basis for grid
	Vec3<double> origin = Vec3<double>(light_global_x_, light_global_y_, light_global_z_);
	Vec3<double> vert_vect = Vec3<double>(0.0,0.0,1.0);
	Vec3<double> or_vect = Vec3<double>(cos(light_orientation), sin(light_orientation), 0);
	or_vect.normalize();
	Vec3<double> horiz_vect = vert_vect.cross(or_vect);
	Vec3<double> vert_spacer = vert_vect*grid_spacing_;
	Vec3<double> horiz_spacer = horiz_vect*grid_spacing_;
	//set loop bounds
	int half_grid_width = grid_width_/2;
	int grid_start_x = -1*half_grid_width;
	int grid_end_x = half_grid_width;
	int half_grid_height = grid_height_/2;
	int grid_start_y = -1*half_grid_height;
	int grid_end_y = half_grid_height;
	//initialize mins and maxs for comparison - want a bounding box on where our grid cells land in the frame
	grid_min_u_ = im_width_;
	grid_max_u_ = 0;
	grid_min_v_ = im_height_;
	grid_max_v_ = 0;
	//iterate through grid cells - y is vertical (aka rows; height), x is horizontal (aka columns; width)
	for (int y = grid_start_y; y <= grid_end_y ; y++)
	{
		for (int x = grid_start_x; x <= grid_end_x; x++)
		{
			Vec3<double> curr_cell_pt = origin + (vert_spacer*y) + (horiz_spacer*x);
			double robot_x_center = -1;
			int cell_center_u, cell_center_v;

			// TODO: replace this with a mathematically simpler solution.
			// solve it out to a linear interpolation and apply a non-projective
			// equation linearly across the grid.
			// The point is to Not use projection for every grid cell.
			// One way of doing this: 
			//   use the undistort matrices in Camera,
			//   make a projection for each of the four grid vertices,
			//   apply the linear interpolation across the undistorted plane,
			//   re-distort the interpolated points.
			if (downsample)
			{
				trans_->globalToUV(twice_width, twice_height, curr_cell_pt.x[0], 
						curr_cell_pt.x[1], curr_cell_pt.x[2],
				      &cell_center_u, &cell_center_v, &robot_x_center, 
						*loc_pose, *curr_pose);
				cell_center_u /= 2;
				cell_center_v /= 2;
			}
			else
			{
				trans_->globalToUV(im_width_, im_height_, curr_cell_pt.x[0], curr_cell_pt.x[1], curr_cell_pt.x[2],
				                   &cell_center_u, &cell_center_v, &robot_x_center, *loc_pose, *curr_pose);
			}
			int grid_index_x = x - grid_start_x;
			int grid_index_y = y - grid_start_y;

			//if the cell center is outside the image area - don't change anything, just set in_frame to false
			if (cell_center_u > im_width_-1 || cell_center_v > im_height_-1 || cell_center_u < 0 || 
					cell_center_v < 0 || robot_x_center < 0)
			{
				grid_cells_[grid_index_y][grid_index_x].in_frame = false;
			}

			else  //otherwise, sample within the cell to get the prediction for this cell
			{
				//calculate distance in pixels halfway to neighbors - this is the cell boundary (and handle edge cases)
				int u1,v1, u2,v2, u3,v3, u4,v4;
				Vec3<double> topleft_neighbor = curr_cell_pt - horiz_spacer + vert_spacer;
				Vec3<double> topright_neighbor = curr_cell_pt  + horiz_spacer + vert_spacer;
				Vec3<double> bottomleft_neighbor = curr_cell_pt  - horiz_spacer - vert_spacer;
				Vec3<double> bottomright_neighbor = curr_cell_pt  + horiz_spacer - vert_spacer;

				if (downsample)
				{

					trans_->globalToUV(twice_width, twice_height, topleft_neighbor.x[0], 
							topleft_neighbor.x[1], topleft_neighbor.x[2],
					      &u1, &v1, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(twice_width, twice_height, topright_neighbor.x[0], 
							topright_neighbor.x[1], topright_neighbor.x[2],
					      &u2, &v2, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(twice_width, twice_height, bottomright_neighbor.x[0], 
							bottomright_neighbor.x[1], bottomright_neighbor.x[2],
					      &u3, &v3, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(twice_width, twice_height, bottomleft_neighbor.x[0], 
							bottomleft_neighbor.x[1], bottomleft_neighbor.x[2],
					      &u4, &v4, &robotX, *loc_pose, *curr_pose);

					u1/=2;
					u2/=2;
					u3/=2;
					u4/=2;
					v1/=2;
					v2/=2;
					v3/=2;
					v4/=2;
				}
				else
				{
					trans_->globalToUV(im_width_, im_height_, topleft_neighbor.x[0], 
							topleft_neighbor.x[1], topleft_neighbor.x[2],
					      &u1, &v1, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(im_width_, im_height_, topright_neighbor.x[0], 
							topright_neighbor.x[1], topright_neighbor.x[2],
					      &u2, &v2, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(im_width_, im_height_, bottomright_neighbor.x[0], 
							bottomright_neighbor.x[1], bottomright_neighbor.x[2],
					      &u3, &v3, &robotX, *loc_pose, *curr_pose);
					trans_->globalToUV(im_width_, im_height_, bottomleft_neighbor.x[0], 
							bottomleft_neighbor.x[1], bottomleft_neighbor.x[2],
					      &u4, &v4, &robotX, *loc_pose, *curr_pose);
				}

				//check for min and max values and make sure the corners still 
				//fall within the image (if not, adjust them so they do)
				//this will also potentially adjust the bounding box of all grid 
				//cells as well, if necessary
				AdjustGridBounds(&u1, &u2, &u3, &u4, &v1, &v2, &v3, &v4);

				GridCell *curr_cell = &grid_cells_[grid_index_y][grid_index_x];
				curr_cell->in_frame = true;
				curr_cell->u_center = cell_center_u;
				curr_cell->v_center = cell_center_v;
				curr_cell->u1 = u1;
				curr_cell->v1 = v1;
				curr_cell->u2 = u2;
				curr_cell->v2 = v2;
				curr_cell->u3 = u3;
				curr_cell->v3 = v3;
				curr_cell->u4 = u4;
				curr_cell->v4 = v4;
				curr_cell->x = grid_index_x;
				curr_cell->y = grid_index_y;
				curr_cell->robot_x = robot_x_center;

			}//end if-else
		}//end vert loop
	}	//end horiz loop
	//printf("from dg: minu %d minv %d maxu %d maxv %d\n", 	grid_min_u_, grid_max_u_, grid_min_v_, grid_max_v_ );
}
