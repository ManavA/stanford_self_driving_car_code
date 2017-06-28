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

#include <cmath>
#include <map>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <highgui.h>
#include <sensor_msgs/image_encodings.h>

#include <global.h>
#include <applanix/ApplanixPose.h>
#include <localize/LocalizePose.h>
#include <driving_common/Heartbeat.h>
#include <lltransform.h>
#include <aw_roadNetwork.h>
#include <kdtree.h>

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <ctype.h>
#include <stdio.h>
#endif

//profiling
//#include "../../ros-pkg/google-perftools/build/google-perftools-1.5/src/google/profiler.h"

//for debug mode
//#include <gui2D.h>
//#include <textures.h>
//--------

#include <driving_common/TrafficLightStates.h>
#include <driving_common/TrafficLightStateRequest.h>
#include <traffic_light_transforms.h>
#include <detection_grid.h>
#include <image_data.h>
#include <histogram_filter_tl.h>

// static memory allocation constants (feel free to increase these)
#define MAX_LIGHTS_PER_INTERSECTION 10
#define MAX_LIGHTS 10000

// Global kill switch
bool quit_signal = false;
	
// defined on command line
static bool NO_YELLOW = false;
static bool NO_PLANNER_MODE = false;
static bool NORMALIZE = true;
static bool writeVideo = false;
static bool writeVideoAsFrames = false; 

// defined by param server
static double start_looking_dist; // how far is too far? [meters]
static double stop_looking_dist;  // how close is too close? [meters]
static double downsample_dist;    // how close before we shrink the image? [meters]
static int grid_width;            // how many cells wide is the plane orthogonal to the light? [cells]
static int grid_height;           // how many cells tall is the plane orthogonal to the light? [cells]
static double grid_spacing;       // how fine is the grid mesh overlaying the plane? [cells]
static bool dropframes = true;    // should we drop frames if more than one image is in the queue?
static int secondary_dist;        // distance from light to switch to secondary
static CameraParams pri, sec;     // primary and secondary cameras' parameters

/*struct DistanceGivenTime
{
	double time;
	double distance;
};
std::vector<DistanceGivenTime> timeDist;
*/
// namespaces
using namespace dgc;
using namespace std;
using namespace vlr;
using namespace rndf;

struct detectionResult
{
	double confidence;
	int intersection;
	int number;
	int distance;
	int accuracy; // { -1, 0, 1 } => { incorrect, unknown, correct }
	char state;
};

struct applanixBuffer
{
	ApplanixPose currPose;
	double globalX;
	double globalY;
	double time_last_update;
};

struct lightAndPos
{
	TrafficLightState light;
	double globalX;
	double globalY;
	double altitude;
	double orientation; //in radians
	double curr_distance;
	double face_angle;
	double robotx;
	char previousState;
	int counter_red;
	int counter_yellow;
	int counter_green;
	bool downsample;
	bool use_secondary_camera;
};

static std::map<std::string, TrafficLightPose> tl_poses;

//traffic-light-specific objects
static TrafficLightTransform tt;
static map<std::string, DetectionGrid*> light_grids;
static map<std::string, HistogramFilterTL*> light_histograms;
static map<std::string, ImageData*>light_image_data;

static std::map<int, std::string> tlid_map;

//for publishing
static TrafficLightList tl_list_msg = {0, NULL};

//camera
static IplImage* full_img = NULL;
static IplImage* half_img = NULL;
static CameraImage *camera_image = NULL;
static CameraInterface *camera_interface = NULL;

// if we have too many pixels in our ROI
static bool downsample = false;
static bool use_secondary_camera = false;

//for threading
pthread_mutex_t camera_mutex   = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t localize_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t applanix_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t tl_poses_mutex = PTHREAD_MUTEX_INITIALIZER;

//lights structures
static map<std::string, lightAndPos> lights;
static dgc_kdtree_p allLights_kdtree;
static set<std::string> active_light_IDs;
static char intersection[MAX_LIGHTS][MAX_LIGHTS_PER_INTERSECTION+1];
static int  interDistance[MAX_LIGHTS][MAX_LIGHTS_PER_INTERSECTION];
static int  interEvaluate[MAX_LIGHTS][MAX_LIGHTS_PER_INTERSECTION+1];
static double interProb[MAX_LIGHTS][MAX_LIGHTS_PER_INTERSECTION][3];
static std::list<detectionResult> results;

//pose and map data
static LocalizePose  loc_pose;
static IpcInterface *ipc;
static applanixBuffer pose_buffer;
static applanixBuffer pose_buffer2;
static std::list<applanixBuffer> recentPoses;
static char* rndf_filename = NULL;
static ShapeTable<uint8_t> red_hue_table;
static ShapeTable<uint8_t> ylw_hue_table;
static ShapeTable<uint8_t> grn_hue_table;
static ShapeTable<uint8_t> saturation_table;

//for heartbeat messages
static Heartbeat heartbeat;

//modes of program operation
static bool debug_mode = false;
static bool evaluation_mode = false;

//for evaluation mode
static vector<double> evaluationTime;
static vector<char> evaluationColor;
static int evaluation_index = 0;
static int totalCorrectFrames  = 0;
static int totalIncorrectFrames = 0;

// for video output
static CvVideoWriter* cvwriter;
static const char* debugWindows[] = { }; //{ "Hist LUT", "Winner", "Prior" };
#define DEBUG_WINDOW_COUNT 0

int evaluate(char color)
{
	if (evaluation_mode == false) return 0;

	color |= 0x20;

	double cam_time = camera_image->timestamp;
	if (cam_time > evaluationTime.back())
	{
		return 0;
	}
	while (evaluationTime[evaluation_index] < cam_time)
	{
	//	printf("cam_time %f evaluationTime %f\n", cam_time, 
	//			evaluationTime[evaluation_index]);
		evaluation_index++;
	}
	
	evaluation_index--; //we went one past the index we care about...
	//printf("color %c passed color %c cam_time %f evaluationTime %f\n", 
	//		(evaluationColor[evaluation_index]|0x20), color, cam_time, 
	//		evaluationTime[evaluation_index]);

	char evalColor = (evaluationColor[evaluation_index]|0x20);

	int correct;

	// don't penalize if we can't see the light
	if (color == 'u' || evalColor == 'u')
	{
		correct = 0;
	}
	else if (color == evalColor)
	{
		totalCorrectFrames++;
		correct = 1;
	}
	else
	{
		totalIncorrectFrames++;
		correct = -1;
	}

	printf("CD:ID  %d:%d\nEvalColor: %c\n", totalCorrectFrames, totalIncorrectFrames, evalColor);

	return correct;
}


//----------------------------------------------------------
//debug gui methods:----------------------------------------
int win_height_gui = 640;
int win_width_gui = 640;
int active_light_index_gui = 0;
bool record_frames = false;
int frame_num_gui = 0;
IplImage *grid_prior_for_viewer;
//static dgc_gl_texture_t* image_texture = NULL;
float SSD_patches[490000];
uchar debug_patch[3000000];
int u_win_data_only_dbg = -1;
int v_win_data_only_dbg = -1;
int posterior_u_dbg = -1;
int posterior_v_dbg = -1;
char win_state_data_only_dbg = 'u';
char posterior_state_dbg = 'u';
int piece_width;
int piece_height;
std::string light_ID_dbg;
IplImage* videoframe;

std::string getLightName()
{
	if (active_light_IDs.size() > 0)
	{
		return *active_light_IDs.begin();
	}
	else
	{
		std::string dummy;
		return dummy;
	}
}

void getUTMOffset(ApplanixPose *pose, double* xOffset, double* yOffset)
{
	double utmX, utmY;
	char utmzone[10];
	latLongToUtm(pose->latitude, pose->longitude, &utmX, &utmY, utmzone);
	printf("utmzone %s utmY %f utmX %f\n", utmzone, utmY, utmX);
	*xOffset = utmX - pose->smooth_x;
	*yOffset = utmY - pose->smooth_y;
	printf("utmoffset x %f offset y %f\n", *xOffset, *yOffset);

}

//Initialization and Shutdown:
void read_traffic_lights_from_file(char *filename)
{
	double x[MAX_LIGHTS];
	double y[MAX_LIGHTS];
	int light_count = 0;

	char arrow = 'n';
	char utmzone[10];

	FILE *f = fopen(filename, "r");
	if (f == NULL)
	{
		printf("Error: unable to open light file: %s\n", filename);
		::exit(0);
	}
	int n=0;
	int inter, number;
	double lat1, lon1, lat2, lon2, alt1, alt2;
	while (fscanf(f, "%d.%d %lf %lf %lf %lf %lf %lf\n%n", 
		&inter, &number, &lat1, &lon1, &alt1, &lat2, &lon2, &alt2, &n) != EOF)
	{
		if (n == 0) continue;

		double utm_x1, utm_y1, utm_x2, utm_y2;
		latLongToUtm(lat1, lon1, &utm_x1, &utm_y1, utmzone);
		latLongToUtm(lat2, lon2, &utm_x2, &utm_y2, utmzone);
		float theta = atan2(utm_y2 - utm_y1, utm_x2 - utm_x1);

		printf("ID %d.%d gx %f gy %f galt %f hasArrowToo %c\n", inter, number, 
			utm_x1, utm_y1, alt1, arrow);

		x[light_count] = utm_x1;
		y[light_count] = utm_y1;

		//add light to map <ID,light>
		//{int,char,char,double,double,double,double,int,int}

		char tmp_cstr[20];
		std::string tmp_str;
		sprintf(tmp_cstr, "%d.%d", inter, number);
		tmp_str = tmp_cstr;
		strcpy(lights[tmp_str].light.name, tmp_cstr);
		lights[tmp_str].light.state =  'u';
		lights[tmp_str].light.state_arrow =  arrow;
		lights[tmp_str].light.timestamp_rg_switch = -1.0;
		lights[tmp_str].light.timestamp_gy_switch = -1.0;
		lights[tmp_str].light.timestamp = -1.0;
		lights[tmp_str].light.confidence =  -1.0;
		lights[tmp_str].light.u =  0;
		lights[tmp_str].light.v =  0;
		lights[tmp_str].globalX = utm_x1;
		lights[tmp_str].globalY = utm_y1;
		lights[tmp_str].altitude = alt1;
		lights[tmp_str].orientation = theta;
		lights[tmp_str].curr_distance = 100000;
		lights[tmp_str].robotx = -10000;
		lights[tmp_str].previousState = 'u';
		lights[tmp_str].counter_red = 0;
		lights[tmp_str].counter_yellow = 0;
		lights[tmp_str].counter_green = 0;
		lights[tmp_str].downsample = false;
		lights[tmp_str].use_secondary_camera = false;

		light_count++;
	}
	//build kdtree
	allLights_kdtree = dgc_kdtree_build_balanced(x, y, light_count);
}

void read_traffic_lights_from_rndf()
{

	//load roadNetwork file
	RoadNetwork rn;
	if (!rn.loadRNDF(rndf_filename))
	{
		dgc_die("invalid roadNetwork file\n");
	}

	//NOTE: Light IDS are REQUIRED to be listed in order, starting at 0
	//because KD tree IDs are based on the order that the lights are 
	//entered into the array

	double x[MAX_LIGHTS];
	double y[MAX_LIGHTS];
	int light_count = 0;

	double globalX = 0;
	double globalY = 0;
	double alt = 0;
	char arrow = 'n';

	const TTrafficLightMap& rn_lights = rn.trafficLights();
	TTrafficLightMap::const_iterator
	tlit = rn_lights.begin(), tlit_end = rn_lights.end();

	for (; tlit != tlit_end; tlit++)
	{
		std::string name = (*tlit).first;
		TrafficLight* tl = (*tlit).second;

		globalX = tl->utm_x();
		globalY = tl->utm_y();
		alt = tl->z();

		//stick light in kdtree
		x[light_count] = globalX;
		y[light_count] = globalY;

		//add light to map <ID,light>
		//{int,char,char,double,double,double,double,int,int}
		TrafficLightState light;
		strcpy(light.name, name.c_str());
		light.state = 'u';
		light.state_arrow = arrow;      // //add arrow info later
		light.timestamp_rg_switch = -1;
		light.timestamp_gy_switch = -1;
		light.timestamp = -1;
		light.confidence = -1;
		light.u = 0;
		light.v = 0;

		lights[name].light = light;
		lights[name].globalX = globalX;
		lights[name].globalY = globalY;
		lights[name].altitude = alt;
		lights[name].orientation = tl->orientation();
		lights[name].curr_distance = 100000;
		lights[name].robotx = -10000;
		lights[name].previousState = 'u';
		lights[name].counter_red = 0;
		lights[name].counter_yellow = 0;
		lights[name].counter_green = 0;
		lights[name].downsample = false;
		lights[name].use_secondary_camera = false;

		printf("ID %s gx %f gy %f galt %f hasArrowToo %c\n",
		       name.c_str(), globalX, globalY, alt, arrow);
		tlid_map.insert(std::make_pair(light_count, name));
		light_count++;
	}

	//build kdtree
	allLights_kdtree = dgc_kdtree_build_balanced(x, y, lights.size());
}

//for comparisons in evaluation mode
void read_ground_truth_from_file(const char* filename)
{
	FILE *f = fopen(filename, "r");
	if (f == NULL)
	{
		printf("Error: unable to open ground truth file: %s\n", filename);
		::exit(0);
	}

	//file format should be
	//<PACKET NUM> <START TIMESTAMP> <COLOR>
	int packet_num;
	double timestamp;
	char color;
	int i = fscanf(f, "%d %lf %c\n", &packet_num, &timestamp, &color);
	while (i != EOF && i != 0)
	{
		evaluationTime.push_back(timestamp);
		evaluationColor.push_back(color);
		i = fscanf(f, "%d %lf %c\n", &packet_num, &timestamp, &color);
	}
}

//CALLBACK FUNCTIONS:

static void shutdown_module(int x)
{
/*	FILE* out = fopen("time_distance.csv", "a");
	for (int i = 0; i < timeDist.size(); i ++)
		fprintf(out, "%14.14g, %14.14g\n", timeDist[i].time, timeDist[i].distance);
	fclose(out);
*/
	if (x == SIGINT)
	{
		quit_signal = true;

		if (evaluation_mode)
		{
			FILE* resultFile = fopen("lightResults_auto", "w");
			std::list<detectionResult>::iterator i = results.begin();

			fprintf(resultFile, "confidence,intersection,number,distance,accuracy,state\n");
			
			while (i != results.end())
			{
				fprintf(resultFile, "%lf,%d,%d,%d,%d,%c\n", i->confidence, i->intersection, 
						i->number, i->distance, i->accuracy, i->state);
				i++;
			}

			fclose(resultFile);
		}

		printf("Shutting down...\n");
		printf("Disabling localization updates.\n"); fflush(stdout);
		pthread_mutex_lock(&localize_mutex);
		printf("Disabling applanix updates.\n"); fflush(stdout);
		pthread_mutex_lock(&applanix_mutex);
		printf("Disabling pose updates.\n"); fflush(stdout);
		pthread_mutex_lock(&tl_poses_mutex);
		printf("Disabling camera updates.\n"); fflush(stdout);
		pthread_mutex_lock(&camera_mutex);

		printf("Destroying windows.\n"); fflush(stdout);
		cvDestroyAllWindows();

		if (evaluation_mode)
			printf("Final evaluation results (CD:ID) -- %d:%d\n", totalCorrectFrames, totalIncorrectFrames);

		if (writeVideo && !writeVideoAsFrames) cvReleaseVideoWriter(&cvwriter);

		if (full_img != NULL) cvReleaseImageHeader(&full_img);
		if (half_img != NULL) cvReleaseImage(&half_img);

		ProfilerStop();
		printf("Disconnecting from IPC.\n");
		ipc->Disconnect();

//		signal(SIGINT, shutdown_module);

		::exit(0);
	}
}

void applanix_handler(ApplanixPose *pose)
{
	pthread_mutex_lock(&localize_mutex);
	
	//update buffer
	if (loc_pose.x_offset == 0 || loc_pose.y_offset == 0)
	{
		getUTMOffset(pose, &loc_pose.x_offset, &loc_pose.y_offset);
	}
	pose_buffer2.globalX = pose->smooth_x + loc_pose.x_offset;
	pose_buffer2.globalY = pose->smooth_y + loc_pose.y_offset;

	pose_buffer2.currPose = *pose;
	pose_buffer2.time_last_update = dgc_get_time();

	recentPoses.push_back(pose_buffer2);
	while( recentPoses.size() > 100) recentPoses.pop_front();

	pthread_mutex_unlock(&localize_mutex);
}

void param_change_handler(void)
{
	dgc_die("#Parameter change detected! This is currently not handled. "
			"Module restart required for parameter change to take effect.\n");
}

void trafficLightPoseHandler(TrafficLightPoseList* new_tl_poses)
{
	if (!new_tl_poses)
	{
		return;
	}

	size_t s = tl_poses.size();
	for (int i=0; i<new_tl_poses->num_traffic_lights; i++)
	{
		// is traffic light already in the list of currently watched
		// traffic lights?
		std::map<std::string, TrafficLightPose>::const_iterator
			tl_pose = tl_poses.find(new_tl_poses->light_pose[i].name);

		if (tl_pose == tl_poses.end())
		{
			tl_poses.insert(make_pair(
			                    std::string(new_tl_poses->light_pose[i].name),
			                    new_tl_poses->light_pose[i]));
		}
	}
	if (s < tl_poses.size()) printf("new poses found.\n");
}

//METHODS FOR REGISTERING AND PUBLISHING TrafficLightState status and heartbeat MESSAGES:

void dgc_TrafficLightState_register_ipc_messages()
{
	int err;

	err = ipc->DefineMessage(TrafficLightListMsgID);
	TestIpcExit(err, "Could not define", TrafficLightListMsgID);

	err = ipc->DefineMessage(HeartbeatID);
	TestIpcExit(err, "Could not define", HeartbeatID);

}

void dgc_TrafficLightList_publish_status_message(TrafficLightList* message)
{
	int err;

	err = ipc->Publish(TrafficLightListMsgID, (TrafficLightList *)message);
	TestIpcExit(err, "Could not publish", TrafficLightListMsgID);
}

void dgc_TrafficLightState_publish_heartbeat_message(void)
{
	int err;
	heartbeat.timestamp = dgc_get_time();
	err = ipc->Publish(HeartbeatID, &heartbeat);
	TestIpcExit(err, "Could not publish", HeartbeatID);
}

//TRAFFIC LIGHT STATE DETECTION METHODS:

void decide_state_posterior_based
(DetectionGrid *dg, HistogramFilterTL *hf, int *posterior_u_win, int *posterior_v_win,
 char *posterior_state, float *confidence)
{
	IplImage *grid_prior = hf->getGridPrior();
	float max_val = 0.0;
	char winner_state = 'u';
	int u_win = -1;
	int v_win = -1;
	for (int y = 0; y < grid_prior->height; y++)
	{
		for (int x = 0; x < grid_prior->width; x++)
		{
			float prob = ((float *)(grid_prior->imageData + y*grid_prior->widthStep))[x];
			if (prob > max_val)
			{
				max_val = prob;
				dg->getBestScore(x, y, &winner_state);
				u_win = dg->getCell(x,y)->u_center;
				v_win = dg->getCell(x,y)->v_center;
			}
		}
	}

	*posterior_u_win = u_win;
	*posterior_v_win = v_win;
	*posterior_state = winner_state;
	*confidence = max_val;
}

bool detect_state
(IplImage *curr_image, std::string& light_ID, int *u, int *v, double *confidence, 
 int *u_win, int *v_win, char *state_final, int index, double* p)
{
	// work around for offset due to inaccurate extrinsic parameters.
	double altitude = lights[light_ID].altitude + 2.5;

	char state = 'u';
	bool first_time = false;
	int im_width = curr_image->width;
	int im_height = curr_image->height;

	// check if we've already initialized structures for this light;
	// if not, create them
	if (light_grids.count(light_ID) == 0)
	{
		light_grids[light_ID] = new DetectionGrid(lights[light_ID].globalX,
		        lights[light_ID].globalY, altitude,
		        grid_width, grid_height, grid_spacing, &tt, NO_YELLOW);
		first_time = true;
	}

	DetectionGrid *curr_grid = light_grids[light_ID];

	int u_prior, v_prior;
	double robotX = 1.0;

	if (first_time)   // calculate proj. of global light pos. onto image plane
	{
		tt.globalToUV(im_width, im_height, lights[light_ID].globalX,
		               lights[light_ID].globalY, lights[light_ID].altitude,
		               &u_prior, &v_prior, &robotX, loc_pose, pose_buffer.currPose);
	}
	else   // data already stored in detection grid
	{
		robotX = curr_grid->getLightRobotX();
	}

	// stop here if this light is behind us or we're taking too long to calculate
	if (robotX < 5.0) return false;

/*	DistanceGivenTime td = { camera_image->timestamp, robotX };
	timeDist.push_back(td);
*/
	if (light_histograms.count(light_ID) == 0)
		light_histograms[light_ID] =
		    new HistogramFilterTL(grid_width, grid_height);

	if (light_image_data.count(light_ID) == 0)
		light_image_data[light_ID] = new ImageData(debug_mode, 
			&red_hue_table, &ylw_hue_table, &grn_hue_table, &saturation_table, 
			NULL);//debugWindows);

	HistogramFilterTL *curr_filter = light_histograms[light_ID];
	ImageData *curr_image_data = light_image_data[light_ID];

	// Update location and projection of corners
	pthread_mutex_lock(&localize_mutex);
	curr_grid->UpdateDetectionGrid(&loc_pose, &pose_buffer.currPose,
			lights[light_ID].orientation, curr_image->width, curr_image->height,
			lights[light_ID].downsample);
	pthread_mutex_unlock(&localize_mutex);

	const int scale = 5;
	CvRect subIm = { 0, videoframe->height - (index+1)*grid_height*scale - 1, 
		grid_width*scale, grid_height*scale };
	
	bool pass = curr_image_data->UpdateDataScores(curr_grid, curr_image,
			lights[light_ID].curr_distance, lights[light_ID].downsample,
			videoframe, subIm, p);

	if (!pass) return false;

	curr_filter->UpdateHistogramFilterTL(curr_grid, 
			pose_buffer.currPose.speed, grid_spacing);

	// decide overall state for light
	float conf = 0.5;
	decide_state_posterior_based(curr_grid, curr_filter, u_win, v_win, &state, &conf);
	*confidence = std::max(p[0], std::max(p[1], p[2]));

	// return current light prior location
	*u = curr_grid->getLightU();
	*v = curr_grid->getLightV();

	*state_final = state;
	return true; 
}

char detect_state_arrow
(IplImage* currImage __attribute__((unused)), std::string& light_ID __attribute__((unused)))
{
	//TODO
	return 'u';
}

// all lights that are sent by planner should be active
// this function checks if we might have passed a light already
// or the angle to the lights are unsuited

bool determine_active_lights()
{
	bool near_lights = active_light_IDs.size() > 0;
	vector<std::string> to_erase;

	if (NO_PLANNER_MODE)
	{
		map<std::string, lightAndPos>::iterator lightIter;
		for (lightIter = lights.begin(); lightIter != lights.end(); 
				lightIter ++)
		{
			double a = lightIter->second.globalX - pose_buffer.globalX;
			double b = lightIter->second.globalY - pose_buffer.globalY;
			double d = sqrt(pow(a,2) + pow(b,2));

			if (stop_looking_dist <= d && d <= start_looking_dist)
			{
				double lat, lon;
				const char zone[] = "51S";
				utmToLatLong(lightIter->second.globalX, 
						lightIter->second.globalY, zone, &lat, &lon);
				TrafficLightPose t =
				{
					"", lat, lon,
					lightIter->second.altitude,
					lightIter->second.orientation
				};

				strncpy(t.name, lightIter->first.c_str(), 20);

				tl_poses[lightIter->first] = t;
			}
		}
	}

	while (!tl_poses.empty())
	{
		TrafficLightPose tl_pose = tl_poses.begin()->second;
		std::string tl_name = tl_poses.begin()->first;
		tl_poses.erase(tl_poses.begin());

		char zone[4];
		double tl_utm_x, tl_utm_y;
		latLongToUtm(tl_pose.lat, tl_pose.lon, &tl_utm_x, &tl_utm_y, zone);

		pthread_mutex_lock(&applanix_mutex);

		double a = tl_utm_x - pose_buffer.globalX;
		double b = tl_utm_y - pose_buffer.globalY;
		double distance = sqrt(pow(a,2) + pow(b,2));
		double face_angle = abs(M_PI - fmod(abs(pose_buffer.currPose.yaw -
		                                        tl_pose.orientation), 2*M_PI));

		LocalizePose lp = loc_pose;
		applanixBuffer ab = pose_buffer;
		int u, v;
		double robotx;
		int width = camera_image->info.width;
		int height = camera_image->info.height;

		pthread_mutex_unlock(&applanix_mutex);

		if ((distance < secondary_dist && use_secondary_camera == false) || 
			 (distance >= secondary_dist && use_secondary_camera == true))
		{
			//first check if we already have grids - if so, delete the ones that relate to old image size values
			if (light_grids.count(tl_name) != 0)
			{
				delete light_grids[tl_name];
				light_grids.erase(tl_name);
			}

			if (light_image_data.count(tl_name) != 0)
			{
				delete light_image_data[tl_name];
				light_image_data.erase(tl_name);
			}

			if (light_histograms.count(tl_name) != 0)
			{
				//light_histograms[tl_name]->Initialize();
			}

			use_secondary_camera = !use_secondary_camera;
		}

		if ((distance < downsample_dist && downsample == false) ||
			 (distance >= downsample_dist && downsample == true))
		{
			//first check if we already have grids - if so, delete the ones that relate to old image size values
			if (light_grids.count(tl_name) != 0)
			{
				delete light_grids[tl_name];
				light_grids.erase(tl_name);
			}

			if (light_image_data.count(tl_name) != 0)
			{
				delete light_image_data[tl_name];
				light_image_data.erase(tl_name);
			}

			if (light_histograms.count(tl_name) != 0)
			{
				//light_histograms[tl_name]->Initialize();
			}
			downsample = !downsample;
		}

		//initialize transform functions for this camera
		if (!use_secondary_camera)
			tt.set_camera_params(pri);
		else
			tt.set_camera_params(sec);

		tt.globalToUV(width, height, tl_utm_x, tl_utm_y, 
				tl_pose.z, &u, &v, &robotx, lp, ab.currPose);

		bool light_was_not_active = lights.find(tl_name) == lights.end();
			
		if ((!NO_PLANNER_MODE || face_angle < 0.75) && robotx >= 5.)
		{
			near_lights = true;
			//
			//push back this node's ID
			active_light_IDs.insert(tl_name);
			lightAndPos& light_and_pos = lights[tl_name];
			
			if (light_was_not_active) 
			{
				strcpy(light_and_pos.light.name, tl_name.c_str());
				light_and_pos.light.state = 'u';
				light_and_pos.light.state_arrow = 'n';      // //add arrow info later
				light_and_pos.light.timestamp_rg_switch = -1;
				light_and_pos.light.timestamp_gy_switch = -1;
				light_and_pos.light.timestamp = -1;
				light_and_pos.light.confidence = -1;
				light_and_pos.light.u = 0;
				light_and_pos.light.v = 0;
			}

			light_and_pos.globalX = tl_utm_x;
			light_and_pos.globalY = tl_utm_y;
			light_and_pos.altitude = tl_pose.z;
			light_and_pos.orientation = tl_pose.orientation;
			light_and_pos.curr_distance = distance;
			light_and_pos.face_angle = face_angle;
			light_and_pos.downsample = downsample;
			light_and_pos.use_secondary_camera = use_secondary_camera;
			light_and_pos.robotx = robotx;

		}
	}

	// look at active lights, see if we've passed them already - if so, delete them from our data structures
	set<std::string>::const_iterator atlit = active_light_IDs.begin(), 
		atlit_end = active_light_IDs.end();
	for (; atlit != atlit_end; atlit++)
	{
		std::string id = *(atlit);
		map<std::string, DetectionGrid*>::iterator x = light_grids.find(id);
		if (x != light_grids.end() && x->second->getLightRobotX() < 5.)
		{
			//delete the actual objects
			delete light_grids[id];
			delete light_histograms[id];
			delete light_image_data[id];
			//remove references to these objects from maps
			light_grids.erase(id);
			light_histograms.erase(id);
			light_image_data.erase(id);

			to_erase.push_back(id);
		}
	}

	while (to_erase.size() > 0)
	{
		active_light_IDs.erase(to_erase.back());
		to_erase.pop_back();
	}

	return near_lights;
}

//LIGHT STATE DETECTION
//if no lights are near, then only a heartbeat message will be published
//otherwise, state messages will be published for each light
void find_lights_detect_state()
{
	//see which set of lights are the closest
	pthread_mutex_lock(&tl_poses_mutex);
	bool close_to_light = determine_active_lights();
	pthread_mutex_unlock(&tl_poses_mutex);

	if (writeVideo || debug_mode)
	{
		cvConvertImage(full_img, videoframe, CV_CVTIMG_SWAP_RB);
	}

	if (!close_to_light || pose_buffer.time_last_update == -1) 
		dgc_TrafficLightState_publish_heartbeat_message();
	
	else
	{
		// Unnecessary if camera is using manual exposure/gain/shutter speed.
		if (NORMALIZE)
		{
			// replace this with [ward02] (or anything better than RGB stretching)
			IplImage* rgb[3];
			for (int i = 0; i < 3; i ++) rgb[i] = cvCreateImage(cvGetSize(full_img), 8, 1);
			cvCvtPixToPlane(full_img, rgb[0], rgb[1], rgb[2], NULL);
			for (int i = 0; i < 3; i ++) cvNormalize(rgb[i], rgb[i], 0, 255, CV_MINMAX);
			cvCvtPlaneToPix(rgb[0], rgb[1], rgb[2], NULL, full_img);
			for (int i = 0; i < 3; i ++) cvReleaseImage(&rgb[i]);
		}

		//iterate through active lights vector
		std::string currID;
		set<std::string>::const_iterator 
			atlit = active_light_IDs.begin(), 
			atlit_end = active_light_IDs.end();
		
		bool this_frame_has_not_been_downsampled = true;

		if (atlit != atlit_end) printf(
				"\nname    dist            face_agl        conf          state\n");
	
		int index = 0;

		std::set<int> interState;

		for (; atlit != atlit_end; atlit++)
		{
			currID = *(atlit);
			IplImage* img;

			TrafficLightState *currLight = &(lights[currID].light);
			std::map<std::string, lightAndPos>::iterator lap = lights.find(currID);

			// Hystoresis counters.
			int& counter_red    = lap->second.counter_red;
			int& counter_yellow = lap->second.counter_yellow;
			int& counter_green  = lap->second.counter_green;

			// Intersection state variables.
			int inter;   // Intersection number
			int number;  // Light number
			int scanCount = sscanf(currID.c_str(), "%d.%d", &inter, &number);
			if( scanCount == 0) 
			{
				inter = 0;
				number = 0;
			}

			interState.insert(inter);

			// Use downsampled or full scale image?
			if (lap->second.downsample)
			{
				if (this_frame_has_not_been_downsampled) 
				{
					cvResize(full_img, half_img, CV_INTER_LINEAR);
					this_frame_has_not_been_downsampled = false;
				}
				img = half_img; 
			}
			else
			{
				img = full_img;
			}

			// Detect state of light.
			int    u_win, v_win;
			bool   reliable_state;
			char   prevState = currLight->state;
			char   state = 'u';
			char   publishedState = 'u';
			double conf = 0;
			double p[3];

			reliable_state = detect_state(img, currID, &currLight->u, 
				&currLight->v, &conf, &u_win, &v_win, &state, index, p);

			// Get time.
			double current_time = dgc_get_time();
			
			// Handle downsample to full scale (u,v) transform.
			if (lap->second.downsample)
			{
				currLight->u *= 2;
				currLight->v *= 2;
			}

			// Handle switch timer tracking.
			if (state != prevState)
			{
				if (prevState == 'r' && state == 'g')
					currLight->timestamp_rg_switch = current_time;

				else if (prevState == 'g' && state == 'y')
					currLight->timestamp_gy_switch = current_time;

			}
			
			// Save light state to 'currLight'.
			currLight->confidence = conf;
			currLight->state = state;
			currLight->timestamp = current_time;

			// Update intersection state probabilities.
			interProb[inter][number][0] = p[0];
			interProb[inter][number][1] = p[1];
			interProb[inter][number][2] = p[2];

			// Display results to screen.
			printf("%7.7s\t%7.7lf\t%7.7lf\t%7.7lf\t", currID.c_str(), lap->second.curr_distance, 
					lap->second.face_angle, conf);

			// Store results to 'result'.
			detectionResult result = { conf, inter, number, lap->second.curr_distance, 0, 0 }; 

			// If we detected something,
			if (reliable_state)
			{
				// Handle hystoresis counters.
				if (currLight->state == 'r')
				{
					counter_red++;
					counter_yellow = 0;
					counter_green = 0;
				}
				else if (currLight->state == 'y')
				{
					counter_red = 0;
					counter_yellow++;
					counter_green = 0;
				}
				else if (currLight->state == 'g')
				{
					counter_red = 0;
					counter_yellow = 0;
					counter_green++;
				}
				else
				{
					counter_red = 0;
					counter_green = 0;
					counter_yellow = 0;
				}

				// If the hystoresis passes,
				if (counter_red > 1 || counter_green > 1 || counter_yellow > 1) 
				{
					// Display and publish state.
					printf("%c\n", state);
					tl_list_msg.num_light_states = 1;
					tl_list_msg.light_state = currLight;
					dgc_TrafficLightList_publish_status_message(&tl_list_msg);
					//if (evaluation_mode) evaluate(currLight->state);
					publishedState = currLight->state;

					intersection[inter][number] = state;
					result.state = state;
					result.confidence = conf;
				}

				// Remove single frame mis-fires
				else if (prevState != state && state != 'u' && prevState != 'u')
				{
					printf("u\n");
					tl_list_msg.num_light_states = 1;
					currLight->state = prevState;
					TrafficLightState tempLight = *currLight;
					tl_list_msg.light_state = &tempLight;
					dgc_TrafficLightList_publish_status_message(&tl_list_msg);
					publishedState = prevState;
					currLight->state = state;

					intersection[inter][number] = 'u';
					result.state = 'u';
					result.confidence = 0.;
				}

				// Hystoresis failed.
				else
				{
					printf("u\n");
					TrafficLightState unknown = *currLight;
					unknown.state = 'u';
					tl_list_msg.num_light_states = 1;
					tl_list_msg.light_state = &unknown;
					publishedState = 'u';
					dgc_TrafficLightList_publish_status_message(&tl_list_msg);

					intersection[inter][number] = 'u';
					result.state = 'u';
					result.confidence = 0.;
				}
			}

			// Detection failed to find a reliable state.
			else
			{
				counter_red = 0;
				counter_green = 0;
				counter_yellow = 0;

				printf("u\n");
				TrafficLightState unknown = *currLight;
				unknown.state = 'u';
				tl_list_msg.num_light_states = 1;
				tl_list_msg.light_state = &unknown;
				publishedState = 'u';
				dgc_TrafficLightList_publish_status_message(&tl_list_msg);

				intersection[inter][number] = 'u';
				result.state = 'u';
				result.confidence = 0.;
			}

			// Update light distances.
			interDistance[inter][number] = lap->second.curr_distance;

			// Evaluate this result.
			if (interState.size() == 1 && result.state != 'u')
			{
				result.accuracy = evaluate(result.state);
				interEvaluate[inter][number] = result.accuracy;
				results.push_back(result);
			}

			// Draw boxes on display frame.
			if (writeVideo || debug_mode)
			{
				int min_u = light_grids[currID]->getMinU();
				int max_u = light_grids[currID]->getMaxU();
				int min_v = light_grids[currID]->getMinV();
				int max_v = light_grids[currID]->getMaxV();

				uchar rvalp, gvalp, bvalp;
				rvalp = gvalp = bvalp = 0;

				switch (publishedState)
				{
					case 'r': rvalp = 255; break;
					case 'g': gvalp = 255; break;
					case 'y': rvalp = gvalp = 255; break;
				}

				int half_width = -1, half_height = -1;

				if (light_image_data.count(currID) > 0)
				{
					 half_width = light_image_data[currID]->getLastTplWidth()/2;
					 half_height = light_image_data[currID]->getLastTplHeight()/2;
				}

				if (half_width > 0 && half_height > 0 && min_u > 0 && max_u > 0 && min_v > 0 && max_v > 0 &&
						min_u < max_u && min_v < max_v && max_u < full_img->width && max_v < full_img->height)
				{
					int mult = (lap->second.downsample) ? 2 : 1;

					CvPoint p1={mult*(u_win - half_width), mult * (v_win - half_height) };
					CvPoint p2={mult*(u_win + half_width), mult * (v_win + half_height) };
					CvScalar color = cvScalar( bvalp, gvalp, rvalp, 0 );
					cvRectangle(videoframe, p1, p2, color, 2);

					CvPoint p3 = {	mult * (min_u), mult * (min_v) };
					CvPoint p4 = {	mult * (max_u), mult * (max_v) };
					CvScalar color2 = cvScalar( 255, 0, 0, 0 );
					cvRectangle(videoframe, p3, p4, color2, 1);

					CvFont font;
					double hScale= mult*((double)(max_u - min_u)/120.);
					double vScale= mult*((double)(max_v - min_v)/160.);
					int    lineWidth=1;
					
					cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);
					CvPoint p5 = { mult * (min_u) + 10*hScale, mult * (min_v) + 30*vScale };
					cvPutText (videoframe, currID.c_str(), p5, &font, cvScalar(255,255,255));
				} // if light is in frame, draw light box
			} // if gui or video capture

			index ++;
		} // active light for-loop

		// calculate state for each intersection
		if (evaluation_mode)
		{
			// interProb
			std::vector<double> p_vec[3];
			std::set<int>::iterator z = interState.begin();
			while (z != interState.end())
			{
				std::map<char,int> stateCount;
				int maxCount = 0;
				int maxState = 'u';
				double avgDist = 0.;
				int n = 0; // number of lights at intersection *z

				for (int j = 0; j < MAX_LIGHTS_PER_INTERSECTION; j ++)
				{
					int state;

/*					switch(intersection[*z][j])
					{
						case 'r' : state = 0;
									  break;
						case 'y' : state = 1;
									  break;
						case 'g' : state = 2;
									  break;
						default  : state = 3;
									  break;
					}
*/
					for (state = 0; state < 3; state ++)
						if (interProb[*z][j][state] >= 0.000001)
							p_vec[state].push_back(1. - interProb[*z][j][state]); 

					//stateCount[intersection[*z][j]] ++;
					if (intersection[*z][j] != 'u') 
					{
						avgDist += (double)interDistance[*z][j];
						n ++;
					}
				}

				if (n != 0) avgDist /= (double)n;
				
				double p[3];
				double pMax = 0.;
				int stateMax = 4;

				if (avgDist > 0.0000000001)
				{

					for (int i = 0; i < 3; i ++)
					{
						p[i] = 1.;

						for (size_t k = 0; k < p_vec[i].size(); k ++)
							p[i] *= p_vec[i][k];

						p[i] = 1. - p[i];

						if (p[i] > pMax) 
						{
							pMax = p[i];
							stateMax = i;
						}
					}
				}

				if (pMax <= .3333334) stateMax = 3;

				char winner;
				switch(stateMax)
				{
					case 0 : winner = 'r';
								break;
					case 1 : winner = 'y';
								break;
					case 2 : winner = 'g';
								break;
					default : winner = 'u';
								 break;
				}

				if (z == interState.begin())
				{
					int detection = evaluate(winner);
					interEvaluate[*z][MAX_LIGHTS_PER_INTERSECTION] = detection;
					intersection[*z][MAX_LIGHTS_PER_INTERSECTION] = winner;

					detectionResult result = { pMax, *z, 0, (int)round(avgDist), detection, winner };
					results.push_back(result);
					printf("inter\tconf\t\tstate\tdist\n"
							"%d\t%7.7lf\t%c\t%7.7lf\n", *z, pMax, winner, avgDist);
				}

				z ++;
			}
		}
	} // if active lights

	fflush(stdout);
}

void keyboard(char key)
{
	FILE* out;

	switch (key)
	{
	case 'D' :
	case 'd' : 
		printf("Toggling debug mode.\n");
		debug_mode = !debug_mode; 
		break;
	case 'e' :
	case 'E' : 
		evaluation_mode = !evaluation_mode; 
		printf("Toggling eval mode to %s.\n", (evaluation_mode ? "true" : "false")); 
		out = fopen("evalLog", "a");
		fprintf(out, "EvalMode %d@%lg\n", evaluation_mode, camera_image->timestamp);
		fclose(out);
		break;
	case 'Y' : 
	case 'y' : 
		printf("Toggling recognizing yellow light state.\n");
		NO_YELLOW = !NO_YELLOW; 
		break;
	case 'W' :
	case 'w' :
		printf("Toggling writing to video file out.avi\n");
		writeVideo = !writeVideo;
		debug_mode = writeVideo;
		break;
	case 'S' :
	case 's' :
		printf("Toggling stretching RGB input channels.\n");
		NORMALIZE = !NORMALIZE;
		break;
	case 27:
	case 'q':
	case 'Q':
		shutdown_module(SIGINT);
		break;
	case 'P' :
	case 'p' :
		if (evaluation_mode)
		{
			printf("saving lightResults...\n");
			FILE* resultFile = fopen("lightResults", "w");
			std::list<detectionResult>::iterator i = results.begin();

			fprintf(resultFile, "confidence,intersection,number,distance,accuracy,state\n");
			
			while (i != results.end())
			{
				fprintf(resultFile, "%lf,%d,%d,%d,%d,%c\n", i->confidence, i->intersection, 
						i->number, i->distance, i->accuracy, i->state);
				i++;
			}

			fclose(resultFile);
		}
		break;
	default:
		break;
	}

}

void display()
{
	if ((writeVideo || debug_mode) && videoframe != NULL)
	{
		int index = 0;
		set<std::string>::const_iterator lightID = active_light_IDs.begin(); 

		while (lightID != active_light_IDs.end())
		{
			map<std::string, HistogramFilterTL*>::iterator 
				currHist = light_histograms.find(*lightID);

			int dist = lights[*lightID].robotx;

			if (dist > 0)
			{
				IplImage *grid_winners = currHist->second->getWinnersImage();
				IplImage *grid_prior   = currHist->second->getGridPrior();

				IplImage* winners = cvCreateImage(cvGetSize(grid_winners), 8, 3);
				IplImage* prior   = cvCreateImage(cvGetSize(grid_prior),   8, 3);

				cvConvertImage(grid_winners, winners, CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);
				cvConvertImage(grid_prior,   prior,   CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);

	//			cvShowImage(debugWindows[1], winners);
	//			cvShowImage(debugWindows[2], prior);

				const int scale = 5;

				IplImage* disp = cvCreateImage(cvSize(scale*grid_width, scale*grid_height), 8, 3);

	/*			CvFont font;
				int chars = light_ID_dbg.size();
				double hScale = (double)(scale*grid_width) /(50.+10.*chars);
				double vScale = (double)(scale*grid_height)/(70.+10.*chars);
				int lineWidth = 2;
				cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);
				CvPoint p5 = { disp->width/2 - chars*25, videoframe->height-(index+1)*(scale*grid_height/3) };
				cvPutText (videoframe, light_ID_dbg.c_str(), p5, &font, cvScalar(255,255,255));
	*/
				cvResize(winners, disp, CV_INTER_LINEAR);
				CvRect subIm = { disp->width, videoframe->height - (index+1)*disp->height - 1, 
					disp->width, disp->height };
				
				if (  subIm.x >= 0 && subIm.x + subIm.width < videoframe->width
					&& subIm.y >= 0 && subIm.y + subIm.height < videoframe->height)
				{
					cvSetImageROI(videoframe, subIm);
					cvCopy(disp, videoframe);
					cvResetImageROI(videoframe);
				}

				cvResize(prior, disp, CV_INTER_LINEAR);
				subIm.x = 2*disp->width;
				
				if (  subIm.x >= 0 && subIm.x + subIm.width < videoframe->width
					&& subIm.y >= 0 && subIm.y + subIm.height < videoframe->height)
				{
					cvSetImageROI(videoframe, subIm);
					cvCopy(disp, videoframe);
					cvResetImageROI(videoframe);
				}

				cvReleaseImage(&disp);
				cvReleaseImage(&winners);
				cvReleaseImage(&prior);

			}

			lightID++;
			index++;
		}

		if (debug_mode) 
		{
/*			light_ID_dbg = getLightName();
			map<std::string, HistogramFilterTL*>::iterator 
				currHist = light_histograms.find(light_ID_dbg);

			if (currHist != light_histograms.end())
			{
				IplImage *grid_winners = currHist->second->getWinnersImage();
				IplImage *grid_prior   = currHist->second->getGridPrior();

				IplImage* winners = cvCreateImage(cvGetSize(grid_winners), 8, 3);
				IplImage* prior   = cvCreateImage(cvGetSize(grid_prior),   8, 3);

				cvConvertImage(grid_winners, winners, CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);
				cvConvertImage(grid_prior,   prior,   CV_CVTIMG_SWAP_RB | CV_CVTIMG_FLIP);

//				cvShowImage(debugWindows[1], winners);
//				cvShowImage(debugWindows[2], prior);

				const int scale = 6;
				int index = 0;

				IplImage* disp = cvCreateImage(cvSize(scale*grid_width, scale*grid_height), 8, 3);

				CvFont font;
				int chars = light_ID_dbg.size();
				double hScale = (double)(scale*grid_width) /(50.+10.*chars);
				double vScale = (double)(scale*grid_height)/(70.+10.*chars);
				int lineWidth = 2;
				cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);
				CvPoint p5 = { disp->width/2 - chars*25, videoframe->height-(index+1)*(scale*grid_height/3) };
				cvPutText (videoframe, light_ID_dbg.c_str(), p5, &font, cvScalar(255,255,255));

				cvResize(winners, disp, CV_INTER_LINEAR);
				CvRect subIm = { 2*disp->width, videoframe->height - (index+1)*disp->height, disp->width, disp->height };
				cvSetImageROI(videoframe, subIm);
				cvCopy(disp, videoframe);
				cvResetImageROI(videoframe);

				cvResize(prior, disp, CV_INTER_LINEAR);
				subIm.x = 3*disp->width;
				cvSetImageROI(videoframe, subIm);
				cvCopy(disp, videoframe);
				cvResetImageROI(videoframe);
				cvReleaseImage(&disp);

				cvReleaseImage(&winners);
				cvReleaseImage(&prior);
			}
*/
			cvShowImage("Frame", videoframe);
			
			keyboard(cvWaitKey(15));
		}
	}
}

void * camera_thread_function(__attribute__ ((unused)) void *ptr)
{
	/* reading images from camera */
	while (!quit_signal)
	{
		while (!camera_interface->ImagesWaiting() && !quit_signal) 
		{
			usleep(10000);
			display();
		}
		
		while (camera_interface->ImagesWaiting() && !quit_signal)
		{
			pthread_mutex_lock(&camera_mutex);

			// Real time
			if (dropframes && camera_interface->ReadCurrentImage(camera_image) < 0)
				dgc_fatal_error("Image read failed.\n");
			
			// Process all images
			if (!dropframes && camera_interface->ReadImage(camera_image) < 0)
				dgc_fatal_error("Image read failed.\n");

			pthread_mutex_lock(&applanix_mutex);
			
			std::list<applanixBuffer>::iterator i = recentPoses.begin();
			double minTimeDiff = 1.e307;

			while( i != recentPoses.end())
			{
				double timeDiff = abs(i->currPose.timestamp - camera_image->timestamp);
				if( timeDiff < minTimeDiff)
				{
					pose_buffer = *i;
					minTimeDiff = timeDiff;
				}
				i++;
			}
			
			pthread_mutex_unlock(&applanix_mutex);

			if (full_img == NULL || (int64_t)camera_image->info.width != (int64_t)full_img->width || 
					(int64_t)camera_image->info.height != (int64_t)full_img->height)
			{
				int width = camera_image->info.width;
				int height = camera_image->info.height;

				if (full_img != NULL) cvReleaseImageHeader(&full_img);

				full_img = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 3);
				full_img->imageData = (char*)camera_image->data;

				if (half_img != NULL) cvReleaseImage(&half_img);
				half_img = cvCreateImage(cvSize(width/2, height/2), IPL_DEPTH_8U, 3);

				if (debug_mode) 
				{
					if (videoframe != NULL) cvReleaseImage(&videoframe);
					videoframe = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
					cvResizeWindow("Frame",2*width/3, 2*height/3);
				}
			}

			find_lights_detect_state();

			pthread_mutex_unlock(&camera_mutex);

			display();

			// write video (or png)
			if (writeVideo && !writeVideoAsFrames) cvWriteFrame(cvwriter, videoframe);
			else if (writeVideoAsFrames)
			{
				char frame_name[25];
				static int frame_number = 764;
				sprintf(frame_name, "frame_%05d.png", frame_number++);
				cvSaveImage(frame_name, videoframe);
			}

		}
	}
	return NULL;
}

void read_parameters(ParamInterface *pint, int argc, char **argv)
{
	char* hsv_table_dir;
	char* red_hue_filename;
	char* ylw_hue_filename;
	char* grn_hue_filename;
	char* saturation_filename;
	char* primary_camera;
	char* secondary_camera;
	char* pri_trans_filename;
	char* sec_trans_filename;

	int pri_cam_num;
	int sec_cam_num;

	std::string strRedHueTable;
	std::string strGrnHueTable;
	std::string strYlwHueTable;
	std::string strSaturationTable;

	// shorthand for DGC_PARAM_... from interface/param_server/param_interface.h
	ParamType P_INT       = DGC_PARAM_INT;
	ParamType P_DOUBLE    = DGC_PARAM_DOUBLE;
	ParamType P_ONOFF     = DGC_PARAM_ONOFF;
	ParamType P_STRING    = DGC_PARAM_STRING;
	ParamType P_FILENAME  = DGC_PARAM_FILENAME;

	Param params[] =
	{
		{"rndf", 			"rndf_file", 				P_FILENAME, &rndf_filename,      1,ParamCB(param_change_handler)},
		{"trafficlights", "hsv_table_directory",	P_FILENAME, &hsv_table_dir,      1,ParamCB(param_change_handler)},
		{"trafficlights", "red_hue_table", 			P_FILENAME, &red_hue_filename,   1,ParamCB(param_change_handler)},
		{"trafficlights", "ylw_hue_table", 			P_FILENAME, &ylw_hue_filename,   1,ParamCB(param_change_handler)},
		{"trafficlights", "grn_hue_table", 			P_FILENAME, &grn_hue_filename,   1,ParamCB(param_change_handler)},
		{"trafficlights", "saturation_table", 		P_FILENAME, &saturation_filename,1,ParamCB(param_change_handler)},
		{"trafficlights", "start_looking_m",      P_DOUBLE,   &start_looking_dist, 1,ParamCB(param_change_handler)},
		{"trafficlights", "stop_looking_m",       P_DOUBLE,   &stop_looking_dist,  1,ParamCB(param_change_handler)},
		{"trafficlights", "downsample_distance_m",P_DOUBLE,   &downsample_dist,    1,ParamCB(param_change_handler)},
		{"trafficlights", "grid_width_cells",     P_INT,      &grid_width,         1,ParamCB(param_change_handler)},
		{"trafficlights", "grid_height_cells",    P_INT,      &grid_height,        1,ParamCB(param_change_handler)},
		{"trafficlights", "grid_spacing_cells",   P_DOUBLE,   &grid_spacing,       1,ParamCB(param_change_handler)},
		{"trafficlights", "dropframes",           P_ONOFF,    &dropframes,         1,ParamCB(param_change_handler)},
		{"trafficlights", "primary_camera",       P_STRING,   &primary_camera,     1,ParamCB(param_change_handler)},
		{"trafficlights", "secondary_camera",     P_STRING,   &secondary_camera,   1,ParamCB(param_change_handler)},
		{"trafficlights", "secondary_distance_m", P_INT,      &secondary_dist,     1,ParamCB(param_change_handler)}
	};
	pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

	Param params2[] =
	{
		{primary_camera,  "camera_num",           P_INT,      &pri_cam_num,        1,ParamCB(param_change_handler)},
		{primary_camera,  "image_fmt",            P_INT,      &pri.info.format,    1,ParamCB(param_change_handler)},
		{primary_camera,  "frame_rate",           P_INT,      &pri.frame_rate,     1,ParamCB(param_change_handler)},
		{primary_camera,  "roi_top",              P_INT,      &pri.roi.y,          1,ParamCB(param_change_handler)},
		{primary_camera,  "roi_left",             P_INT,      &pri.roi.x,          1,ParamCB(param_change_handler)},
		{primary_camera,  "roi_height",           P_INT,      &pri.roi.height,     1,ParamCB(param_change_handler)},
		{primary_camera,  "roi_width",            P_INT,      &pri.roi.width,      1,ParamCB(param_change_handler)},
		{primary_camera,  "pixel_size_um",        P_DOUBLE,   &pri.pixel_size,     1,ParamCB(param_change_handler)},
		{primary_camera,  "max_res_width",        P_INT,      &pri.info.width,     1,ParamCB(param_change_handler)},
		{primary_camera,  "max_res_height",       P_INT,      &pri.info.height,    1,ParamCB(param_change_handler)},
		{primary_camera,  "cx",                   P_DOUBLE,   &pri.cx,             1,ParamCB(param_change_handler)},
		{primary_camera,  "cy",                   P_DOUBLE,   &pri.cy,             1,ParamCB(param_change_handler)},
		{primary_camera,  "Fx",                   P_DOUBLE,   &pri.Fx,             1,ParamCB(param_change_handler)},
		{primary_camera,  "Fy",                   P_DOUBLE,   &pri.Fy,             1,ParamCB(param_change_handler)},
		{"transform",     primary_camera,         P_FILENAME, &pri_trans_filename, 1,ParamCB(param_change_handler)}, 

		{secondary_camera,"camera_num",           P_INT,      &sec_cam_num,        1,ParamCB(param_change_handler)},
		{secondary_camera,"image_fmt",            P_INT,      &sec.info.format,    1,ParamCB(param_change_handler)},
		{secondary_camera,"frame_rate",           P_INT,      &sec.frame_rate,     1,ParamCB(param_change_handler)},
		{secondary_camera,"roi_top",              P_INT,      &sec.roi.y,          1,ParamCB(param_change_handler)},
		{secondary_camera,"roi_left",             P_INT,      &sec.roi.x,          1,ParamCB(param_change_handler)},
		{secondary_camera,"roi_height",           P_INT,      &sec.roi.height,     1,ParamCB(param_change_handler)},
		{secondary_camera,"roi_width",            P_INT,      &sec.roi.width,      1,ParamCB(param_change_handler)},
		{secondary_camera,"pixel_size_um",        P_DOUBLE,   &sec.pixel_size,     1,ParamCB(param_change_handler)},
		{secondary_camera,"max_res_width",        P_INT,      &sec.info.width,     1,ParamCB(param_change_handler)},
		{secondary_camera,"max_res_height",       P_INT,      &sec.info.height,    1,ParamCB(param_change_handler)},
		{secondary_camera,"cx",                   P_DOUBLE,   &sec.cx,             1,ParamCB(param_change_handler)},
		{secondary_camera,"cy",                   P_DOUBLE,   &sec.cy,             1,ParamCB(param_change_handler)},
		{secondary_camera,"Fx",                   P_DOUBLE,   &sec.Fx,             1,ParamCB(param_change_handler)},
		{secondary_camera,"Fy",                   P_DOUBLE,   &sec.Fy,             1,ParamCB(param_change_handler)},
		{"transform",     secondary_camera,       P_FILENAME, &sec_trans_filename, 1,ParamCB(param_change_handler)}
	};
	pint->InstallParams(argc, argv, params2, sizeof(params2) / sizeof(params2[0]));

	dgc_transform_read(pri.trans, pri_trans_filename);
	dgc_transform_read(sec.trans, sec_trans_filename);

	strRedHueTable = hsv_table_dir;
	strYlwHueTable = hsv_table_dir;
	strGrnHueTable = hsv_table_dir;
	strSaturationTable = hsv_table_dir;

	strRedHueTable += "/";
	strYlwHueTable += "/";
	strGrnHueTable += "/";
	strSaturationTable += "/";

	strRedHueTable += red_hue_filename;
	strYlwHueTable += ylw_hue_filename;
	strGrnHueTable += grn_hue_filename;
	strSaturationTable += saturation_filename;
	
	if (NO_PLANNER_MODE)
		printf("start/downsample/stop distances: %lf %lf %lf\n", 
			start_looking_dist, downsample_dist, stop_looking_dist);

	printf("Using shape tables: \n\t%s\n", strRedHueTable.c_str());
	printf("\t%s\n", strYlwHueTable.c_str());
	printf("\t%s\n", strGrnHueTable.c_str());
	printf("\t%s\n", strSaturationTable.c_str());

	red_hue_table.Load(strRedHueTable.c_str());
	ylw_hue_table.Load(strYlwHueTable.c_str());
	grn_hue_table.Load(strGrnHueTable.c_str());
	saturation_table.Load(strSaturationTable.c_str());
}


int main(int argc, char **argv)
{
	ProfilerStart("tl_profile");
	ippSetNumThreads(1);
	cvNamedWindow("Frame", 0);
	cvMoveWindow("Frame", 0, 0);

	bool showUsage = false;
	bool exitAfter = false;
	int  lightsTxt = -1;
	int  statesTxt = -1;
	for (int i = 1; i < argc; i ++)
	{
		switch (argv[i][0])
		{
			case 'D' :
			case 'd' : 
				printf("Switching to debug mode.\n");
				debug_mode = true; 
				break;
			case 'e' :
			case 'E' : 
				printf("Switching to eval mode.\n"); 
				statesTxt = ++i;
				evaluation_mode = true; 
				break;
			case 'Y' : 
			case 'y' : 
				printf("Not recognizing yellow light state.\n");
				NO_YELLOW = true; 
				break;
			case 'N' : 
			case 'n' : 
				printf("No planner mode. Using lights.txt.\n");
				lightsTxt = ++i;
				NO_PLANNER_MODE = true; 
				break;
			case 'W' :
			case 'w' :
				printf("Writing to video file out.avi\n");
				writeVideo = true;
				debug_mode = true;
				break;
			case 'S' :
			case 's' :
				printf("Not stretching RGB input channels.\n");
				NORMALIZE = false;
				break;
			case '-' :
				if (strcmp(argv[i], "--help") == 0) 
				{
					showUsage = true;
					exitAfter = true;
					break;
				}
				// don't break here.
			default  : 
				printf("Unknown option: %s\n", argv[i]);
				showUsage = true;
				break;
		}
	}

	if (argc == 1) showUsage = true;

	if (showUsage)
	{
		// TODO: update this
		fprintf(stderr, "\nUsage: %s [D] [N <text file>] [E <text file>]\n\n"
				 "D\t\tDebug (graphics) mode\n"
				 "N [lights filename]\tNo planner mode, use text file\n"
				 "E [states filename]\tEvaluation mode, use text file\n\n"
				 "All other options are given by param_server.\n\n", argv[0]);
	}

	if (exitAfter) return 0;

	// initialize structures
	for (int i = 0; i < MAX_LIGHTS; i ++)
	{
		for (int j = 0; j < MAX_LIGHTS_PER_INTERSECTION; j ++)
		{
			intersection[i][j] = 'u';
			interDistance[i][j] = INT_MAX;
			interEvaluate[i][j] = 0;
			interProb[i][j][0] = 0.;
			interProb[i][j][1] = 0.;
			interProb[i][j][2] = 0.;
		}
		intersection[i][MAX_LIGHTS_PER_INTERSECTION] = 'u';
		interEvaluate[i][MAX_LIGHTS_PER_INTERSECTION] = 0;
	}

	/* connect to the IPC server, register messages */
	ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	ParamInterface *pint = new ParamInterface(ipc);
	read_parameters(pint, argc, argv);

	for (int i = 0; i < DEBUG_WINDOW_COUNT; i ++)
	{
		cvNamedWindow(debugWindows[i], 0);
		cvResizeWindow(debugWindows[i], 3*grid_width, 3*grid_height);
		cvMoveWindow(debugWindows[i], 3*(i+5)*(grid_width+1), pri.info.height/2+grid_height);
	}

	if (writeVideo)
	{
		cvwriter = cvCreateVideoWriter("out.avi", CV_FOURCC('P','I','M','1'),
				15, cvSize(pri.info.width, pri.info.height), 1);

		if (cvwriter == NULL)
		{
			fprintf(stderr, "CV_FOURCC returns %d\n", CV_FOURCC('P','I','M','1'));
			fprintf(stderr, "couldn't initialize video writer.\n");
			fprintf(stderr, "saving frames individually; just use the following from the command line:\n\t"
					"ffmpeg -sameq -r 15 -i frame_%%05d.png front.mp4\n\n");
			writeVideoAsFrames = true;
		}
	}
	
	// read in file containing traffic light ID's, global coordinates
	if (NO_PLANNER_MODE)
		read_traffic_lights_from_file(argv[lightsTxt]);
	else
		read_traffic_lights_from_rndf();

	//initialize heartbeat message
	strncpy(heartbeat.host, dgc_hostname(), 10);
	strcpy(heartbeat.modulename, "TRAFFIC_LIGHTS");

	//register messages
	dgc_TrafficLightState_register_ipc_messages();

	//subscribe to external messages
	ApplanixPose pose;
	memset(&pose, 0, sizeof(pose));
	pose_buffer.globalX = 0;
	pose_buffer.globalY = 0;
	pose_buffer.currPose = pose;
	pose_buffer.time_last_update = -1;

	//setup a shutdown handler

	if (ipc->Subscribe(ApplanixPoseID, &applanix_handler, 
				DGC_SUBSCRIBE_ALL, &applanix_mutex) == -1)
	{
		fprintf(stderr, "Couldn't subscribe to ipc applanix messages.\n");
		return -1;
	}

	if (ipc->Subscribe(LocalizePoseID, &loc_pose, 
				DGC_SUBSCRIBE_ALL, &localize_mutex) == -1)
	{
		fprintf(stderr, "Couldn't subscribe to ipc localize messages.\n");
		return -1;
	}

	if (ipc->Subscribe(TrafficLightPoseListMsgID, &trafficLightPoseHandler, 
				DGC_SUBSCRIBE_ALL, &tl_poses_mutex) == -1)
	{
		fprintf(stderr, "Couldn't subscribe to ipc traffic light pose messages.\n");
		return -1;
	}

	//EVALUTAION MODE
	//if e or E is entered as the second parameter, it will try to load an evaluation text file
	//and run in evaluation mode -- meaning it will compare the light classifications with ground truth
	if (evaluation_mode)
	{
		read_ground_truth_from_file(argv[statesTxt]); 
	}

	//CAMERA:
	pthread_t camera_thread;

	//connect to camera interface
	camera_interface = new CameraShmInterface;
	if (camera_interface->CreateClient(pri.info.camera_number) < 0)
		dgc_fatal_error("Could not connect to camera %d interface.", pri.info.camera_number);
	camera_image = new CameraImage;
	if (!camera_image)
	{
		fprintf(stderr, "Could not allocate camera image buffer.\n");
		return -1;
	}

	//start the camera thread
	pthread_create(&camera_thread, NULL, camera_thread_function, NULL);

	// work around for IPC error upon initial light publication
	usleep(100000);
	dgc_TrafficLightState_publish_heartbeat_message();
	TrafficLightState light;
	strcpy(light.name, "notalight");
	light.state = 'u';
	light.state_arrow = false; 
	light.timestamp_rg_switch = -1;
	light.timestamp_gy_switch = -1;
	light.timestamp = -1;
	light.confidence = -1;
	light.u = 0;
	light.v = 0;
	tl_list_msg.num_light_states = 1;
	tl_list_msg.light_state = &light;
	dgc_TrafficLightList_publish_status_message(&tl_list_msg);

	ipc->Dispatch();

	return 0;

}

#ifdef _EiC
main(1,"TRAFFIC LIGHTS");
#endif
