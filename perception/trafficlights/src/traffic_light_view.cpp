#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <param_interface.h>
#include <applanix_interface.h>
#include <localize_messages.h>
#include <heartbeat_interface.h>
#include <heartbeat_messages.h>
#include <lltransform.h>
#include <rndf.h>
#include <gui2D.h>
#include <camera_shm_interface.h>
#include <traffic_light_messages.h>
#include <transform.h>
#include <passat_constants.h>
#include <map>
#include <textures.h>
#include <cv.h>
#include <highgui.h>
#include <cxtypes.h>
#include <cxcore.h>
#include <cmath>
#include <fstream>
#include "traffic_light_transforms.h"

#define TIME_THRESHOLD 2

using namespace dgc;
using namespace std;
using namespace vlr;

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

//globals
bool debug_mode = false;
bool record = false;
static bool collectLights = false;
static bool NORMALIZE = false;

TrafficLightTransform tt;
static CameraImage *camera_image = NULL;
CameraInterface *camera_interface = NULL;
static pthread_mutex_t camera_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t current_lights_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t applanix_mutex = PTHREAD_MUTEX_INITIALIZER;
static ApplanixPose currPose;
static ApplanixPose currPose2;
static std::list<ApplanixPose> recentPoses;
static CameraParams pri;
static bool writeVideo = false;
static bool writeVideoAsFrames = false; 
static CvVideoWriter* cvwriter;

/*struct RadiusGivenTime
{
	double time;
	double radius;
};
std::vector<RadiusGivenTime> timeRadius;
*/
struct lightPos
{
	double gx;
	double gy;
	double alt;
};

map<std::string, TrafficLightState> currentLights;

bool quit_signal = false;

int win_height = 480;
int win_width = 640;

IpcInterface *ipc;


//Camera information
double cx, cy, Fx, Fy;
dgc_transform_t cameraTrans;

//COORDINATE TRANSFORM
//for drawing in openGL object coordinates
bool uvToObj(int u, int v, double *objX, double *objY)
{
	tt.imagePlane_to_OpenGLObj(u, v, camera_image->info.width, camera_image->info.height, objX, objY);
	return true;
}

//CALLBACK FUNCTIONS:

FILE* lightFile;

/*void applanix_handler(ApplanixPose *pose)
{
	pthread_mutex_lock(&applanix_mutex);
	//update buffer
	if (loc_pose.x_offset == 0 || loc_pose.y_offset == 0)
	{
		getUTMOffset(pose, &loc_pose.x_offset, &loc_pose.y_offset);
	}
	pose_buffer.globalX = pose->smooth_x + loc_pose.x_offset;
	pose_buffer.globalY = pose->smooth_y + loc_pose.y_offset;
	pthread_mutex_unlock(&applanix_mutex);

	pose_buffer.currPose = *pose;
	pose_buffer.time_last_update = dgc_get_time();
}
*/
void applanix_handler(ApplanixPose *pose)
{
	pthread_mutex_lock(&applanix_mutex);
	recentPoses.push_back(*pose);
	while( recentPoses.size() > 100) recentPoses.pop_front();
	pthread_mutex_unlock(&applanix_mutex);
}

void light_handler(TrafficLightList *light_list)
{

	std::string ID = light_list->light_state->name;

	//update current lights
	pthread_mutex_lock(&current_lights_mutex);
	if (currentLights.count(ID) > 0)
	{
		currentLights[ID].state = light_list->light_state->state;
		currentLights[ID].state_arrow = light_list->light_state->state_arrow;
		currentLights[ID].timestamp = light_list->light_state->timestamp;
		currentLights[ID].u = light_list->light_state->u;
		currentLights[ID].v = light_list->light_state->v;
		currentLights[ID].confidence = light_list->light_state->confidence;
	}
	else
	{
		currentLights[ID] = light_list->light_state[0];
	}
	pthread_mutex_unlock(&current_lights_mutex);

	fprintf(stderr, "got light message for ID %s, state %c\n", ID.c_str(), currentLights[ID].state);

}

//SUBSRIBE

void  ipc_subscribe_TrafficLightList()
{
	static int subscribed = 0;
	static int callback_id = -1;

	if (!subscribed)
	{
		callback_id = ipc->Subscribe(TrafficLightListMsgID, &light_handler, DGC_SUBSCRIBE_ALL);
		subscribed = 1;
	}
}

void setColor(char state, double confidence)
{
	switch (state)
	{
	case 'r':
		glColor3f(confidence, 0, 0);
		break;
	case 'g':
		glColor3f(0, confidence, 0);
		break;
	case 'y':
		glColor3f(confidence, confidence, 0);
		break;
	case 'u':
		glColor3f(0.7, 0.7, 0.7);
		break;
	default:
		break;
	}

}

void keyboard(unsigned char key, int x __attribute__ ((unused)),
              int y __attribute__ ((unused)))
{
	static bool storeLight = false;
	static char lightBuffer[80] = "";

	switch (key)
	{
	case 27:
	case 'q':
	case 'Q':
		exit(0);
		break;
	case 'M':
	case 'm':
		if (!storeLight)
			sprintf(lightBuffer,"%f %f %f", currPose.latitude, currPose.longitude, currPose.altitude); 
		else
			fprintf(lightFile, "%f %f %f %s\n", currPose.latitude, currPose.longitude, currPose.altitude, lightBuffer);
		storeLight = !storeLight;
		break;
	case 'R':
	case 'r':
		record = !record;
		break;
	case 'd':
	case 'D':
		debug_mode = !debug_mode;
		break;
	default:
		break;
	}

}

//INITialization and Shutdown:

static void shutdown_module(int x)
{
/*	FILE* out = fopen("time_radius.csv", "a");
	for (int i = 0; i < timeRadius.size(); i ++)
		fprintf(out, "%14.14g, %14.14g\n", timeRadius[i].time, timeRadius[i].radius);
	fclose(out);
*/
	if (x == SIGINT)
	{
		cvDestroyAllWindows();
		if (writeVideo && !writeVideoAsFrames)
			cvReleaseVideoWriter(&cvwriter);
		quit_signal = true;
		fprintf(stderr, "\nTRAFFIC_LIGHT_VIEW: Caught SIGINT. exiting.\n");
		fflush(stderr);
		fclose(lightFile);
		ipc->Disconnect();
		
		exit(1);
	}
}
//
IplImage *image = 0, *hsv = 0, *hue = 0, *mask = 0, *backproject = 0, *histimg = 0;
CvHistogram *hist = 0;

struct TrackPoint
{
	double u;
	double v;
	double x;
	double y;
	double z;
	double x_abs;
	double y_abs;
	double z_abs;
	ApplanixPose p;
	double a;
	double b;
	double c;
};


int backproject_mode = 0;
int select_object = 0;
int track_object = 0;
int show_hist = 1;
CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 16;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 55, vmax = 256, smin = 170;
bool newline = false;
std::vector<TrackPoint> track_points;
double scale = 1.;


void on_mouse( int event, int x, int y, int flags, void* param)
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);

        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, image->width );
        selection.height = MIN( selection.height, image->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
    }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
}


CvScalar hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
        {{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}

char gpsTxt[180];
int fade_count = 0;

void process_frame()
{
	std::vector<std::string> files;
	getdir("./", files);
	std::string s("light_");
	static int lightNum = 0;

	if (lightNum == 0)
		for (size_t i = 0; i < files.size(); i ++)
			if (files[i].substr(0, s.length()) == s)
			{
				int x = atoi(files[i].substr(s.length(), files[i].length()-1).c_str());
				if (x > lightNum) lightNum = x;
			}

	int bin_w;
	IplImage* frame = cvCreateImageHeader(cvSize(camera_image->info.width, camera_image->info.height), 8, 3);
	frame->imageData = (char*)camera_image->data;
	cvConvertImage(frame, frame, CV_CVTIMG_SWAP_RB);

	if( !image )
	{
		/* allocate all the buffers */

		image       = cvCreateImage( cvGetSize(frame), 8, 3 );
		hsv         = cvCreateImage( cvGetSize(frame), 8, 3 );
		hue         = cvCreateImage( cvGetSize(frame), 8, 1 );
		mask        = cvCreateImage( cvGetSize(frame), 8, 1 );
		backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
		histimg     = cvCreateImage( cvSize(320,200),  8, 3 );
		hist        = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
		cvZero( histimg );
	}

	cvCopy(frame,image);

	if (NORMALIZE)
	{
		// replace this with [ward02] (or anything better than RGB stretching)
		IplImage* rgb[3];
		for (int i = 0; i < 3; i ++) rgb[i] = cvCreateImage(cvGetSize(frame), 8, 1);
		cvCvtPixToPlane(frame, rgb[0], rgb[1], rgb[2], NULL);
		for (int i = 0; i < 3; i ++) cvNormalize(rgb[i], rgb[i], 0, 255, CV_MINMAX);
		cvCvtPlaneToPix(rgb[0], rgb[1], rgb[2], NULL, frame);
		for (int i = 0; i < 3; i ++) cvReleaseImage(&rgb[i]);
	}

	cvCvtColor( frame, hsv, CV_BGR2HSV );

	if( track_object )
	{
		int _vmin = vmin, _vmax = vmax;

		cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0), cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
		cvSplit( hsv, hue, 0, 0, 0 );

		if( track_object < 0 )
		{
			 float max_val = 0.f;
			 cvSetImageROI( hue, selection );
			 cvSetImageROI( mask, selection );
			 cvCalcHist( &hue, hist, 0, mask );
			 cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
			 cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
			 cvResetImageROI( hue );
			 cvResetImageROI( mask );
			 track_window = selection;
			 track_object = 1;

			 cvZero( histimg );
			 bin_w = histimg->width / hdims;
			 for(int i = 0; i < hdims; i++ )
			 {
				  int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
				  CvScalar color = hsv2rgb(i*180.f/hdims);
				  cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
									cvPoint((i+1)*bin_w,histimg->height - val),
									color, -1, 8, 0 );
			 }
		}

		cvCalcBackProject( &hue, backproject, hist );
		cvAnd( backproject, mask, backproject, 0 );
		try
		{
			cvCamShift( backproject, track_window,
							cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
							&track_comp, &track_box );
		} catch (cv::Exception& e) {
			fprintf(stderr, "Selection out of bounds.\n");
		}

		track_window = track_comp.rect;

		if( backproject_mode ) cvCvtColor( backproject, image, CV_GRAY2BGR );
		if( !frame->origin ) track_box.angle = -track_box.angle;

		double box_hw = (double)(track_box.size.height) / (double)(track_box.size.width);
		double box_wh = 1./box_hw;

		if (track_box.size.width > 1. && track_box.size.height > 1. && box_hw < 3.0 && box_wh < 3.0)
		{
			/*RadiusGivenTime dr = { camera_image->timestamp,
				std::min(track_box.size.height, track_box.size.width) };
			timeRadius.push_back(dr);
*/
			char zone[4];

			pthread_mutex_lock(&applanix_mutex);

			TrackPoint tp = { track_box.center.x, track_box.center.y, 0, 0, 0, 0, 0, 0, currPose, 0, 0, 0 }; 
			latLongToUtm(currPose.latitude, currPose.longitude, &tp.x_abs, &tp.y_abs, zone);
			tp.z_abs = currPose.altitude;

			pthread_mutex_unlock(&applanix_mutex);

			if (track_points.size() == 0)
			{
				tp.x = 0;
				tp.y = 0;
				tp.z = 0;
			}
			else
			{
				tp.x = tp.x_abs - track_points.front().x_abs;
				tp.y = tp.y_abs - track_points.front().y_abs;
				tp.z = tp.z_abs - track_points.front().z_abs;
			}

			double* d = new double[3];
			tt.BackProject(tp.u, tp.v, (unsigned)image->width, (unsigned)image->height, tp.p, &d);
			tp.a = d[0];
			tp.b = d[1];
			tp.c = d[2];
			delete [] d;

			double e = 10000.;
			static double f = 0.;
			
			if (track_points.size() > 0)
			{
				double ex = tp.x - track_points.back().x;
				double ey = tp.y - track_points.back().y;
				double ez = tp.z - track_points.back().z;
				e = sqrt(ex*ex + ey*ey + ez*ez);
			}

			if (e > .150) 
			{
				track_points.push_back(tp);
				f += e;
			}

			if (f > 5.)
			{
				f = 0.;	
				if ( collectLights )
				{
					char filename[20];
					sprintf(filename, "light_%05d.bmp", lightNum++);

					int side = std::min(track_box.size.width, track_box.size.height);
					int x0 = track_box.center.x - side/2;
					int y0 = track_box.center.y - side/2;

					CvRect roi = { x0, y0, side, side };
					cvSetImageROI(image, roi);
					cvSaveImage(filename, image);
					cvResetImageROI(image);
				}
			}

			try
			{
				cvEllipseBox( image, track_box, CV_RGB(255,0,255), 2, CV_AA, 0 );
			} catch (cv::Exception& e) {
				fprintf(stderr, "Elipse out of bounds.\n");
			}

			newline = true;
		}
		else if (newline)
		{
			if (track_points.size() >= 3)
			{
	/*			std::ofstream ofile("real_data.dat", std::ios::binary);
				for (std::vector<TrackPoint>::iterator t = track_points.begin(); t != track_points.end(); t++)
					ofile.write(reinterpret_cast<char*>(&(*t)), sizeof(TrackPoint));
				ofile.close();
	*/
				CvMat* A = cvCreateMat(3,3,CV_64FC1);
				CvMat* x = cvCreateMat(3,1,CV_64FC1);
				CvMat* b = cvCreateMat(3,1,CV_64FC1);

				cvZero(A);
				cvZero(x);
				cvZero(b);

				for (std::vector<TrackPoint>::iterator t = track_points.begin(); t != track_points.end(); t++)
				{
					*( (double*)CV_MAT_ELEM_PTR( *A, 0, 0 ) ) += 1. - (t->a * t->a);
					*( (double*)CV_MAT_ELEM_PTR( *A, 0, 1 ) ) +=    - (t->a * t->b);
					*( (double*)CV_MAT_ELEM_PTR( *A, 0, 2 ) ) +=    - (t->a * t->c);
					*( (double*)CV_MAT_ELEM_PTR( *A, 1, 0 ) ) +=    - (t->a * t->b);
					*( (double*)CV_MAT_ELEM_PTR( *A, 1, 1 ) ) += 1. - (t->b * t->b);
					*( (double*)CV_MAT_ELEM_PTR( *A, 1, 2 ) ) +=    - (t->b * t->c);
					*( (double*)CV_MAT_ELEM_PTR( *A, 2, 0 ) ) +=    - (t->a * t->c);
					*( (double*)CV_MAT_ELEM_PTR( *A, 2, 1 ) ) +=    - (t->b * t->c);
					*( (double*)CV_MAT_ELEM_PTR( *A, 2, 2 ) ) += 1. - (t->c * t->c);

					*( (double*)CV_MAT_ELEM_PTR( *b, 0, 0 ) ) += 
						(1. - t->a*t->a)*t->x - t->a*t->b*t->y - t->a*t->c*t->z;
					
					*( (double*)CV_MAT_ELEM_PTR( *b, 1, 0 ) ) += 
						-t->a*t->b*t->x + (1 - t->b*t->b)*t->y - t->b*t->c*t->z;

					*( (double*)CV_MAT_ELEM_PTR( *b, 2, 0 ) ) += 
						-t->a*t->c*t->x - t->b*t->c*t->y + (1 - t->c*t->c)*t->z;
				}

				cvSolve(A, b, x, CV_SVD);

				// compensate for d-vectors converging too early.
				//cvAddWeighted(x,1.1,x,0.,0.,x);
				
				*( (double*)CV_MAT_ELEM_PTR( *x, 0, 0 ) ) += track_points.front().x_abs;
				*( (double*)CV_MAT_ELEM_PTR( *x, 1, 0 ) ) += track_points.front().y_abs;
				*( (double*)CV_MAT_ELEM_PTR( *x, 2, 0 ) ) += track_points.front().z_abs;

	/*			cvSave("A.xml", A);
				cvSave("x.xml", x);
				cvSave("b.xml", b);
	*/
/*				TrackPoint& p = track_points.front();
				printf("10S %8.8lf %8.8lf %8.8lf %8.8lf %8.8lf %8.8lf\n",
					CV_MAT_ELEM( *x, double, 0, 0 ), 
					CV_MAT_ELEM( *x, double, 1, 0 ),
					CV_MAT_ELEM( *x, double, 2, 0 ), 
					p.x_abs, p.y_abs, p.z_abs); 
				fflush(stdout);
*/
				double lat, lon;
				const char zone[] = "10S";
				utmToLatLong(CV_MAT_ELEM( *x, double, 0, 0 ), CV_MAT_ELEM( *x, double, 1, 0 ), zone, &lat, &lon);
				sprintf(gpsTxt, "%f %f %f %f %f %f",  (float)lat, (float)lon, 
						(float)CV_MAT_ELEM(*x, double, 2, 0), (float)track_points.front().p.latitude, 
						(float)track_points.front().p.longitude, (float)track_points.front().p.altitude);
				printf("%s\n", gpsTxt);
				
				track_points.clear();
				cvReleaseMat(&A);
				cvReleaseMat(&b);
				cvReleaseMat(&x);

				fade_count = 50;

				newline = false;
				track_object = 0;
			}
		}
	}

	if( select_object && selection.width > 0 && selection.height > 0 )
	{
		cvSetImageROI( image, selection );
		cvXorS( image, cvScalarAll(255), image, 0 );
		cvResetImageROI( image );
	}

	if (fade_count > 0)
	{
		double alpha = std::min(20., (double)fade_count)/20.;
		CvFont font;
		double hScale = .7;
		double vScale = .7;
		int lineWidth = 1;
		cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, hScale, vScale, 0, lineWidth);
		CvPoint p5 = { 15, 30 };
		IplImage* imgTxt = cvCreateImage(cvSize(900,50), 8, 3);
		cvZero(imgTxt);
		cvPutText (imgTxt, gpsTxt, p5, &font, cvScalar(255,255,255));
		cvSetImageROI(image, cvRect(15, 15, imgTxt->width, imgTxt->height));
		cvAddWeighted(imgTxt, alpha, image, 1., 0, image);
		cvResetImageROI(image);
		cvReleaseImage(&imgTxt);
		fade_count --;
	}

	cvShowImage( "main", image );
	cvShowImage( "Histogram", histimg );

	// write video (or png)
	if (writeVideo && !writeVideoAsFrames) cvWriteFrame(cvwriter, image);
	else if (writeVideoAsFrames)
	{
		char frame_name[25];
		static int frame_number = 0;
		sprintf(frame_name, "frame_%05d.png", frame_number++);
		cvSaveImage(frame_name, image);
	}

	cvReleaseImageHeader(&frame);
}

//THREADS:
void * camera_thread_function(__attribute__ ((unused)) void *ptr)
{
	/* reading images from camera */
	while (true)
	{
		while (camera_interface->ImagesWaiting())
		{
			pthread_mutex_lock(&camera_mutex);
			if (camera_interface->ReadCurrentImage(camera_image) < 0)
			{
				dgc_fatal_error("Image read failed.\n");
			}
		
			pthread_mutex_lock(&applanix_mutex);
			
			std::list<ApplanixPose>::iterator i = recentPoses.begin();
			double minTimeDiff = 1.e307;

			while( i != recentPoses.end())
			{
				double timeDiff = abs(i->timestamp - camera_image->timestamp);
				if( timeDiff < minTimeDiff)
				{
					currPose = *i;
					minTimeDiff = timeDiff;
				}
				i++;
			}
			
			pthread_mutex_unlock(&applanix_mutex);

			process_frame();
			pthread_mutex_unlock(&camera_mutex);
		}

		char c = cvWaitKey(10);
		keyboard(c,0,0);
		usleep(1000);
	}
	return NULL;
}

void read_parameters(ParamInterface *pint, int argc, char** argv)
{
	char* primary_camera;
	char* pri_trans_filename;

	// shorthand for DGC_PARAM_... from interface/param_server/param_interface.h
	ParamType P_INT       = DGC_PARAM_INT;
	ParamType P_DOUBLE    = DGC_PARAM_DOUBLE;
	ParamType P_STRING    = DGC_PARAM_STRING;
	ParamType P_FILENAME  = DGC_PARAM_FILENAME;

	Param params[] =
	{
		{"trafficlights", "primary_camera",       P_STRING,   &primary_camera,     1,NULL}
	};
	pint->InstallParams(argc, argv, params, sizeof(params) / sizeof(params[0]));

	fprintf(stderr, "primary camera: %s\n", primary_camera);

	Param params2[] =
	{
		{primary_camera,  "camera_num",           P_INT,      &pri.info.camera_number,1,NULL},
//		{primary_camera,  "image_fmt",            P_STRING,   &pri.info.format,    1,NULL},
		{primary_camera,  "frame_rate",           P_INT,      &pri.frame_rate,     1,NULL},
		{primary_camera,  "roi_top",              P_INT,      &pri.roi.y,          1,NULL},
		{primary_camera,  "roi_left",             P_INT,      &pri.roi.x,          1,NULL},
		{primary_camera,  "roi_height",           P_INT,      &pri.roi.height,     1,NULL},
		{primary_camera,  "roi_width",            P_INT,      &pri.roi.width,      1,NULL},
		{primary_camera,  "pixel_size_um",        P_DOUBLE,   &pri.pixel_size,     1,NULL},
		{primary_camera,  "max_res_width",        P_INT,      &pri.info.width,     1,NULL},
		{primary_camera,  "max_res_height",       P_INT,      &pri.info.height,    1,NULL},
		{primary_camera,  "cx",                   P_DOUBLE,   &pri.cx,             1,NULL},
		{primary_camera,  "cy",                   P_DOUBLE,   &pri.cy,             1,NULL},
		{primary_camera,  "Fx",                   P_DOUBLE,   &pri.Fx,             1,NULL},
		{primary_camera,  "Fy",                   P_DOUBLE,   &pri.Fy,             1,NULL},
		{"transform",     primary_camera,         P_FILENAME, &pri_trans_filename, 1,NULL}
	};

	pint->InstallParams(argc, argv, params2, sizeof(params2) / sizeof(params2[0]));

	dgc_transform_read(pri.trans, pri_trans_filename);
}

int main(int argc, char **argv)
{
	cvNamedWindow("main", 0);
	cvMoveWindow("main", 0, 0);
	cvNamedWindow("Histogram");
	cvSetMouseCallback( "main", on_mouse, 0 );
	cvCreateTrackbar( "Vmin", "main", &vmin, 256, 0 );
	cvCreateTrackbar( "Vmax", "main", &vmax, 256, 0 );
	cvCreateTrackbar( "Smin", "main", &smin, 256, 0 );
	
	bool showUsage = false;
	bool exitAfter = false;
	for (int i = 1; i < argc; i ++)
	{
		switch (argv[i][0])
		{
			case 'D' :
			case 'd' : 
				fprintf(stderr, "Switching to debug mode.\n");
				debug_mode = true; 
				break;
			case 'C' : 
			case 'c' : 
				fprintf(stderr, "Collecting lights to BGR bmp files (use bgr2hsv in TOOLS folder).\n");
				collectLights = true; 
				break;
			case 'W' :
			case 'w' :
				fprintf(stderr, "Writing to video file out.avi\n");
				writeVideo = true;
				debug_mode = true;
				break;
			case 'S' :
			case 's' :
				fprintf(stderr, "Toggling input channel RGB stretching.\n");
				NORMALIZE = !NORMALIZE;
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
				fprintf(stderr, "Unknown option: %s\n", argv[i]);
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
	
	
	/* connect to the IPC server, register messages */
	ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	ParamInterface *pint = new ParamInterface(ipc);
	read_parameters(pint, argc, argv);

	ApplanixPose pose;
	memset(&pose, 0, sizeof(pose));

	/* setup a shutdown handler */
	signal(SIGINT, shutdown_module);

	lightFile = fopen("light.txt", "a");
	
	//subscribe to messages:
	ipc->Subscribe(ApplanixPoseID, &applanix_handler, DGC_SUBSCRIBE_ALL, &applanix_mutex);
	ipc_subscribe_TrafficLightList();

	//CAMERA:
	//initialize camera-specific information:
	tt.set_camera_params(pri);

	param_struct_t param;
	pthread_t thread1;

	/* connect to camera interface */
	camera_interface = new CameraShmInterface;
	if (camera_interface->CreateClient(0) < 0)
		dgc_fatal_error("Could not connect to camera %d interface.", 0);
	//if (camera_interface->CreateClient(pri.info.camera_number) < 0)
	//	dgc_fatal_error("Could not connect to camera %d interface.", pri.info.camera_number);
	camera_image = new CameraImage;

	scale = 700. / (double)(pri.info.height);
	cvResizeWindow("main", (int)((double)(pri.info.width)*scale), (int)((double)(pri.info.height)*scale));

	/* start the graphics thread */
	param.argc = argc;
	param.argv = argv;

	/* start reading images from camera */
	pthread_create(&thread1, NULL, camera_thread_function, NULL);

	ipc->Dispatch();
	return 0;
}

