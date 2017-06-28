
#include "traffic_light_transforms.h"
#include <opencv/cv.h>

using namespace dgc;

void TrafficLightTransform::set_camera_trans_matrix(dgc_transform_t cameraT)
{
	dgc_transform_copy(cameraTrans, cameraT);
}

void TrafficLightTransform::set_camera_params(CameraParams& _params)
{
	set_camera_trans_matrix(_params.trans);
	cx = _params.cx;
	cy = _params.cy;
	Fx = _params.Fx;
	Fy = _params.Fy;
};

//global->smooth, with z = altitude
void TrafficLightTransform::global_to_smooth(double gx, double gy, double gz, double x_offset, double y_offset, double *sx, double *sy, double *sz)
{
	*sx = gx - x_offset;
	*sy = gy - y_offset;
	*sz = gz;

	//for debug:
	//printf("\nLIGHT COORDINATES:\n");
	//printf("globalX %f globalY %f altitude %f\n", gx, gy, gz);
	//printf("localizer: offset_x %f offset_y %f\n", x_offset, y_offset);
	//printf("smoothX %f smoothY %f smoothZ %f\n", *sx, *sy, *sz);
}

//smooth->robot
void TrafficLightTransform::smooth_to_robot
(double sx, double sy, double sz, const ApplanixPose& currPose, double *rx, double *ry, double *rz)
{
	*rx = sx - currPose.smooth_x;
	*ry = sy - currPose.smooth_y;
	*rz = sz - currPose.altitude;
	dgc_transform_t trans, transInv;
	dgc_transform_identity(trans);
	dgc_transform_rotate_x(trans, currPose.roll);
	dgc_transform_rotate_y(trans, currPose.pitch);
	dgc_transform_rotate_z(trans, currPose.yaw);
	dgc_transform_inverse(trans, transInv);
	dgc_transform_point(rx, ry, rz, transInv);
}

//robot->camera
void TrafficLightTransform::robot_to_Cam
(double rx, double ry, double rz, double *camX, double *camY, double *camZ)
{
	dgc_transform_t cameraTransInv;
	*camX = rx;
	*camY = ry;
	*camZ = rz;
	dgc_transform_inverse(cameraTrans, cameraTransInv);
	dgc_transform_point(camX, camY, camZ, cameraTransInv);
}

/*use the projective equations to get u,v point:
/ u = Fx*(Y/X) + cx
/ v = Fy*(Z/X) + cy*/
void TrafficLightTransform::Cam_to_imagePlane
(double camX, double camY, double camZ, unsigned int im_width, unsigned int im_height, double *u, double *v)
{
	// images are lower left in origin; subtract height, width from results to compensate.
	*u = im_width  - round(Fx*(camY/camX) + cx);
	*v = im_height - round(Fy*(camZ/camX) + cy);
}

static void PrintMatrix(const CvMat* A, const char* name) 
{
	printf("%s = [ \n", name);
	for (int i = 0; i < A->rows; i ++)
	{
		for (int j = 0; j < A->cols; j ++)
			printf("\t%5.5lf", CV_MAT_ELEM(*A,double,i,j));
		printf("\n");
	}
	printf(" ];\n");
}

void TrafficLightTransform::BackProject
(double u, double v, unsigned int im_width, unsigned int im_height, const ApplanixPose& currPose, double *d[3])
{
	// Initialize opencv matrix header refering to 'd'.
	CvMat* d_ = cvCreateMatHeader(3, 1, CV_64FC1);
	d_->data.db = *d;

	*( (double*)CV_MAT_ELEM_PTR( *d_, 0, 0 ) ) = u;
	*( (double*)CV_MAT_ELEM_PTR( *d_, 1, 0 ) ) = v;
	*( (double*)CV_MAT_ELEM_PTR( *d_, 2, 0 ) ) = 1.;

//	PrintMatrix(d_, "d");

	// Intrinsic back-solver.
	CvMat* K = cvCreateMat(3, 3, CV_64FC1);
	cvZero(K);
	*( (double*)CV_MAT_ELEM_PTR( *K, 0, 0 ) ) = Fx;
	*( (double*)CV_MAT_ELEM_PTR( *K, 0, 2 ) ) = cx;
	*( (double*)CV_MAT_ELEM_PTR( *K, 1, 1 ) ) = Fy;
	*( (double*)CV_MAT_ELEM_PTR( *K, 1, 2 ) ) = cy;
	*( (double*)CV_MAT_ELEM_PTR( *K, 2, 2 ) ) = 1.;

	cvSolve(K, d_, d_, CV_SVD);

/*	PrintMatrix(K, "K");
	PrintMatrix(d_, "d");
*/
	// TODO: calibrate for extrinsic parameters and back-solve here.
	
	// Permute from 'image processing' frame to 'automotive industry' frame.
	//    P = [ 0 0 1; -1 0 0; 0 -1 0 ];
	CvMat* P = cvCreateMat(3, 3, CV_64FC1);
	cvZero(P);
	*( (double*)CV_MAT_ELEM_PTR( *P, 0, 2 ) ) =  1.;
	*( (double*)CV_MAT_ELEM_PTR( *P, 1, 0 ) ) = -1.;
	*( (double*)CV_MAT_ELEM_PTR( *P, 2, 1 ) ) = -1.;

	cvMatMul(P, d_, d_);

/*	PrintMatrix(P, "P");
	PrintMatrix(d_, "d");
*/	
	// Rotate from camera coordinates to world coordinates.
	//    R = [ cos(w)*cos(p), cos(w)*sin(p)*sin(r) - sin(w)*cos(r), cos(w)*sin(p)*cos(r) + sin(w)*sin(r)
	//          sin(w)*cos(p), sin(w)*sin(p)*sin(r) + cos(w)*cos(r), sin(w)*sin(p)*cos(r) - cos(w)*sin(r)
	//          -sin(p),       cos(p)*sin(r),                        cos(p)*cos(r) ];

	double sw    = sin(currPose.yaw);
	double sp    = sin(currPose.pitch);
	double sr    = sin(currPose.roll);

	double cw    = cos(currPose.yaw);
	double cp    = cos(currPose.pitch);
	double cr    = cos(currPose.roll);

	CvMat* R = cvCreateMat(3, 3, CV_64FC1);
	*( (double*)CV_MAT_ELEM_PTR( *R, 0, 0 ) ) = cw*cp;
	*( (double*)CV_MAT_ELEM_PTR( *R, 0, 1 ) ) = cw*sp*sr - sw*cr;
	*( (double*)CV_MAT_ELEM_PTR( *R, 0, 2 ) ) = cw*sp*cr + sw*sr;

	*( (double*)CV_MAT_ELEM_PTR( *R, 1, 0 ) ) = sw*cp;
	*( (double*)CV_MAT_ELEM_PTR( *R, 1, 1 ) ) = sw*sp*sr + cw*cr;
	*( (double*)CV_MAT_ELEM_PTR( *R, 1, 2 ) ) = sw*sp*cr - cw*sr;

	*( (double*)CV_MAT_ELEM_PTR( *R, 2, 0 ) ) = -sp;
	*( (double*)CV_MAT_ELEM_PTR( *R, 2, 1 ) ) = cp*sr;
	*( (double*)CV_MAT_ELEM_PTR( *R, 2, 2 ) ) = cp*cr;

	cvMatMul(R, d_, d_);
	cvNormalize(d_, d_);

/*	PrintMatrix(R, "R");
	PrintMatrix(d_, "d");
*/	
	cvReleaseMat(&R);
	cvReleaseMat(&P);
	cvReleaseMat(&K);
	cvReleaseMatHeader(&d_);
}

void TrafficLightTransform::imagePlane_to_OpenGLObj
(int u, int v, int im_width, int im_height, double *objX, double *objY)
{
	*objX = u/(double)im_width;
	*objY = 1 - v/(double)im_height;
}


void TrafficLightTransform::globalToUV
(int im_width, int im_height, double gxLight, double gyLight, double gzLight,
 int *u, int *v, double *robotXp, const LocalizePose& loc_pose, const ApplanixPose& currPose)
{
	//global->smooth
	double smoothX, smoothY, smoothZ;
	global_to_smooth(gxLight, gyLight, gzLight, loc_pose.x_offset, loc_pose.y_offset, &smoothX, &smoothY, &smoothZ);

	//smooth->robot
	double robotX, robotY, robotZ;
	smooth_to_robot(smoothX, smoothY, smoothZ, currPose, &robotX, &robotY, &robotZ);

	//for seeing if we're still looking at the light ahead of us -- check x coordinate here
	*robotXp = robotX;

	//robot->camera
	double camX, camY, camZ;
	robot_to_Cam(robotX, robotY, robotZ, &camX, &camY, &camZ);

	double ud, vd;
	Cam_to_imagePlane(camX, camY, camZ, im_width, im_height, &ud, &vd);

	*u = round(ud);
	*v = round(vd);
}

