#include <roadrunner.h>
#include <ipc_std_interface.h>
#include <transform.h>
#include <passat_constants.h>
#include <param_interface.h>
#include <blf.h>
#include <blf_id.h>
#include <camera_interface.h>
#include <gui2D.h>
#include <cmath>
#include <map>
#include <textures.h>
#include <string>

#ifndef round
#define round(x) (x<0?ceil((x)-0.5):floor((x)+0.5))
#endif

#define CAMERA 0

using namespace dgc;
using namespace vlr;

const int kFleaMaxPacketSize = 20000000;
static dgc_gl_texture_t* image_texture = NULL;

struct FleaPacket
{
	FleaPacket();
	~FleaPacket();
	double timestamp;
	unsigned int len, max_len;
	unsigned char *data;
};

FleaPacket::FleaPacket()
{
	len = 0;
	max_len = kFleaMaxPacketSize;
	data = new unsigned char[max_len];
}

FleaPacket::~FleaPacket()
{
	if (data != NULL)
		delete data;
}


blf_t                   *blf = NULL;
blf_index_t             *blf_index = NULL;
int                     correct_falloff = true;
int                     firsttime = 1;

//flea stuff
//double prevFlea_timestamp = 0;
double currFlea_timestamp = 0;
FleaPacket* pkt;
int flea_pkt_num = 0;
CameraImage* camstruct;
int structOffset;
double cx, cy, Fx, Fy;
double fplane_dist = 1.0;

/* parameters */
int delta = 0;

bool clicked = false;

//same aspect ratio as flea images
int win_width = 640;
int win_height = 480;

//flea image resolution
int image_height = 960;
int image_width = 1280;

void read_flea_pkt( int index )
{
	int                           ret;
	unsigned short                pkt_id;

	if (index>=0 && index < blf_index->num_blocks)
	{
		blf->seek(blf_index->block[index].offset, SEEK_SET);
		//printf("did seeking\n");

		ret = blf->read_data(&pkt_id, &(pkt->timestamp), &(pkt->data), &(pkt->len), &(pkt->max_len));

		//prevFlea_timestamp = currFlea_timestamp;
		currFlea_timestamp = pkt->timestamp;

		//printf("\nreturnedTimestamp %f\n", pkt->timestamp);
		//printf("read data %d bytes\n", pkt->len);
	}
}


void keyboard(unsigned char key, int x __attribute__ ((unused)),
              int y __attribute__ ((unused)))
{
	delta = 0;

	if (('0' <= key && key <= '9') || (key == '.') || (key == ' ')) fprintf(stderr, "%c", key);

	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'q':
		delta = 1;
		break;
	case 'a':
		delta = -1;
		break;
	case 's':
		delta = -10;
		break;
	case 'w':
		delta = 10;
		break;
	case 'd':
		delta = -100;
		break;
	case 'e':
		delta = 100;
		break;
	case 'R' :
	case 'r':
		fprintf(stderr, " r %lf\n", currFlea_timestamp);
		break;
	case 'v' :
	case 'G' :
	case 'g':
		fprintf(stderr, " g %lf\n", currFlea_timestamp);
		break;
	case 'f' :
	case 'Y' :
	case 'y':
		fprintf(stderr, " y %lf\n", currFlea_timestamp);
		break;
	case 'u' :
	case 'U' :
		fprintf(stderr, " u %lf\n", currFlea_timestamp);
		break;
	case 'M' :
	case 'm':
		fprintf(stderr, "undo\n");
		printf("mark previous as a mistake\n");
		break;
	case 'z' :
		{
/*			int inter, number, isActive, state;
			char states[] = { 'r', 'y', 'g', 'u' };
			scanf("%02d%01d%01d%01d\n", &inter, &number, &isActive, &state);
			printf("%d %d %d %c %16.16lf\n", inter, number, isActive, states[state-1], currFlea_timestamp);
*/
			fprintf(stderr, "%16.16lf\n", currFlea_timestamp);
		}
	default:
		break;
	}

	if (delta != 0)
	{
		flea_pkt_num += delta;

		if (flea_pkt_num < 0)
		{
			flea_pkt_num = 0;
		}

		read_flea_pkt(flea_pkt_num);
	}
	glFlush();
	gui2D_forceRedraw();

}

void drawCamImage()
{

	// generate a texture variable
	if (image_texture == NULL)
	{
		image_texture = dgc_gl_empty_texture(image_width, image_height, 2048, 0);
	}

	//actual camera data starts after the bytes for the CameraImage struct end, so use structOffset
	glTexImage2D(GL_TEXTURE_2D, 0, 3, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, &(pkt->data[structOffset]));

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, image_texture->texture_id);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glColor3f(1, 1, 1);
	glBegin(GL_QUADS);
	glTexCoord2f(0,0);
	glVertex2f(0, 1);

	glTexCoord2f(1,0);
	glVertex2f(1, 1);

	glTexCoord2f(1,1);
	glVertex2f(1, 0);

	glTexCoord2f(0,1);
	glVertex2f(0, 0);
	glEnd();

	glDisable(GL_TEXTURE_2D);


}

void display(void)
{
	/* clear window */
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT);

	set_display_mode_2D(1,1);

	drawCamImage();

}


//void read_parameters(ParamInterface *pint, int argc, char **argv)
//{
//no parameters for now
//return;

//}

int main(int argc, char **argv)
{
	if (argc!=2)
	{
		fprintf( stderr, "usage: %s <BLF-FILE> \n", argv[0] );
		exit(0);
	}

	IpcInterface *ipc = new IpcStandardInterface;
	if (ipc->Connect(argv[0]) < 0)
		dgc_fatal_error("Could not connect to IPC network.");

	//ParamInterface *pint = new ParamInterface(ipc);
	//read_parameters(pint, argc, argv);

	blf = new blf_t;
	if (blf->open(argv[1], "r") != BLF_OK)
		dgc_fatal_error("Could not open BLF file %s for reading.", argv[1]);


	blf_index = blf_index_load(argv[1]);
	if (blf_index == NULL)
		dgc_fatal_error("blf file must have index file (did you run blf-index?).");

	pkt = new FleaPacket;
	camstruct = new CameraImage;
	structOffset= sizeof(*camstruct)-1;

	read_flea_pkt(0);


	//gui 2d loop:
	gui2D_initialize(argc, argv, 10, 10, win_width, win_height, 30.0);
	gui2D_set_displayFunc(display);
	gui2D_set_keyboardFunc(keyboard);
	gui2D_setInitialCameraPos(0,0,0,0);
	gui2D_mainloop();

	return 0;
}
