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


#include <param_interface.h>
#include <grid.h>
#include <lltransform.h>
#include <ipc_std_interface.h>
#include <velo_support.h>
#include <terrainmap.h>

#define MAX_RANGE 50.0

using namespace dgc;

IpcInterface   *ipc = NULL;

/* parameters */

char *cal_filename = NULL;
dgc_transform_t velodyne_offset;

int first = 1;
double last_x = 0, last_y = 0;

typedef struct {
  terrain_tile *tile;
} grid_cell_t;

dgc_grid_p tile_grid = NULL;

typedef struct {
  vision_tile *tile;
} grid_v_cell_t;

dgc_grid_p vision_grid = NULL;

FILE *rndf = NULL;

double utmx[100];
double utmy[100];
double utmyaw[100];
int utmc = 0;

void process_tile(grid_v_cell_t *v_cell);

void rndf_create(char *name) {
	char fname[255];
	sprintf(fname, "%s.rndf", name);
	rndf = fopen(fname, "w");
	fprintf(rndf, "RNDF_name %s\n", name);
	fprintf(rndf, "num_segments\t1\n");
	fprintf(rndf, "num_zones\t0\n");
	fprintf(rndf, "format_version\t1.0\n");
	fprintf(rndf, "creation_date\t06/25/2009\n");
	fprintf(rndf, "segment\t1\n");
	fprintf(rndf, "num_lanes\t1\n");
	fprintf(rndf, "lane\t1.1\n");
	fprintf(rndf, "num_waypoints\t2000\n");
	fprintf(rndf, "lane_width\t12\n");
}

void rndf_add_waypoint(ApplanixPose *pose) {
	static int w_num = 1;
	static float last_yaw = 0.0;
	static double last_x = 0.0, last_y = 0.0;
	double utm_x = 0, utm_y = 0;
	char utmzone[5];
  	vlr::latLongToUtm(pose->latitude, pose->longitude, &utm_x, &utm_y, utmzone);
	float dist = hypot(utm_x - last_x, utm_y - last_y);
	if(dist > 2.0 && (fabs(pose->yaw - last_yaw) > M_PI / 60 || dist > 30.0)) {
		printf("1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
		fprintf(rndf, "1.1.%d\t%.7lf\t%.7lf\n", w_num, pose->latitude, pose->longitude);
		last_x = utm_x;
		last_y = utm_y;
		last_yaw = pose->yaw;
		w_num++;
	} else {
		printf("Skipping waypoint. Distance: %f    Yaw: %f\n", dist, pose->yaw);
	}
}

void rndf_close() {
	fprintf(rndf, "end_lane\n");
	fprintf(rndf, "end_segment\n");
	fclose(rndf);
}

void grid_v_clear_handler(void *cell) {
  vision_tile *tile = ((grid_v_cell_t *)cell)->tile;
  if(tile != NULL) {
    fprintf(stderr, "Processing vision tile...\n");
    process_tile((grid_v_cell_t *)cell);
    fprintf(stderr, "Saving-vision %s\n", tile->filename);
    tile->save(tile->filename);
    delete ((grid_v_cell_t *)cell)->tile;
  }
}

void grid_clear_handler(void *cell) {
  terrain_tile *tile = ((grid_cell_t *)cell)->tile;
  if(tile != NULL) {
    fprintf(stderr, "NOT Saving %s\n", tile->filename);
    // tile->save(tile->filename);
    delete ((grid_cell_t *)cell)->tile;
  }
}

double nearest_yaw(double x, double y) {
	int i;
	double min_dist = 10000000, best_yaw = 0;
	for(i = 0; i < 100; i++) {
		double d = hypot(x - utmx[i], y - utmy[i]);
		if(d < min_dist) {
			min_dist = d;
			best_yaw = utmyaw[i];
		}
	}
	return best_yaw;
}

void process(ApplanixPose *applanix_pose) {
//	printf("PROCESS-A\n");
  int r, c;
  double utm_x, utm_y;
  grid_cell_t *cell;
  grid_v_cell_t *v_cell;
  char utmzone[10];
  
  vlr::latLongToUtm(applanix_pose->latitude, applanix_pose->longitude,  &utm_x, &utm_y, utmzone);

  //printf("Process: utm_x: %f\n", utm_x);
  if(!first && hypot(utm_x - last_x, utm_y - last_y) < 2.0) 
	return;
  if(fabs(applanix_pose->speed) < dgc_mph2ms(1.0))
	return;
  last_x = utm_x;
  last_y = utm_y;

	utmx[utmc] = utm_x;
	utmy[utmc] = utm_y;
	utmyaw[utmc] = applanix_pose->yaw;
	utmc++;
	utmc = utmc % 100;

	int index = utmc - 50;
	if(index < 0) index += 100;
	utm_x = utmx[index];
	utm_y = utmy[index];
	dgc_grid_recenter_grid(tile_grid, utm_x, utm_y);
	dgc_grid_recenter_grid(vision_grid, utm_x, utm_y);
	rndf_add_waypoint(applanix_pose);
	first = 0;

      /* find the beam's terrain tile */
      r = (int)floor(utm_y / tile_grid->resolution);
      c = (int)floor(utm_x / tile_grid->resolution);
	printf("utm_x: %f   c: %d   r: %d\n", utm_x, c, r);
      v_cell = (grid_v_cell_t *)dgc_grid_get_rc_global(vision_grid, r, c);

      int r2, c2;
      for(r2 = r - 2; r2 <= r + 2; r2++) {
	for(c2 = c - 2; c2 <= c + 2; c2++) {
      		cell = (grid_cell_t *)dgc_grid_get_rc_global(tile_grid, r2, c2);
      		if(cell != NULL) {
			if(cell->tile == NULL) {
	  			cell->tile = new terrain_tile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
	  			sprintf(cell->tile->filename, "lmap-%s-%d-%d-%06d-%06d.tf.gz", 
		  			utmzone, (int)rint(TERRAIN_TILE_RESOLUTION * 100), 
		  			TERRAIN_TILE_SIZE, c2, r2);
	  			fprintf(stderr, "Loading %s\n", cell->tile->filename);
				if(cell->tile->load(cell->tile->filename) < 0) {
					cell->tile->utm_x0 = 0;
				}
			}
      		}
	}
      }

      if(v_cell != NULL) {
	if(v_cell->tile == NULL) {
		v_cell->tile = new vision_tile(TERRAIN_TILE_SIZE, TERRAIN_TILE_SIZE);
		sprintf(v_cell->tile->filename, "vmap-%s-%d-%d-%06d-%06d.tf.gz",
			utmzone, (int)rint(TERRAIN_TILE_RESOLUTION * 100),
			TERRAIN_TILE_SIZE, c, r);
		fprintf(stderr, "V-Loading %s\n", v_cell->tile->filename);
		if(v_cell->tile->load(v_cell->tile->filename) < 0) {
			fprintf(stderr, " Could not load vision file.\n");
	    		v_cell->tile->utm_x0 = c * tile_grid->resolution;
	    		v_cell->tile->utm_y0 = r * tile_grid->resolution;
	    		v_cell->tile->resolution = TERRAIN_TILE_RESOLUTION;
	    		strcpy(v_cell->tile->utmzone, utmzone);
		}
	}
      }
}

void process_tile(grid_v_cell_t *v_cell) {
	terrain_tile_cell *grid_cell = NULL;
	vision_tile_cell *grid_v_cell = NULL;
	int dx, dy, n;
	double utm_x0 = v_cell->tile->utm_x0;
	double utm_y0 = v_cell->tile->utm_y0;
	//float yaw = applanix_pose->yaw;
	for(dx = 0; dx < TERRAIN_TILE_SIZE; dx++) {
		for(dy = 0; dy < TERRAIN_TILE_SIZE; dy++) {
			double utm_x2 = utm_x0 + dx * TERRAIN_TILE_RESOLUTION;
			double utm_y2 = utm_y0 + dx * TERRAIN_TILE_RESOLUTION;
			double best_yaw = nearest_yaw(utm_x2, utm_y2);
			double x_step = cos(best_yaw);
			double y_step = sin(best_yaw);
			for(n = -20; n <= 20; n++) {	
				double p_x = utm_x0 + (dx + n * x_step) * TERRAIN_TILE_RESOLUTION;
				double p_y = utm_y0 + (dy + n * y_step) * TERRAIN_TILE_RESOLUTION;
      				int r2 = (int)floor(p_y / tile_grid->resolution);
      				int c2 = (int)floor(p_x / tile_grid->resolution);
      				grid_cell_t *cell2 = (grid_cell_t *)dgc_grid_get_rc_global(tile_grid, r2, c2);
				if(!cell2) continue;
				if(!cell2->tile) continue;
				if(!cell2->tile->utm_x0) continue;
 				int tile_x = (int)floor((p_x - cell2->tile->utm_x0) / TERRAIN_TILE_RESOLUTION);
 				int tile_y = (int)floor((p_y - cell2->tile->utm_y0) / TERRAIN_TILE_RESOLUTION);
				grid_cell = &(cell2->tile->cell[tile_x][tile_y]);
				grid_v_cell = &(v_cell->tile->cell[dx][dy]);
				if(!(grid_cell && grid_v_cell))
					continue;
				if(cell2->tile->utm_x0 == 0)
					continue;
				if(grid_cell->i_count > 0)
					grid_v_cell->blur += (1.0 / 40.0) * (1.0/255.0) * grid_cell->intensity / grid_cell->i_count;
				if(n == 0 && grid_cell->i_count > 0)
					grid_v_cell->orig = grid_cell->intensity / grid_cell->i_count;
			}
		}
	}
	for(dx = 5; dx < TERRAIN_TILE_SIZE - 5; dx++) {
		for(dy = 5; dy < TERRAIN_TILE_SIZE - 5; dy++) {
			double utm_x2 = utm_x0 + dx * TERRAIN_TILE_RESOLUTION;
			double utm_y2 = utm_y0 + dx * TERRAIN_TILE_RESOLUTION;
			double best_yaw = nearest_yaw(utm_x2, utm_y2);
			int x1 = int(dx + .5 + 3 * cos(best_yaw + M_PI/2));
			int y1 = int(dy + .5 + 3 * sin(best_yaw + M_PI/2));
			int x2 = int(dx + .5 - 3 * cos(best_yaw + M_PI/2));
			int y2 = int(dy + .5 - 3 * sin(best_yaw + M_PI/2));
			float b1 = v_cell->tile->cell[x1][y1].blur;
			float b2 = v_cell->tile->cell[x2][y2].blur;
			float b0 = v_cell->tile->cell[dx][dy].blur;
			if(!(b1 > 0 && b2 > 0 && b0 > 0))
				continue;
			if(b1 > b2) b2 = b1;
			if(b0 - b2 > 0) {
				//float line = (b0 - b2) * (.01 * v_cell->tile->cell[dx][dy].orig);
				float line = (b0 - b2);
				v_cell->tile->cell[dx][dy].line_lat = line;
			}
		}
	}
	int i, j;
	for(i = 0; i < 100; i++) {
		double x = utmx[i];
		double y = utmy[i];
		double yaw = utmyaw[i];
		int tile_x = (int)floor((x - v_cell->tile->utm_x0) / TERRAIN_TILE_RESOLUTION);
 		int tile_y = (int)floor((y - v_cell->tile->utm_y0) / TERRAIN_TILE_RESOLUTION);
		//printf("Tile coords for pose %2d: (%d, %d)\n", i, tile_x, tile_y);
		if(tile_x > 30 && tile_x < TERRAIN_TILE_SIZE - 30 && tile_y > 30 && tile_y < TERRAIN_TILE_SIZE - 30) {
			printf("Tile coords for pose %2d: (%d, %d)\n", i, tile_x, tile_y);
			for(j = -25; j <= 25; j++) {
				int x = int(tile_x + .5 + j * cos(yaw + M_PI/2));
				int y = int(tile_y + .5 + j * sin(yaw + M_PI/2));
				float lane = v_cell->tile->cell[x][y].line_lat;
				//float blur = v_cell->tile->cell[x][y].blur;
				printf("%3d ", (int)(100*lane));
			}
			printf("\n");
		}
	}
}

void read_parameters(ParamInterface *pint, int argc, char **argv) {
  Param params[] = {
    {"transform", "velodyne", DGC_PARAM_TRANSFORM, &velodyne_offset, 0, NULL},
    {"velodyne", "cal_file", DGC_PARAM_FILENAME, &cal_filename, 0, NULL},
  };
  pint->InstallParams(argc, argv, 
		      params, sizeof(params)/sizeof(params[0]));
}

int 
main(int argc, char **argv) 
{
  grid_cell_t *default_tile = NULL;
  grid_v_cell_t *default_tile_v = NULL;

  if(argc < 3)
    dgc_die("Error: not enough arguments.\n"
            "Usage: %s log-file velodyne-file \n", argv[0]);

  /* connect to IPC server, get parameters, and disconnect */
  ipc = new IpcStandardInterface;
  if (ipc->Connect("map_velodyne") < 0)
    dgc_fatal_error("Could not connect to IPC network.");
  dgc::ParamInterface  *pint;
  pint = new dgc::ParamInterface(ipc);
  read_parameters(pint, argc, argv);
  ipc->Disconnect();

  default_tile = (grid_cell_t *)calloc(1, sizeof(grid_cell_t));
  dgc_test_alloc(default_tile);

  tile_grid = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE,
				  TERRAIN_GRID_NUM_TILES + 2, 
				  TERRAIN_GRID_NUM_TILES + 2,
				  sizeof(grid_cell_t), default_tile);
  dgc_grid_set_clear_handler(tile_grid, grid_clear_handler);

  default_tile_v = (grid_v_cell_t *)calloc(1, sizeof(grid_v_cell_t));
  dgc_test_alloc(default_tile_v);

  vision_grid = dgc_grid_initialize(TERRAIN_TILE_RESOLUTION * TERRAIN_TILE_SIZE,
				  1, 
				  1,
				  sizeof(grid_v_cell_t), default_tile_v);
  dgc_grid_set_clear_handler(vision_grid, grid_v_clear_handler);

  int i;
  for(i = 0; i < 100; i++)
	utmx[i] = utmy[i] = utmyaw[i] = 0.0;

  rndf_create("OneLane");
  LineBuffer *line_buffer = NULL;
  ApplanixPose pose;
  char *line = NULL;
  dgc_FILE *fp;
  fp = dgc_fopen(argv[1], "r");
  if(fp == NULL)
	dgc_die("Error: could not open file %s for reading.\n", argv[1]);
  fprintf(stderr, "Reading logfile...\n");
  line_buffer = new LineBuffer;
  do {
	line = line_buffer->ReadLine(fp);
	if(line != NULL) {
		if(strncmp(line, "APPLANIX_POSE_V2", 16) == 0) {
			StringV2ToApplanixPose(dgc_next_word(line), &pose);
			process(&pose);
		}
	}
  } while(line != NULL);
  dgc_fclose(fp);
  fprintf(stderr, "Done.\n");
  dgc_grid_clear(tile_grid);
  dgc_grid_clear(vision_grid);
  printf("Vision processing complete.");
  rndf_close();
  return 0;
}
