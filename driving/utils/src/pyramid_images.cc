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


#include <string>
#include <iostream>
#include <global.h>
#include <imagery.h>
#include <dirent.h>
#include <opencv2/core/core.hpp>

using namespace dgc;
using namespace vlr;

std::string image_root;
std::string laser_image_root;

void source_filename(char *srcdir, char *utmzone, int image_size, double image_resolution, int x, int y, char *filename) {
  sprintf(filename, "%s/lmap-%s-%d-%d-%06d-%06d.png", srcdir, utmzone, (int) rint(image_resolution * 100), image_size, x, y);
}

void make_halfres_image(char *srcdir, char *destdir, int x1, int y1, double image_resolution, char *utmzone) {
  int x, y;
  cv::Mat* big_image = NULL, *small_image = NULL;
  char filename[200], bigfilename[200];
  int big_width = 0;
  int yt, dx0, dy0;

  source_filename(destdir, utmzone, 500, image_resolution * 2, x1 / 2, y1 / 2, bigfilename);
  if (dgc_file_exists(bigfilename)) {
    fprintf(stderr, "Skipping %s\n", bigfilename);
    return;
  }

  for (x = x1; x < x1 + 2; x++)
    for (y = y1; y < y1 + 2; y++) {
      source_filename(srcdir, utmzone, 500, image_resolution, x, y, filename);
      if (!dgc_file_exists(filename)) continue;

      else if (big_image == NULL) {
        big_image = new cv::Mat(1000, 1000, CV_8UC3);
        big_width = 1000;
      }

      cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_ANYCOLOR);
      cv::Mat flipped_img(500, 500, CV_8UC3);
      cv::flip(image, flipped_img, 0);
      image = flipped_img;
      dx0 = (x - x1) * 500;
      dy0 = (y - y1) * 500;
      for (yt = 0; yt < image.rows; yt++)
//        memcpy(big_image->data + ((dy0 + yt) * big_width + dx0) * 3, image.data + yt * image.cols * 3, image.cols * 3);
      memcpy(big_image->data + ((dy0 + 499 - yt) * big_width + dx0) * 3, image.data + yt * image.cols * 3, image.cols * 3);
    }

  if (big_image != NULL) {
    small_image = new cv::Mat(500, 500, CV_8UC3);
    for (x = 0; x < 500; x++)
      for (y = 0; y < 500; y++)
        memcpy(small_image->data + (y * 500 + x) * 3, big_image->data + (y * 2 * 1000 + x * 2) * 3, 3);

    fprintf(stderr, "Writing %s\n", bigfilename);
    cv::imwrite(bigfilename, *small_image);
    delete big_image;
    delete small_image;
  }
}

#define UTMZONE "10S"

int filter(const struct dirent* d)
{
  char comparestr[100];

  sprintf(comparestr, "lmap-%s-15-500-", UTMZONE);
  if(strncmp(d->d_name, comparestr, strlen(comparestr)) == 0) {return 1;}
  return 0;
}

int main(int argc, char **argv) {
  struct dirent **namelist;
  char numstr[100];
  int n, num;
  int x, y;
  double res;
  int min_x = 1000000, min_y = 1000000, max_x = -1, max_y = -1;

  if(argc < 2) {
    std::cout << "Usage: " << argv[0] << " <imagery dir>\n";
    exit(-5);
  }
  image_root = argv[1];
  
  /* get the laser image root directory */
  laser_image_root = image_root;
  laser_image_root = laser_image_root + "/laser/";

  /* find the 5 cm images - compute x, y bounds */
  n = scandir(laser_image_root.c_str(), &namelist, filter, alphasort);
  if (n < 0) dgc_die("Error: could not get file list\n");
  else {
    while (n--) {
      strncpy(numstr, namelist[n]->d_name + 16, 6);
      numstr[6] = '\0';
      num = atoi(numstr);
      if (num < min_x) min_x = num;
      if (num > max_x) max_x = num;

      strncpy(numstr, namelist[n]->d_name + 23, 6);
      numstr[6] = '\0';
      num = atoi(numstr);
      if (num < min_y) min_y = num;
      if (num > max_y) max_y = num;

      free(namelist[n]);
    }
  }

  res = 0.15;
  while (res < 10) {
    fprintf(stderr, "Making %2fm images...\n", res);

    /* start on an even image number */
    min_x = min_x / 2 * 2;
    min_y = min_y / 2 * 2;

    /* make the half-res images */
    for (x = min_x; x <= max_x; x += 2)
      for (y = min_y; y <= max_y; y += 2)
        make_halfres_image((char*)laser_image_root.c_str(), (char*)laser_image_root.c_str(), x, y, res, UTMZONE);

    /* change the tile coordinates for next layer up */
    min_x /= 2;
    min_y /= 2;
    min_x--;
    min_y--;
    max_x /= 2;
    max_y /= 2;
    max_x++;
    max_y++;
    res *= 2;
  }
  return 0;
}
