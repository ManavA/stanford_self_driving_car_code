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


#include <sys/types.h>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <malloc.h>
#include <stdlib.h>
#include <string>
#include <global.h>

using namespace dgc;

namespace vlr {

static int myFilter(const dirent* a) {
  if (strncmp(a->d_name, "tty", 3) == 0) return 1;
  return 0;
}

static int fileInt(const std::string& filename) {
  FILE *fp;
  int i;

  fp = fopen(filename.c_str(), "r");
  if (fp == NULL) return -1;
  if (fscanf(fp, "%x", &i) != 1) {
    dgc_error("Error reading int from file (usbfind)");
  }
  fclose(fp);
  return i;
}

static void fileString(const std::string& filename, char* str, int max_l) {
  FILE *fp;

  fp = fopen(filename.c_str(), "r");
  if (fp == NULL) {
    str[0] = '\0';
    return;
  }
  if (fgets(str, max_l, fp) != str) dgc_error("Error reading string from file (usbfind)");
  if (strlen(str) > 0 && str[strlen(str) - 1] == '\n') str[strlen(str) - 1] = '\0';
  fclose(fp);
}

int dgc_usbfind_lookup(int device_vendor, int device_product, char *device_serial, char *device) {
  struct dirent **namelist;
  char filename[300], serial[200];
  int i, n, vendor, product, found = 0;

  n = scandir("/sys/bus/usb-serial/devices/", &namelist, vlr::myFilter, 0);
  if (n < 0) perror("scandir");
  else {
    for (i = 0; i < n; i++) {
      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../idVendor", namelist[i]->d_name);
      vendor = fileInt(filename);

      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../idProduct", namelist[i]->d_name);
      product = fileInt(filename);

      sprintf(filename, "/sys/bus/usb-serial/devices/%s/../../serial", namelist[i]->d_name);
      fileString(filename, serial, 200);

      if (vendor == device_vendor && product == device_product && (device_serial == NULL || strcasecmp(serial, device_serial) == 0)) {
        sprintf(device, "/dev/%s", namelist[i]->d_name);
        found = 1;
        break;
      }
    }
    for (i = 0; i < n; i++)
      free(namelist[i]);
    free(namelist);
  }
  return found;
}

#define _MAX_NAME_LENGTH   300

static char* get_word(char* str) {
  return (strtok(str, " "));
}

bool usbFindLookupParamString(const std::string& str, std::string& res) {
  char usb[_MAX_NAME_LENGTH], vendor[_MAX_NAME_LENGTH];
  char product[_MAX_NAME_LENGTH], serial[_MAX_NAME_LENGTH];
  char name[_MAX_NAME_LENGTH];
  int ret;
  int n = sscanf(str.c_str(), "%[^(](%[^:]:%[^):]:%[^)]", usb, vendor, product, serial);

  if (!strcasecmp(get_word(usb), "usb")) {
    if (n == 4) {
      ret = dgc_usbfind_lookup(strtol(get_word(vendor), NULL, 16), strtol(get_word(product), NULL, 16), get_word(serial), name);
      if (!ret) return false;
      else {
        res = name; // port
        return true;
      }
    }
    else if (n == 3) {
      ret = dgc_usbfind_lookup(strtol(get_word(vendor), NULL, 16), strtol(get_word(product), NULL, 16), NULL, name);
      if (!ret) return false;
      else {
        res = name; // port
        return true;
      }
    }
    else {
      return false;
    }
  }

  res = str;    // ?!?
  return true;
}

} //namespace vlr
