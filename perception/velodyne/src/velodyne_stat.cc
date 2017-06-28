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


#include <lltransform.h>
#include <velocore.h>
#include <fnmatch.h>
#include <signal.h>

namespace dgc {

#define MAX_NAME_LENGTH       256
#define MAX_TIME_DELAY        1.0
#define MAX_NUM_SCANS         20000

void
quit_program( int sig __attribute__ ((unused)) )
{
  fprintf( stderr, "\n# INFO: quit program with CTRL-C\n" );
  exit(0);
}

void
print_usage( char *prgname )
{
  dgc_die( "Usage: %s <FILENAME>\n", prgname );
}

} // namespace dgc

using namespace dgc;

int 
main( int argc, char **argv )
{
  dgc_velodyne_packet_t     pkt;
  int                       log_reading = TRUE;
  int                       err;
  long                      num_pkts = 0, gaps = 0, big_gaps = 0;
  double                    avg_ts = 0;
  double                    max_gap = 0;
  double                    last_ts = 0, diff_ts, info_ts = 0;
  double                    start_ts = 0, duration_ts = 0;
  int                       firsttime = TRUE;
  struct tm               * local_time;
  time_t                    current_time;

  dgc_velodyne_scan_p       scans        = NULL;
  dgc_velodyne_file_p       velodyne     = NULL;

  if (argc!=2) {
    print_usage(argv[0]);
    exit(0);
  }

  /* read logfile */
  fprintf(stderr, "# INFO: reading in logfile data... \n");

  /* allocate memory */
  scans     = 
    (dgc_velodyne_scan_p) malloc(MAX_NUM_SCANS * sizeof(dgc_velodyne_scan_t) );
  dgc_test_alloc(scans);
  
  velodyne = dgc_velodyne_open_file(argv[argc - 1]);
  if(velodyne == NULL)
    dgc_die("Error: could not open velodyne file %s\n", argv[argc - 1]);

  signal( SIGINT, quit_program );

  while (log_reading) {
    /* read one package to get first timestamp of the velodyne data */
    err = dgc_velodyne_read_packet(velodyne, &pkt);
    if(err < 0) {
      log_reading = FALSE;
    }
    if (firsttime) {
      firsttime = FALSE;
      info_ts = start_ts = pkt.timestamp;
      current_time = (long) start_ts;
      local_time = localtime(&current_time);
      fprintf( stderr, "# INFO: start time: %f (%02d.%02d.%04d - %02d:%02d.%02d)\n", 
	       start_ts, 
	       local_time->tm_mon + 1, 
	       local_time->tm_mday, 
	       local_time->tm_year + 1900,
	       local_time->tm_hour, 
	       local_time->tm_min, 
	       local_time->tm_sec );

    } else { 
      if (pkt.timestamp-info_ts>10) {
	info_ts = pkt.timestamp;
	fprintf( stderr, "\r                                            \r"
		 "# INFO: time = %f", info_ts );
      }
      diff_ts = pkt.timestamp-last_ts;
      avg_ts += diff_ts;
      num_pkts++;
      if (diff_ts>0.01) {
	gaps++;
	if (diff_ts>1.0) {
	  big_gaps++;
	  fprintf( stderr, "\r                                            \r"
		   "# INFO: large gap %ld - %.2f sec at %f\n", 
		   big_gaps, diff_ts, pkt.timestamp );
	}
	if (diff_ts>max_gap) {
	  max_gap = diff_ts;
	}
      }
    }
    last_ts =  pkt.timestamp;
  }
  duration_ts = pkt.timestamp - start_ts;
  fprintf( stderr, "\r                                            \r" );
  fprintf( stderr, "# INFO: number of pkts:               %ld\n", num_pkts );
  fprintf( stderr, "# INFO: duration of file:             %.2f\n", duration_ts );
  fprintf( stderr, "# INFO: average time between packets: %f\n", 
	   avg_ts / (double) num_pkts );
  fprintf( stderr, "# INFO: number of gaps:               %ld (%f%%)\n", 
	   gaps, 100.0 * gaps / (double) num_pkts );
  fprintf( stderr, "# INFO: number of large gaps:         %ld\n", big_gaps );
  fprintf( stderr, "# INFO: biggest gap:                  %f\n", max_gap );
  return(0);
}

