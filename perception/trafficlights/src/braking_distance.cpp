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

#include <roadrunner.h>
#include <can_interface.h>
#include <controller_interface.h>
#include <logio.h>
#include <gnuplot.h>

using namespace dgc;

/*given a logfile, this program calculates how many times the vehicle came to a complete stop, within a certain stopping tolerance,
and the g's and distance involved in coming to a stop*/

int main(int argc, char **argv)
{
	LineBuffer *line_buffer = NULL;
	ControllerTarget target;
	CanStatus can;
	char *line = NULL, *left;
	int received_target = 0;
	double log_timestamp, prevlog_timestamp;
	double start_time = 0;
	FILE *data_fp;
	dgc_FILE *fp;

	double currVelocity = 0;
	double prevVelocity = 0;
	double maxPrevVelocity = 0;

	bool in_braking_section = false;
	double cumulative_distance = 0;
	int brakeCount = 0;

	double stop_tolerance;
	double start_after;

	/* interpet command line parameters */
	if (argc < 4)
		dgc_die("Error: not enough arguments\n"
		        "Usage: %s <logfile> <stop tolerance (i.e. do you want to count rolling stops?) in m/s> <how many seconds into logfile do you want to start? 0 = the beginning> \n", argv[0]);

	/*data_fp = fopen("data.txt", "w");
	if(data_fp == NULL)
	  dgc_die("Error: could not open temporary file data.txt");*/

	fp = dgc_fopen(argv[1], "r");
	if (fp == NULL)
		dgc_die("Error: could not open file %s for reading.\n", argv[1]);

	stop_tolerance = atof(argv[2]);
	start_after = atof(argv[3]);

	fprintf(stderr, "Reading logfile... \n\n");
	line_buffer = new LineBuffer;
	do
	{
		line = line_buffer->ReadLine(fp);
		if (line != NULL)
		{
			if (strncmp(line, "CAN3", 4) == 0)
			{
				left = StringV3ToCanStatus(dgc_next_word(line), &can);
				log_timestamp = READ_DOUBLE(&left);

				currVelocity = 0.5 * dgc_kph2ms(can.wheel_speed_rl + can.wheel_speed_rr);

				if (in_braking_section)
				{
					//integration by trapezoid method to get cumulative distance
					cumulative_distance += (log_timestamp - prevlog_timestamp)*((prevVelocity + currVelocity)/2.0);

					if (currVelocity > maxPrevVelocity)
					{
						maxPrevVelocity = currVelocity;
						start_time = log_timestamp;
						in_braking_section = false;
						cumulative_distance = 0;
					}
					else if (currVelocity < stop_tolerance) //we've stopped
					{
						in_braking_section = false;
						double deccel = (maxPrevVelocity/(log_timestamp - start_time))/9.8;
						printf("braking distance at time elapsed = %f sec, starting from %f m/s at a deceleration rate of %fg's, was %fm\n",
						       start_time, maxPrevVelocity, deccel, cumulative_distance);
						brakeCount++;
						maxPrevVelocity = 0;
						cumulative_distance = 0;
					}
				}
				if (prevVelocity > 1 && prevVelocity > currVelocity && !in_braking_section && log_timestamp > start_after)
				{
					in_braking_section = true;
				}
			}
		}
		prevlog_timestamp = log_timestamp;
		prevVelocity = currVelocity;

	}
	while (line != NULL);
	dgc_fclose(fp);
	//fclose(data_fp);
	fprintf(stderr, "done.\n");

	printf("you came to a complete stop %d times at a stop tolerance of %f m/s\n", brakeCount, stop_tolerance);
	return 0;
}

