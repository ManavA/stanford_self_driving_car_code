/*
 * Copyright (c) 2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: player_laser_log_to_msg.cpp 27908 2009-12-18 06:34:19Z sachinc $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

\author Radu Bogdan Rusu

@b player_laser_log_to_msg converts Player (http://player.sf.net) laser logs to valid laser scans in ROS format.

 **/

// ROS core
#include <ros/ros.h>
// ROS messages
#include <sensor_msgs/LaserScan.h>

#include <fstream>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace ros;
using namespace sensor_msgs;

class PlayerLogToMsg
{
  protected:
    string tf_frame_;
    NodeHandle nh_;

  public:

    // ROS messages
    LaserScan msg_scan_;

    string file_name_, msg_topic_;
    Publisher scan_pub_;

    ifstream logfile_stream_;
    bool is_file_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PlayerLogToMsg () : tf_frame_ ("laser_tilt_mount_link"),
                        is_file_ (true)
    {
      msg_topic_ = "/laser_scan";
      scan_pub_ = nh_.advertise<LaserScan> (msg_topic_.c_str (), 1);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PlayerLogToMsg () { }

    ////////////////////////////////////////////////////////////////////////////////
    // Start
    int
      start ()
    {
      // Open file
      logfile_stream_.open (file_name_.c_str ());
      long int nr_lines = count (istreambuf_iterator<char>(logfile_stream_), istreambuf_iterator<char> (), '\n');

      if (nr_lines == 0)
        ROS_WARN ("No lines found in %s", file_name_.c_str ());
      logfile_stream_.seekg (0, ios::beg);
      ROS_INFO ("Extracting points from %s ...", file_name_.c_str ());
      msg_scan_.header.frame_id = tf_frame_;
      return (0);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Spin (!)
    bool
      spin ()
    {
      double ti, tj = 0, tdif = 0;
      int total_nr_points = 0, total_laser_scans = 0;
      string line;
      vector<string> st;

      while (nh_.ok ())
      {
        getline (logfile_stream_, line);
        // Do we assume that the input is a file? If so, and EOF, break
        if (logfile_stream_.eof () && is_file_)
          break;
        // If any bad/eof/fail flags are set, continue
        if (!logfile_stream_.good ())
        {
          usleep (500);
          logfile_stream_.clear ();
          continue;
        }
        // If the line is empty, continue
        if (line == "")
          continue;

        // Split a line into tokens
        boost::trim (line);
        boost::split (st, line, boost::is_any_of (" "), boost::token_compress_on);

        string line_type = st.at (0);
        if (line_type.substr (0, 1) == "#")
          continue;

        // Get the interface name
        string interface = st.at (3);

        // ---[ laser
        if (interface.substr (0, 5) != "laser")
          continue;

        ti = atof (line_type.c_str ());

        tdif = fabs (ti - tj);                    // Just in case time decides to go backwards :)

        msg_scan_.header.stamp = Time (ti);

        // index = st.at (4)
        int type    = atoi (st.at (5).c_str ());
        int subtype = atoi (st.at (6).c_str ());

        if (type != 1 || subtype != 1)            // we only process PLAYER_LASER_DATA_SCAN packets for now (1:1)
          continue;

        msg_scan_.angle_min = atof (st.at (8).c_str ());
        msg_scan_.angle_max = atof (st.at (9).c_str ());
        msg_scan_.angle_increment = atof (st.at (10).c_str ());
        msg_scan_.range_min = 0;
        msg_scan_.range_max = atof (st.at (11).c_str ());
        msg_scan_.ranges.resize (atoi (st.at (12).c_str ()));
        msg_scan_.intensities.resize (msg_scan_.ranges.size ());

        if (msg_scan_.ranges.size () == 0)        // If no points found, continue to the next packet
          continue;

        int j = 0;
        for (unsigned int i = 0; i < msg_scan_.ranges.size (); i++)
        {
          msg_scan_.ranges[i] = atof (st.at (13 + i + j).c_str ());
          msg_scan_.intensities[i] = atoi (st.at (14 + i + j).c_str ());
          j++;
        }
        total_nr_points += msg_scan_.ranges.size ();
        ++total_laser_scans;

        if (is_file_)
          ROS_DEBUG ("Publishing data (%d) with %d points on topic %s in frame %s (%f). Angle min/max/resulution: %f/%f/%f, Range min/max: %f/%f",
                     total_laser_scans, (int)msg_scan_.ranges.size (), nh_.resolveName (msg_topic_).c_str (), msg_scan_.header.frame_id.c_str (), msg_scan_.header.stamp.toSec (), 
                     msg_scan_.angle_min, msg_scan_.angle_max, msg_scan_.angle_increment, msg_scan_.range_min, msg_scan_.range_max);
        scan_pub_.publish (msg_scan_);

        // Sleep for a certain number of seconds (tdif)
        if (tj != 0 && is_file_)
        {
          Duration tictoc (tdif);
          tictoc.sleep ();
        }

        spinOnce ();
        tj = ti;

        logfile_stream_.clear ();
      }

      // Close the file and finish the movie
      logfile_stream_.close ();
      ROS_INFO ("[done : %d measurements with %d points extracted]", total_laser_scans, total_nr_points);

      return (true);
    }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  if (argc < 3)
  {
    ROS_WARN ("Syntax is: %s <file.log> [bool is_file (0/1)]", argv[0]);
    ROS_INFO ("           [note: set 'is_file' to 0 if the input .log file is a stream (i.e., data is still written to it during the execution of %s)]", argv[0]);
    ROS_INFO ("Usage example: %s 1190378714.log 0", argv[0]);
    return (-1);
  }

  init (argc, argv, "player_laser_log_to_msg");

  PlayerLogToMsg p;
  p.file_name_ = string (argv[1]);
  p.is_file_   = atoi (argv[2]);

  if (p.start () == -1)
  {
    ROS_ERROR ("Error processing %s. Exiting.", argv[1]);
    return (-1);
  }
  p.spin ();

  return (0);
}
/* ]--- */
