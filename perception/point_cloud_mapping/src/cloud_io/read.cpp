/*
 * Copyright (c) 2008-2009 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: read.cpp 31245 2010-02-15 07:12:07Z rusu $
 *
 */

/** \author Radu Bogdan Rusu */

#include <stdlib.h>
#include <point_cloud_mapping/cloud_io.h>
#include <boost/algorithm/string.hpp>

namespace cloud_io
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Load point cloud data from a PCD file containing n-D points.
  * Returns -1 on error, > 0 on success (denoting the underlying PCD file type: 0=ascii, 1=binary)
  * \note All lines besides:
  * - the ones beginning with # (treated as comments)
  * - COLUMNS ...
  * - POINTS ...
  * - DATA ascii/binary
  * ...are intepreted as data points.
  * \param file_name the name of the file to load
  * \param points the resulting point array
  */
  int
    loadPCDFile (const char* file_name, sensor_msgs::PointCloud &points)
  {
    bool binary_data = false;
    int nr_points = 0;
    std::ifstream fs;
    std::string line;

    int specified_channel_count = 0;
    // for reading image 2d coordinates if ARRAY is specified
    int array_width = 0;
    int array_height = 0;

    int idx = 0;
    // Open file
    fs.open (file_name);
    if (!fs.is_open () || fs.fail ())
      return (-1);

    // Read the header and fill it in with wonderful values
    while (!fs.eof ())
    {
      getline (fs, line);
      if (line == "")
        continue;

      std::vector<std::string> st;
      boost::split (st, line, boost::is_any_of (" "), boost::token_compress_on);

      std::string line_type = st.at (0);

      // ---[ Perform checks to see what does this line represents
      if (line_type.substr (0, 1) == "#")
        continue;
      // Get the column indices
      if ( (line_type.substr (0, 7) == "COLUMNS") || (line_type.substr (0, 6) == "FIELDS") )  
      {
        int remaining_tokens = st.size () - (1 + 3);
        //specified_channel_count = st.size () - 1; // used to be this, fixed below
        specified_channel_count = remaining_tokens; // need this to make ARRAY work
        points.set_channels_size (remaining_tokens);
        for (int i = 0; i < remaining_tokens; i++)
        {
          std::string col_type = st.at (i + 4);
          points.channels[i].name = col_type;
        }

        continue;
      }
      if ( (line_type.substr (0, 4) == "SIZE") || (line_type.substr (0, 4) == "TYPE") || (line_type.substr (0, 5) == "WIDTH") || (line_type.substr (0, 6) == "HEIGHT") )
        continue;
      // Get the number of points
      if (line_type.substr (0, 6) == "POINTS")
      {
        nr_points = atoi (st.at (1).c_str ());
        points.set_points_size (nr_points);

        for (unsigned int d = 0; d < points.get_channels_size (); d++)
          points.channels[d].set_values_size (nr_points);

        continue;
      }

      // Check DATA type
      if (line_type.substr (0, 4) == "DATA")
      {
        if (st.at (1).substr (0, 6) == "binary")
        {
          binary_data = true;
          break;
        }
        continue;
      }

      // load array dimensions from disparity image if present
      // this assumes that you will already have seen the "real" channel names
      if (line_type.substr (0, 5) == "ARRAY")
      {
        array_width = atoi (st.at (1).c_str ());
        array_height = atoi (st.at (2).c_str ());
        points.set_channels_size (specified_channel_count + 2);
        points.channels[specified_channel_count].name = "array_width";
        points.channels[specified_channel_count+1].name = "array_height";
        points.channels[specified_channel_count].set_values_size (1);
        points.channels[specified_channel_count+1].set_values_size (1);
        points.channels[specified_channel_count].values[0] = array_width;
        points.channels[specified_channel_count+1].values[0] = array_height;
        continue;
      }

      // Nothing of the above? We must have points then
      // Convert the first token to float and use it as the first point coordinate
      if (idx >= nr_points)
      {
        //fprintf (stderr, "Error: input file %s has more points than advertised (%d)!\n", file_name, nr_points);
        break;
      }

      // Assume x-y-z to be the first dimensions in the file
      points.points[idx].x = atof (st.at (0).c_str ());
      points.points[idx].y = atof (st.at (1).c_str ());
      points.points[idx].z = atof (st.at (2).c_str ());
      for (int i = 0; i < specified_channel_count; i++)
        points.channels[i].values[idx] = atof (st.at (i+3).c_str ());

      idx++;
    }
    // Close file
    fs.close ();

    /// ---[ Binary mode only
    /// We must re-open the file and read with mmap () for binary
    if (binary_data)
    {
      // Open for reading
      int fd = open (file_name, O_RDONLY);
      if (fd == -1)
        return (-1);

      // Compute how much a point (with it's N-dimensions) occupies in terms of bytes
      int point_size = sizeof (float) * (points.channels.size () + 3);

      // Prepare the map
      char *map = (char*)mmap (0, points.points.size () * point_size, PROT_READ, MAP_SHARED, fd, getpagesize ());
      if (map == MAP_FAILED)
      {
        close (fd);
        return (-1);
      }

      // Read the data
      for (unsigned int i = 0; i < points.points.size (); i++)
      {
        int idx_j = 0;

        // Copy the x-y-z point coordinates
        memcpy (&points.points[i].x, (char*)&map[i * point_size + idx_j], sizeof (float));
        idx_j += sizeof (float);
        memcpy (&points.points[i].y, (char*)&map[i * point_size + idx_j], sizeof (float));
        idx_j += sizeof (float);
        memcpy (&points.points[i].z, (char*)&map[i * point_size + idx_j], sizeof (float));
        idx_j += sizeof (float);

        for (unsigned int j = 0; j < points.channels.size (); j++)
        {
          memcpy (&points.channels[j].values[i], (char*)&map[i * point_size + idx_j], sizeof (float));
          idx_j += sizeof (float);
        }
      }

      // Unmap the pages of memory
      if (munmap (map, points.points.size () * point_size) == -1)
      {
        close (fd);
        return (-1);
      }
      close (fd);
    }

    if ( (idx != nr_points) && (!binary_data) )
    {
      ROS_ERROR ("Warning! Number of points read (%d) is different than expected (%d)", idx, nr_points);
      return (-1);
    }

    return (binary_data);
  }
}
