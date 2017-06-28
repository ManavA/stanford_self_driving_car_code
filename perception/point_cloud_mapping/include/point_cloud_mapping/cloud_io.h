/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu -=- cs.tum.edu>
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
 * $Id: cloud_io.h 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_IO_CLOUDIO_H_
#define _CLOUD_IO_CLOUDIO_H_

// ROS includes
#include <sensor_msgs/PointCloud.h>

#include <vector>
#include <fstream>
#include <sys/mman.h>
#include <fcntl.h>

namespace cloud_io
{
  int loadPCDFile (const char* file_name, sensor_msgs::PointCloud &points);

  int savePCDFile (const char* file_name, const sensor_msgs::PointCloud &points, bool binary_mode = false);

  int savePCDFileASCII (const char* file_name, const sensor_msgs::PointCloud &points, int precision);
  int savePCDFileBinary (const char* file_name, const sensor_msgs::PointCloud &points);

  int getIndex (sensor_msgs::PointCloud *points, std::string value);

  std::string addCurrentHeader (const sensor_msgs::PointCloud &points, bool binary_type);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create the PCD header comments */
  inline std::string
    createNewHeader ()
  {
    return std::string (
      "# .PCD v0.3 - Point Cloud Data file format\n"
      "#\n"
      "# Column legend:\n"
      "#   x,  y,  z - 3d point coordinates\n"
      "#   r,  g,  b - RGB point color information\n"
      "#\n"
      "# DATA specifies whether points are stored in binary format or ascii text. Header must be page aligned (4096 on most systems)\n#\n");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create shorter PCD header comments */
  inline std::string
    createNewHeaderShort ()
  {
    return std::string (
      "# .PCD v0.3 - Point Cloud Data file format\n"
      "#\n");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Get the available point cloud channels as a space separated string
    * \param points a pointer to the PointCloud message
    */
  inline std::string
    getAvailableDimensions (const sensor_msgs::PointCloud &points)
  {
    std::string result = "x y z";
    unsigned int i;
    for (i = 0; i < points.channels.size (); i++)
    {
      if (points.channels[i].values.size () == 0)
        continue;

      result = result + " " + points.channels[i].name;
    }
    return (result);
  }

}
#endif
