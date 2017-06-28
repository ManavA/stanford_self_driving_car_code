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
 * $Id: intersections.h 21045 2009-08-07 20:52:44Z stuglaser $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_GEOMETRY_INTERSECTIONS_H_
#define _CLOUD_GEOMETRY_INTERSECTIONS_H_

// ROS includes
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>

namespace cloud_geometry
{

  namespace intersections
  {

    bool planeWithPlaneIntersection (const std::vector<double> &plane_a, const std::vector<double> &plane_b, std::vector<double> &line);
    bool lineWithPlaneIntersection (const std::vector<double> &plane, const std::vector<double> &line, geometry_msgs::Point32 &point);
    bool lineWithLineIntersection (const std::vector<double> &line_a, const std::vector<double> &line_b, geometry_msgs::Point32 &point, double sqr_eps);
    bool planeWithCubeIntersection (const std::vector<double> &plane, const std::vector<double> &cube, geometry_msgs::Polygon &polygon);
    bool lineToBoxIntersection (const std::vector<double> &line, const std::vector<double> &cube);
  }
}

#endif
