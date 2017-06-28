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

#ifndef AW_OCTREET_H
#define AW_OCTREET_H

//== INCLUDES =================================================================
#include <vec3.h>
#include <vector>
#include <stack>
#include <sstream>
#include <limits>

namespace vlr {

//== CLASS DEFINITION =========================================================
  template <class T>
  class OctreeT
  {

  public:
    typedef T Scalar;
    typedef typename vlr::Vec3<Scalar> Point;
    typedef typename std::vector<Point> PointList;
    typedef typename PointList::iterator PointListIterator;
    typedef typename PointList::const_iterator PointListConstIterator;
    typedef typename std::vector<int> IndexList;
    typedef typename IndexList::const_iterator IndexListConstIterator;

    OctreeT(
      int               num_points,
      const Point*      points,
      Scalar            minRadius,
      int               minPoints);

    OctreeT(
      int               num_points,
      const Point*      points,
      Point             center,
      Scalar            radius,
      Scalar            minRadius,
      int               minPoints);

    OctreeT(
      int               num_points,
      const Point*      points,
      Point             center,
      Scalar            radius,
      IndexList&        indices,
      Scalar            minRadius,
      int               minPoints = 1,
      int               parentIndex = 0,
      OctreeT<T>*       parentTree = NULL);

    ~OctreeT(void);

    bool searchNearest(const Point &p, int& nearestNeighbor, Scalar& distance);
    void refineTree(Scalar minRadius, int minPoints);

    void getCubes(PointList &points, std::vector<int> &edges);
    void getCubeCenters(PointList &centers);
    void getPoints(PointList &points);
    void getIndices(IndexList& indices);
    void getLeaves(std::vector< OctreeT<T>* >& leafes);

    Point center() { return m_center; }
    Scalar radius() { return m_radius; }
    bool isLeaf() { return m_isLeaf; }


  private:
    void initialize(IndexList& indices, Scalar& minRadius, int& minVertices);
    bool closerToPoint(const Point &p, const int& otherIndex, Scalar& distance);

  private:
    int                       m_num_points;
    const Point*              m_points;
    Point                     m_center;
    Scalar                    m_radius;
    int                       m_parentIndex;
    OctreeT<T>*               m_parentTree;
    OctreeT<T>*               m_child[8];

    bool                      m_isLeaf;
    std::vector<int>          m_indices; // point indices, groups of three
  };
//=============================================================================

} // namespace vlr

#if !defined(OCTREE_TEMPLATE)
#include "aw_octree_template.h"
#endif

#endif // AW_OCTREET_H defined

