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

#define OCTREE_TEMPLATE
#define DEBUG_LEVEL 0

//== INCLUDES =================================================================
#include <vec3.h>
#include <aw_geometry.h>

//== NAMESPACES ===============================================================
using namespace std;

namespace vlr {

//== IMPLEMENTATION ==========================================================
template <class T>
OctreeT<T>::OctreeT(
  int num_points,
  const Point* points,
  Scalar  minRadius,
  int minPoints)
  : m_num_points(num_points), m_points(points)
{
  // calculate bounding box
  Point bbMin(points[0]), bbMax(points[0]);

  // create index list
  IndexList indices;
  //indices.reserve(points.size);

  // generate point indices
  for (int i=0; i<num_points;++i) {
    indices.push_back(i);
    bbMin.minimize(points[i]);
    bbMax.maximize(points[i]);
  }

  //cout << "bbMin " << bbMin << " bbMax " << bbMax << endl;

  // set members
  m_center = (bbMax-bbMin)/(Scalar)2.0;
  m_radius = bbMax.max();
  m_parentIndex = 0;
  m_parentTree = NULL;

  // initialize octree
  initialize(indices,minRadius,minPoints);
}

template <class T>
OctreeT<T>::OctreeT(
  int               num_points,
  const Point*      points,
  Point             center,
  Scalar            radius,
  Scalar            minRadius,
  int               minPoints)
  : m_num_points(num_points), m_points(points), m_center(center), m_radius(radius)
{
  // create index list
  IndexList indices;
  //indices.reserve(points.size);

  // generate point indices
  for (int i=0; i<num_points;++i) {
    indices.push_back(i);
  }

  // set members
  m_parentIndex = 0;
  m_parentTree = NULL;

  // initialize octree
  initialize(indices,minRadius,minPoints);
}

template <class T>
OctreeT<T>::OctreeT(
  int               num_points,
  const Point*      points,
  Point             center,
  Scalar            radius,
  IndexList&        indices,
  Scalar            minRadius,
  int               minPoints,
  int               parentIndex,
  OctreeT<T>*       parentTree) :
  m_num_points(num_points), m_points(points), m_center(center), m_radius(radius), m_parentIndex(parentIndex), m_parentTree(parentTree)
{
  initialize(indices,minRadius,minPoints);
}

template <class T>
OctreeT<T>::~OctreeT()
{
  if (m_isLeaf) return;
  for (int i=0; i<8; i++) {
    if (m_child[i])  {
      delete m_child[i];
    }
  }
}

template <class T>
void OctreeT<T>::
initialize(IndexList& indices, Scalar& minRadius, int& minPoints)
{
  bool subdivided=false;
  Point vert;
  m_indices.clear();
  m_isLeaf = false;

  // check whether we need to subdivide more
  if (m_radius < 2.0*minRadius) {
    m_isLeaf = true;
    m_indices = indices;
    m_child[0] = m_child[1] = m_child[2] = m_child[3] = NULL;
    m_child[4] = m_child[5] = m_child[6] = m_child[7] = NULL;
    return;
  }

  // if so, try for triangle and each child whether they intersect
  Scalar halfR = m_radius*.5;
  Point childCtr;

  IndexList xp, xm; // x plus, x minus
  IndexList yp, ym; // y plus, y minus
  IndexList zp, zm; // z plus, z minus
  xp.reserve(indices.size());
  xm.reserve(indices.size());

  // split about x axis
  // find closest vertex
  IndexListConstIterator vIt, vIt_end;
  for(vIt=indices.begin(), vIt_end=indices.end(); vIt!=vIt_end; ++vIt)
  {
    vert = m_points[*vIt];
    if (vert[0] >= m_center[0]) {
      xp.push_back(*vIt);
    } else {
      xm.push_back(*vIt);
    }
  }

  // split about y axis
  IndexList &xv = xm;
  IndexList &yv = ym;
  for (int i=0; i<2; i++) {
    if (i) xv = xp;
    else   xv = xm;
    yp.clear(); yp.reserve(xv.size());
    ym.clear(); ym.reserve(xv.size());

    for(vIt=xv.begin(), vIt_end=xv.end(); vIt!=vIt_end; ++vIt)
    {
      vert = m_points[*vIt];
      if (vert[1] >= m_center[1]) {
        yp.push_back(*vIt);
      } else  {
        ym.push_back(*vIt);
      }
    }

    // split about z axis
    for (int j=0; j<2; j++) {
      if (j) yv = yp;
      else   yv = ym;
      zp.clear(); zp.reserve(yv.size());
      zm.clear(); zm.reserve(yv.size());

      for(vIt=yv.begin(), vIt_end=yv.end(); vIt!=vIt_end; ++vIt)
      {
        vert = m_points[*vIt];
        if (vert[2] >= m_center[2]) {
          zp.push_back(*vIt);
        } else {
          zm.push_back(*vIt);
        }
      }

      // create the children, if needed
      childCtr[0] = m_center[0] + (i ? halfR : -halfR);
      childCtr[1] = m_center[1] + (j ? halfR : -halfR);
      childCtr[2] = m_center[2] - halfR;

      int k = i+2*j;
      if (zm.size()>=static_cast<unsigned int>(minPoints)) {
        m_child[k] = new OctreeT(m_num_points, m_points, childCtr, halfR, zm, minRadius, minPoints, k, this);
        subdivided = true;
      } else {
        m_child[k] = NULL;
      }

      childCtr[0] = m_center[0] + (i ? halfR : -halfR);
      childCtr[1] = m_center[1] + (j ? halfR : -halfR);
      childCtr[2] = m_center[2] + halfR;

      k += 4;
      if (zp.size()>=static_cast<unsigned int>(minPoints)) {
        m_child[k] = new OctreeT(m_num_points, m_points, childCtr, halfR, zp, minRadius, minPoints, k, this);
        subdivided = true;
      } else {
        m_child[k] = NULL;
      }
    }
  }

  // no subdivision made
  if(!subdivided)
  {
    m_indices = indices;
    m_isLeaf = true;
    m_child[0] = m_child[1] = m_child[2] = m_child[3] = NULL;
    m_child[4] = m_child[5] = m_child[6] = m_child[7] = NULL;
  }
}

template <class T>
void OctreeT<T>::
refineTree(Scalar minRadius, int minPoints)
{
  if(m_isLeaf)
  {
    if(m_radius<=minRadius)
      return;

    IndexList indices = m_indices;
    initialize(indices, minRadius, minPoints);
    return;
  }

  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->refineTree(minRadius,minPoints);
  }
}


template <class T>
bool
  OctreeT<T>::
  searchNearest(const Point &p, int& nearestNeighbor, Scalar &distance)
{
  bool bFound=false;

  // is this a leaf node?
  if (m_isLeaf) {
    // look for a new closest distance
    IndexListConstIterator vIt, vIt_end;
    for(vIt=m_indices.begin(), vIt_end=m_indices.end(); vIt!=vIt_end; ++vIt)
    {
      // check if distance to othis triangle is smaller
      if(closerToPoint(p,*vIt,distance))
        nearestNeighbor = *vIt;
    }
    return ball_within_bounds(p, sqrtf(distance), m_center, m_radius);
  }

  // which child contains p?
  int iChild = 4*(m_center[2]<p[2]) + 2*(m_center[1]<p[1]) + (m_center[0]<p[0]);

  // check that child first
  if (m_child[iChild] && m_child[iChild]->searchNearest(p, nearestNeighbor, distance)) return true;

  // now see if the other children need to be checked
  for (int i=0; i<8; i++) {
    if (i==iChild) continue;
    if (m_child[i] && bounds_overlap_ball(p,sqrtf(distance),m_child[i]->m_center, m_child[i]->m_radius)) {
      if (m_child[i]->searchNearest(p, nearestNeighbor, distance)) return true;
    }
  }
  return ball_within_bounds(p, sqrtf(distance), m_center, m_radius);
}

template <class T>
bool OctreeT<T>::
closerToPoint(const Point &p, const int& otherIndex, Scalar& distance)
{
  float distance_temp = dist2(p,m_points[otherIndex]);
  if (distance_temp < distance) {
    distance = distance_temp;
    return true;
  }
  else {
    return false;
  }
}

template <class T>
void OctreeT<T>::getPoints(PointList &p)
{
  if (m_isLeaf) {
    p.push_back(m_center + Point(-m_radius,-m_radius,-m_radius));//0
    p.push_back(m_center + Point( m_radius,-m_radius,-m_radius));//1
    p.push_back(m_center + Point(-m_radius, m_radius,-m_radius));//2
    p.push_back(m_center + Point( m_radius, m_radius,-m_radius));//3
    p.push_back(m_center + Point(-m_radius,-m_radius, m_radius));//4
    p.push_back(m_center + Point( m_radius,-m_radius, m_radius));//5
    p.push_back(m_center + Point(-m_radius, m_radius, m_radius));//6
    p.push_back(m_center + Point( m_radius, m_radius, m_radius));//7
    return;
  }
  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->getPoints(p);
  }

}

template <class T>
void OctreeT<T>::getCubes(PointList &p, std::vector<int> &edgeindices)
{

  if (m_isLeaf) {
    int s = p.size();
    p.push_back(m_center + Point(-m_radius,-m_radius,-m_radius));//0
    p.push_back(m_center + Point( m_radius,-m_radius,-m_radius));//1
    p.push_back(m_center + Point(-m_radius, m_radius,-m_radius));//2
    p.push_back(m_center + Point( m_radius, m_radius,-m_radius));//3
    p.push_back(m_center + Point(-m_radius,-m_radius, m_radius));//4
    p.push_back(m_center + Point( m_radius,-m_radius, m_radius));//5
    p.push_back(m_center + Point(-m_radius, m_radius, m_radius));//6
    p.push_back(m_center + Point( m_radius, m_radius, m_radius));//7

    edgeindices.push_back(s+0);edgeindices.push_back(s+1);
    edgeindices.push_back(s+0);edgeindices.push_back(s+4);
    edgeindices.push_back(s+1);edgeindices.push_back(s+3);
    edgeindices.push_back(s+2);edgeindices.push_back(s+0);
    edgeindices.push_back(s+3);edgeindices.push_back(s+2);
    edgeindices.push_back(s+4);edgeindices.push_back(s+6);
    edgeindices.push_back(s+6);edgeindices.push_back(s+2);
    edgeindices.push_back(s+6);edgeindices.push_back(s+7);
    edgeindices.push_back(s+7);edgeindices.push_back(s+3);
    edgeindices.push_back(s+7);edgeindices.push_back(s+5);
    edgeindices.push_back(s+5);edgeindices.push_back(s+1);
    edgeindices.push_back(s+5);edgeindices.push_back(s+4);
    return;
  }
  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->getCubes(p,edgeindices);
  }
}

template <class T>
void OctreeT<T>::getCubeCenters(PointList &centers)
{
  if (m_isLeaf) {
    centers.push_back(m_center);
    return;
  }
  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->getCubeCenters(centers);
  }
}


template <class T>
void OctreeT<T>::getIndices(IndexList& indices)
{
  if (m_isLeaf) {
    int nf = m_indices.size();
    for(int i=0;i<nf;++i)
      indices.push_back(m_indices[i]);
    return;
  }
  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->getIndices(indices);
  }
}


template <class T>
void OctreeT<T>::getLeaves(std::vector< OctreeT<T>* >& leafes)
{
  if (m_isLeaf) {
    leafes.push_back(this);
    return;
  }
  for (int i=0; i<8; i++) {
    if (m_child[i])
      m_child[i]->getLeaves(leafes);
  }

}

} // namespace vlr

