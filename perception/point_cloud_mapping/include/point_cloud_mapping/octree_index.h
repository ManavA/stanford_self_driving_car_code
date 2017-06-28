/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 * $Id: octree_index.h 16379 2009-05-29 19:20:46Z hsujohnhsu $
 *
 */

/** \author Radu Bogdan Rusu */

#ifndef _CLOUD_OCTREE_OCTREEINDEX_H_
#define _CLOUD_OCTREE_OCTREEINDEX_H_

#include <vector>

namespace cloud_octree
{

  /** \brief Octree index class. Stores an internal i-j-k index to a leaf represented by an unsigned long int (32 bits). */
  class OctreeIndex
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Default empty OctreeIndex constructor */
      OctreeIndex () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief OctreeIndex constructor
        * \param i the first leaf coordinate
        * \param j the second leaf coordinate
        * \param k the third leaf coordinate
        */
      OctreeIndex (int i, int j, int k)
      {
        setIndex (i, j, k);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return the i-j-k leaf coordinates in the octree for this Index object
        * \param i the first leaf coordinate
        * \param j the second leaf coordinate
        * \param k the third leaf coordinate
        */
      inline void
        getIndex (int &i, int &j, int &k)
      {
        unsigned long int lijk = ijk_;
        i = (lijk & 0x3ff);     // Unmask I
        lijk >>= 10;            // Shift
        j = (lijk & 0x3ff);     // Unmask J
        lijk >>= 10;            // Shift
        k = (lijk & 0x3ff);     // Unmask K
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the internal index representation for a given i-j-k leaf coordinate in the octree
        * \param i the first leaf coordinate
        * \param j the second leaf coordinate
        * \param k the third leaf coordinate
        */
      inline void
        setIndex (int i, int j, int k)
      {
        ijk_ = (k & 0x3ff);     // Mask K
        ijk_ <<= 10;            // Shift
        ijk_ += (j & 0x3ff);    // Mask J
        ijk_ <<= 10;            // Shift
        ijk_ += (i & 0x3ff);    // Mask I
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Overloaded < operator */
      bool
        operator < (OctreeIndex const & o) const
      {
        return (this->ijk_ < o.ijk_);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Overloaded > operator */
      bool
        operator > (OctreeIndex const & o) const
      {
        return (this->ijk_ > o.ijk_);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Overloaded == operator */
      bool
        operator == (OctreeIndex const & o) const
      {
        return (this->ijk_ == o.ijk_);
      }

      /** \brief Internal index stored as Z | Y | X */
      unsigned long int ijk_;
  };

}

#endif
