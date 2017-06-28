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
 * $Id: cloud_octree.h 16379 2009-05-29 19:20:46Z hsujohnhsu $
 *
 */

/** \author Matei Ciocarlie and Radu Bogdan Rusu */

#ifndef _CLOUD_OCTREE_OCTREE_H_
#define _CLOUD_OCTREE_OCTREE_H_

#include <iostream>
#include <list>
#include <set>
#include <cmath>
#include <point_cloud_mapping/octree_node.h>
#include <point_cloud_mapping/octree_index.h>
// #include <scan_utils/OctreeMsg.h>

namespace cloud_octree
{
  /** \brief Main octree class */
  class Octree
  {
    public:
      Octree (float cx, float cy, float cz, float dx, float dy, float dz, int max_depth);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for Octree. Recursively deletes the tree by deleting the root. */
      virtual ~Octree () { delete root_; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Clears and deallocates the entire Octree. */
      inline void
        clear ()
      {
        delete root_;
        root_ = new Branch ();
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Enable/disable the octree auto-expand capability
        * \param val true to expand, false otherwise
        */
      inline void setAutoExpand (bool val) { m_auto_expand_ = val; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Sets the max depth of this Octree. Does NOT change the inner data. Smallest accepted value is 1. */
      inline void
        setMaxDepth (int d)
      {
        if (d <= 0)
          d = 1;
        m_max_depth_ = d;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns the max. depth of the Octree. */
      inline int getMaxDepth () const { return (m_max_depth_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns the maximum number of cells along one dimension of this Octree. Equals 2^(mMaxDepth). 
        * Overall, the Octree might contain up to getNumCells()^3 cells.
        */
      inline int getNumCells () const { return (1 << m_max_depth_); }

      std::vector<Leaf*> getOccupiedLeaves ();
      std::vector<Leaf*> getMinOccupiedLeaves (unsigned int min_points);


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Sets the center of this Octree. Does NOT change the inner data.
        * \param cx the x coordinate of the center
        * \param cy the y coordinate of the center
        * \param cz the z coordinate of the center
        */
      inline void
        setCenter (float cx, float cy, float cz)
      {
        m_cen_[0] = cx; m_cen_[1] = cy; m_cen_[2] = cz;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Sets the size of this Octree. Does NOT change the inner data.
        * \note Smallest cell is then 2^(-maxdepth) * total_octree_size
        * \param dx the size over the x coordinate
        * \param dy the size over the y coordinate
        * \param dz the size over the z coordinate
        */
      inline void
        setSize (float dx, float dy, float dz)
      {
        m_dim_[0] = dx; m_dim_[1] = dy; m_dim_[2] = dz;
      }

      void serialize (char **destination, unsigned int *size) const;
      bool deserialize (char *source, unsigned int size);


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Converts cell indices to spatial coordinates. See main class description for details.
        * \param i the i index of the leaf/cell
        * \param j the j index of the leaf/cell
        * \param k the k index of the leaf/cell
        * \param x the x coordinate of the point
        * \param y the y coordinate of the point
        * \param z the z coordinate of the point
        */
      inline bool
        cellToCoordinates (int i, int j, int k, float *x, float *y, float *z) const
      {
        int num_cells = getNumCells ();
        *x = m_cen_[0] - (m_dim_[0] / 2.0) + (m_dim_[0] / num_cells) * (i + 0.5);
        *y = m_cen_[1] - (m_dim_[1] / 2.0) + (m_dim_[1] / num_cells) * (j + 0.5);
        *z = m_cen_[2] - (m_dim_[2] / 2.0) + (m_dim_[2] / num_cells) * (k + 0.5);
        if (i < 0 || j < 0 || k < 0)
          return (false);
        if (i >= num_cells || j >= num_cells || k >= num_cells)
          return (false);
        return (true);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Converts spatial coordinates to cell indices. See main class description for details.
        * \param x the x coordinate of the point
        * \param y the y coordinate of the point
        * \param z the z coordinate of the point
        * \param i the i index of the leaf/cell
        * \param j the j index of the leaf/cell
        * \param k the k index of the leaf/cell
        */
      inline bool
        coordinatesToCell (float x, float y, float z, int *i, int *j, int *k) const
      {
        int num_cells = getNumCells ();
        *i = (int)(floor ((x - m_cen_[0]) / num_cells) + (num_cells / 2));
        *j = (int)(floor ((y - m_cen_[1]) / num_cells) + (num_cells / 2));
        *k = (int)(floor ((z - m_cen_[2]) / num_cells) + (num_cells / 2));
        if (!testBounds (x, y, z))
          return (false);
        return (true);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns \a true if the point at x,y,z is inside the volume of this Octree.
        * \param x the x coordinate of the point to test
        * \param y the y coordinate of the point to test
        * \param z the z coordinate of the point to test
        */
      inline bool
        testBounds (float x, float y, float z) const
      {
        float dx = m_dim_[0] / 2.0;
        float dy = m_dim_[1] / 2.0;
        float dz = m_dim_[2] / 2.0;
        if (x > m_cen_[0] + dx) return (false); if (x < m_cen_[0] - dx) return (false);
        if (y > m_cen_[1] + dy) return (false); if (y < m_cen_[1] - dy) return (false);
        if (z > m_cen_[2] + dz) return (false); if (z < m_cen_[2] - dz) return (false);
        return (true);
      }

      std::vector<int> get (float x, float y, float z) const;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Compute the 3D bounds (min, max) of a leaf/cell
        * \param leaf the leaf to compute bounds for
        * \param bounds the resultant bounds
        */
      inline void
        computeCellBounds (Leaf* leaf, std::vector<double> &bounds)
      {
        int num_cells = getNumCells ();
        bounds[0] = leaf->cen_[0] - (m_dim_[0] / num_cells) / 2;
        bounds[1] = leaf->cen_[1] - (m_dim_[0] / num_cells) / 2;
        bounds[2] = leaf->cen_[2] - (m_dim_[0] / num_cells) / 2;
        bounds[3] = leaf->cen_[0] + (m_dim_[0] / num_cells) / 2;
        bounds[4] = leaf->cen_[1] + (m_dim_[0] / num_cells) / 2;
        bounds[5] = leaf->cen_[2] + (m_dim_[0] / num_cells) / 2;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Delete a value stored at the lowermost leaf \a x,y,z. Any subsequent query will then return
        * \a empty_indices_. On success, the function will navigate back towards the top of the tree, aggregating leaves
        * where appropriate.
        * \param x the x index of the leaf/cell
        * \param y the y index of the leaf/cell
        * \param z the z index of the leaf/cell
        * \note need to test this.
        */
      void
        erase (float x, float y, float z)
      {
        // Check whether the cell exists
        if (!testBounds (x, y, z))
          return;

        // Cell exists, continue
        float cx = m_cen_[0], cy = m_cen_[1], cz = m_cen_[2];
        float dx = m_dim_[0] / 2.0, dy = m_dim_[1] / 2.0, dz = m_dim_[2] / 2.0;

        int depth = 0;
        unsigned char address;
        Branch *current_node = root_;
        Node *next_node;

        std::list<Branch*> visited_branches;

        while (1)
        {
          dx /= 2.0;
          dy /= 2.0;
          dz /= 2.0;
          depth++;

          address = 0;
          if ( x > cx ) { address += 4; cx += dx; }
          else          { cx -= dx; }
          if ( y > cy ) { address += 2; cy += dy; }
          else          { cy -= dy; }
          if ( z > cz ) { address += 1; cz += dz; }
          else          { cz -= dz; }

          // The current node is a branch by definition
          visited_branches.push_back (current_node);
          next_node = current_node->getChild (address);

          // Unexplored region of space
          if (!next_node)
            break;
          else if (next_node->isLeaf ())
          {
            // We have reached the max depth
            if (depth >= m_max_depth_)
            {
              current_node->setChild (address, NULL);
              break;
            }

            // Create a new branch with the all children leaves with the old value
            next_node = new Branch (((Leaf*)next_node)->getIndices ());
            current_node->setChild (address, next_node);
          }

          // Advance the recursion
          current_node = (Branch*)next_node;
        }

        // Go back through the list of visited nodes and check if we need to aggregate
        bool agg = true;
        Leaf *new_leaf;
        while (agg && (int)visited_branches.size () > 1)
        {
          current_node = visited_branches.back ();
          visited_branches.pop_back ();
          agg = current_node->aggregate (&new_leaf);
          if (agg)
            // Delete the old branch
            visited_branches.back ()->replaceChild (current_node, new_leaf);
        }
        visited_branches.clear ();
      }

      void insert (float x, float y, float z, int new_idx);
      void expandTo (float x, float y, float z);

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Recursively computes and returns the number of branches of the tree */
      inline int getNumBranches () const { return (root_->getNumBranches ()); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Recursively computes and returns the number of leaves of the tree */
      inline int getNumLeaves () const { return (root_->getNumLeaves ()); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns the space in memory occupied by the tree */
      inline long long unsigned int getMemorySize() const
      {
        unsigned int nr_leaves = root_->getNumLeaves ();
        unsigned int nr_branches = root_->getNumBranches();
        return (nr_leaves * sizeof (Leaf) + nr_branches * sizeof (Branch));
      }


      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns the value at given cell coordinates
        * \param i the first cell coordinate
        * \param j the second cell coordinate
        * \param k the third cell coordinate
        */
      inline std::vector<int>
        cellGet (int i, int j, int k) const
      {
        float x, y, z;
        if (!cellToCoordinates (i, j, k, &x, &y, &z))
          return (empty_indices_);
        return (get (x, y, z));
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Inserts a point index at given cell coordinates.
        * \param i the first cell coordinate
        * \param j the second cell coordinate
        * \param k the third cell coordinate
        * \param new_idx the point index to insert
        */
      inline void
        cellInsert (int i, int j, int k, int new_idx)
      {
        float x, y, z;
        if (!cellToCoordinates (i, j, k, &x, &y, &z))
          return;
        insert (x, y, z, new_idx);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Deletes data at given cell coordinates
        * \param i the first cell coordinate
        * \param j the second cell coordinate
        * \param k the third cell coordinate
        */
      inline void
        cellErase (int i, int j, int k)
      {
        float x, y, z;
        if (!cellToCoordinates (i, j, k, &x, &y, &z))
          return;
        erase (x, y, z);
      }

    private:
      /** \brief The root of the Octree - always a branch and never NULL. */
      Branch *root_;
      /** \brief The value that is returned for a never visited leaf. */
      const std::vector<int> empty_indices_;

      /** \brief The max depth of the octree, minimum 1 (root and 8 leaves).
        * The smallest leaf is guaranteed to have size of 2^( - max_depth) * total_octree_size
        */
      int m_max_depth_;

      /** \brief The total size of the Octree. Can be thought of as the dimensions of the root branch. */
      float m_dim_[3];
      /** \brief The location of the center of the octree. */
      float m_cen_[3];

      /** \brief Expand Octree automatically if data to be inserted is out of bounds */
      bool m_auto_expand_;
  };
}

#endif
