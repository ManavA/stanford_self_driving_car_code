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
 * $Id: octree.cpp 8082 2008-12-15 00:40:22Z veedee $
 *
 */

/** \author Matei Ciocarlie and Radu Bogdan Rusu */

#include <point_cloud_mapping/cloud_octree.h>
#include <string.h>
#include <cstdio>

namespace cloud_octree
{

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Constructor for Octree. Initalizes the root to a branch with all NULL children.
    * \param cx the center of the octree in space (X coordinate)
    * \param cy the center of the octree in space (Y coordinate)
    * \param cz the center of the octree in space (Z coordinate)
    * \param dx the dimensions of the octree along the X axis
    * \param dy the dimensions of the octree along the Y axis
    * \param dz the dimensions of the octree along the Z axis
    * \param max_depth the maximum depth of the Octree
    */
  Octree::Octree (float cx, float cy, float cz, float dx, float dy, float dz, int max_depth)
  {
    root_ = new Branch ();
    setCenter (cx, cy, cz);
    {
      //assert (max_depth >= 0);
    }
    if (max_depth == 0)
    {
      max_depth = 1;
      dx *= 2; dy *= 2; dz *= 2;
    }
    setSize (dx, dy, dz);
    setMaxDepth (max_depth);
    m_auto_expand_ = true;        // automatically expand the octree by default
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Serializes this octree to a string.
    * \note This only saves the inner structure of the tree, not center, extents, maxdepth etc.
    * \param destination the destination buffer
    * \param size the total size
    */
  void
    Octree::serialize (char **destination, unsigned int *size) const
  {
    int n_leaves   = root_->getNumLeaves ();
    int n_branches = root_->getNumBranches ();

    // Each branch stores a byte for each child
    // Each leaf stores its value
    *size = 8 * n_branches + sizeof (empty_indices_) * n_leaves;

    *destination = new char[*size];

    unsigned int address = 0;

    root_->serialize (*destination, address);
    // Sanity check
    if (address != *size)
      fprintf (stderr, "[Octree::serialize] Serialization error; unexpected size!\n");
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Reads in the content of this Octree. Returns \a true if the Octree is deserialized succesfully.
    * \note The old Octree content is deleted!
    * \note This is equivalent to calling \a clear(...) first and then \a deserialize(...)
    * \param source the source buffer
    * \param size - the total size of the string passed in. It is used for checking correctness and for avoiding memory
    * corruption.
    */
  bool
    Octree::deserialize (char *source, unsigned int size)
  {
    unsigned int address = 0;
    bool result = root_->deserialize (source, address, size);
    if (!result || address != size)
    {
      fprintf (stderr, "[Octree::deserialize] Octree deserialization error!\n");
      return (false);
    }
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Expand the Octree to contain the point at \a x,y,z. Works by adding new leaves above the current root.
    * \note It does not actually insert anyting at \a x,y,z. All the current structure is left unchanged, and the
    * smallest leaves will still have the same size. They will just be deeper down inside the tree.
    * \param x the x index of the leaf/cell
    * \param y the y index of the leaf/cell
    * \param z the z index of the leaf/cell
    */
  void
    Octree::expandTo (float x, float y, float z)
  {
    // While <x,y,z> is outside the tree bounds
    while (!testBounds (x, y, z))
    {
      unsigned char address = 0;
      // Compute the address of the current root in the list of the future root which will be added on top.
      // It is reversed from the address computation used in insert/erase(...)
      if (x < (m_cen_[0] - m_dim_[0] / 2.0)) { address +=4; m_cen_[0] -= m_dim_[0] / 2.0; }
      else                                   { m_cen_[0] += m_dim_[0] / 2.0; }

      if (y < (m_cen_[1] - m_dim_[1] / 2.0)) { address +=2; m_cen_[1] -= m_dim_[1] / 2.0; }
      else                                   { m_cen_[1] += m_dim_[1] / 2.0; }

      if (z < (m_cen_[2] - m_dim_[2] / 2.0)) { address +=1; m_cen_[2] -= m_dim_[2] / 2.0; }
      else                                   { m_cen_[2] += m_dim_[2] / 2.0; }

      // We have doubled the size
      m_dim_[0] *= 2.0; m_dim_[1] *= 2.0; m_dim_[2] *= 2.0;

      // Create the new root and set current root as child
      Branch* new_root = new Branch ();
      new_root->setChild (address, root_);

      // See if we can aggregate the old root
      Leaf* new_leaf;
      if (root_->aggregate (&new_leaf))
        // This also deletes the old root
        new_root->setChild (address, new_leaf);

      // Set the new root
      root_ = new_root;

      // We have increased the depth
      m_max_depth_++;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Returns the value at the given spatial coordinates. If that region of space is unvisited, returns
    * \a empty_indices_.
    * \param x the x index of the leaf/cell
    * \param y the y index of the leaf/cell
    * \param z the z index of the leaf/cell
    */
  std::vector<int>
    Octree::get (float x, float y, float z) const
  {
    if (!testBounds (x, y, z))
      return (empty_indices_);

    float cx = m_cen_[0], cy = m_cen_[1], cz = m_cen_[2];
    float dx = m_dim_[0] / 2.0, dy = m_dim_[1] / 2.0, dz = m_dim_[2] / 2.0;

    unsigned char address;
    Branch *currentNode = root_;
    Node *nextNode;

    while (1)           // Need to return within this loop
    {
      dx /= 2.0; dy /= 2.0; dz /= 2.0;

      address = 0;
      if ( x > cx) {address += 4; cx += dx;}
      else { cx -= dx;}
      if ( y > cy) {address += 2; cy += dy;}
      else {cy -= dy;}
      if ( z > cz) {address += 1; cz += dz;}
      else {cz -= dz;}

      nextNode = currentNode->getChild (address);
      if (!nextNode)
        return (empty_indices_);
      else if (nextNode->isLeaf ())
        return ((Leaf*)nextNode)->getIndices ();

      currentNode = (Branch*)nextNode;
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Insert a new value at the spatial location specified by \a x,y,z. If the given region of space is
    * previously unvisited, it will travel down creating Branches as it goes, until reaching a depth
    * of \a m_max_depth_. At that point it will create a Leaf with the value \a new_value.
    * On success, the method will navigate back towards the top of the tree, aggregating leaves
    * where appropriate. The method will also update the internal \a occupied_leaves_ set.
    * \note
      If the point at \a x,y,z is out of bounds: if the \a m_auto_expand_
      is true, the tree will expand (by adding new leaves on top of the
      current root) until it contains the point at \a x,y,z. It will
      then proceed with insertion normally. If the flag is false, this
      will just return without inserting anything.
    * \param x the x index of the leaf/cell
    * \param y the y index of the leaf/cell
    * \param z the z index of the leaf/cell
    * \param new_idx the point index to insert
    */
  void
    Octree::insert (float x, float y, float z, int new_idx)
  {
    // Check whether the cell exists
    if (!testBounds (x, y, z))
    {
      if (m_auto_expand_)
        expandTo (x, y, z);
      else
        return;
    }

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
      {
        // We have reached the max depth, create a new leaf
        if (depth >= m_max_depth_)
        {
          std::vector<int> indices (1);
          indices[0] = new_idx;
          next_node = new Leaf (indices);
          current_node->setChild (address, next_node);

          /*int num_cells = getNumCells ();
          int i, j, k;
          i = (int)(floor ((cx - m_cen_[0]) / num_cells) + (num_cells / 2));
          j = (int)(floor ((cy - m_cen_[1]) / num_cells) + (num_cells / 2));
          k = (int)(floor ((cz - m_cen_[2]) / num_cells) + (num_cells / 2));
          // Update the occupied_leaves_ list
          OctreeIndex leaf_index (i, j, k);
          occupied_leaves_.insert (leaf_index);
      //     std::cerr << "New Leaf at " << i << " " << j << " " << k << std::endl;
          for (std::set<OctreeIndex>::iterator it = occupied_leaves_.begin (); it != occupied_leaves_.end (); ++it)
          {
            OctreeIndex oi = *it;
            oi.getIndex (i, j, k);
      //       std::cerr << "Leaf at " << i << " " << j << " " << k << std::endl;
          }*/
          break;
        }
        else
        {
          // Create a new unexplored branch
          next_node = new Branch ();
          current_node->setChild (address, next_node);
        }
      }
      else if (next_node->isLeaf ())      // The node exists, and is a leaf
      {
        // We have reached the max depth, set the leaf to new value then done
        if (depth >= m_max_depth_)
        {
          ((Leaf*)next_node)->insertIndex (new_idx);
          break;
        }

        // Create a new branch with the all children leaves with the old value
        next_node = new Branch(((Leaf*)next_node)->getIndices ());
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
        // This will also delete the old branch
        visited_branches.back ()->replaceChild (current_node, new_leaf);
    }
    visited_branches.clear ();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Returns the list of occupied leaves in the tree. */
  std::vector<Leaf*>
    Octree::getOccupiedLeaves ()
  {
    int num_cells = getNumCells ();
    std::vector<Leaf*> leaves;

    std::list<SpatialNode*> stack;
    SpatialNode* sn = new SpatialNode;

    sn->node_ = root_;
    sn->cx_ = m_cen_[0]; sn->cy_ = m_cen_[1]; sn->cz_ = m_cen_[2];
    sn->dx_ = m_dim_[0]; sn->dy_ = m_dim_[2]; sn->dz_ = m_dim_[2];
    stack.push_front (sn);

    while (!stack.empty ())
    {
      sn = stack.front ();
      stack.pop_front ();

      // Check if we have a valid node
      if (sn->node_)
      {
        // Check if the node is a leaf
        if (sn->node_->isLeaf ())
        {
          ((Leaf*)sn->node_)->i_ = (int)(floor ((sn->cx_ - m_cen_[0]) / num_cells) + (num_cells / 2));
          ((Leaf*)sn->node_)->j_ = (int)(floor ((sn->cy_ - m_cen_[1]) / num_cells) + (num_cells / 2));
          ((Leaf*)sn->node_)->k_ = (int)(floor ((sn->cz_ - m_cen_[2]) / num_cells) + (num_cells / 2));
          ((Leaf*)sn->node_)->cen_[0] = sn->cx_;
          ((Leaf*)sn->node_)->cen_[1] = sn->cy_;
          ((Leaf*)sn->node_)->cen_[2] = sn->cz_;
          leaves.push_back ((Leaf*)sn->node_);
        }
        // Must be a branch then, get it's children
        else
          for (int i = 0; i < 8; i++)
            stack.push_front (((Branch*)sn->node_)->getSpatialChild (i, sn->cx_, sn->cy_, sn->cz_, sn->dx_, sn->dy_, sn->dz_));
      }
      delete sn;
    }

    // Cleanup
    while (!stack.empty ())
    {
      delete stack.front ();
      stack.pop_front ();
    }
    return (leaves);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Returns the list of occupied leaves in the tree that have a minimum imposed number of points
    * \param min_points the minimum number of points that a leaf must contain in order to be selected
    */
  std::vector<Leaf*>
    Octree::getMinOccupiedLeaves (unsigned int min_points)
  {
    std::vector<Leaf*> leaves;

    std::list<SpatialNode*> stack;
    SpatialNode* sn = new SpatialNode;

    sn->node_ = root_;
    sn->cx_ = m_cen_[0]; sn->cy_ = m_cen_[1]; sn->cz_ = m_cen_[2];
    sn->dx_ = m_dim_[0]; sn->dy_ = m_dim_[2]; sn->dz_ = m_dim_[2];
    stack.push_front (sn);

    while (!stack.empty ())
    {
      sn = stack.front ();
      stack.pop_front ();

      // Check if we have a valid node
      if (sn->node_)
      {
        // Check if the node is a leaf
        if (sn->node_->isLeaf ())
        {
          if (((Leaf*)sn->node_)->getIndices ().size () >= min_points)
            leaves.push_back ((Leaf*)sn->node_);
        }
        // Must be a branch then, get it's children
        else
          for (int i = 0; i < 8; i++)
            stack.push_front (((Branch*)sn->node_)->getSpatialChild (i, sn->cx_, sn->cy_, sn->cz_, sn->dx_, sn->dy_, sn->dz_));
      }
      delete sn;
    }

    // Cleanup
    while (!stack.empty ())
    {
      delete stack.front ();
      stack.pop_front ();
    }
    return (leaves);
  }

}
