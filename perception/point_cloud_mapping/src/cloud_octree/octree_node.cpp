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
 * $Id: octree_node.cpp 8082 2008-12-15 00:40:22Z veedee $
 *
 */

/** \author Matei Ciocarlie and Radu Bogdan Rusu */

#include <point_cloud_mapping/octree_node.h>
#include <string.h>
#include <vector>

namespace cloud_octree
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Serializes the content of this leaf.
    * \param destination the destination buffer
    * \param address the address
    */
  void
    Leaf::serialize (char *destination, unsigned int &address) const
  {
    memcpy (&destination[address], &indices_, sizeof (indices_));
    address += sizeof (indices_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Deserialize and read in the contents of this leaf.
    * \param source the source buffer
    * \param address the address
    * \param size the total size
    */
  bool
    Leaf::deserialize (char *source, unsigned int &address, unsigned int size)
  {
    if (address + sizeof (indices_) > size)
    {
      address = size;
      return (false);
    }
    memcpy (&indices_, &source[address], sizeof (indices_));
    address += sizeof (indices_);
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recursively serializes everything below this branch.
    * \param destination the destination buffer
    * \param address the address
    */
  void
    Branch::serialize (char *destination, unsigned int &address) const
  {
    for (int i = 0; i < 8; i++)
    {
      if (!m_children_[i])
      {
        destination[address] = child_type::NULL_CHILD;
        address++;
        continue;
      }
      if (m_children_[i]->isLeaf ())
        destination[address] = child_type::LEAF;
      else
        destination[address] = child_type::BRANCH;
      address++;
      m_children_[i]->serialize (destination, address);
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recursively reads in everything below this branch.
    * \param source the source buffer
    * \param address the address
    * \param size the total size
    */
  bool
    Branch::deserialize (char *source, unsigned int &address, unsigned int size)
  {
    for (int i = 0; i < 8; i++)
    {
      if (address >= size) return (false);
      if (source[address] == child_type::NULL_CHILD)
      {
        setChild (i, NULL);
        address++;
        continue;
      }
      if (source[address] == child_type::LEAF)
        setChild (i, new Leaf ());
      else if (source[address] == child_type::BRANCH)
        setChild (i, new Branch () );
      else                // Error
      {
        address = size;
        return (false);
      }
      address++;
      if (!m_children_[i]->deserialize (source, address, size))
      {                   // Error
        address = size;
        return (false);
      }
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Check if all children of this branch are identical. If true, place in \a new_leaf a pointer to a new leaf
    * that can replace this branch.
    * \param new_leaf pointer to a leaf that can replace the branch
    */
  bool
    Branch::aggregate (Leaf **new_leaf) const
  {
    if (!m_children_[0])
    {
      // First child is NULL
      for (int i = 1; i < 8; i++)
        // Some children are NULL, some not. We are done
        if (m_children_[i])
          return (false);

      // All children are NULL
      *new_leaf = NULL;
      return (true);
    }

    // If any child is a branch, return false
    if (!m_children_[0]->isLeaf ())
      return (false);

    std::vector<int> indices = ((Leaf*)(m_children_[0]))->getIndices ();

    for (int i = 1; i < 8; i++)
    {
      // Some children are NULL, some not. We are done
      if (!m_children_[i])
        return (false);
      // If any child is a branch, return false
      if (!m_children_[i]->isLeaf ())
        return (false);

      // Two children leaves have different values
//      if ( ((Leaf*)(m_children_[i]))->getValue () != indices)
//        return (false);
    }

    // All children are leaves and they have the same value
    *new_leaf = new Leaf (indices);
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recursively returns the number of leaves. */
  int
    Branch::getNumLeaves () const
  {
    int n = 0;
    for (int i = 0; i < 8; i++)
    {
      if (!m_children_[i])
        continue;
      if (m_children_[i]->isLeaf ())
        n += 1;
      else
        n += ((Branch*)m_children_[i])->getNumLeaves ();
    }
    return (n);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Recursively returns the total number of branches below this one (including this one). */
  int
    Branch::getNumBranches() const
  {
    int n = 1;
    for (int i = 0; i < 8; i++)
      if (m_children_[i] && !m_children_[i]->isLeaf ())
        n += ((Branch*)m_children_[i])->getNumBranches ();
    return (n);
  }
}
