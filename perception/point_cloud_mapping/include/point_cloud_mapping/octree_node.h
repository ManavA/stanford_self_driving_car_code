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
 * $Id: octree_node.h 16379 2009-05-29 19:20:46Z hsujohnhsu $
 *
 */

/** \author Matei Ciocarlie and Radu Bogdan Rusu */

#ifndef _CLOUD_OCTREE_OCTREENODE_H_
#define _CLOUD_OCTREE_OCTREENODE_H_

#include <vector>

namespace cloud_octree
{

  /** \brief Constants that need to be chars to save space */
  namespace child_type
  {
    const char NULL_CHILD = 0;
    const char BRANCH     = 1;
    const char LEAF       = 2;
  }


  /** \brief  An Octree node (leaf or branch). We could define this privately inside the Octree class, but in
    * the future we might implement faster accessors that return the node that they accessed, so that subsequent calls
    * can use this information.
    */
  class Node
  {
    public:
      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructor for base Node. */
      Node () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for base Node. */
      virtual ~Node () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return true if the underlying Node is a Leaf. */
      virtual bool isLeaf () const = 0;

      virtual void serialize (char*, unsigned int&) const = 0;
      virtual bool deserialize (char*, unsigned int&, unsigned int)  = 0;

      virtual int computeMaxDepth () const { return (0); }
  };


  /** \brief Spatial Node index class. Holds a pointer to a node as well as
    * the spatial coordinates that the node is responsible for. It is
    * designed to facilitate navigation through the Octree without using
    * recursion.
    */
  class SpatialNode
  {
    public:
      Node *node_;
      float cx_, cy_, cz_;
      float dx_, dy_, dz_;
  };


  /** \brief A leaf holds a vector of point indices. */
  class Leaf : public Node
  {
    private:
      std::vector<int> indices_;

    public:

      float cen_[3];
      int i_, j_, k_;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Constructors for Leaf. */
      Leaf () { }
      Leaf (std::vector<int> indices) { setIndices (indices); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for Leaf. */
      virtual ~Leaf () { }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Get the point indices stored in this leaf. */
      inline std::vector<int> getIndices () { return (indices_); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Set the underlying point indices.
        * \param indices the point indices to set
        */
      inline void setIndices (std::vector<int> indices) { indices_ = indices; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Insert a new point index in the leaf
        * \param index the point index to insert
        */
      inline void insertIndex (int index) { indices_.push_back (index); }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      bool isLeaf () const { return (true); }

      virtual void serialize (char *destination, unsigned int &address) const;
      virtual bool deserialize (char *source, unsigned int &address, unsigned int size);
  };

  /** \brief An Octree branch. Always contains exactly 8 children pointers. A NULL child pointer means that the
    * respective child points to an unexplored region of space and thus is equivalent to having a child with an
    * empty index vector in the Octree.
   */
  class Branch : public Node
  {
    public:

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Branch constructor. Initializes a branch with all NULL (unexplored) children. */
      Branch ()
      {
        m_children_ = new Node*[8];
        for (int i = 0; i < 8; i++)
          m_children_[i] = NULL;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Branch constructor. Initializes a branch with all its children set to \a indices */
      Branch (std::vector<int> indices)
      {
        m_children_ = new Node*[8];
        for (int i = 0; i < 8; i++)
          m_children_[i] = new Leaf (indices);
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Destructor for Branch. Delete all children first.
        * \note to delete an entire octree, delete its root.
        */
      virtual ~Branch ()
      {
        for (int i = 0; i < 8; i++)
          if (m_children_[i])
            delete m_children_[i];
        delete m_children_;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      inline bool isLeaf () const { return (false); }

      virtual void serialize (char *destination, unsigned int &address) const;
      virtual bool deserialize (char *source, unsigned int &address, unsigned int size);

      bool aggregate (Leaf **new_leaf) const;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Sets the child at address \a adress to point at \a child.
        * \note If a child was already present at that address it is deleted.
        * \param address the address
        * \param child the child
        */
      inline void
        setChild (unsigned char address, Node *child)
      {
        if (m_children_[address])
          delete (m_children_[address]);
        m_children_[address] = child;
      }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return the child at address \a adress (between 0 and 7) */
      inline Node* getChild (unsigned char address) { return m_children_[address]; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Return the child at address \a adress (between 0 and 7). Const version. */
      inline const Node* getChild (unsigned char address) const { return m_children_[address]; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Replaces \a old_child with \a new_child. \a old_child is deleted.
        * \param old_child the old child
        * \param new_child the new child
        */
      inline void
        replaceChild (Node *old_child, Node *new_child)
      {
        for (int i = 0; i < 8; i++)
        {
          if (m_children_[i] == old_child)
          {
            setChild (i, new_child);
            return;
          }
        }
      }

      int getNumLeaves () const;
      int getNumBranches() const;

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Returns a structure that contains both a pointer to the child and its spatial coordinates
        * \param address the requested node address
        * \param cx, cy, cz, dx, dy, dz the node's spatial coordinates
        */
      inline SpatialNode*
        getSpatialChild (int address, float cx, float cy, float cz, float dx, float dy, float dz)
      {
        SpatialNode *sn = new SpatialNode;
        sn->node_ = getChild (address);

        sn->dx_ = dx / 2.0;
        sn->dy_ = dy / 2.0;
        sn->dz_ = dz / 2.0;

        if (address / 4 == 0)
          sn->cx_ = cx - sn->dx_ / 2.0;
        else
          sn->cx_ = cx + sn->dx_ / 2.0;

        if ((address % 4) / 2 == 0)
          sn->cy_ = cy - sn->dy_ / 2.0;
        else
          sn->cy_ = cy + sn->dy_ / 2.0;
        if ((address % 4) % 2 == 0)
          sn->cz_ = cz - sn->dz_ / 2.0;
        else
          sn->cz_ = cz + sn->dz_ / 2.0;

        return (sn);
      }

    private:
      /** \brief Placeholder for the 8 childrens of the branch. */
      Node **m_children_;
  };

}

#endif
