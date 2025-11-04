/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file DigitalSetByOctree.h
 * @author Bastien DOIGNIES 
 * LIRIS 
 *
 * @date 2025/09/05
 *
 * Header file for module DigitalSetByOctree.cpp
 *
 * This file is part of the DGtal library.
 */

#pragma once

// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

//#include "DGtal/io/Display3D.h"


namespace DGtal
{
  /**
   * @brief A DigitalSet that stores voxel as an octree, or a DAG
   * 
   * This class allows for a compact representation of voxel as a
   * sparse voxel octree (SVO). Optionally, this can be further
   * compressed as a Sparse Voxel Directed Acyclic Graph where 
   * nodes are shared when they share a common structure.
   * 
   * Leaves are never explicitly stored to further reduce memory
   * usage. 
   * 
   * Common operation complexity:
   * (L = depth of the tree computed with domain size, N = number of voxels)
   *  - Insertion: O(log(L))
   *  - Erase: O(N * log(L))
   *  - find: O(log(L))
   *  - Iterator next: O(log(L))
   *  - Listing all voxels: O(N * log(L))
   *  - Memory usage (octree): O(N * log(L))
   * 
   * When converted to a DAG, the digital set can not be modified anymore
   * (neither insertion, or erase).
   *
   * References:
   *  @cite Laine2010SVO
   *  @cite Kampe2013SVDag
   */
  template <class Space>
    class DigitalSetByOctree 
    {
      public:
        template<class>
        friend class SVOWriter;
        template<class>
        friend class SVOReader;

        using DimIndex = std::uint8_t;
        using CellIndex = typename Space::UnsignedInteger;
        using Size = size_t;

        // Only HyperRectDomain are supported 
        using Domain = HyperRectDomain<Space>;
        using Point  = typename Domain::Point;
        
        // Define a value type for this class to be somewhat compatible
        // with image-based code. 
        using Value = void;

        static constexpr DGtal::Dimension D = Space::dimension;
        static constexpr DGtal::Dimension dimension = Space::dimension;
        
        static constexpr CellIndex CELL_COUNT  = (1 << D);
        static constexpr CellIndex INVALID_IDX = std::numeric_limits<CellIndex>::max();

        // For each child node, store on which side of the octree split it is.
        static constexpr std::array<std::array<DimIndex, D>, CELL_COUNT> SIDES_FROM_INDEX = []()
        {
         std::array<std::array<DimIndex, D>, CELL_COUNT> sides_from_index{};
         for (CellIndex i = 0; i < CELL_COUNT; ++i) 
         {
           CellIndex coord = i;
           for (DimIndex d = 0; d < D; ++d) 
           {
             sides_from_index[i][d] = coord % 2;
             coord /= 2;
           }
         }

         return sides_from_index;
        }();

        /** 
         * @brief Node for octree
         * 
         * Only stores the index of its children
         */
        struct Node 
        {
          Node() 
          {
            for (CellIndex i = 0; i < CELL_COUNT; ++i)
              children[i] = INVALID_IDX;
          }

          CellIndex children[CELL_COUNT];
        };
        
        /**
         * @brief Helper struct for computing local estimators
         *
         * It serves as a cache key to identify a neighborhood:
         *  - The parent that encloses the whole neighborhood
         *  - The neighborhood code
         */
        struct ComputationCacheKey 
        {
          CellIndex parentLvl; 
          CellIndex parentIdx;
          std::vector<DimIndex> code;

          bool operator<(const ComputationCacheKey& other) const 
          {
            if (parentLvl != other.parentLvl) return parentLvl < other.parentLvl;
            if (parentIdx != other.parentIdx) return parentIdx < other.parentIdx;
            return code < other.code;
          }
        };

        /**
         * @brief Helper struct to store traversal and go to next leaf
         */
        struct TraversalMemory 
        {
          Domain domain; //< Domain represented by the node
          CellIndex lvl; //< Level of the node
          CellIndex idx; //< Index of the node 
          CellIndex currentChildIdx; //< Which child is currently explored.

          bool operator==(const TraversalMemory& other) const 
          {
            return lvl == other.lvl && idx == other.idx && currentChildIdx == other.currentChildIdx;
          }
        };

        /**
         * @brief Iterator over the octree
         */
        struct OctreeIterator 
        {
          public:
            friend class DigitalSetByOctree;

            // Should be std::output_iterator_tag, but CDigitalSet
            // does not allow this; so we upgrade the category
            using iterator_category = std::forward_iterator_tag;
            using difference_type = std::ptrdiff_t;
            using value_type = Point;
            using reference = value_type&;
            using pointer = value_type*;

            /** 
             * @brief Constuctor to end of an octree
             */
            OctreeIterator(const DigitalSetByOctree* container) 
            {
              myContainer = container;
            }

            /**
             * @brief Constructor from any node to explore subtree
             * 
             * The main purpose of this constructor is to pass the 
             * root node.
             */
            OctreeIterator(const DigitalSetByOctree* container, 
                           TraversalMemory init) 
            {
              myContainer = container;
              myMemory.push_back(std::move(init));

              findNextLeaf();
            }

            /** 
             * @brief Constructor from an entire traversal
             * 
             * The main purpose of this constructor is for the 
             * find method to directly build the iterator without
             * searching through the whole tree.
             */
            OctreeIterator(const DigitalSetByOctree* container, 
                           std::vector<TraversalMemory>& memory) 
            {
              myContainer = container;
              myMemory = memory;
            }

            /**
             * @brief Compares two iterator
             * 
             * Note: The end of an octree is represented by 
             * an empty traversal memory
             */
            bool operator==(const OctreeIterator& other) const 
            {
              if (myContainer != other.myContainer) return false;
              if (myMemory.size() != other.myMemory.size()) return false;

              if (myMemory.size() != 0) 
              {
                return myMemory.back() == other.myMemory.back();
              }
              return true;
            }

            /**
             * @brief Not equal comparison operator
             */
            bool operator!=(const OctreeIterator& other) const 
            {
              return !(*this == other);
            }

            /**
             * @brief Dereference operator
             */
            Point operator*() const 
            {
              const auto& sides = SIDES_FROM_INDEX[myMemory.back().currentChildIdx];
              return splitDomain(myMemory.back().domain, sides.data()).lowerBound();
            }

            /**
             * @brief Prefix increment
             */
            OctreeIterator& operator++() 
            {
              findNextLeaf();
              return *this;
            }

            /**
             * @brief Postfix increment
             */
            OctreeIterator operator++(int) 
            {
              auto it = *this;
              findNextLeaf();
              return it;
            }
          private:
            /**
             * @brief Finds the next leaf, if any
             */
            void findNextLeaf();
          private:
            const DigitalSetByOctree* myContainer; //< Pointer to the original octree
            std::vector<TraversalMemory> myMemory;  //< Current traversal information
        };

        using Iterator = OctreeIterator;
        using ConstIterator = OctreeIterator;

        /**
         * @brief Constructor from a domain
         * 
         * Only HyperRectDomains are supported for octrees.
         * 
         * The given domain will be extended to ensure
         * it is an hyper cube with sides that is a power of 2
         * 
         * @param d The domain 
         */
        DigitalSetByOctree(const Domain& d);

        /**
         * @brief Returns the domain of the digital set
         */
        const Domain& domain() const { return *myAdmissibleDomain; }

        /**
         * @brief Returns the domain of the voxels
         */
        CowPtr<Domain> domainPointer() const { return myAdmissibleDomain; }
      public:
        /**
         * @brief Inserts a new point in the octree
         * 
         * This function can be called multiple times with the same
         * point without any risk.
         * 
         * If the point is not in bounds or the octree has been 
         * converted to a DAG, this function does nothing.
         * 
         * @param p The point to insert
         */
        void insert(const Point& p);

        /**
         * @brief Inserts a new point in the octree
         * 
         * This function can be called multiple times with the same
         * point without any risk.
         * 
         * If the point is not in bounds or the octree has been 
         * converted to a DAG, this function does nothing.
         * 
         * @param p The point to insert
         * @see insert
         */
        void insertNew(const Point& p) { insert(p); }

        /**
         * @brief Check if a voxel is set or not
         * 
         * @param p The point to check for
         * @see find
         */
        bool operator()(const Point& p) const { return find(p) != end(); }

        /**
         * @brief Finds a point within the Octree
         * 
         * @param p The point to find
         * @return An iterator to the point if possible otherwise end()
         */
        Iterator find(const Point& p) const;

        /** 
         * @brief Returns the number of voxel in the set
         */
        size_t size() const { return mySize; }

        /** 
         * @brief Check if the octree is empty or not
         */
        bool empty() const { return size() == 0; }

        /**
         * @brief Returns the memory occupied by node storage in bytes
         * 
         * @see shrink
         */
        size_t memoryFootprint() const 
        {
          size_t count = 0;
          for (CellIndex i = 0; i < myNodes.size(); ++i) 
          {
            count += myNodes[i].capacity() * sizeof(Node);
          }
          return count;
        }

        /**
         * @brief Removes all nodes from the octree
         * 
         * Note: if the octree has been converted to a dag, 
         * this function reset the flag and insertion 
         * is available afterwards. 
         */
        void clear() 
        {
          myNodes.clear();
          myState = State::OCTREE;
        }

        /** 
         * @brief Remove a voxel from the octree
         * 
         * This functions is undefined behavior if the Iterator
         * is invalid.
         * 
         * @param it The iterator pointing to the voxel to remove
         */
        size_t erase(const Iterator& it);

        /** 
         * @brief Removes a range of voxels
         * 
         * @param begin The begining of the range
         * @param end The end of the range
         */
        size_t erase(Iterator begin, Iterator end) 
        {
          size_t count = 0;
          for (; begin != end; ++begin) 
          {
            count += erase(begin);
          }
          return count;
        }

        /**
         * @brief Removes a voxel from the octree
         * 
         * @param p The point to remove
         */
        size_t erase(const Point& p) 
        {
          return erase(find(p));
        }

        /**
         * @brief Shrinks storage to reduce memory usage
         * 
         * This function removes extra capacity added for 
         * faster insertion with vector and can free some
         * memory once insertion phase is over.
         *
         * This function is called automatically by convertToDAG.
         *
         * @see memoryFootprint
         */
        void shrink_to_fit() 
        {
          for (CellIndex i = 0; i < myNodes.size(); ++i) 
          {
            myNodes[i].shrink_to_fit();
          }
        }

        /**
         * @brief Appends an octree to another
         * 
         * This is equivalent to looping through an octree and inserting
         * every node into another
         * 
         * @param other The octree to append
         */
        DigitalSetByOctree& operator+=(const DigitalSetByOctree& other) 
        {
          if (myState == State::OCTREE) 
          {
            for (auto it = other.begin(); it != other.end(); ++it) 
            {
              insert(*it);
            }
          }
          return *this;
        }

        /**
         * @brief Computes the complement of the octree
         * 
         * This is equivalent to looping through an octree and inserting
         * 
         * @param out The output iterator
         */
        template<typename It>
        void computeComplement(It out) const 
        {
          DigitalSetByOctree complement(*myDomain);
          for (auto it = myDomain->begin(); it != myDomain->end(); ++it) 
          {
            if (!this->operator()(*it)) 
            {
              *out = *it;
              ++out;
            }
          }
        }

        /**
         * @brief Assigns the octree as the complement of another set
         * 
         * This is equivalent to looping through a set and inserting
         * every node
         * 
         * @param other The digital set to compute complement of
         */
        template<class DSet>
        void assignFromComplement(const DSet& other) 
        {
          if (myState == State::OCTREE) 
          {
            clear();
            for (auto it = myDomain->begin(); it != myDomain->end(); ++it) 
            {
              if (!other(*it)) 
                  insert(*it);
            }
          }
        }
        
        /**
         * @brief Computes the bounding box of stored points
         * 
         * This functions runs in O(N log(L)). 
         * 
         * @param lb Output lower bound
         * @param ub Output upper bound
         */
        void computeBoundingBox(Point& lb, Point& ub) const 
        {
          lb = myDomain->upperBound();
          ub = myDomain->lowerBound();

          for (auto it = begin(); it != end(); ++it) 
          {
            const auto p = *it;
            for (DimIndex d = 0; d < D; d++) 
            {
              lb[d] = std::min(lb[d], p[d]);
              ub[d] = std::max(ub[d], p[d]);
            }
          }
        }

        /**
         * @brief Returns an iterator to the begining of the octree
         */
        Iterator begin() const 
        {
          TraversalMemory mem;
          mem.domain = *myDomain;
          mem.lvl = 0;
          mem.idx = 0;
          mem.currentChildIdx = INVALID_IDX;
          
          return Iterator(this, mem);
        }

        /**
         * @brief Returns an iterator to the begining of the octree
         */
        Iterator begin() { return static_cast<const DigitalSetByOctree&>(*this).begin(); }

        /**
         * @brief Returns an iterator to the end of the octree
         */
        Iterator end() { return Iterator(this); }

        /**
         * @brief Returns an iterator to the end of the octree
         */
        Iterator end() const { return Iterator(this); }

    public:
        /**
         * @brief Converts the octree to DAG
         */
        void convertToDAG();

        template<class Func>
        auto computeFunction(OctreeIterator start, OctreeIterator end, CellIndex range, const Func& f);

        /**
         * @brief Dumps the octree to std out
         * 
         * Note: This function is meant for debug purposes only
         */
        void dumpOctree() const;
    private:
        /**
         * @brief Helper function to split and select among 2^D subdomains
         * 
         * @param domain The domain to split
         * @param sides Selection between left or right subdomain for each dimension
         */
        static Domain splitDomain(const Domain& domain, const DimIndex* sides);
    
    private:
        /**
         * @brief The state the container is in
         *  
         * After the octree is converted to a DAG, it is not possible
         * to insert or remove nodes. 
         */
        enum class State 
        {
          OCTREE, 
          DAG
        };


    private:
        // We store two domains:
        //  - myDomain is the domain used for computation, that 
        //    requires its size to be whole power of two for 
        //    computationnal purposes
        //  - myAdmissibleDomain is the domain of points that can be
        //    stored within the octree. It shares the same lower bound
        //    with domain, but its upper bound is lowered by one.
        CowPtr<Domain> myAdmissibleDomain;     // Domain of point that are allowed. 
        CowPtr<Domain> myDomain;       // Pointer to domain, as required by CDigitalSet.
        State myState = State::OCTREE; // Current state
        
        size_t mySize;                 // Current number of stored voxel.
        std::vector<std::vector<Node>> myNodes; // Nodes storage
    };
};

#include "DigitalSetByOctree.ih"
