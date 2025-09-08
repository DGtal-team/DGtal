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

#pragma once

/**
 * @file DigitalSetBySTLSet.h
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
#include <stack>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

//#include "DGtal/io/Display3D.h"


namespace DGtal
{
    template <class Space>
    class DigitalSetByOctree {
    public:
        using DimIndex = std::uint8_t;
        using CellIndex = typename Space::UnsignedInteger;
        using Size = size_t;

        using Domain = HyperRectDomain<Space>;
        using Point  = typename Domain::Point;


        struct TraversalMemory {
            Domain domain;
            CellIndex lvl;
            CellIndex idx;
            CellIndex currentChildIdx;

            bool operator==(const TraversalMemory& other) const {
                return lvl == other.lvl && idx == other.idx && currentChildIdx == other.currentChildIdx;
            }
        };
        friend class Iterator;
        struct OctreeIterator {
            // Should be std::output_iterator_tag, but CDigitalSet
            // does not allow this; so we upgrade the category
            using iterator_category = std::forward_iterator_tag;
            using difference_type = std::ptrdiff_t;
            using value_type = Point;
            using reference = value_type&;
            using pointer = value_type*;

            friend class DigitalSetByOctree;

            OctreeIterator(const DigitalSetByOctree* container) {
                myContainer = container;
            }

            OctreeIterator(const DigitalSetByOctree* container, 
                           TraversalMemory init) {
                myContainer = container;
                myMemory.push(std::move(init));

                findNextLeaf();
            }

            OctreeIterator(const DigitalSetByOctree* container, 
                           std::stack<TraversalMemory>& memory) {
                myContainer = container;
                myMemory = memory;
            }

            void findNextLeaf() {
                while (myMemory.size() != 0) {
                    TraversalMemory& m = myMemory.top();
                    CellIndex k = (m.currentChildIdx == INVALID_IDX) ? 0 : (m.currentChildIdx + 1);
                    for (; k < CELL_COUNT; ++k) {
                        if (myContainer->myNodes[m.lvl][m.idx].children[k] != INVALID_IDX) {
                            // Update explored child
                            m.currentChildIdx = k;
                            break;
                        }
                    }

                    if (k < CELL_COUNT) {
                        if (myMemory.top().lvl == myContainer->myNodes.size() - 1) break;
                        // Child found, go down

                        TraversalMemory newMemory;
                        newMemory.lvl = m.lvl + 1;
                        newMemory.domain = splitDomain(m.domain, SIDES_FROM_INDEX[k].data());
                        newMemory.idx = myContainer->myNodes[m.lvl][m.idx].children[k];
                        newMemory.currentChildIdx = INVALID_IDX;

                        myMemory.push(std::move(newMemory));
                    } else {
                        // No children unexplored, go back to parent
                        myMemory.pop();
                    }

                }

                if (myMemory.size() != 0) {
                    // For leaves, find first non 0 index
                    auto& mem = myMemory.top();
                    if (mem.currentChildIdx == INVALID_IDX) {
                        for (CellIndex k = 0; k < CELL_COUNT; ++k) {
                            if (myContainer->myNodes[mem.lvl][mem.idx].children[k] != 0) {
                                mem.currentChildIdx = k;
                                break;
                            }
                        }
                    }
                }
            }

            void next() {
                findNextLeaf();
            }

            bool operator==(const OctreeIterator& other) const {
                if (myContainer != other.myContainer) return false;
                if (myMemory.size() != other.myMemory.size()) return false;

                if (myMemory.size() != 0) {
                    return myMemory.top() == other.myMemory.top();
                }
                return true;
            }

            bool operator!=(const OctreeIterator& other) const {
                return !(*this == other);
            }

            Point operator*() const {
                const auto& sides = SIDES_FROM_INDEX[myMemory.top().currentChildIdx];
                return splitDomain(myMemory.top().domain, sides.data()).lowerBound();
            }

            OctreeIterator& operator++() {
                next();
                return *this;
            }

            OctreeIterator operator++(int) {
                auto it = *this;
                next();
                return it;
            }

        private:
            const DigitalSetByOctree* myContainer;
            std::stack<TraversalMemory> myMemory;
        };

        using Iterator = OctreeIterator;
        using ConstIterator = OctreeIterator;

        BOOST_STATIC_CONSTANT(DGtal::Dimension, D = Space::dimension);
        BOOST_STATIC_CONSTANT(CellIndex, CELL_COUNT  = (1 << D));
        BOOST_STATIC_CONSTANT(CellIndex, INVALID_IDX = (1 << CELL_COUNT) - 1);
        constexpr static std::array<std::array<DimIndex, D>, CELL_COUNT> SIDES_FROM_INDEX = []() constexpr {
            std::array<std::array<DimIndex, D>, CELL_COUNT> sides_from_index{};
            for (CellIndex i = 0; i < CELL_COUNT; ++i) {
                CellIndex coord = i;
                for (DimIndex d = 0; d < D; ++d) {
                    sides_from_index[i][d] = coord % 2;
                    coord /= 2;
                }
            }

            return sides_from_index;
        }();

        DigitalSetByOctree(const Domain& d) {
            const auto lb = d.lowerBound();
            const auto ub = d.upperBound();
            auto size = ub - lb;
            
            // Enforce cubical domain with side that are powers of 2
            CellIndex newSize = 1;
            for (DimIndex d = 0; d < D; ++d) {
                if (!(size[d] & (size[d] - 1))) {
                    newSize = (newSize > size[d]) ? newSize : size[d];
                } else {
                    CellIndex s = 1 << (static_cast<CellIndex>(std::ceil(std::log(size[d]))) + 1);
                    newSize = (newSize > s) ? newSize : s;
                }
            }

            for (DimIndex d = 0; d < D; ++d) {
                size[d] = newSize;
            }

            size_t lvl = static_cast<size_t>(std::ceil(std::log(newSize)));
            myNodes.resize(lvl + 1);
            myNodes[0].push_back(Node());

            myDomain = CowPtr(new Domain(lb, lb + size));
            mySize = 0;
        };

        const Domain& domain() const {
            return *myDomain;
        }

        CowPtr<Domain> domainPointer() const {
            return myDomain;
        }

        static Domain splitDomain(const Domain& domain, const DimIndex* sides) {
                  auto lb = domain.lowerBound();
            const auto ub = domain.upperBound();
            const auto size = (ub - lb) / 2;

            for (DimIndex d = 0; d < D; ++d)
                lb[d] += sides[d] * size[d];
            
            return Domain(lb, lb + size);
        }

        void insert(const Point& p) {
            if (myState == State::OCTREE) {
                Domain domain = *myDomain;
                CellIndex nodeIdx = 0;
                
                if (!domain.isInside(p)) return;
                for (size_t i = 0; i < myNodes.size() - 1; ++i) {
                    const auto lb = domain.lowerBound();
                    const auto ub = domain.upperBound();
                    const Point middle = domain.lowerBound() + (ub - lb) / 2;

                    CellIndex childIdx = 0;
                    DimIndex sides[D]{};
                    for (DimIndex d = 0; d < D; ++d) {
                        sides[d]  = (p[d] >= middle[d]);
                        childIdx += sides[d] * (1 << (D - d - 1));
                    }

                    domain = splitDomain(domain, sides);
                    if (myNodes[i][nodeIdx].children[childIdx] == INVALID_IDX) {
                        myNodes[i + 1].push_back(Node());
                        myNodes[i + 1].back().parent = nodeIdx;
                        myNodes[i][nodeIdx].children[childIdx] = myNodes[i + 1].size() - 1;

                        mySize ++;
                    }

                    nodeIdx = myNodes[i][nodeIdx].children[childIdx];
                }

                // Do not create nodes for single voxels. So we repeat 
                const auto lb = domain.lowerBound();
                const auto ub = domain.upperBound();
                const Point middle = domain.lowerBound() + (ub - lb) / 2;

                CellIndex childIdx = 0;
                for (DimIndex d = 0; d < D; ++d) {
                    childIdx += (p[d] >= middle[d]) * (1 << (D - d - 1));
                }
                myNodes[myNodes.size() - 1][nodeIdx].children[childIdx] = 1;
            }
        }

        void insertNew(const Point& p) {
            insert(p);
        }

        bool operator()(const Point& p) const {
            return find(p) != end();
        }

        Iterator find(const Point& p) const {
            std::stack<TraversalMemory> traversal;
            Domain domain = *myDomain;
            CellIndex nodeIdx = 0;
            
            TraversalMemory root;
            root.lvl = 0;
            root.domain = domain;
            root.idx = 0;
            traversal.push(root);
            
            if (!domain.isInside(p)) return end();
            for (size_t i = 0; i < myNodes.size() - 1; ++i) {
                const auto lb = domain.lowerBound();
                const auto ub = domain.upperBound();
                const Point middle = domain.lowerBound() + (ub - lb) / 2;

                CellIndex childIdx = 0;

                DimIndex sides[D]{};
                for (DimIndex d = 0; d < D; ++d) {
                    sides[d]  = (p[d] >= middle[d]);
                    childIdx += sides[d] * (1 << (D - d - 1));
                }
                
                if (myNodes[i][nodeIdx].children[childIdx] == INVALID_IDX) {
                    return end();
                }
                
                domain = splitDomain(domain, sides);
                nodeIdx = myNodes[i][nodeIdx].children[childIdx];
    
                TraversalMemory memory;
                memory.domain = domain;
                memory.lvl = i + 1;
                memory.idx = nodeIdx;
                memory.currentChildIdx = INVALID_IDX;
                
                traversal.top().currentChildIdx = childIdx;
                traversal.push(memory);
            }
            
            const auto lb = domain.lowerBound();
            const auto ub = domain.upperBound();
            const Point middle = domain.lowerBound() + (ub - lb) / 2;
            CellIndex childIdx = 0;
            for (DimIndex d = 0; d < D; ++d) {
                childIdx += (p[d] >= middle[d]) * (1 << (D - d - 1));
            }
            traversal.top().currentChildIdx = childIdx;

            return Iterator(this, traversal);
        }

        size_t size() const {
            return mySize;
        }

        size_t memoryFootprint() const {
            size_t count = 0;
            for (CellIndex i = 0; i < myNodes.size(); ++i) {
                count += myNodes[i].size() * sizeof(Node);
            }
            return count;
        }

        bool empty() const {
            return mySize == 0;
        }

        void clear() {
            myNodes.clear();
            myState = State::OCTREE;
        }

        size_t erase(const Iterator& it) {
            if (myState == State::OCTREE) {
                std::vector<std::pair<CellIndex, CellIndex>> reindex;
                bool childRemoved = true;

                auto mem = it.myMemory;
                while (!mem.empty()) {
                    const auto& tmem = mem.top();
                    if (childRemoved) { // Can disconnect parent from the child
                        CellIndex child = myNodes[tmem.lvl][tmem.idx].children[tmem.currentChildIdx];
                        myNodes[tmem.lvl][tmem.idx].children[tmem.currentChildIdx] = INVALID_IDX;
                        
                        // Reindex childrens because one was removed in previous layer
                        for (CellIndex i = 0; i < myNodes[tmem.lvl].size(); ++i) {
                            auto& node = myNodes[tmem.lvl][i];
                            for (CellIndex j = 0; j < CELL_COUNT; ++j) {
                                if (node.children[j] != INVALID_IDX && node.children[j] >= child) {
                                    node.children[j] --;
                                }
                            }
                        }
                    }

                    size_t count = 0;
                    for (CellIndex i = 0; i < CELL_COUNT; ++i) {
                        count += (myNodes[tmem.lvl][tmem.idx].children[i] != INVALID_IDX);
                    }

                    if (count == 0) {
                        myNodes[tmem.lvl].erase(myNodes[tmem.lvl].begin() + tmem.idx);
                        childRemoved = true;
                    } else {
                        childRemoved = false;
                    }

                    mem.pop();
                }
            }
            return 0;
        }

        size_t erase(Iterator begin, Iterator end) {
            size_t count = 0;
            for (; begin != end; ++begin) {
                count += erase(begin);
            }
            return count;
        }

        size_t erase(const Point& p) {
            return erase(find(p));
        }

        void shrink() {
            for (CellIndex i = 0; i < myNodes.size(); ++i) {
                myNodes[i].shrink_to_fit();
            }
        }

        DigitalSetByOctree& operator+=(const DigitalSetByOctree& other) {
            if (myState == State::OCTREE) {
                for (auto it = other.begin(); it != other.end(); ++it) {
                    insert(*it);
                }
            }
            return *this;
        }
        
        template<typename It>
        void computeComplement(It out) const {
            DigitalSetByOctree complement(*myDomain);
            for (auto it = myDomain->begin(); it != myDomain->end(); ++it) {
                if (!this->operator()(*it)) {
                    *out = *it;
                    ++out;
                }
            }
        }
        
        template<class DSet>
        void assignFromComplement(const DSet& other) {
            if (myState == State::OCTREE) {
                for (auto it = myDomain->begin(); it != myDomain->end(); ++it) {
                    if (!other(*it)) {
                        insert(*it);
                    }
                }
            }
        }
        
        void computeBoundingBox(Point& lb, Point& ub) const {
            lb = myDomain->upperBound();
            ub = myDomain->lowerBound();

            for (auto it = begin(); it != end(); ++it) {
                const auto p = *it;
                for (DimIndex d = 0; d < D; d++) {
                    lb[d] = std::min(lb[d], p[d]);
                    ub[d] = std::max(ub[d], p[d]);
                }
            }
        }

        Iterator begin() const {
            CellIndex nodeIdx  = 0;
            CellIndex childIdx = 0;
            Domain domain = *myDomain;

            TraversalMemory mem;
            mem.domain = *myDomain;
            mem.lvl = 0;
            mem.idx = 0;
            mem.currentChildIdx = INVALID_IDX;
            
            return Iterator(this, mem);
        }

        Iterator begin() {
            return static_cast<const DigitalSetByOctree&>(*this).begin();
        }

        Iterator end() { return Iterator(this); }
        Iterator end() const { return Iterator(this); }

    public:
        void convertToDAG() {

            myState = State::DAG;
        }

    private:
        /**
         * @brief The state the container is in
         *  
         * After the octree is converted to a DAG, it is not possible
         * to insert or remove nodes. 
         *
         */
        enum class State {
            OCTREE, 
            DAG
        };

        struct Node {
            Node() {
                parent = INVALID_IDX;
                for (CellIndex i = 0; i < CELL_COUNT; ++i)
                    children[i] = INVALID_IDX;
            }

            CellIndex parent;
            CellIndex children[CELL_COUNT];
        };
    private:
        CowPtr<Domain> myDomain;
        State myState = State::OCTREE;
        
        size_t mySize;
        std::vector<std::vector<Node>> myNodes;
    };
};

