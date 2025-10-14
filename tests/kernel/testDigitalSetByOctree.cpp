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
 * @file
 * @ingroup Tests
 * @author Bastien DOIGNIES 
 * LIRIS
 *
 * @date 2025/09/05
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_DigitalSetByOctree <p>
 * Aim: simple tesst of DigitalSetByOctree
 */

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/sets/DigitalSetByOctree.h"
#include "DGtalCatch.h"

#include "DGtal/io/writers/VolWriter.h"
#include "DGtal/io/readers/VolReader.h"

using namespace DGtal;
using namespace std;

struct TestFixture {
    inline static const Z3i::Domain domain        {Z3i::Point(-1, -2, -1), Z3i::Point(14, 3, 9)};
    inline static const Z3i::Domain expectedDomain{Z3i::Point(-1, -2, -1), Z3i::Point(14, 13, 14)};

    // Some test cases
    inline static const Z3i::Point testPoints[8] = {
        // Valid points inside the domain
        Z3i::Point(0, 0, 0)   , Z3i::Point(2, 1, 4),    // Some points
        Z3i::Point(-1, -1, -1), Z3i::Point(14, 13, 14), // On the edge
        Z3i::Point(8, 7, 4),                            // Outside the original domain, but inside the excpected one  
                                                        // We pay for the bigger domain, we might accept those points as well
        // Duplicates
        Z3i::Point(0, 0, 0), Z3i::Point(8, 7, 4), 
        // Outside the domain
        Z3i::Point(-5, 152, -123)
    };
    inline static const size_t testPointCount = sizeof(testPoints) / sizeof(Z3i::Point);

    inline static const bool testPointsValid[8] = {
        true, true, 
        true, true, 
        true, 
        true, true, 
        false, 
    };
    // Valid points of testPoints, in expected order
    inline static const Z3i::Point validPoints[5] = {
        Z3i::Point(-1, -1, -1), Z3i::Point(0, 0, 0), 
        Z3i::Point(2, 1, 4)   , Z3i::Point(8, 7, 4), 
        Z3i::Point(14, 13, 14)
    };
    inline static const size_t validPointCount = sizeof(validPoints) / sizeof(Z3i::Point);
};

TEST_CASE_METHOD(TestFixture, "Benchmarking DigitalSetByOctree using Catch2", "[catch]")
{
    // This is checked at compile time !
    using namespace DGtal;
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<DigitalSetByOctree< Z2i::Space >> ));
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<DigitalSetByOctree< Z3i::Space >> ));

    SECTION("Test octree domain") {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        REQUIRE(octree.domain().lowerBound() == expectedDomain.lowerBound());
        REQUIRE(octree.domain().upperBound() == expectedDomain.upperBound());
    };

    SECTION("Test octree insert") {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        REQUIRE(octree.size() == validPointCount);
    };
    
    SECTION("Testing if points exists") {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        for (int i = 0; i < sizeof(testPoints) / sizeof(Z3i::Point); ++i) {
            REQUIRE(testPointsValid[i] == octree(testPoints[i]));
        }
    };

    SECTION("Testing iterating over octree") {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        size_t i = 0;
        for (auto it = octree.begin(); it != octree.end(); ++it, ++i) {
            if (i >= validPointCount) {
                REQUIRE(false);
                return;
            }
            REQUIRE(*it == validPoints[i]);
        }
    };

    SECTION("Testing erasing points") {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        octree.erase(testPoints[0]);
        octree.erase(octree.end()); // Invalid iterator, but we accept it anyway

        for (auto it = octree.begin(); it != octree.end(); ++it) {
            REQUIRE(*it != testPoints[0]);
        }

        REQUIRE(octree.size() == validPointCount - 1);
    };

    SECTION("Test DAG on simple case") {
        const unsigned int lvl = 4;
        const int size = (1 << lvl);

        const size_t expectedMemory = lvl * sizeof(typename DigitalSetByOctree<Z3i::Space>::Node);
        std::vector<size_t> expectedRslt(size, 3);
        expectedRslt.front() = 2;
        expectedRslt.back()  = 2;

        Z3i::Domain domain(Z3i::Point{0, 0, 0}, Z3i::Point{size, size, size});
        
        DigitalSetByOctree<Z3i::Space> octree = DigitalSetByOctree<Z3i::Space>(domain);
        
        // One of best cases for dag: all points on the diagonal of the domain
        // This is compressed as a single node per level.
        for (size_t i = 0; i < size; ++i) {
            octree.insert(Z3i::Point{(int)i, (int)i, (int)i});
        }
        
        octree.convertToDAG();

        auto lmbd = [](Z3i::Point, const std::vector<Z3i::Point>& neighborhood) {
            return neighborhood.size();
        };
        auto rslt = octree.computeFunction(octree.begin(), octree.end(), 1, lmbd);

        REQUIRE(octree.memoryFootprint() == expectedMemory);
        REQUIRE(rslt == expectedRslt);

    };

    SECTION("Test Vol I/O") {
        const unsigned int start = 5;
        const unsigned int end = 10;
        DigitalSetByOctree<Z3i::Space> octree1(Z3i::Domain(Z3i::Point{start, start, start}, {end, end, end}));
        for (unsigned int i = 0; i < (end - start); ++i) {
            const int c = (int)start + (int)i;
            octree1.insert(Z3i::Point{c, c, c});
        }
        octree1.convertToDAG();

        VolWriter<DigitalSetByOctree<Z3i::Space>> writer;
        VolReader<DigitalSetByOctree<Z3i::Space>, int> reader;

        writer.exportVol("tmp.vol", octree1, true);
        auto octree2 = reader.importVol("tmp.vol");

        auto it1 = octree1.begin();
        auto it2 = octree2.begin();
        for (; it1 != octree1.end() && it2 != octree2.end(); ++it1, ++it2) {
            REQUIRE(*it1 == *it2);
        }
        REQUIRE(it1 == octree1.end());
        REQUIRE(it2 == octree2.end());
    };
};

