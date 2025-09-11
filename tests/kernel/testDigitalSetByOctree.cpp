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

void checkConcept() {
    using namespace DGtal;
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<DigitalSetByOctree< Z3i::Space >> ));
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<DigitalSetByOctree< Z2i::Space >> ));
}

bool testDiagonalPointsDAG() {
    using namespace DGtal;
    const unsigned int lvl = 4;
    const int size = (1 << lvl);

    bool ok = true;
    Z3i::Domain domain(Z3i::Point{0, 0, 0}, Z3i::Point{size, size, size});
    DigitalSetByOctree<Z3i::Space> octree(domain);
    trace.emphase() << octree.domain() << std::endl;

    // One of best cases for dag: all points on the diagonal of the domain
    // This is compressed as a single node per level.
    for (size_t i = 0; i < size; ++i) {
        octree.insert(Z3i::Point{(int)i, (int)i, (int)i});
    }

    trace.beginBlock( "Simple check for DAG compression ");
    {
        const size_t mem_before = octree.memoryFootprint();
            octree.convertToDAG();
        const size_t mem_after  = octree.memoryFootprint();

        trace.info() << "Before conversion: " << mem_before << " bytes occupied" << "\n";
        trace.info() << "After conversion: "  << mem_after  << " bytes occupied" << "\n";

        ok = ok && (mem_after == lvl * sizeof(typename DigitalSetByOctree<Z3i::Space>::Node));
    } trace.endBlock();

    auto lmbd = [](Z3i::Point, const std::vector<Z3i::Point>& neighborhood) {
        return neighborhood.size();
    };
    auto rslt = octree.computeFunction(octree.begin(), octree.end(), 1, lmbd);

    return ok;
}

bool testInsertAndIterate() {
    using namespace DGtal;

    bool ok = true;
    
    const Z3i::Domain domain  (Z3i::Point{-1, -2, -1}, Z3i::Point{14, 3, 9});        
    const Z3i::Domain expected(Z3i::Point{-1, -2, -1}, Z3i::Point{14, 13, 14});        

    // Some test cases
    const Z3i::Point testPoints[] = {
        // Valid points inside the domain
        Z3i::Point(0, 0, 0)   , Z3i::Point(2, 1, 4),    // Some points
        Z3i::Point(-1, -1, -1), Z3i::Point(14, 13, 14), // On the edge
        Z3i::Point(8, 7, 4),                            // Outside the original domain, but inside the excpected one  
        // Duplicates
        Z3i::Point(0, 0, 0), Z3i::Point(8, 7, 4), 
        // Outside the domain
        Z3i::Point(-5, 152, -123)
    };
    const size_t testPointCount = sizeof(testPoints) / sizeof(Z3i::Point);
    const bool testPointsValid[8] = {
        true, true, 
        true, true, 
        true, 
        true, true, 
        false, 
    };
    // Valid points of testPoints, in expected order
    const Z3i::Point validPoints[] = {
        Z3i::Point(-1, -1, -1), Z3i::Point(0, 0, 0), 
        Z3i::Point(2, 1, 4)   , Z3i::Point(8, 7, 4), 
        Z3i::Point(14, 13, 14)
    };
    const size_t validPointCount = sizeof(validPoints) / sizeof(Z3i::Point);


    trace.beginBlock ( "Checking domain " );
    {
        DigitalSetByOctree<Z3i::Space> octree(domain);

        trace.info() << "Domain = " << domain << "\n";
        trace.info() << "Octree Domain   = " << octree.domain() << "\n";
        trace.info() << "Expected Domain = " << expected << "\n";

        ok = ok && (octree.domain().lowerBound() == expected.lowerBound()) && 
                   (octree.domain().upperBound() == expected.upperBound());
    } trace.endBlock();

    trace.beginBlock( "Testing insertion ");
    {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        trace.info() << "Attempts of insertion = " << 8  
                     << ", octree size = "  << octree.size()
                     << ", valid points = " << validPointCount << "\n";

        ok = ok && (octree.size() == validPointCount);
    } trace.endBlock();

    trace.beginBlock( "Testing existence ");
    {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        for (int i = 0; i < sizeof(testPoints) / sizeof(Z3i::Point); ++i) {
            trace.info() << "Testing for:" << testPoints[i] 
                         << ", octree = "    << octree(testPoints[i])
                         << ", expected = "  << testPointsValid[i]  << "\n";

            ok = ok && (testPointsValid[i] == octree(testPoints[i]));
        }
    } trace.endBlock();

    trace.beginBlock( "Test iterating ");
    {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        size_t i = 0;
        for (auto it = octree.begin(); it != octree.end(); ++it, ++i) {
            if (i >= validPointCount) {
                trace.info() << "Error: more valid points than expected. \n";
                ok = false;
                break;
            }
            trace.info() << i << ": " << *it << ", expected = " << validPoints[i] << "\n";
            ok = ok && (*it == validPoints[i]);
        }
    } trace.endBlock();

    trace.beginBlock( "Test erase" );
    {
        DigitalSetByOctree<Z3i::Space> octree(domain);
        for (int i = 0; i < testPointCount; ++i) {
            octree.insert(testPoints[i]);
        }

        octree.erase(testPoints[0]);
        octree.erase(octree.end());

        trace.info() << "Removing point:"  
                     << " octree size = "  << octree.size()
                     << " valid points = " << validPointCount - 1 << "\n";
        ok = ok && (octree.size() == validPointCount - 1);
    } trace.endBlock();

    return ok;
}

int main(int, char**) { 
    const bool res = testInsertAndIterate() && testDiagonalPointsDAG();
    DGtal::trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
    return !res;
}
