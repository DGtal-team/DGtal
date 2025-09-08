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

int main(int argc, char** argv) { 
    using namespace DGtal;
    
    
    Z3i::Domain d(Z3i::Point{-1, -1, -1}, Z3i::Point{14, 3, 9});
    DigitalSetByOctree<Z3i::Space> octree(d);
    BOOST_CONCEPT_ASSERT(( concepts::CDigitalSet<DigitalSetByOctree< Z3i::Space >> ));

    std::cout << octree.domain() << std::endl;
    std::cout << "====" << std::endl;
    octree.insert(Z3i::Point{0, 0, 0});
    std::cout << "====" << std::endl;
    octree.insert(Z3i::Point{1, 1, 1});
    std::cout << "====" << std::endl;
    octree.insert(Z3i::Point{-1, -1, -1});
    std::cout << "====" << std::endl;
    octree.insert(Z3i::Point{10, 10, 10});
    
    std::cout << "====" << std::endl;
    std::cout << std::boolalpha;
    std::cout << octree(Z3i::Point{ 0, 0, 0}) << std::endl;;
    std::cout << octree(Z3i::Point{-1, -1, -1}) << std::endl;;
    std::cout << octree(Z3i::Point{ 12, 18, 24}) << std::endl;;
    std::cout << octree(Z3i::Point{2, 6, 3}) << std::endl;;

    std::cout << "====" << std::endl;
   
    auto it = octree.find(Z3i::Point{10, 10, 10});
    std::cout << *it << std::endl;
    std::cout << "====" << std::endl;
    std::cout << "====" << std::endl;
    for (auto it = octree.begin(); it != octree.end(); it.next()) {
        std::cout << *it << std::endl;
    }

    std::cout << "====" << std::endl;
    std::cout << "====" << std::endl;
    auto it2 = octree.find(Z3i::Point{10, 10, 10});
    std::cout << *it2 << std::endl;
    std::cout << std::endl;
    std::cout << octree.memoryFootprint() << std::endl;
    octree.erase(it2);
    for (auto it = octree.begin(); it != octree.end(); it.next()) {
        std::cout << *it << std::endl;
    }
    std::cout << octree.memoryFootprint() << std::endl;
    return 0;
}
