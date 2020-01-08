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
 * @file testSplitFunctions.cpp
 * @ingroup Tests
 * @author Pablo Hernandez-Cerdan (\c pablo.hernandez.cerdan@outlook.com)
 *
 * @date 2019/01/08
 *
 * Testing class for SplitFunctions
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/SplitFunctions.h"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;

#define INBLOCK_TEST(x) \
  nbok += ( x ) ? 1 : 0; \
  nb++; \
  trace.info() << "(" << nbok << "/" << nb << ") " \
         << #x << std::endl;

bool testComputeSplitsEasy() {
    unsigned int nbok = 0;
    unsigned int nb = 0;
    using Point = DGtal::Z3i::Point;
    Point lowerBound = {0,0,0};
    Point upperBound = {4,6,8};
    size_t requested_number_of_splits = 2;
    std::vector<unsigned int> splits(lowerBound.dimension);
    trace.beginBlock("computeSplits Easy");
    trace.info() << "lowerBound: " << lowerBound << std::endl;
    trace.info() << "upperBound: " << upperBound << std::endl;
    auto number_of_splits = functions::computeSplits(
            requested_number_of_splits,
            lowerBound, upperBound,
            splits.data());
    trace.info() << "number_of_splits: " << number_of_splits << std::endl;
    trace.info() << "splits: " << splits[ 0 ] << ", " << splits[ 1 ] << ", "
                 << splits[ 2 ] << std::endl;
    INBLOCK_TEST( number_of_splits == requested_number_of_splits );
    INBLOCK_TEST( splits[0] == 1 );
    INBLOCK_TEST( splits[1] == 1 );
    INBLOCK_TEST( splits[2] == 2 );
    trace.endBlock();
    trace.beginBlock("getSplit Easy");
    for (size_t split_index = 0; split_index < number_of_splits; split_index++) {
        auto outputBounds =
            DGtal::functions::getSplit(split_index, requested_number_of_splits,
                    lowerBound, upperBound);
        const auto & outputLowerBound = outputBounds[0];
        const auto & outputUpperBound = outputBounds[1];
        trace.info() << "split_index: " << split_index << std::endl;
        trace.info() << "outputLowerBound: " << outputLowerBound << std::endl;
        trace.info() << "outputUpperBound: " << outputUpperBound << std::endl;
    }
    size_t split_index = 1;
    auto outputBounds =
        DGtal::functions::getSplit(split_index, requested_number_of_splits,
                lowerBound, upperBound);
    const auto & outputLowerBound = outputBounds[0];
    const auto & outputUpperBound = outputBounds[1];
    INBLOCK_TEST( outputLowerBound[2] == 4 );
    INBLOCK_TEST( outputUpperBound[2] == 8 );
    trace.endBlock();
    return nbok == nb;
};

int main()
{
  trace.beginBlock ( "Testing SplitFunctions" );

  bool res = testComputeSplitsEasy();

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
