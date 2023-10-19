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
#include "DGtalCatch.h"
#include "DGtal/topology/KhalimskyCellHashFunctions.h"
#include "DGtal/topology/VoxelComplex.h"
#include "DGtal/topology/VoxelComplexFunctions.h"
///////////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;
using Point = DGtal::Z3i::Point;
using DigitalSet = DGtal::DigitalSetByAssociativeContainer<
DGtal::Z3i::Domain, std::unordered_set<typename DGtal::Z3i::Domain::Point>>;
DigitalSet fillSet(const Point & lowerBound, const Point & upperBound) {
    using Value = typename Point::Component;
    using Domain = DGtal::Z3i::Domain;
    Domain domain(lowerBound, upperBound);
    DigitalSet a_set(domain);
    for(Value x = lowerBound[0]; x <= upperBound[0]; x++)
        for(Value y = lowerBound[1]; y <= upperBound[1]; y++)
            for(Value z = lowerBound[2]; z <= upperBound[2]; z++) {
                a_set.insert(Point(x,y,z));
            }
    return a_set;
}

TEST_CASE("getBlocksFromSplits", "[blocks]"){
    using namespace DGtal::functions;
    using KSpace = DGtal::Z3i::KSpace;
    Point lowerBound = {0,0,0};
    Point upperBound = {13,13,13};
    using Complex = DGtal::VoxelComplex<KSpace>;
    KSpace kspace;
    kspace.init(lowerBound, upperBound, true);
    Complex vc(kspace);
    vc.construct(fillSet(lowerBound, upperBound));
    const size_t requested_number_of_splits = 4;
    auto splitted_complexes = splitComplex(vc, requested_number_of_splits);
    auto & splits_bounds = splitted_complexes.splits_bounds;
    auto & splits = splitted_complexes.splits;
    SECTION("No wide_of_block_sub_complex") {
        auto block_lower_upper_bounds =
            getBorderBlocksFromSplits(lowerBound, upperBound, splits_bounds);
        auto number_of_blocks = getNumberOfBorderBlocksFromSplits(splits);
        const auto expected_number_of_blocks = 2;
        CHECK(number_of_blocks == expected_number_of_blocks);
        REQUIRE(block_lower_upper_bounds.size() == expected_number_of_blocks);

        Point expected_lowerBound_x0 = Point(6, 0, 0);
        Point expected_upperBound_x0 = Point(6, 13, 13);
        Point expected_lowerBound_y0 = Point(0, 6, 0);
        Point expected_upperBound_y0 = Point(13, 6, 13);
        CHECK(block_lower_upper_bounds[0][0] == expected_lowerBound_x0);
        CHECK(block_lower_upper_bounds[0][1] == expected_upperBound_x0);
        CHECK(block_lower_upper_bounds[1][0] == expected_lowerBound_y0);
        CHECK(block_lower_upper_bounds[1][1] == expected_upperBound_y0);
    }
    SECTION("wide_of_block_sub_complex = 2") {
        size_t wide_of_block_sub_complex = 2;
        auto block_lower_upper_bounds =
            getBorderBlocksFromSplits(lowerBound, upperBound, splits_bounds, wide_of_block_sub_complex);
        Point expected_lowerBound_x0 = Point(4, 0, 0);
        Point expected_upperBound_x0 = Point(7, 13, 13);
        Point expected_lowerBound_y0 = Point(0, 4, 0);
        Point expected_upperBound_y0 = Point(13, 7, 13);
        CHECK(block_lower_upper_bounds[0][0] == expected_lowerBound_x0);
        CHECK(block_lower_upper_bounds[0][1] == expected_upperBound_x0);
        CHECK(block_lower_upper_bounds[1][0] == expected_lowerBound_y0);
        CHECK(block_lower_upper_bounds[1][1] == expected_upperBound_y0);
    }
}

TEST_CASE("computeSplitsEasy", "[computeSplits, getSplit]") {
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
    CHECK( number_of_splits == requested_number_of_splits );
    CHECK( splits[0] == 1 );
    CHECK( splits[1] == 1 );
    CHECK( splits[2] == 2 );
    trace.endBlock();
    trace.beginBlock("getSplit Easy");
    for (size_t split_index = 0; split_index < number_of_splits; split_index++) {
        auto split_bounds =
            DGtal::functions::getSplit(split_index, requested_number_of_splits,
                    lowerBound, upperBound);
        const auto & outputLowerBound = split_bounds.lowerBound;
        const auto & outputUpperBound = split_bounds.upperBound;
        trace.info() << "split_index: " << split_index << std::endl;
        trace.info() << "outputLowerBound: " << outputLowerBound << std::endl;
        trace.info() << "outputUpperBound: " << outputUpperBound << std::endl;
    }
    size_t split_index = 1;
    auto split_bounds =
        DGtal::functions::getSplit(split_index, requested_number_of_splits,
                lowerBound, upperBound);
    const auto & outputLowerBound = split_bounds.lowerBound;
    const auto & outputUpperBound = split_bounds.upperBound;
    CHECK( outputLowerBound[2] == 4 );
    CHECK( outputUpperBound[2] == 8 );
    trace.endBlock();
}
////////////////////// splitComplex ///////////////////////////
// Fixture for a X
struct Fixture_X_with_tight_bounds {
    ///////////////////////////////////////////////////////////
    // type aliases
    ///////////////////////////////////////////////////////////
    using Point = DGtal::Z3i::Point;
    using Domain = DGtal::Z3i::Domain;
    using KSpace = DGtal::Z3i::KSpace;

    using FixtureDigitalTopology = DGtal::Z3i::DT26_6;
    using FixtureDigitalSet = DGtal::DigitalSetByAssociativeContainer<
        DGtal::Z3i::Domain,
        std::unordered_set<typename DGtal::Z3i::Domain::Point>>;
    using FixtureMap = std::unordered_map<KSpace::Cell, CubicalCellData>;
    using FixtureComplex = DGtal::VoxelComplex<KSpace, FixtureMap>;

    ///////////////////////////////////////////////////////////
    // fixture data
    FixtureComplex complex_fixture;
    FixtureDigitalSet set_fixture;
    KSpace ks_fixture; // needed because ConstAlias in CC constructor.
    ///////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    // Constructor
    ///////////////////////////////////////////////////////////
    Fixture_X_with_tight_bounds() : complex_fixture(ks_fixture), set_fixture(create_set()) {
        create_complex_from_set(set_fixture);
    }

    ///////////////////////////////////////////////////////////
    // Function members
    ///////////////////////////////////////////////////////////
    FixtureDigitalSet create_set() {
        using namespace DGtal;

        // with_tight_bounds
        Point p1(-6, -6, -1);
        Point p2(6, 6, 1);
        Domain domain(p1, p2);

        FixtureDigitalSet a_set(domain);
        std::vector<Point> center_set;
        center_set.reserve(9);

        Point c00(0, 0, 0);
        center_set.push_back(c00);
        Point c01x(-1, 0, 0);
        center_set.push_back(c01x);
        Point c10x(1, 0, 0);
        center_set.push_back(c10x);
        Point c02x(-2, 0, 0);
        center_set.push_back(c02x);
        Point c20x(2, 0, 0);
        center_set.push_back(c20x);

        Point c01y(0, -1, 0);
        center_set.push_back(c01y);
        Point c10y(0, 1, 0);
        center_set.push_back(c10y);
        Point c02y(0, -2, 0);
        center_set.push_back(c02y);
        Point c20y(0, 2, 0);
        center_set.push_back(c20y);

        Point z_pos(0, 0, 1);
        int branch_length(4);
        std::vector<Point> diagonals;
        diagonals.reserve(6);
        for (const auto &p : center_set) {
            diagonals.clear();
            for (int l = 0; l <= branch_length; ++l) {
                diagonals.push_back({l, l, 0});
                diagonals.push_back({l, -l, 0});
                diagonals.push_back({-l, l, 0});
                diagonals.push_back({-l, -l, 0});
                for (int z = -1; z <= 1; ++z)
                    for (const auto &d : diagonals)
                        a_set.insert(p + d + (z * z_pos));
            }
        }

		return a_set;
    }

    FixtureComplex &create_complex_from_set(FixtureDigitalSet &input_set) {

        ks_fixture.init(input_set.domain().lowerBound(),
                        input_set.domain().upperBound(), true);
        complex_fixture = FixtureComplex(ks_fixture);
        complex_fixture.construct(input_set);
        return complex_fixture;
    }
};

TEST_CASE_METHOD(Fixture_X_with_tight_bounds, "splitComplex", "[parallel]") {
    using namespace DGtal::functions;
    auto & vc = complex_fixture;
    // CHECK(vc.nbCells(0) == 528);
    // CHECK(vc.nbCells(1) == 1276);
    // CHECK(vc.nbCells(2) == 1016);
    // CHECK(vc.nbCells(3) == 267);
    // Modify data of a cell to check if data is copied to sub complexes
    Point p{2, 0, 0};
    const auto modified_cell = vc.space().uSpel(p);
    const auto modified_cell_dim = vc.space().uDim(modified_cell);
    auto it_cell= vc.findCell(modified_cell_dim, modified_cell);
    CHECK(it_cell != vc.end(modified_cell_dim));
    DGtal::uint32_t magic_number = 15;
    it_cell->second.data = magic_number;
    SECTION("SplitComplex")  {
        size_t requested_number_of_splits = 2;
        trace.beginBlock("SplitComplex");
        trace.info() << "lowerBound" <<  vc.space().lowerBound() << std::endl;
        trace.info() << "upperBound" <<  vc.space().upperBound() << std::endl;
        auto out = splitComplex(vc, requested_number_of_splits);
        auto & sub_complexes = out.sub_complexes;
        CHECK(out.number_of_splits == requested_number_of_splits);
        CHECK(sub_complexes.size() == requested_number_of_splits);
        {
            size_t sub_index = 0;
            auto & sc = sub_complexes[sub_index];
            CHECK(sc.nbCells(0) != 0);
            CHECK(sc.nbCells(1) != 0);
            CHECK(sc.nbCells(2) != 0);
            CHECK(sc.nbCells(3) != 0);
            trace.info() << "lowerBound S0" <<  sc.space().lowerBound() << std::endl;
            trace.info() << "upperBound S0" <<  sc.space().upperBound() << std::endl;
            CHECK(sc.space().lowerBound() == typename KSpace::Point(-6,-6,-1));
            CHECK(sc.space().upperBound() == typename KSpace::Point(-1,6,1));
            CHECK(sc.space().lowerBound() == out.splits_bounds[sub_index].lowerBound);
            CHECK(sc.space().upperBound() == out.splits_bounds[sub_index].upperBound);
        }
        {
            size_t sub_index = 1;
            auto & sc = sub_complexes[sub_index];
            CHECK(sc.nbCells(0) != 0);
            CHECK(sc.nbCells(1) != 0);
            CHECK(sc.nbCells(2) != 0);
            CHECK(sc.nbCells(3) != 0);
            trace.info() << "lowerBound S1" <<  sc.space().lowerBound() << std::endl;
            trace.info() << "upperBound S1" <<  sc.space().upperBound() << std::endl;
            CHECK(sc.space().lowerBound() == typename KSpace::Point(0,-6,-1));
            CHECK(sc.space().upperBound() == typename KSpace::Point(6,6,1));
            CHECK(sc.space().lowerBound() == out.splits_bounds[sub_index].lowerBound);
            CHECK(sc.space().upperBound() == out.splits_bounds[sub_index].upperBound);
            // Check cell exist in sub_complex
            auto sc_it_cell = sc.findCell(it_cell->first);
            CHECK(sc_it_cell != sc.end(modified_cell_dim));
            // Check data is copied into sub_complexes
            CHECK(sc[modified_cell].data == magic_number);
        }
        const size_t row_voxels_sc0 = 7;
        CHECK(sub_complexes[0].nbCells(0) + sub_complexes[1].nbCells(0) ==
                vc.nbCells(0) + (row_voxels_sc0 + 1) * 4);
        CHECK(sub_complexes[0].nbCells(1) + sub_complexes[1].nbCells(1) ==
                vc.nbCells(1) + ( (row_voxels_sc0 + 1) * 3 + row_voxels_sc0 * 4));
        CHECK(sub_complexes[0].nbCells(2) + sub_complexes[1].nbCells(2) ==
                vc.nbCells(2) + row_voxels_sc0 * 3);
        CHECK(sub_complexes[0].nbCells(3) + sub_complexes[1].nbCells(3) ==
                vc.nbCells(3));
        trace.endBlock();
    }
    SECTION("getBorderVoxels")  {
        trace.beginBlock("getBorderVoxels");
        size_t requested_number_of_splits = 4;
        auto out = splitComplex(vc, requested_number_of_splits);
        auto & sub_complexes = out.sub_complexes;
        const auto lowerBound_to_ignore = vc.space().lowerBound();
        const auto upperBound_to_ignore = vc.space().upperBound();
        typename FixtureComplex::Point wide_point = {2,2,2};
        std::vector<std::vector<typename FixtureComplex::CellMapIterator>> borders;
        {
            size_t sub_index = 0;
            auto & sc = sub_complexes[sub_index];
            const auto lowerBound = sc.space().lowerBound();
            const auto upperBound = sc.space().upperBound();
            auto border_iterators =
                getBorderVoxels(sc, lowerBound, upperBound, &wide_point,
                        &lowerBound_to_ignore, &upperBound_to_ignore);
            CHECK(border_iterators.size() == 30);
            borders.push_back(border_iterators);
        }
        {
            size_t sub_index = 1;
            auto & sc = sub_complexes[sub_index];
            const auto lowerBound = sc.space().lowerBound();
            const auto upperBound = sc.space().upperBound();
            auto border_iterators =
                getBorderVoxels(sc, lowerBound, upperBound, &wide_point,
                        &lowerBound_to_ignore, &upperBound_to_ignore);
            CHECK(border_iterators.size() == 30);
            borders.push_back(border_iterators);
        }
        trace.endBlock();
    }
}
////////////////////// end splitComplex ////////////////////////
