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
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2021/02/12
 *
 * Functions for testing class DGtal::ArithmeticalDSSComputerOnSurfels.
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <string>
#include <iterator>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/topology/DigitalSurface2DSlice.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"

#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/surfaces/ArithmeticalDSSComputerOnSurfels.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

using namespace std;
using namespace DGtal;

using KSpace = Z3i::KSpace;
using SH3    = Shortcuts<KSpace>;
using Surfel = KSpace::SCell;

using SegmentComputerOnSurfels = ArithmeticalDSSComputerOnSurfels<KSpace, std::vector<Surfel>::const_iterator, int>;
using SegmentationSurfels   = SaturatedSegmentation<SegmentComputerOnSurfels>;

using SegmentComputer = ArithmeticalDSSComputer<std::vector<Z2i::Point>::const_iterator, int, 4>;
using Segmentation = SaturatedSegmentation<SegmentComputer>;

struct Slice
{
    Dimension dim1;
    Dimension dim2;
    Surfel start;
    std::vector<Surfel> contour;
};

std::pair<KSpace, Slice> getSlice (std::string const& shape = "ellipsoid", double gridstep = 1.0)
{
    using SurfaceSlice = DigitalSurface2DSlice<SH3::DigitalSurface::DigitalSurfaceTracker>;

    auto params = SH3::defaultParameters();
    params("polynomial", shape)("gridstep", gridstep);

    auto implicit_shape = SH3::makeImplicitShape3D(params);
    auto digitized_shape = SH3::makeDigitizedImplicitShape3D(implicit_shape, params);
    auto binary_image = SH3::makeBinaryImage(digitized_shape, params);
    auto kspace = SH3::getKSpace(binary_image, params);
    auto surface = SH3::makeDigitalSurface(binary_image, kspace, params);

    Surfel surfel = Surfaces<KSpace>::findABel(kspace, *binary_image, 10000);

    KSpace::DirIterator q1 = kspace.sDirs(surfel);
    KSpace::DirIterator q2 = kspace.sOrthDirs(surfel);
    Dimension dim1 = *q1, dim2 = *q2;
    auto tracker = surface->container().newTracker(surfel);
    SurfaceSlice surfaceSlice(tracker, dim1);
    delete tracker;

    std::vector<Surfel> contour(surfaceSlice.begin(), surfaceSlice.end());

    Slice slice{dim1, dim2, surfel, contour};

    return { kspace, slice };
}

std::vector<Z2i::Point> extractPoints (SegmentComputerOnSurfels const& sc, Slice const& slice)
{
    std::vector<Z2i::Point> points;

    auto initialPoints = sc.getProjectedPointsFromSurfel(slice.start);
    points.push_back(initialPoints.first);
    points.push_back(initialPoints.second);

    for (auto sit = slice.contour.begin() + 1; sit != slice.contour.end(); ++sit)
    {
        Surfel s = *sit;
        auto pt = sc.getNextProjectedPoint(s);
	points.push_back(pt);
    }

    return points;
}

//////////////////////////////////////////////////////////////
TEST_CASE("Testing ArithmeticalDSSComputerOnSurfels")
{
    // Construct and extract a slice of a digital surface
    KSpace kspace;
    Slice slice;
    std::tie(kspace, slice) = getSlice();

    // Do a segmentation using the surfel class
    SegmentComputerOnSurfels recognitionAlgorithmSurfels(kspace, slice.dim1, slice.dim2);
    SegmentationSurfels segmentationSurfels(slice.contour.begin(), slice.contour.end(), recognitionAlgorithmSurfels);

    // Extract the projected points
    std::vector<Z2i::Point> points = extractPoints(recognitionAlgorithmSurfels, slice);

    // Do a segmentation on the projected points
    SegmentComputer recognitionAlgorithm;
    Segmentation segmentation(points.begin(), points.end(), recognitionAlgorithm);

    // The two segmentations must be the same
    bool allEqual = true;
    auto segIt = segmentation.begin();
    auto segSurfelIt = segmentationSurfels.begin();
    while (segIt != segmentation.end() && segSurfelIt != segmentationSurfels.end()) {
      
      allEqual = allEqual && (segIt->primitive() == segSurfelIt->primitive());
      ++segIt;
      ++segSurfelIt;
    }

    REQUIRE(allEqual);
}

/** @ingroup Tests **/
