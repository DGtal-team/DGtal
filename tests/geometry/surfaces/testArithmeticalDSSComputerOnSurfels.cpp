#include <iostream>
#include <string>
#include <iterator>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/topology/DigitalSurface2DSlice.h"

#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/surfaces/ArithmeticalDSSComputerOnSurfels.h"
#include "DGtal/geometry/curves/GreedySegmentation.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"

#include "DGtal/io/boards/Board2D.h"

using namespace std;
using namespace DGtal;

using KSpace = Z3i::KSpace;
using SH3    = Shortcuts<KSpace>;
using Surfel = KSpace::SCell;

using SegmentComputerOnSurfels = ArithmeticalDSSComputerOnSurfels<KSpace, std::vector<Surfel>::const_iterator, int, 4>;
using SegmentationSurfels      = GreedySegmentation<SegmentComputerOnSurfels>;
// using SegmentationSurfels   = SaturatedSegmentation<SegmentComputerOnSurfels>;
//
using SegmentComputer = ArithmeticalDSSComputer<std::vector<Z2i::Point>::const_iterator, int, 4>;
using Segmentation    = GreedySegmentation<SegmentComputer>;
// using Segmentation = SaturatedSegmentation<SegmentComputer>;

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
    Dimension dim1 = *q1, dim2 = kspace.sOrthDir(surfel);
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

    auto initialPoints = sc.projectSurfel(slice.start);
    points.push_back(initialPoints.first);
    points.push_back(initialPoints.second);

    int currentIdx = 0;
    bool firstIt = true;
    for (auto sit = slice.contour.begin() + 1; sit != slice.contour.end(); ++sit)
    {
        Surfel s = *sit;
        auto projectedPoints = sc.projectSurfel(s);

        if (firstIt) {
            if (projectedPoints.first == points[currentIdx]) {
                points.push_back(projectedPoints.second);
            } else if (projectedPoints.first == points[currentIdx + 1]) {
                points.push_back(projectedPoints.second);
            } else if (projectedPoints.second == points[currentIdx]) {
                points.push_back(projectedPoints.first);
            } else if (projectedPoints.second == points[currentIdx + 1]) {
                points.push_back(projectedPoints.first);
            } else {
                assert(false);
            }

            firstIt = false;
        } else {
            if (projectedPoints.first == points[currentIdx]) {
                points.push_back(projectedPoints.second);
            } else if (projectedPoints.second == points[currentIdx]) {
                points.push_back(projectedPoints.first);
            } else {
                assert(false);
            }
        }

        currentIdx = points.size() - 1;
    }

    return points;
}

template < typename Segmentation >
void displaySegmentation (Segmentation const& segmentation,
                          std::vector<Z2i::Point> const& points,
                          std::string const& filename)
{
    Board2D board;

    // Initial points (starting surfel)
    board << CustomStyle("PointVector", new CustomColors(Color::Red, Color::None)) << points[0] << points[1];

    // All the projected points
    for (const auto& p: points)
    {
        board << SetMode("PointVector", "Grid") << p;
    }

    // The segmentations
    board << SetMode( "ArithmeticalDSS", "BoundingBox" );
    for (auto sit = segmentation.begin(); sit != segmentation.end(); ++sit)
    {
        auto s = *sit;
        board << CustomStyle("ArithmeticalDSS/BoundingBox", new CustomPenColor(Color::Red)) << s.primitive();
    }

    board.saveSVG(filename.c_str());
}

//////////////////////////////////////////////////////////////
int main(void)
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

    // TODO: the two segmentations must be the same

#if 0
    // Display some stuff
    displaySegmentation(segmentationSurfels, points, "recoSurfels.svg");
    displaySegmentation(segmentation, points, "reco.svg");
#endif

    return 0;
}
