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
 * @ingroup Examples
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/07
 *
 * An example file named examplePlaneProbingSurfaceLocalEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingParallelepipedEstimator.h"
#include "DGtal/geometry/surfaces/DigitalSurfacePredicate.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingDigitalSurfaceLocalEstimator.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/io/viewers/Viewer3D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

using KSpace           = Z3i::KSpace;
using SH3              = Shortcuts<KSpace>;
using SHG3             = ShortcutsGeometry<KSpace>;
using Surface          = SH3::DigitalSurface;
using Surfel           = SH3::Surfel;
using SurfacePredicate = DigitalSurfacePredicate<Surface>;
using ProbingAlgorithm = PlaneProbingTetrahedronEstimatorParallelepiped<SurfacePredicate, ProbingMode::R1>;
// using ProbingAlgorithm = PlaneProbingParallelepipedEstimator<SurfacePredicate, ProbingMode::H>;
using Estimator        = PlaneProbingDigitalSurfaceLocalEstimator<Surface, ProbingAlgorithm>;
using ProbingFactory   = Estimator::ProbingFactory;
using Quantity         = Estimator::Quantity;
using Point            = SH3::Point;
using RealPoint        = SH3::RealPoint;
using Integer          = Point::Coordinate;

RealPoint centerSurfel (KSpace const& K, SH3::SCell const& s)
{
    auto pointels = SH3::getPrimalVertices(K, s, true);

    Point p = K.uCoords(pointels[0]),
          u = K.uCoords(pointels[1]) - p,
          v = K.uCoords(pointels[3]) - p;

    static const RealPoint shift(-0.5, -0.5, -0.5);

    return p + 0.5 * u + 0.5 * v + shift;
}

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    QApplication application(argc, argv);

    std::string volfile = (argc > 1) ? argv[1] : (examplesPath + "samples/Al.100.vol");

    auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
    auto bimage    = SH3::makeBinaryImage(volfile, params);
    auto K         = SH3::getKSpace(bimage, params);
    auto surface   = SH3::makeDigitalSurface(bimage, K, params);
    auto surfels   = SH3::getSurfelRange(surface);

    Integer bound = params["maxAABB"].as<Integer>();
    double gridstep = params["gridstep"].as<double>();

    Viewer3D<> viewer(K);
    viewer << SetMode3D(Surfel().className(), "Basic");
    viewer.show();

    // Parameters
    ProbingFactory probingFactory = [&bound](const auto& frame, const auto& surfacePredicate) {
        // Tetrahedron-based estimator
        return new ProbingAlgorithm(frame.p, { frame.b1, frame.b2, frame.normal }, surfacePredicate);

        // Parallelepiped-based estimator
        // return new ProbingAlgorithm(frame.p, { frame.b1, frame.b2, frame.normal }, surfacePredicate, bound);
    };

    bool verbose = true;

    std::map<Surfel, RealPoint> preEstimations;
    // The user can provide the pre-estimation
    // auto preEstimationsVector = SHG3::getCTrivialNormalVectors(surface, surfels, params);
    // for (std::size_t i = 0; i < surfels.size(); ++i)
    // {
    //     preEstimations[surfels[i]] = preEstimationsVector[i];
    // }
    // Or it is directly done inside the Estimator::eval function

    Estimator estimator;
    estimator.init(gridstep, surfels.begin(), surfels.end());
    estimator.attach(surface);
    estimator.setParams(probingFactory, preEstimations, verbose);

    std::vector<Quantity> quantities;
    estimator.eval(surfels.begin(), surfels.end(), std::back_inserter(quantities));

    Color fillColor = viewer.getFillColor();

    for (std::size_t i = 0; i < surfels.size(); ++i)
    {
        const Surfel& s = surfels[i];
        const Quantity& n = quantities[i];

        RealPoint origin = centerSurfel(K, s);

        viewer.setFillColor(fillColor);
        viewer << s;

        // Pre-estimation in red
        RealPoint const& preEstimation = estimator.getPreEstimation(s);
        viewer.setLineColor(Color::Red);
        viewer.addLine(origin, origin + 1.5 * preEstimation.getNormalized(), 0.3);

        // Estimated normal in green;
        viewer.setLineColor(Color::Green);
        viewer.addLine(origin, origin + 1.5 * n.getNormalized(), 0.3);
    }

    viewer << Viewer3D<>::updateDisplay;
    application.exec();

    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
