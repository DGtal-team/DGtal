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
#include "DGtal/io/viewers/Viewer3D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

using KSpace           = Z3i::KSpace;
using SH3              = Shortcuts<KSpace>;
using Surface          = SH3::DigitalSurface;
using SurfacePredicate = DigitalSurfacePredicate<Surface>;
using ProbingAlgorithm = PlaneProbingTetrahedronEstimator<SurfacePredicate, ProbingMode::H>;
// using ProbingAlgorithm = PlaneProbingParallelepipedEstimator<SurfacePredicate, ProbingMode::H>;
using Estimator        = PlaneProbingDigitalSurfaceLocalEstimator<Surface, ProbingAlgorithm>;
using Quantity         = Estimator::Quantity;
using Point            = SH3::Point;
using RealPoint        = SH3::RealPoint;

// TODO: ProbingAlgorithm have different constructors (extra bound in Parallelepiped)

RealPoint centerSurfel (KSpace const& K, SH3::SCell const& s)
{
    auto pointels = SH3::getPrimalVertices(K, s, true);

    Point p = K.uCoords(pointels[0]),
          u = K.uCoords(pointels[1]) - p,
          v = K.uCoords(pointels[3]) - p;

    const RealPoint shift(-0.5, -0.5, -0.5);

    return p + 0.5 * u + 0.5 * v + shift;
}

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
    QApplication application(argc, argv);

    std::string volfile = (argc > 1) ? argv[1] : (examplesPath + "samples/Al.100.vol");

    auto params    = SH3::defaultParameters();
    auto bimage    = SH3::makeBinaryImage(volfile, params);
    auto K         = SH3::getKSpace(bimage, params);
    auto surface   = SH3::makeDigitalSurface(bimage, K, params);
    auto surfels   = SH3::getSurfelRange(surface);

    Viewer3D<> viewer(K);
    viewer << SetMode3D(SH3::Surfel().className(), "Basic");
    viewer.show();

    SurfacePredicate surfacePredicate(surface);

    Estimator estimator;
    estimator.init(params["gridstep"].as<double>(), surfels.begin(), surfels.end());
    estimator.attach(surface);
    estimator.setParams(true);

    std::vector<Quantity> quantities;
    estimator.eval(surfels.begin(), surfels.end(), std::back_inserter(quantities));

    Color fillColor = viewer.getFillColor();

    for (std::size_t i = 0; i < surfels.size(); ++i)
    {
        const auto& s = surfels[i];
        const auto& n = quantities[i];

        RealPoint origin = centerSurfel(K, s);

        viewer.setFillColor(fillColor);
        viewer << s;

        // viewer.setFillColor(Color::Red);
        // viewer.addCube(origin, 0.1);

        viewer.setLineColor(Color::Green);
        viewer.addLine(origin, origin + 1.5 * n.getNormalized(), 0.3);
    }

    viewer << Viewer3D<>::updateDisplay;
    application.exec();

    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
