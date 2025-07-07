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
 * An example file that shows how to construct and use a PlaneProbingDigitalSurfaceLocalEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <unordered_map>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingParallelepipedEstimator.h"
#include "DGtal/geometry/surfaces/DigitalSurfacePredicate.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingDigitalSurfaceLocalEstimator.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

using KSpace           = Z3i::KSpace;
using SH3              = Shortcuts<KSpace>;
using SHG3             = ShortcutsGeometry<KSpace>;
using Surface          = SH3::DigitalSurface;
using Surfel           = SH3::Surfel;
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
    std::string volfile = (argc > 1) ? argv[1] : (examplesPath + "samples/cat10.vol");

    auto params    = SH3::defaultParameters() | SHG3::defaultParameters();
    auto bimage    = SH3::makeBinaryImage(volfile, params);
    auto K         = SH3::getKSpace(bimage, params);
    auto surface   = SH3::makeDigitalSurface(bimage, K, params);
    auto surfels   = SH3::getSurfelRange(surface);

    Integer bound = params["maxAABB"].as<Integer>();
    double gridstep = params["gridstep"].as<double>();

    PolyscopeViewer<> viewer(K);

    //! [PlaneProbingDigitalSurfaceLocalEstimatorConstruction]
    using SurfacePredicate = DigitalSurfacePredicate<Surface>;
    using ProbingAlgorithm = PlaneProbingParallelepipedEstimator<SurfacePredicate, ProbingMode::R1>;
    // The general form is PlaneProbingDigitalSurfaceLocalEstimator<SurfaceType, ProbingAlgorithm>
    using Estimator        = PlaneProbingDigitalSurfaceLocalEstimator<Surface, ProbingAlgorithm>;

    // Parameters of the estimator:
    // - the probing factory
    Estimator::ProbingFactory probingFactory = [&bound](const Estimator::ProbingFrame& frame, const SurfacePredicate& surfacePredicate) {
        // If the base estimator is a PlaneProbingTetrahedronEstimator
        // return new ProbingAlgorithm(frame.p, { frame.b1, frame.b2, frame.normal }, surfacePredicate);

        // For a PlaneProbingParallelepipedEstimator
        return new ProbingAlgorithm(frame.p, { frame.b1, frame.b2, frame.normal }, surfacePredicate, bound);
    };

    // - an optional hashmap of pre-estimations
    std::unordered_map<Surfel, RealPoint> preEstimations;
    // The user can provide the pre-estimation
    // auto preEstimationsVector = SHG3::getCTrivialNormalVectors(surface, surfels, params);
    // for (std::size_t i = 0; i < surfels.size(); ++i)
    // {
    //     preEstimations[surfels[i]] = preEstimationsVector[i];
    // }
    // Or if it is not given, it is implicitly done inside the Estimator::eval function (using the MaximalSegmentSliceEstimation estimator)

    // - a verbosity flag
    bool verbose = true;

    Estimator estimator(surface, probingFactory, preEstimations, verbose);
    estimator.init(gridstep, surfels.begin(), surfels.end());
    //! [PlaneProbingDigitalSurfaceLocalEstimatorConstruction]

    //! [PlaneProbingDigitalSurfaceLocalEstimatorUsage]
    // Evaluation on a range of surfels
    std::vector<Estimator::Quantity> quantities;
    estimator.eval(surfels.begin(), surfels.end(), std::back_inserter(quantities));

    // Or on one surfel 's'
    // Estimator::Quantity q = estiamtor.eval(s);
    //! [PlaneProbingDigitalSurfaceLocalEstimatorUsage]
    Color fillColor = Color::White;
    int i = 0;
    for (auto it = surfels.begin(); it != surfels.end(); ++it, ++i)
    {
        const Surfel& s = *it;
        const Estimator::Quantity& n = quantities[i];

        RealPoint origin = centerSurfel(K, s);

        viewer.drawColor(fillColor);
        viewer << s;

        // Pre-estimation in red
        RealPoint const& preEstimation = estimator.getPreEstimation(it);
        viewer.drawColor(Color::Red);
        viewer.drawLine(origin, origin + 1.5 * preEstimation.getNormalized());

        // Estimated normal in green;
        viewer.drawColor(Color::Green);
        viewer.drawLine(origin, origin + 1.5 * n.getNormalized());
    }

    viewer.show();
    return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
