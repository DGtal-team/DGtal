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
 * @date 2020/12/04
 *
 * An example file that shows how to construct and use a DGtal::PlaneProbingTetrahedronEstimator on an analytical digital plane.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/DigitalPlanePredicate.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

using Integer = int;
using Space   = SpaceND<3, Integer>;
using Point   = Space::Vector;
using Vector  = Space::Point;

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
  //! [PlaneProbingTetrahedronEstimatorConstruction]
  // The general form is ProbingEstimator<Predicate, mode> where
  // - Predicate is a model of concepts::PointPredicate, see DigitalPlanePredicate or DigitalSurfacePredicate for instance,
  // - mode specifies the candidate set, it is one of { ProbingMode::H, ProbingMode::R, ProbingMode::R1, ProbingMode::L }.
  using DigitalPlane = DigitalPlanePredicate<Space>;
  using Estimator    = PlaneProbingTetrahedronEstimator<DigitalPlane, ProbingMode::R1>;

  // We start by constructing the predicate, here a standard digital plane of normal (2, 6, 15)
  Vector n(2, 6, 15);
  DigitalPlane plane(n, 0, n.norm1());

  // Instantiation: estimator(startingPoint, initialFrame, predicate) where
  // (startingPoint, initialFrame) describes the initial tetrahedron.
  Point o(0, 0, 0);
  std::array<Point, 3> m = { Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1) };
  Estimator estimator(o, m, plane);
  //! [PlaneProbingTetrahedronEstimatorConstruction]

  //! [PlaneProbingTetrahedronEstimatorUsage]
  int it = 0;
  while (estimator.advance().first) {
      it++;

      // You can examine the current configuration of the H-neighborhood, using PlaneProbingTetrahedronEstimator::hexagonState
      auto state = estimator.hexagonState();
      if (state == Estimator::Neighborhood::HexagonState::Planar) {
          std::cout << "Planar" << std::endl;
      } else if (state == Estimator::Neighborhood::HexagonState::Empty) {
          std::cout << "Empty" << std::endl;
      } else if (state == Estimator::Neighborhood::HexagonState::NonPlanar) {
          std::cout << "NonPlanar" << std::endl;
      } else if (state == Estimator::Neighborhood::HexagonState::NonConvex) {
          std::cout << "NonConvex" << std::endl;
      }

      // Here, we display the current frame (the vectors m_k) and the current estimation
      std::clog << "it = " << it << " "
          << estimator.m(0) << " " << estimator.m(1) << " " << estimator.m(2) << " "
          << estimator.getNormal() << std::endl;
  }

  // This loop can also be reduced to:
  // Point n = estimator.compute()
  //! [PlaneProbingTetrahedronEstimatorUsage]

  ASSERT(estimator.getNormal() == n);

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
