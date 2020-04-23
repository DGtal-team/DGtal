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
 * @author Roland Denis (\c denis@math.univ-lyon1.fr )
 * CNRS, Institut Camille Jordan (UMR 5208), Université Lyon 1, France
 *
 * @date 2018/11/23
 *
 * @brief An example file for @ref DGtal::functors::PointFunctorHolder.
 *
 * This file is part of the DGtal library.
 */


//////////////////////////////////////////////////////////////////////////////
// Inclusions

//! [examplePointFunctorHolder-includes]
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/kernel/PointFunctorHolder.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"

#include <boost/algorithm/minmax_element.hpp>
//! [examplePointFunctorHolder-includes]

///////////////////////////////////////////////////////////////////////////////
using namespace DGtal;

int main()
{
  //! [examplePointFunctorHolder-hold]
  auto mickey = functors::holdPointFunctor<Z2i::Point>(
    [] ( Z2i::Point const& pt ) {
      return
           (pt - Z2i::Point(0, -5)).norm() > 3
        && (    pt.norm() <= 20
             || (pt - Z2i::Point(-18,21)).norm() <= 10
             || (pt - Z2i::Point( 18,21)).norm() <= 10
           );
    });
  //! [examplePointFunctorHolder-hold]

  //! [examplePointFunctorHolder-DT]
  Z2i::Domain domain( Z2i::Point(-35,-25), Z2i::Point(35, 35) );

  using DTL2 = DistanceTransformation<Z2i::Space, decltype(mickey), Z2i::L2Metric>;

  DTL2 dt( domain, mickey, Z2i::l2Metric );
  //! [examplePointFunctorHolder-DT]


  //! [examplePointFunctorHolder-DTvis]
  DTL2::Value maxDT = *boost::first_max_element( dt.constRange().begin(), dt.constRange().end() );
  using HueTwice = HueShadeColorMap<DTL2::Value, 1>;

  Board2D aBoard;
  aBoard << domain;
  Display2DFactory::drawImage<HueTwice>(aBoard, dt, 0, maxDT);
  aBoard.saveEPS("examplePointFunctorHolder.eps");
  //! [examplePointFunctorHolder-DTvis]

  return 0;

}
