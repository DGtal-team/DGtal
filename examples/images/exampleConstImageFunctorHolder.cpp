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
 * @date 2018/07/18
 *
 * @brief An example file for @ref DGtal::functors::ConstImageFunctorHolder
 *
 * This file is part of the DGtal library.
 */


//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/boards/Board2D.h"

#include <cmath>
#include <DGtal/io/colormaps/HueShadeColorMap.h>

//! [include]
#include "DGtal/images/ConstImageFunctorHolder.h"
//! [include]

int main()
{
  using namespace DGtal::Z2i;

  DGtal::Board2D aBoard;

  //! [def]
  using Value = double; // value type  of the image
  using HueShadeDouble = DGtal::HueShadeColorMap<Value>;  // a simple HueShadeColorMap varying on 'double' values
  //! [def]

  // Image from a unary lambda
  //! [example1]
  const Domain domain1(Point(1,1), Point(16,16));

  auto image1 = DGtal::functors::holdConstImageFunctor(
      domain1,
      [] (Point const& pt) { return 25 * ( std::cos( (pt - Point(4,4)).norm() ) + 1 ); }
  );
  //! [example1]

  aBoard.clear();
  DGtal::Display2DFactory::drawImage<HueShadeDouble>(aBoard, image1, 0, 225);
  aBoard.saveSVG("ConstImageFunctorHolder_example1.svg");

  // Image from a binary lambda
  //! [example2]
  const Domain domain2(Point(-1,1), Point(18,18));

  auto image2 = DGtal::functors::holdConstImageFunctor(
      domain2,
      [] (Point const& pt, Domain const& d) { // we could also capture the domain
        return 2 * std::min( ( pt - d.lowerBound() ).norm(), ( pt - d.upperBound() ).norm() );
      }
  );
  //! [example2]

  aBoard.clear();
  DGtal::Display2DFactory::drawImage<HueShadeDouble>(aBoard, image2, 0, 225);
  aBoard.saveSVG("ConstImageFunctorHolder_example2.svg");

  // Image that depends on other images
  //! [example3]
  auto image3 = DGtal::functors::holdConstImageFunctor(
      domain1,
      [&image1, &image2] (Point const& pt) { return image1(pt) + image2(pt); }
  );
  //! [example3]

  aBoard.clear();
  DGtal::Display2DFactory::drawImage<HueShadeDouble>(aBoard, image3, 0, 225);
  aBoard.saveSVG("ConstImageFunctorHolder_example3.svg");
  return 0;
}
