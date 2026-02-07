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
 * @file geometry/curves/estimation/exampleLMST3D.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2015/09/27
 *
 * An example file named LambdaMST3D.
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows the basic usage of the Lambda maximal segment tangent estimation
   in 3D by the method using only 2D projections.

@see \ref moduleArithDSSReco

\example geometry/curves/estimation/exampleLMST3DBy2D.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iterator>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/helpers/StdDefs.h"

//! [LambdaMST3DBy2DHeader]
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.h"
//! [LambdaMST3DBy2DHeader]
///////////////////////////////////////////////////////////////////////////////


using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  //! [LambdaMST3DBy2DStandardCtor]
  typedef vector < Point > Container;
  typedef Container::const_iterator ConstIterator;
  LambdaMST3DBy2D < ConstIterator > lmst;
  //! [LambdaMST3DBy2DStandardCtor]

  // Input points
  Container contour;
  contour.push_back ( Point ( 18, 25, 18 ) );
  contour.push_back ( Point ( 17, 25, 19 ) );
  contour.push_back ( Point ( 16, 25, 20 ) );
  contour.push_back ( Point ( 15, 25, 21 ) );
  contour.push_back ( Point ( 14, 25, 22 ) );
  contour.push_back ( Point ( 13, 25, 23 ) );
  contour.push_back ( Point ( 12, 25, 24 ) );
  contour.push_back ( Point ( 11, 25, 25 ) );
  contour.push_back ( Point ( 10, 25, 26 ) );
  contour.push_back ( Point ( 9, 25, 27 ) );
  contour.push_back ( Point ( 8, 25, 28 ) );

  lmst.init ( contour.cbegin ( ), contour.cend ( ), LambdaMST3DBy2D < ConstIterator >::MAIN_AXIS::X ) ;
  //! [LambdaMST3DBy2DPoint]
  for ( const auto & p : contour )
    trace.info ( ) << lmst.eval ( p ) << std::endl;
  //! [LambdaMST3DBy2DPoint]

  //! [LambdaMST3DBy2DRange]
  lmst.init ( contour.cbegin ( ), contour.cend ( ), LambdaMST3DBy2D < ConstIterator >::MAIN_AXIS::X );
  vector < RealVector > tangent;
  lmst.eval ( contour.cbegin ( ), contour.cend ( ), back_insert_iterator < vector < RealVector > > ( tangent ) );
  //! [LambdaMST3DBy2DRange]
  for ( const auto & t : tangent )
    trace.info ( ) << t << std::endl;
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
