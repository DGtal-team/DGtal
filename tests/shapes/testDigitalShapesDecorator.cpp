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
 * @file testDigitalShapesDecorator.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/12/07
 *
 * Functions for testing class DigitalShapesDecorator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/GaussDigitizer.h"

/////// UnaryOperators
#include "DGtal/shapes/DigitalShapesDecorator.h"

/////// Shapes 2D
#include "DGtal/shapes/parametric/Ball2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalShapesDecorator.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDigitalShapesDecorator()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing Unary operation on digital shapes ..." );

  typedef Ball2D< Z2i::Space > ShapeA;
  typedef GaussDigitizer< Z2i::Space, ShapeA > MyGaussDigitizerA;

  typedef ShapeA::Point Point;

  double h = 0.5;

  ShapeA shapeA(-2.501, 0.0, 2.5013);
  ShapeA shapeB(2, 0.0, 2.5013);

  MyGaussDigitizerA digShapeA;
  digShapeA.attach( shapeA );
  digShapeA.init( shapeA.getLowerBound(), shapeA.getUpperBound(), h );

  MyGaussDigitizerA digShapeB;
  digShapeB.attach( shapeB );
  digShapeB.init( shapeB.getLowerBound(), shapeB.getUpperBound(), h );

  typedef DigitalShapesUnion< MyGaussDigitizerA, MyGaussDigitizerA > Union;
  Union s_union ( digShapeA, digShapeB );

  ShapeA shapeC(0.0, 0.0, 2.5);
  MyGaussDigitizerA digShapeC;
  digShapeC.attach( shapeC );
  digShapeC.init( shapeC.getLowerBound(), shapeC.getUpperBound(), h );

  typedef DigitalShapesIntersection< Union, MyGaussDigitizerA > Intersection;
  Intersection s_intersec ( s_union, digShapeC );

  typedef DigitalShapesMinus< MyGaussDigitizerA, MyGaussDigitizerA > Minus;
  Minus s_minus ( digShapeA, digShapeC );

  nbok += s_union( Point( -12, 0 )) ? 0 : 1;
  nbok += s_union( Point( 0, 0 )) ? 1 : 0;
  nbok += s_union( Point( 10, 0 )) ? 0 : 1;
  nbok += s_union( Point( 9, 0 )) ? 1 : 0;
  nbok += s_union( Point( -10, 0 )) ? 1 : 0;



  nbok += s_intersec( Point( -6, 0 )) ? 0 : 1;
  nbok += s_intersec( Point( 4, 0 )) ? 1 : 0;
  nbok += s_intersec( Point( 6, 0 )) ? 0 : 1;
  nbok += s_intersec( Point( 0, 5 )) ? 0 : 1;



  nbok += s_minus( Point( -9, 0 )) ? 1 : 0;
  nbok += s_minus( Point( -2, 0 )) ? 0 : 1;
  nbok += s_minus( Point( 0, 2 )) ? 0 : 1;
  nbok += s_minus( Point( -8, 0 )) ? 1 : 0;

  nb = 13;


  trace.info() << "(" << nbok << "/" << nb << ") "
               << "true == true" << std::endl;
  trace.endBlock();
  return  nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DigitalShapesDecorator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << std::endl;

  bool res = testDigitalShapesDecorator(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
