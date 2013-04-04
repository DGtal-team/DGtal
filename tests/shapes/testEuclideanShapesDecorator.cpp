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
 * @file testEuclideanShapesDecorator.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/12/07
 *
 * Functions for testing class EuclideanShapesDecorator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

/////// UnaryOperators
#include "DGtal/shapes/EuclideanShapesDecorator.h"

/////// Shapes 2D
#include "DGtal/shapes/parametric/Ball2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class EuclideanShapesDecorator.
///////////////////////////////////////////////////////////////////////////////

bool testEuclideanShapesDecorator()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing Unary operation on euclidean shapes ..." );

  typedef Ball2D< Z2i::Space > ShapeA;
  typedef ShapeA::RealPoint RealPoint;

  ShapeA shapeA(-2.501, 0.0, 2.5013);
  ShapeA shapeB(2, 0.0, 2.5013);

  typedef EuclideanShapesUnion< ShapeA, ShapeA > Union;
  Union s_union ( shapeA, shapeB );

  ShapeA shapeC(0.0, 0.0, 2.5);

  typedef EuclideanShapesIntersection< Union, ShapeA > Intersection;
  Intersection s_intersec ( s_union, shapeC );

  typedef EuclideanShapesMinus< ShapeA, ShapeA > Minus;
  Minus s_minus ( shapeA, shapeC );

  nbok += s_union.isInside( RealPoint( -5.1, 0.0 )) ? 0 : 1;
  nbok += s_union.isInside( RealPoint( 0.0, 0.0 )) ? 1 : 0;
  nbok += s_union.isInside( RealPoint( 4.6, 0.0 )) ? 0 : 1;
  nbok += s_union.isInside( RealPoint( 4.5, 0.0 )) ? 1 : 0;
  nbok += s_union.isInside( RealPoint( -5.0, 0.0 )) ? 1 : 0;



  nbok += s_intersec.isInside( RealPoint( -2.6, 0.0 )) ? 0 : 1;
  nbok += s_intersec.isInside( RealPoint( 2.5, 0.0 )) ? 1 : 0;
  nbok += s_intersec.isInside( RealPoint( 2.6, 0.0 )) ? 0 : 1;
  nbok += s_intersec.isInside( RealPoint( 0.0, 2.5 )) ? 0 : 1;



  nbok += s_minus.isInside( RealPoint( -5.0, 0.0 )) ? 1 : 0;
  nbok += s_minus.isInside( RealPoint( -1.0, 0.0 )) ? 0 : 1;
  nbok += s_minus.isInside( RealPoint( 0.0, 2.0 )) ? 0 : 1;
  nbok += s_minus.isInside( RealPoint( -2.6, 0.0 )) ? 1 : 0;

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
  trace.beginBlock ( "Testing class EuclideanShapesDecorator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << std::endl;

  bool res = testEuclideanShapesDecorator(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
