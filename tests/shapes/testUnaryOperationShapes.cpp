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
 * @file testUnaryOperationShapes.cpp
 * @ingroup Tests
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/28
 *
 * Functions for testing class UnaryOperationShapes.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

/////// UnaryOperators
#include "DGtal/shapes/DigitalShapesAdapter.h"
#include "DGtal/shapes/EuclideanShapesAdapter.h"

/////// Shapes 2D
#include "DGtal/shapes/parametric/Ball2D.h"

/////// Render 2D
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"

/////// Digitalization
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/DigitalSurface.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class UnaryOperationShapes.
///////////////////////////////////////////////////////////////////////////////

int minimalistEuclideanShapesTests2D()
{
  typedef Ball2D< Z2i::Space > ShapeA;
  typedef typename ShapeA::Space::Point Point;
  typedef typename ShapeA::Space::RealPoint RealPoint;

  ShapeA shapeA(-2.5, 0.0, 2.5);
  ShapeA shapeB(2.5, 0.0, 2.5);

  typedef EuclideanShapesUnion< ShapeA, ShapeA > Union;
  Union s_union ( shapeA, shapeB );

  ShapeA shapeC(0.0, 0.0, 2.5);

  typedef EuclideanShapesIntersection< Union, ShapeA > Intersection;
  Intersection s_intersec ( s_union, shapeC );

  typedef EuclideanShapesMinus< Union, ShapeA > Minus;
  Minus s_minus ( s_union, shapeC );

  std::cout << "==== Union tests ====" << std::endl;
  std::cout << "NO " << s_union.isInside( RealPoint(-5.5, 0.0) ) << std::endl;
  std::cout << "YES (M) " << s_union.isInside( RealPoint(0.0, 0.0) ) << std::endl;
  std::cout << "NO " << s_union.isInside( RealPoint(5.5, 0.0) ) << std::endl;
  std::cout << "YES " << s_union.isInside( RealPoint(-5.0, 0.0) ) << std::endl;

  std::cout << "==== Intersection tests ====" << std::endl;
  std::cout << "NO " << s_intersec.isInside( RealPoint(-5.0, 0.0) ) << std::endl;
  std::cout << "YES (M) " << s_intersec.isInside( RealPoint(2.5, 0.0) ) << std::endl;
  std::cout << "NO " << s_intersec.isInside( RealPoint(0.0, 2.0) ) << std::endl;
  std::cout << "YES " << s_intersec.isInside( RealPoint(-1.0, 0.0) ) << std::endl;

  std::cout << "==== Minus tests ====" << std::endl;
  std::cout << "YES (M) " << s_minus.isInside( RealPoint(-5.0, 0.0) ) << std::endl;
  std::cout << "NO (M) " << s_minus.isInside( RealPoint(2.5, 0.0) ) << std::endl;
  std::cout << "NO " << s_minus.isInside( RealPoint(0.0, 2.0) ) << std::endl;
  std::cout << "YES " << s_minus.isInside( RealPoint(-4.0, 0.0) ) << std::endl;

  std::cout << "==== Digitalization ====" << std::endl;
  typedef Minus CurrentShape;
  typedef typename CurrentShape::Space Space;
  typedef typename Space::Integer Integer;
  typedef typename Space::Vector Vector;
  typedef KhalimskySpaceND< Space::dimension, Integer > KSpace;
  typedef typename KSpace::SCell SCell;
  typedef HyperRectDomain<Space> Domain;
  typedef GaussDigitizer< Space, CurrentShape > Digitizer;
  typedef LightImplicitDigitalSurface< KSpace, Digitizer > LightImplicitDigSurface;
  typedef DigitalSurface< LightImplicitDigSurface > MyDigitalSurface;
  typedef typename LightImplicitDigSurface::SurfelConstIterator ConstIterator;

  Digitizer dig;
  dig.attach( s_minus );
  dig.init( s_minus.getLowerBound(), s_minus.getUpperBound(), 0.1 );
  Domain domain = dig.getDomain();
  KSpace K;
  bool ok = K.init( dig.getLowerBound(), dig.getUpperBound(), true );
  if ( ! ok )
  {
    std::cerr << "[testUnaryOperationShapes]"
              << " error in creating KSpace." << std::endl;
    return false;
  }

  SurfelAdjacency<KSpace::dimension> SAdj( true );

  SCell bel = Surfaces<KSpace>::findABel( K, dig, 10000 );
  LightImplicitDigSurface LightImplDigSurf( K, dig, SAdj, bel );
  MyDigitalSurface digSurf( LightImplDigSurf );



  std::cout << "==== Render ====" << std::endl;
  Board2D board;
  board << SetMode( domain.className(), "Paving" ) << domain;

  for ( ConstIterator itbegin = digSurf.begin(), itend = digSurf.end(), it = itbegin; it != itend; ++it )
  {
    ////////////////////
    board << CustomStyle( (*it).className(),
                          new CustomColors( Color( 0, 200, 0 ),
                                            Color( 100, 255, 100 ) ) )
          << *it;
  }

  board << CustomStyle( "olol",
                        new CustomColors( Color( 200, 0, 0 ),
                                          Color( 255, 100, 100 ) ) )
        << Point(0,0);

  board.saveSVG("testUnaryOparationShapes-s_minus.svg");

  return 42;
}





int minimalistDigitalShapesTests2D()
{
  //DigitalShapesUnion s_union ();

  return 42;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( )//int argc, char** argv )
{
  trace.beginBlock ( "Testing class UnaryOperationShapes" );

  minimalistEuclideanShapesTests2D();

  minimalistDigitalShapesTests2D();
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
