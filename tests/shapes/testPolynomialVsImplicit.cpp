/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testPolynomial.cpp
 * @ingroup tests
 * @author Anis Benyoub (anis.benyoub@insa-lyon.fr)
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/06/21
 *
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <QtGui/QApplication>
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/shapes/implicit/ImplicitFunctionDiff1LinearCellEmbedder.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/topology/SCellsFunctors.h"
#include "DGtal/topology/helpers/BoundaryPredicate.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include <boost/math/special_functions/fpclassify.hpp>

#include "DGtal/images/ImageHelper.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/shapes/implicit/ImplicitHyperCube.h"
#include "DGtal/shapes/EuclideanShapesDecorator.h"
#include "DGtal/shapes/Shapes.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////


  typedef Space::RealPoint RealPoint;
  typedef RealPoint::Coordinate Ring;
  typedef MPolynomial<3, Ring> Polynomial3;
  typedef MPolynomialReader<3, Ring> Polynomial3Reader;
  typedef ImplicitPolynomial3Shape<Space> ImplPolyBall;
  typedef ImplicitHyperCube< Space > ImplCube;
  typedef ImplicitBall< Space > ImplBall;
  typedef EuclideanShapesMinus< ImplBall, ImplPolyBall > MinusOperatorBall;
  typedef EuclideanShapesMinus< MinusOperatorBall, ImplCube > MinusOperatorPrevBall;
  typedef GaussDigitizer<Space, MinusOperatorPrevBall> DigitalShape;
  typedef DigitalShape::PointEmbedder DigitalEmbedder;

template < typename TKSpace, typename TShape >
class NearestPointEmbedder
{
public:
  typedef TKSpace KSpace;
  typedef TShape Shape;
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::Space Space;
  typedef typename Space::RealPoint RealPoint;
  typedef Cell Argument;
  typedef RealPoint Value;

  NearestPointEmbedder()
    :shape(0)
  {}

  ~NearestPointEmbedder(){}

  void init( const KSpace & K, const double h, const Shape & s )
  {
    k = K;
    step = h;
    shape = &s;
  }

  RealPoint operator()( const Cell & cell ) const
  {
    Point B = k.uKCoords( cell );
    RealPoint A( B[ 0 ], B[1], B[2] );
    A /= 2.0;
    A *= step;
    A = shape->nearestPoint( A, 0.01 * step, 200, 0.1 * step );
    return A;
  }

protected:
  KSpace k;
  double step;
  const Shape* shape;
};


int main( int argc, char** argv )
{

  double p1[ 3 ] = {-5.0, -5.0, -5.0};
  double p2[ 3 ] = {5.0, 5.0, 5.0};

  double step = 0.1;


  Polynomial3 P;
  Polynomial3Reader reader;
  std::string poly_str = "x^2 + y^2 + z^2 - 25.0";
  std::string::const_iterator iter
      = reader.read( P, poly_str.begin(), poly_str.end() );
  if ( iter != poly_str.end() )
  {
    std::cerr << "ERROR: I read only <"
              << poly_str.substr( 0, iter - poly_str.begin() )
              << ">, and I built P=" << P << std::endl;
    return 1;
  }


  ImplPolyBall ipshape( P );

  ImplCube icube( RealPoint( 5.0, 0.0, 0.0 ), 5.0 );

  ImplBall ishape( RealPoint( 0.0, 0.0, 0.0 ), 5.0 );

  MinusOperatorBall minus( ishape, ipshape );

  MinusOperatorPrevBall minusprev( minus, icube );

  std::cout << ipshape.orientation(RealPoint(0.0,0.0,0.0)) << std::endl;
  std::cout << ipshape.orientation(RealPoint(4.9,4.9,4.9)) << std::endl;


  DigitalShape dshape;
  dshape.attach( minusprev );
  dshape.init( RealPoint( p1 ), RealPoint( p2 ), step );
  Domain domain = dshape.getDomain();

  typedef typename DigitalSetSelector< Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type MySet;
  MySet set( domain );
  Shapes< Domain >::digitalShaper( set, dshape );

  QApplication application( argc, argv );
  Viewer3D viewer;
  viewer.show();
  viewer << SetMode3D( domain.className(), "BoundingBox" ) << domain;
  viewer << set;


  viewer << Viewer3D::updateDisplay;

  return application.exec();
}
