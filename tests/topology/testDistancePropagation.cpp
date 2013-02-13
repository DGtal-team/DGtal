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
 * @file testDistancePropagation.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2012/06/18
 *
 * Functions for testing objects as graph.
 *
 * This file is part of the DGtal library.
 */
 
 ///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Lambda2To1.h"
#include "DGtal/kernel/CanonicEmbedder.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/graph/CUndirectedSimpleGraph.h"
#include "DGtal/graph/DistanceVisitor.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/shapes/Shapes.h"
///////////////////////////////////////////////////////////////////////////////


using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing objects as graph.
///////////////////////////////////////////////////////////////////////////////

typedef ImageSelector < Z2i::Domain, int>::Type Image;
void testDistancePropagation()
{
  typedef Z2i::Space Space;
  typedef Z2i::Point Point;
  typedef Z2i::Domain Domain;
  typedef Z2i::DigitalSet DigitalSet;
  typedef Z2i::Object4_8 Object;
  
  BOOST_CONCEPT_ASSERT(( CUndirectedSimpleGraph<Z2i::Object4_8> ));
  
  Point p1( -50, -50 );
  Point p2( 50, 50 );
  Domain domain( p1, p2 );
  Point c1( -2, -1 );
  Point c2( -14, 5 );
  Point c3( -30, -15 );
  Point c4( -10, -20 );
  Point c5( 12, -1 );
  DigitalSet shape_set( domain );
  
  Shapes<Domain>::addNorm2Ball( shape_set, c1, 9 );
  Shapes<Domain>::addNorm1Ball( shape_set, c2, 9 );
  Shapes<Domain>::addNorm1Ball( shape_set, c3, 10 );
  Shapes<Domain>::addNorm2Ball( shape_set, c4, 12 );
  Shapes<Domain>::addNorm1Ball( shape_set, c5, 4 );

  Object obj(Z2i::dt4_8, shape_set);
  
  
  GradientColorMap<int> cmap_grad( 0, 25);
  cmap_grad.addColor( Color( 0, 0, 255 ) );
  cmap_grad.addColor( Color( 0, 255, 0 ) );
  cmap_grad.addColor( Color( 255, 0, 0 ) );
  
  Board2D board;
  board << SetMode( domain.className(), "Paving" )
        << domain
        << SetMode( p1.className(), "Paving" );
  
  Image image = ImageFromSet<Image>::create(shape_set, 1);
  
  // Type definitions
  typedef CanonicEmbedder<Space> VertexEmbedder;
  typedef VertexEmbedder::Value RealPoint;
  typedef RealPoint::Coordinate Scalar;
  typedef ExactPredicateLpSeparableMetric<Space,2> Distance;
  typedef std::binder1st< Distance > DistanceToPoint; 
  typedef Composer<VertexEmbedder, DistanceToPoint, Scalar> VertexFunctor;
  typedef DistanceVisitor< Object, VertexFunctor, std::set<Point> > Visitor;

  VertexEmbedder embedder;
  Distance distance;
  DistanceToPoint distanceToPoint = std::bind1st( distance, embedder( c1 ) );
  VertexFunctor vfunctor( embedder, distanceToPoint );
  Visitor visitor( obj, vfunctor, c1 );
  
  while( ! visitor.finished() )
    {
      Scalar v = visitor.current().second;
      image.setValue( visitor.current().first, v ); 
      visitor.expand();
    }
  
  string specificStyle = p1.className() + "/Paving";
  
  for ( DigitalSet::ConstIterator it = shape_set.begin();
        it != shape_set.end();
        ++it )
    {
      if( image(*it) == 0)
        board << CustomStyle( specificStyle,
                              new CustomColors( Color::Black,
                                                Color::Red ) );
      else if( image(*it) > 0 )
        board << CustomStyle( specificStyle,
                              new CustomColors( Color::Black,
                                                cmap_grad( image(*it) ) ) );
      else
        board << CustomStyle( specificStyle,
                              new CustomColors( Color::Black,
                                                cmap_grad( 0 ) ) );
      board << *it;
    }
  
  board.saveEPS("testDistancePropagation.eps");
}

int main( int /*argc*/, char** /*argv*/ )
{
  testDistancePropagation();
  return 0;
}




