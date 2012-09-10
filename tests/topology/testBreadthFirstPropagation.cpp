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
 * @file testAdjacency.cpp
 * @ingroup Tests
 * @author Jérémy Gaillard (\c jeremy.gaillard@insa-lyon.fr )
 *
 * @date 2012/06/18
 *
 * Functions for testing objects as graph.
 *
 * This file is part of the DGtal library.
 */
 
 ///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/topology/CUndirectedSimpleGraph.h"
#include "DGtal/topology/BreadthFirstVisitor.h"
#include <set>
#include <iterator>
///////////////////////////////////////////////////////////////////////////////


using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing objects as graph.
///////////////////////////////////////////////////////////////////////////////

typedef ImageSelector < Z2i::Domain, int>::Type Image;
void testBreadthFirstPropagation()
{
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
  
  
  GradientColorMap<int> cmap_grad( 0, 52);
  cmap_grad.addColor( Color( 0, 0, 200 ) );
  cmap_grad.addColor( Color( 0, 0, 50 ) );
  
  Board2D board;
  board << SetMode( domain.className(), "Paving" )
  << domain
  << SetMode( p1.className(), "Paving" );
  
  Image image = ImageFromSet<Image>::create(shape_set, 1);
  
  
  BreadthFirstVisitor<Object, set<Point> > bfv (obj, c1);
  
  
  while( !bfv.finished() )
  {
    image.setValue(bfv.current().first, bfv.current().second);
    bfv.expand();
  }
  
  string specificStyle = p1.className() + "/Paving";
  
  for ( DigitalSet::ConstIterator it = shape_set.begin();
  it != shape_set.end();
  ++it )
  {
    if( image(*it) == 0)
    {
      board << CustomStyle( specificStyle,
        new CustomColors( Color::Black,
        Color::Red ) )
      << *it;
    }
    else
    {
      if( image(*it) > 0 )
      {
	board << CustomStyle( specificStyle,
	  new CustomColors( Color::Black,
	  cmap_grad( image(*it) ) ) )
	<< *it;
      }
      else
      {
	board << CustomStyle( specificStyle,
	  new CustomColors( Color::Black,
	  cmap_grad( 0 ) ) )
	<< *it;
      }
    }
  }
  
  board.saveEPS("testBreadthFirstPropagation.eps");
}

int main( int /*argc*/, char** /*argv*/ )
{
  testBreadthFirstPropagation();
  return 0;
}




