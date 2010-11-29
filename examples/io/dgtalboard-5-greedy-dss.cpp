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
 * @file dgtalboard-5-greedy-dss.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/26
 *
 * An example file named dgtalboard-5-greedy-dss.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <sstream>
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/GreedyDecomposition.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example dgtalboard-5-greedy-dss" );

  typedef ArithmeticalDSS<StandardBase<int> > DSS;
  typedef FreemanChain<int> ContourType; 
  typedef GreedyDecomposition< ContourType, DSS > Decomposition;
  // A Freeman chain code is a string composed by the coordinates of the first pixel, and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "31 16 11121212121212212121212212122122222322323233323333333323333323303330330030300000100010010010001000101010101111" << endl;
  
  // Construct the Freeman chain
  ContourType theContour( ss );
  //Segmentation
  Decomposition theDecomposition( theContour );
  Point p1( 0, 0 );
  Point p2( 31, 31 );
  Domain domain( p1, p2 );
  DGtalBoard aBoard;
  aBoard << SetMode( domain.styleName(), "Grid" )
	 << domain
	 << theContour;
  //for each segment
  DSS segment;
  aBoard << SetMode( segment.styleName(), "BoundingBox" );
  string styleName = segment.styleName() + "/BoundingBox";
  for ( Decomposition::ConstIterator i = theDecomposition.begin();
	i != theDecomposition.end(); ++i ) 
    {
      segment = *i;
      aBoard << CustomStyle( styleName, 
			     new CustomPenColor( DGtalBoard::Color::Blue ) )
	     << segment; // draw each segment
    } 
  aBoard.saveSVG("dgtalboard-5-greedy-dss.svg");
  aBoard.saveSVG("dgtalboard-5-greedy-dss.eps");

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
