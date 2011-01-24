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
 * @file convex-and-concave-parts.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2011/01/24
 *
 * An example file named convex-and-concave-parts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <sstream>
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/MaximalSegments.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example convex-and-concave-parts" );

  typedef ArithmeticalDSS<int,4> DSS4;
  typedef FreemanChain<int> Contour4; 
  typedef MaximalSegments< Contour4::ConstIterator, DSS4 > MDSSs4;

  // A Freeman chain code is a string composed by the coordinates of the first pixel, 
	//and the list of elementary displacements. 
  std::stringstream ss(stringstream::in | stringstream::out);
  ss << "1 11 030030330303303030300001010101101011010000030330303303030300001010110101011010000033" << endl;
  
  // Construct the Freeman chain
  Contour4 theContour( ss );

  //Maximal Segments
  MDSSs4 theCover( theContour.begin(),theContour.end(),false );
  Point p1( 0, 0 );
  Point p2( 48, 12 );
  Domain domain( p1, p2 );
  DGtalBoard aBoard;
  aBoard << SetMode( domain.styleName(), "Grid" )
	 << domain
	 << SetMode( "PointVector", "Grid" )
	 << theContour;

  //For each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string aStyleName = "ArithmeticalDSS/BoundingBox";
  for ( MDSSs4::ConstIterator i = theCover.begin();
	i != theCover.end(); ++i ) 
    {
			//first and last points
			Contour4::ConstIterator front = i.getFront();
			Contour4::ConstIterator back = i.getBack();	
			//parameters
			DSS4 segment(*i);
			int mu = segment.getMu();
			int omega = segment.getOmega();

			//choose pen color
			CustomPenColor* aPenColor;
			if (back == theContour.begin()) {
				aPenColor = new CustomPenColor( DGtalBoard::Color::Black );
			} else {
				--back;
				//the front of the last segment points at the end
				if (front == theContour.end()) {
					aPenColor = new CustomPenColor( DGtalBoard::Color::Black );
				} else {
					++front;
					if ( (segment.getRemainder(*back)<mu-1)&&
							 (segment.getRemainder(*front)<mu-1) ) {                //concave
						aPenColor = new CustomPenColor( DGtalBoard::Color::Green);
					} else if ( (segment.getRemainder(*back)>mu+omega)&&
									    (segment.getRemainder(*front)>mu+omega) ) {     //convex
						aPenColor = new CustomPenColor( DGtalBoard::Color::Blue );
					} else if ( (segment.getRemainder(*back)>mu+omega)&&
									    (segment.getRemainder(*front)<mu-1) ) {         //convex to concave
						aPenColor = new CustomPenColor( DGtalBoard::Color::Yellow );
					} else if ( (segment.getRemainder(*back)<mu-1)&&
									    (segment.getRemainder(*front)>mu+omega) ) {     //concave to convex
						aPenColor = new CustomPenColor( DGtalBoard::Color::Yellow );
					} else {                                                    //pb
						aPenColor = new CustomPenColor( DGtalBoard::Color::Red );
					}
				}
			}

			 
			// draw each segment
			aBoard << CustomStyle( aStyleName, aPenColor )
						 << segment; 

    } 
  aBoard.saveSVG("convex-and-concave-parts.svg");

  trace.endBlock();

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
