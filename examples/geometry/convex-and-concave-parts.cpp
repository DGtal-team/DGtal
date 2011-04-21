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
#include <fstream>
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/geometry/2d/MaximalSegments.h"
///////////////////////////////////////////////////////////////////////////////
#include "ConfigExamples.h"

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

typedef FreemanChain<int> Sequence; 
typedef Sequence::ConstIterator Iterator;
typedef ArithmeticalDSS<Iterator,int,4> DSS4Computer;
typedef MaximalSegments<DSS4Computer> Cover;


int main( int argc, char** argv )
{

  trace.beginBlock ( "Example convex-and-concave-parts" );

  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;


	string codes; 
	if (argc >= 2) codes = argv[1];
	else codes = "0300303303033030303000010101011010110100000303303033030303000010101101010110100000333"; 

  stringstream ss(stringstream::in | stringstream::out);
  ss << "0 0 " << codes << endl;
  Sequence theContour( ss );
  
  trace.info() << "FreemanChain: " << ss.str() << endl;

  //Maximal Segments
  DSS4Computer computer;
  Cover theCover( theContour.begin(),theContour.end(),computer,true);

  DGtalBoard aBoard;
  aBoard
	 << SetMode( "PointVector", "Grid" )
	 << theContour;

  //For each segment
  aBoard << SetMode( "ArithmeticalDSS", "BoundingBox" );
  string aStyleName = "ArithmeticalDSS/BoundingBox";
  for ( Cover::SegmentIterator i = theCover.begin();
	i != theCover.end(); ++i ) {

		//segment
	  DSS4Computer segment(*i);

    //choose pen color
    CustomPenColor* aPenColor;

		if ( !(i.intersectNext() && i.intersectPrevious()) ) {

			aPenColor = new CustomPenColor( DGtalBoard::Color::Black );

		} else {
	    //begin and end iterators
	    //(back point on the first point)
	    //(front point after the last point)
	    Iterator front = i.getFront();
	    Iterator back = i.getBack();	
			if (back == theContour.begin()) {
				back = theContour.end();
			} 
			--back;

	    //parameters
	    int mu = segment.getMu();
	    int omega = segment.getOmega();

			//configurations
			if ( (segment.getRemainder(*back)<=mu-1)&&
				   (segment.getRemainder(*front)<=mu-1) ) {                //concave
				aPenColor = new CustomPenColor( DGtalBoard::Color::Green);
			} else if ( (segment.getRemainder(*back)>=mu+omega)&&
					  (segment.getRemainder(*front)>=mu+omega) ) {           //convex
				aPenColor = new CustomPenColor( DGtalBoard::Color::Blue );
			} else if ( (segment.getRemainder(*back)>=mu+omega)&&
					  (segment.getRemainder(*front)<=mu-1) ) {               //convex to concave
				aPenColor = new CustomPenColor( DGtalBoard::Color::Yellow );
			} else if ( (segment.getRemainder(*back)<=mu-1)&&
					  (segment.getRemainder(*front)>=mu+omega) ) {           //concave to convex
				aPenColor = new CustomPenColor( DGtalBoard::Color::Yellow );
			} else {                                                    //pb
				aPenColor = new CustomPenColor( DGtalBoard::Color::Red );
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
