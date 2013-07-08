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
 * @file exampleArithmeticalDSS.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/07/08
 *
 * An example file named exampleArithmeticalDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i; 
///////////////////////////////////////////////////////////////////////////////




///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  trace.beginBlock ( "Example exampleArithmeticalDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  {
    std::string filename = "DSS4.svg";

    //DSS type
    typedef StandardDSS4<DGtal::int32_t> DSS;
    typedef DSS::Point Point; 
  
    DSS theDSS( Point(0,0), Point(8,5) ); 
  
    //! [StandardDSS4DrawingUsage]
    Board2D board;
  
    // Draw the grid
    Domain domain( Point(0,0), Point(8,5) );
    board << SetMode(domain.className(), "Grid")
  	  << domain;    

    // Draw the points of the DSS
    board << SetMode("PointVector", "Grid")
  	  << SetMode(theDSS.className(), "Points") 
  	  << theDSS;

    // Draw the bounding box
    board << SetMode(theDSS.className(), "BoundingBox") 
  	  << theDSS;
  
    board.saveSVG( filename.c_str() );
    //! [StandardDSS4DrawingUsage]
  }

  {
    std::string filename = "DSS4bis.svg";

    //DSS type
    typedef ArithmeticalDSS<DGtal::int32_t, DGtal::int32_t, 4> DSS;
    typedef DSS::Point Point; 
  
    DSS theDSS( Point(0,0), Point(8,5) ); 
  
    //! [ArithmeticalDSSDrawingUsage]
    Board2D board;
  
    // Draw the pixels
    Domain domain( Point(0,0), Point(8,5) );
    board << SetMode(domain.className(), "Paving")
	  << domain;    
  
    //Draw the points of the DSS
    board << SetMode("PointVector", "Both");
    board << SetMode(theDSS.className(), "Points") 
	  << theDSS;

    // Draw the bounding box
    board << SetMode(theDSS.className(), "BoundingBox") 
	  << theDSS;
  
    board.saveSVG( filename.c_str() );
    //! [ArithmeticalDSSDrawingUsage]
  }

  {
    std::string filename = "DSS8.svg";

    //DSS type
    typedef NaiveDSS8<DGtal::int32_t> DSS;
    typedef DSS::Point Point; 
  
    DSS theDSS( Point(0,0), Point(8,5) ); 
  
    //! [NaiveDSS8DrawingUsage]
    Board2D board;
  
    // Draw the pixels
    Domain domain( Point(0,0), Point(8,5) );
    board << SetMode(domain.className(), "Paving")
  	  << domain;    
  
    //Draw the points of the DSS
    board << SetMode("PointVector", "Both");
    board << SetMode(theDSS.className(), "Points") 
  	  << theDSS;

    // Draw the bounding box
    board << SetMode(theDSS.className(), "BoundingBox") 
  	  << theDSS;
  
    board.saveSVG( filename.c_str() );
    //! [NaiveDSS8DrawingUsage]
  }

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
