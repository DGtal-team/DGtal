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
 * @file ArithmeticalDSSComputer.cpp
 * @ingroup Examples
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France

 * @date 2010/11/30
 *
 * An example file named ArithmeticalDSSComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

#include "DGtal/geometry/curves/ArithmeticalDSSComputer.h"
#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/helpers/StdDefs.h"

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  

  /////////////////////////// DSS4 //////////////////////////////////
  {
    //! [ArithmeticalDSSComputer4Usage]
    typedef PointVector<2,int> Point;
    typedef std::vector<Point> Range;
    typedef std::vector<Point>::const_iterator ConstIterator;
    typedef ArithmeticalDSSComputer<ConstIterator,int,4> DSSComputer;  

    // Input points
    Range contour;
    contour.push_back(Point(0,0));
    contour.push_back(Point(1,0));
    contour.push_back(Point(1,1));
    contour.push_back(Point(2,1));
    contour.push_back(Point(3,1));
    contour.push_back(Point(3,2));
    contour.push_back(Point(4,2));
    contour.push_back(Point(5,2));
    contour.push_back(Point(6,2));
    contour.push_back(Point(6,3));
    contour.push_back(Point(6,4));
    
    // Add points while it is possible
    DSSComputer theDSSComputer;    
    theDSSComputer.init( contour.begin() );
    while ( ( theDSSComputer.end() != contour.end() )
          &&( theDSSComputer.extendForward() ) ) {}

    // Output parameters
    cout << theDSSComputer << endl;
    //! [ArithmeticalDSSComputer4Usage]

    DSSComputer::Primitive theDSS = theDSSComputer.primitive();  
  
    // Display on board
    string filename = "DSS.svg"; 
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
  }

  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
