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
 * @file ArithmeticalDSS.cpp
 * @ingroup Examples
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France

 * @date 2010/11/30
 *
 * An example file named ArithmeticalDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

#include "DGtal/geometry/curves/ArithmeticalDSS.h"
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
    //! [ArithmeticalDSS4Usage]
    typedef PointVector<2,int> Point;
    typedef std::vector<Point> Range;
    typedef std::vector<Point>::const_iterator ConstIterator;
    typedef ArithmeticalDSS<ConstIterator,int,4> DSS4;  

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
    DSS4 theDSS4;    
    theDSS4.init( contour.begin() );
    while ( ( theDSS4.end() != contour.end() )
          &&( theDSS4.extendFront() ) ) {}

    // Output parameters
    cout << theDSS4 << endl;
    //! [ArithmeticalDSS4Usage]
            
    //! [ArithmeticalDSS4DrawingUsage]
    // Draw the grid
    Board2D board;
  
    Domain domain( Point(0,0), Point(8,8) );
    board << SetMode(domain.className(), "Grid")
    << domain;    

    // Draw the points of the DSS
    board << SetMode("PointVector", "Grid")
    << SetMode(theDSS4.className(), "Points") 
    << theDSS4;
    // Draw the bounding box
    board << SetMode(theDSS4.className(), "BoundingBox") 
    << theDSS4;
  
    board.saveSVG("DSS4.svg");
    //! [ArithmeticalDSS4DrawingUsage]
  }

  ////////////////////// DSS8 ///////////////////////////////
  {
    //! [ArithmeticalDSS8Usage]
    typedef PointVector<2,int> Point;
    typedef std::vector<Point> Range;
    typedef std::vector<Point>::const_iterator ConstIterator;
    typedef ArithmeticalDSS<ConstIterator,int,8> DSS8;  

    // Input points
    Range boundary;
    boundary.push_back(Point(0,0));
    boundary.push_back(Point(1,1));
    boundary.push_back(Point(2,1));
    boundary.push_back(Point(3,2));
    boundary.push_back(Point(4,2));
    boundary.push_back(Point(5,2));
    boundary.push_back(Point(6,3));
    boundary.push_back(Point(6,4));

    // Add points while it is possible
    DSS8 theDSS8;    
    theDSS8.init( boundary.begin() );
    while ( ( theDSS8.end() != boundary.end() )
          &&( theDSS8.extendFront() ) ) {}


    // Output parameters
    cout << theDSS8 << endl;
    //! [ArithmeticalDSS8Usage]
    
    //! [ArithmeticalDSS8DrawingUsage]
    //Draw the pixels
    Board2D board;
    Domain domain( Point(0,0), Point(8,8) );
    board << SetMode(domain.className(), "Paving")
    << domain;    
  
    //Draw the points of the DSS
    board << SetMode("PointVector", "Both");
    board << SetMode(theDSS8.className(), "Points") 
    << theDSS8;

    //Draw the bounding box of the DSS
    board << SetMode(theDSS8.className(), "BoundingBox") 
    << theDSS8;
    
    board.saveSVG("DSS8.svg");
    //! [ArithmeticalDSS8DrawingUsage]
  }

  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
