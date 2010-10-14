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
//LICENSE-END
/**
 * @file test_arithDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2010/07/02
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_arithDSS <p>
 * Aim: simple test of \ref arithDSS, \ref arithDSS4
 */

#include <cstdio>
#include <cmath>
// #include <iostream>
#include <fstream>
#include <vector>
// #include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/ArithDSS4.h"
// #include "DGtal/kernel/SpaceND.h"
// #include "DGtal/kernel/domains/HyperRectDomain.h"

#include <iostream>
#include <iterator>
#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/sets/DigitalSetConverter.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/DomainMetricAdjacency.h"
#include "DGtal/topology/DomainAdjacency.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/Expander.h"
#include "DGtal/io/DGtalBoard.h"

using namespace DGtal;
using namespace std;
using namespace LibBoard;




int main(int argc, char **argv)
{


  typedef SpaceND<2> Space2Type;
  typedef HyperRectDomain<Space2Type> Domain2D;
  typedef Space2Type::Point Point;
  
	std::vector<Point> contour;
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

  // Bad initialisation
  trace.beginBlock("Bad init");
	trace.info() << "same point two times" << std::endl;
	try {
  	ArithDSS4<Domain2D> theDSS(Point(0,0),Point(0,0));		
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
	trace.info() << "not connected points" << std::endl;
	try {
	  ArithDSS4<Domain2D> theDSS(Point(0,0),Point(1,1));	
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
  trace.endBlock();


  // Print the result of the initialisation
  trace.beginBlock("Init of a DSS");
  ArithDSS4<Domain2D> theDSS(contour.at(0),contour.at(1));		
  trace.info() << theDSS << " " << theDSS.isValid() << std::endl;
  trace.endBlock();
  
  
  // Print the result of the adding
  trace.beginBlock("Add some points");
	for (int i = 2; i < contour.size(); i++) {
			trace.info() << contour.at(i) << std::endl;
			theDSS.addFront(contour.at(i));
		  trace.info() << theDSS << " " << theDSS.isValid() << std::endl;
	}
  trace.endBlock();
  
  // Draw the DSS in a SVG file
  trace.info()<< "Draw the DSS in the SVG file DSS.svg"<< endl; 
  Point p1(  -10, -10  );
  Point p2( 10, 10  );
  Domain2D domain( p1, p2 );

  Board board;
  board.setUnit(Board::UCentimeter);
  
  domain.selfDrawAsGrid(board);
  theDSS.selfDraw(board);
  
  board.saveSVG("DSS.svg");

  // Print the result of the removing
  trace.beginBlock("Remove the first point as many times as possible");
	while (theDSS.removeBack()) { 
		trace.info() << theDSS << " " << theDSS.isValid() << std::endl;
	}
  trace.endBlock();


  
 

    return 0;
}
