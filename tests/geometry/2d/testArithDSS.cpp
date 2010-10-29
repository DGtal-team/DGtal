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


	typedef int Coordinate;
	typedef PointVector<2,Coordinate> Point;
	typedef ArithDSS4<Coordinate> DSS;
  
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
	contour.push_back(Point(6,4));

  // Bad initialisation
  trace.beginBlock("Bad init");
	trace.info() << "same point two times" << std::endl;
	try {
  	DSS theDSS(Point(0,0),Point(0,0));		
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
	trace.info() << "not connected points" << std::endl;
	try {
	  DSS theDSS(Point(0,0),Point(1,1));	
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
  trace.endBlock();


  // Good Initialisation
  trace.beginBlock("Init of a DSS");
  DSS theDSS(contour.at(0),contour.at(1));		
  trace.info() << theDSS << " " << theDSS.isValid() << std::endl;
  trace.endBlock();

 
  
  // Adding step
  trace.beginBlock("Add points while it is possible and draw the result in DSS.svg");
	{
		int i = 2;
		while (theDSS.addFront(contour.at(i))) {
			i++;
		}
	  trace.info() << theDSS << " " << theDSS.isValid() << std::endl;
	}

	HyperRectDomain<SpaceND<2> > domain( Point(  -10, -10  ), Point(  10, 10  ) );

  Board board;
  board.setUnit(Board::UCentimeter);
  
  domain.selfDrawAsGrid(board);
  theDSS.selfDraw(board);
  
  board.saveSVG("DSS.svg");

  trace.endBlock();
  

  // Removing step and checks consistency with the adding step.
  trace.beginBlock("Checks consistency between adding and removing");

		std::deque<DSS > v1,v2;
  	DSS newDSS(contour.at(0),contour.at(1));	 
	  	v1.push_back(newDSS);

		//forward scan and store each DSS
		trace.info() << "forward scan" << std::endl;

		int i = 2;
		while (newDSS.addFront(contour.at(i))) {
	  	v1.push_back(newDSS);
			i++;
		}

		//backward scan
		trace.info() << "backward scan" << std::endl;

  	DSS reverseDSS(contour.at(i-1),contour.at(i-2));
		int j = i-3;
		while ( (j>=0)&&(reverseDSS.addFront(contour.at(j))) ) {
			j--;
		}
		trace.info() << "removing" << std::endl;
		trace.info() << reverseDSS << std::endl;

		//removing step, store each DSS for comparison
	  v2.push_front(reverseDSS);
		i--;
		while (reverseDSS.removeBack()) {
	  	v2.push_front(reverseDSS);
			i--;
		}		
		
		//comparison
		trace.info() << "comparison" << std::endl;
		ASSERT(v1.size() == v2.size());
		bool isOk = true;
		for (int k = 0; k < v1.size(); k++) {
			if (v1.at(k) != v2.at(k)) isOk = false;
			trace.info() << "DSS :" << k << std::endl;
			trace.info() << v1.at(k) << v2.at(k) << std::endl;
		}

		if (isOk) trace.info() << "ok for the " << v1.size() << " DSS" << std::endl;
		else trace.info() << "failure" << std::endl;

  trace.endBlock();


  
 

  return 0;
}
