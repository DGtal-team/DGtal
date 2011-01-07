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
 * @file testArithDSS.cpp
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
 * Description of testArithDSS <p>
 * Aim: simple test of \ref ArithmeticalDSS
 */




#include <iostream>
#include <iterator>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>

#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/io/DGtalBoard.h"
#include <gmpxx.h>

using namespace DGtal;
using namespace std;
using namespace LibBoard;




int main(int argc, char **argv)
{

 typedef SpaceND<2,long long int> Space2Type;
 typedef Space2Type::Point Point;
 typedef Space2Type::Integer Coordinate;

//--------------------- DSS4 --------------------------

//	typedef ArithmeticalDSS<StandardBase<Coordinate> > DSS4;
	typedef experimental::ArithmeticalDSS<Coordinate,4> DSS4;  

	std::vector<Point> contour;
/*	contour.push_back(Point(0,0));
	contour.push_back(Point(1,0));
	contour.push_back(Point(2,0));
	contour.push_back(Point(3,0));
	contour.push_back(Point(3,1));
	contour.push_back(Point(4,1));
	contour.push_back(Point(5,1));
	contour.push_back(Point(5,2));
*/

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

/*
  // Bad initialisation
  trace.beginBlock("Bad init");
	trace.info() << "same point two times" << std::endl;
	try {
  	DSS4 theDSS4(Point(0,0),Point(0,0));		
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
	trace.info() << "not connected points" << std::endl;
	try {
	  DSS4 theDSS4(Point(0,0),Point(1,1));	
	} catch (InputException e) {
		trace.info() << e.what() << std::endl;
	}
  trace.endBlock();
*/

  // Good Initialisation
  trace.beginBlock("Init of a DSS4");
  DSS4 theDSS4(contour.at(0));		
  trace.info() << theDSS4 << " " << theDSS4.isValid() << std::endl;
  trace.endBlock();
  
  
  
  // Adding step
  trace.beginBlock("Add points while it is possible and draw the result");
	{
		int i = 1;
		while ( (i<contour.size())
					&&(theDSS4.extend(contour.at(i))) ) {
			i++;
		}
	  trace.info() << theDSS4 << " " << theDSS4.isValid() << std::endl;

		HyperRectDomain< Space2Type > domain( Point(0,0), Point(10,10) );

		DGtalBoard board;
		board.setUnit(Board::UCentimeter);
		
  	board << SetMode(domain.styleName(), "Grid")
				  << domain;		
    board << SetMode("PointVector", "Grid");

//  	board << SetMode(theDSS4.styleName(), "Both") 
//					<< theDSS4;
//does not draw the default style

  	board << SetMode(theDSS4.styleName(), "Points") 
					<< theDSS4;
  	board << SetMode(theDSS4.styleName(), "BoundingBox") 
					<< theDSS4;


		
		board.saveSVG("DSS4.svg");
	
	}

  trace.endBlock();
  


  // Removing step and checking consistency with the adding step.
  trace.beginBlock("Checking consistency between adding and removing");

		std::deque<DSS4 > v1,v2;
  	DSS4 newDSS4(contour.at(0));	 
	  	v1.push_back(newDSS4);

		//forward scan and store each DSS4
		trace.info() << "forward scan" << std::endl;

		int i = 1;
		while  ( (i<contour.size())
					&&(newDSS4.extend(contour.at(i))) ) {
	  	v1.push_back(newDSS4);
			i++;
		}

		//backward scan
		trace.info() << "backward scan" << std::endl;

  	DSS4 reverseDSS4(contour.at(i-1));
		int j = i-2;
		while ( (j>=0)&&(reverseDSS4.extend(contour.at(j))) ) {
			j--;
		}
		trace.info() << "removing" << std::endl;
		trace.info() << reverseDSS4 << std::endl;

		//removing step, store each DSS4 for comparison
	  v2.push_front(reverseDSS4);
		i--;
		while (reverseDSS4.retract()) {
	  	v2.push_front(reverseDSS4);
			i--;
		}		
		
		//comparison
		trace.info() << "comparison" << std::endl;
		ASSERT(v1.size() == v2.size());
		bool isOk = true;
		for (unsigned int k = 0; k < v1.size(); k++) {
			if (v1.at(k) != v2.at(k)) isOk = false;
			trace.info() << "DSS4 :" << k << std::endl;
			trace.info() << v1.at(k) << v2.at(k) << std::endl;
		}

		if (isOk) trace.info() << "ok for the " << v1.size() << " DSS4" << std::endl;
		else trace.info() << "failure" << std::endl;

  trace.endBlock();

/*
  
	typedef experimental::ArithmeticalDSS<Coordinate,8> DSS8; 

	std::vector<Point> boundary;
	boundary.push_back(Point(0,0));
	boundary.push_back(Point(1,1));
	boundary.push_back(Point(2,1));
	boundary.push_back(Point(3,2));
	boundary.push_back(Point(4,2));
	boundary.push_back(Point(5,2));
	boundary.push_back(Point(6,3));
	boundary.push_back(Point(6,4));

  // Good Initialisation
  trace.beginBlock("test of a DSS8");
  DSS8 theDSS8(boundary.at(0));		
  trace.info() << theDSS8 << " " << theDSS8.isValid() << std::endl;

	{
		int i = 1;
		while (theDSS8.extend(boundary.at(i))) {
			i++;
		}
	  trace.info() << theDSS8 << " " << theDSS8.isValid() << std::endl;

		HyperRectDomain<Space2Type> domain( Point(0,0), Point(10,10) );

		
		DGtalBoard board;
		board.setUnit(Board::UCentimeter);
		
  	board << SetMode(domain.styleName(), "Paving")
				  << domain;		
    board << SetMode("PointVector", "Both");


//  	board << SetMode(theDSS8.styleName(), "Both") 
//					<< theDSS8;
//does not work


  	board << SetMode(theDSS8.styleName(), "Points") 
					<< theDSS8;
  	board << SetMode(theDSS8.styleName(), "BoundingBox") 
					<< theDSS8;
		
		
		board.saveSVG("DSS8.svg");

	}

  trace.endBlock();
*/

  return 0;
}
