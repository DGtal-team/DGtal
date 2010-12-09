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

using namespace DGtal;
using namespace std;
using namespace LibBoard;




int main(int argc, char **argv)
{

 typedef SpaceND<2> Space2Type;
 typedef Space2Type::Point Point;
 typedef SpaceND<2>::Integer Coordinate;

//--------------------- DSS4 --------------------------

	typedef ArithmeticalDSS<StandardBase<Coordinate> > DSS4;
  
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


  // Good Initialisation
  trace.beginBlock("Init of a DSS4");
  DSS4 theDSS4(contour.at(0),contour.at(1));		
  trace.info() << theDSS4 << " " << theDSS4.isValid() << std::endl;
  trace.endBlock();
  
  
  
  // Adding step
  trace.beginBlock("Add points while it is possible and draw the result in DSS4.svg");
	{
		int i = 2;
		while (theDSS4.addFront(contour.at(i))) {
			i++;
		}
	  trace.info() << theDSS4 << " " << theDSS4.isValid() << std::endl;

	  Point a(-10,-10);
	  Point b(10,10);
	HyperRectDomain< Space2Type > domain( a , b );

		DGtalBoard board;
		board.setUnit(Board::UCentimeter);
		
		board << DrawDomainPaving() << domain;
		// domain.selfDrawAsGrid(board);
		
		//board << DrawDSSBoundingBox()
		//    << theDSS4;
		board << DrawPavingPixel() <<  theDSS4;
		
		board.saveEPS("DSS4.eps");
		
		



	}

  trace.endBlock();
  


  // Removing step and checking consistency with the adding step.
  trace.beginBlock("Checking consistency between adding and removing");

		std::deque<DSS4 > v1,v2;
  	DSS4 newDSS4(contour.at(0),contour.at(1));	 
	  	v1.push_back(newDSS4);

		//forward scan and store each DSS4
		trace.info() << "forward scan" << std::endl;

		int i = 2;
		while (newDSS4.addFront(contour.at(i))) {
	  	v1.push_back(newDSS4);
			i++;
		}

		//backward scan
		trace.info() << "backward scan" << std::endl;

  	DSS4 reverseDSS4(contour.at(i-1),contour.at(i-2));
		int j = i-3;
		while ( (j>=0)&&(reverseDSS4.addFront(contour.at(j))) ) {
			j--;
		}
		trace.info() << "removing" << std::endl;
		trace.info() << reverseDSS4 << std::endl;

		//removing step, store each DSS4 for comparison
	  v2.push_front(reverseDSS4);
		i--;
		while (reverseDSS4.removeBack()) {
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

//--------------------- DSS8 -----------------------------
  
	typedef ArithmeticalDSS<NaiveBase<Coordinate> > DSS8; 

	std::vector<Point> bord;
	bord.push_back(Point(0,0));
	bord.push_back(Point(1,1));
	bord.push_back(Point(2,1));
	bord.push_back(Point(3,2));
	bord.push_back(Point(4,2));
	bord.push_back(Point(5,2));
	bord.push_back(Point(6,3));
	bord.push_back(Point(6,4));

  // Good Initialisation
  trace.beginBlock("test of a DSS8");
  DSS8 theDSS8(bord.at(0),bord.at(1));		
  trace.info() << theDSS8 << " " << theDSS8.isValid() << std::endl;

	{
		int i = 2;
		while (theDSS8.addFront(bord.at(i))) {
			i++;
		}
	  trace.info() << theDSS8 << " " << theDSS8.isValid() << std::endl;

		HyperRectDomain<SpaceND<2> > domain( Point(  -10, -10  ), Point(  10, 10  ) );

		
		DGtalBoard board;
		board.setUnit(Board::UCentimeter);
		
		board << DrawDSSBoundingBox() << theDSS8;
		
		

		// Board board;
		// board.setUnit(Board::UCentimeter);
		
		// domain.selfDrawAsGrid(board);
		// theDSS8.selfDraw(board);
		
		board.saveSVG("DSS8.svg");

	}

  trace.endBlock();

  return 0;
}
