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
 * @file testHalfPlane.cpp
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
 * @brief Aim: simple test of \ref Preimage2D
 */

#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <iterator>




#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/2d/GridCurve.h"
#include "DGtal/io/boards/Board2D.h"

#include "DGtal/geometry/2d/StraightLineFrom2Points.h"
#include "DGtal/geometry/2d/Preimage2D.h"


#include "ConfigTest.h"


using namespace DGtal;


int main()
{

  typedef int Coordinate;
  typedef PointVector<2, Coordinate> Point;
  typedef StraightLineFrom2Points<Coordinate> StraightLine;
  typedef Preimage2D<StraightLine> Preimage2D;

  //data
  std::vector<Point> bInf, bSup;

//////////////////////////// test 1 //////////////////////////////////////
  bInf.push_back(Point(0, 10));
  bInf.push_back(Point(1, 11));
  bInf.push_back(Point(2, 11));
  bInf.push_back(Point(3, 12));

  bSup.push_back(Point(0, 12));
  bSup.push_back(Point(1, 13));
  bSup.push_back(Point(2, 13));
  bSup.push_back(Point(3, 14));

  Board2D board;
  board.setUnit(Board2D::UCentimeter);


  //SimpleTest
  trace.beginBlock("Simple Preimage test");
  {
    int i = 0;
    Preimage2D thePreimage(bInf.at(i), bSup.at(i));
    
    //draw range
    Point P(bInf.at(i));
    Point Q(bSup.at(i));
    board << P << Q; 
    board.drawLine(P[0], P[1], Q[0], Q[1]);

    i++;
    int n;
    n = bInf.size();
    while ( (i < n) &&
        (thePreimage.addFront(bInf.at(i), bSup.at(i))) )
    {

      trace.info() << "--- Constraint number " << i << " ---" << std::endl;
      trace.info() << thePreimage << std::endl;

      //draw range
      Point P2(bInf.at(i));
      Point Q2(bSup.at(i));
      board << P2 << Q2; 
      board.drawLine(P2[0], P2[1], Q2[0], Q2[1]);

      i++;
    }

    board << thePreimage; 
    board.saveEPS( "testPreimage-simple.eps", Board2D::BoundingBox, 5000 );
  }
  trace.endBlock();


  bInf.push_back(Point(4, 10));
  bInf.push_back(Point(5, 10));
  bInf.push_back(Point(6, 11));
  bInf.push_back(Point(7, 12));
  bInf.push_back(Point(8, 12));
  bInf.push_back(Point(9, 13));

  bSup.push_back(Point(4, 13));
  bSup.push_back(Point(5, 12));
  bSup.push_back(Point(6, 13));
  bSup.push_back(Point(7, 14));
  bSup.push_back(Point(8, 14));
  bSup.push_back(Point(9, 15));

  //preimage
  trace.beginBlock("test limit case");

  board.clear(); 
  
  {
    int i = 0;
    Preimage2D thePreimage2(bInf.at(i), bSup.at(i));

    //draw range
    Point P(bInf.at(i));
    Point Q(bSup.at(i));
    board << P << Q; 
    board.drawLine(P[0], P[1], Q[0], Q[1]);

    i++;
    int n;
    n = bInf.size();
    while ( (i < n) &&
        (thePreimage2.addFront(bInf.at(i), bSup.at(i))) )
    {

      trace.info() << "--- Constraint number " << i << " ---" << std::endl;
      trace.info() << thePreimage2 << std::endl;

      //draw range
      Point P2(bInf.at(i));
      Point Q2(bSup.at(i));
      board << P2 << Q2;
      board.drawLine(P2[0], P2[1], Q2[0], Q2[1]);

      i++;
    }

    //draw preimage
    board << thePreimage2; 
    board.saveEPS("testPreimage-limitCase.eps");

  }
  trace.endBlock();

//////////////////////////// test 2 //////////////////////////////////////

  bInf.clear();
  bSup.clear();

  bInf.push_back(Point(154, 154));
  bInf.push_back(Point(154, 154));
  bInf.push_back(Point(167, 201));
  bInf.push_back(Point(167, 201));
  bInf.push_back(Point(167, 201));
  bInf.push_back(Point(210, 213));
  bInf.push_back(Point(199, 246));
  bInf.push_back(Point(199, 246));
  bInf.push_back(Point(236, 249));
  bInf.push_back(Point(256, 275));
  bInf.push_back(Point(256, 275));
  bInf.push_back(Point(286, 295));
  bInf.push_back(Point(286, 295));


  bSup.push_back(Point(74, 211));
  bSup.push_back(Point(122, 210));
  bSup.push_back(Point(122, 210));
  bSup.push_back(Point(139, 239));
  bSup.push_back(Point(159, 243));
  bSup.push_back(Point(159, 243));
  bSup.push_back(Point(159, 243));
  bSup.push_back(Point(184, 271));
  bSup.push_back(Point(184, 271));
  bSup.push_back(Point(184, 271));
  bSup.push_back(Point(225, 294));
  bSup.push_back(Point(225, 294));
  bSup.push_back(Point(234, 338));


  //preimage
  trace.beginBlock("test General Case");

  board.clear();
  {
    int i = 0;
    Preimage2D thePreimage(bInf.at(i), bSup.at(i));

    //draw range
    Point P(bInf.at(i));
    Point Q(bSup.at(i));
    board << P << Q;
    board.drawLine(P[0], P[1], Q[0], Q[1]);

    i++;
    int n;
    n = bInf.size();
    while ( (i < n) &&
        (thePreimage.addFront(bInf.at(i), bSup.at(i))) )
    {

      trace.info() << "--- Constraint number " << i << " ---" << std::endl;
      trace.info() << thePreimage << std::endl;

      //draw range
      Point P2(bInf.at(i));
      Point Q2(bSup.at(i));
      board << P2 << Q2;
      board.drawLine(P2[0], P2[1], Q2[0], Q2[1]);

      i++;
    }

    //draw preimage
    board << thePreimage; 
    board.saveEPS("testPreimage-generalCase.eps");

  }
  trace.endBlock();


  return 0;
}
