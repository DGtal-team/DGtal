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
  
  Point firstPoint ( 0, 0  );
  Point secondPoint (  1, 0  );
  
  // Initialisation of a DSS
  ArithDSS4<Domain2D> theDSS(firstPoint,secondPoint);		
  

  // Print the result of the initialisation
  trace.beginBlock("Init of a DSS");
  std::cout << theDSS;
  trace.endBlock();
  
  // Add some points
  Point a(1,1);
  theDSS.addFront(a);
  
  Point b(2,1);
  theDSS.addFront(b);
  
  Point c(3,1);
  theDSS.addFront(c);
  
  // Print the result
  trace.beginBlock("Add some points");
  std::cout << theDSS;
  trace.endBlock();
  
  // Test draw
  
  Point p1(  -10, -10  );
  Point p2( 10, 10  );
  Domain2D domain( p1, p2 );

  Board board;
  board.setUnit(Board::UCentimeter);
  
  domain.selfDrawAsGrid(board);
  theDSS.selfDraw(board);
  
  board.saveSVG("DSS.svg");
  
 

    return 0;
}
