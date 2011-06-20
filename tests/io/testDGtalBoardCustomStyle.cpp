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
 * @file testDGtalBoardCustomStyle.cpp
 * @ingroup Tests
 * @author davidcoeurjolly (login) (\c Unknown )
 * Unknown
 *
 * @date 2010/10/24
 *
 * Functions for testing class DGtalBoardCustomStyle.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
  
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/io/DGtalBoard.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;



/**
 * Custom style.
 */
struct MyDrawStyleCustomGreen : public DrawableWithDGtalBoard
{
  virtual void selfDraw( LibBoard::Board & aBoard ) const
   {
     aBoard.setFillColorRGBi(0,160,0);
     aBoard.setPenColorRGBi(80,0,0);
   }
};

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DGtalBoardCustomStyle.
//////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDGtalBoardCustomStyle()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef SpaceND<2> Z2;
 
  typedef HyperRectDomain<Z2> Domain;
  typedef Z2::Point Point;
  Point p1(  -10, -10  );
  Point p2(  10, 10  );
  Domain domain( p1, p2 );
  typedef DigitalSetSelector
    < Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type SpecificSet; 
  SpecificSet mySet( domain );
 
  Point c(  0, 0  );
  mySet.insert( c );
  Point d(  5, 2  );
  mySet.insert( d );
  Point e(  1, -3  );
  mySet.insert( e );


  DGtalBoard board;
  board.setUnit(LibBoard::Board::UCentimeter);
  
  board << DrawDomainGrid()
	<< domain 
	<< mySet;
  board.saveSVG("testcustom-prev.svg");

  board.clear();

  board << DrawDomainGrid()
	<< domain 
	<< CustomStyle( mySet.styleName(), new MyDrawStyleCustomGreen )
	<< mySet;
  board.saveSVG("testcustom-next.svg");

 
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DGtalBoardCustomStyle" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDGtalBoardCustomStyle(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
