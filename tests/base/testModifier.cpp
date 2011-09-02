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
 * @file testModifier.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/02
 *
 * Functions for testing class Modifier.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/base/Modifier.h"

#include "DGtal/topology/KhalimskySpaceND.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Modifier.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testModifier()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  
  //scell 2 point
  {
    typedef KhalimskySpaceND<3> K3;
    K3 theKSpace; 
    K3::SCell s = theKSpace.sPointel( K3::Point(3,3,4) );
    K3::Point aPoint = SCellToPoint<K3>::get( theKSpace, s );
    trace.info() << s << aPoint <<std::endl;  
    nbok += ( aPoint == K3::Point(3,3,4) ) ? 1 : 0; 
    nb++;
  }
  //scell 2 midPoint
  {
    typedef KhalimskySpaceND<2> K2;
    K2 theKSpace; 
    K2::SCell s = theKSpace.sCell( K2::Point(0,1) );
    RealPointVector<K2::dimension> aPoint = SCellToMidPoint<K2>::get( theKSpace, s );
    trace.info() << s << aPoint <<std::endl;  
    nbok += ( aPoint == RealPointVector<K2::dimension>(0,0.5) ) ? 1 : 0; 
    nb++;
  }  

    //scell 2 arrow
  {
    typedef KhalimskySpaceND<2> K2;
    K2 theKSpace; 
    K2::SCell s = theKSpace.sCell( K2::Point(0,1) );
    std::pair<K2::Point, K2::Vector> aArrow = SCellToArrow<K2>::get( theKSpace, s );
    trace.info() << s << aArrow.first << aArrow.second <<std::endl;  
    K2::Point p(0,1); 
    K2::Vector v(0,-1); 
    nbok += ( ((aArrow.first == p) && (aArrow.second == v)) ) ? 1 : 0; 
    nb++;
  }  
  
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class Modifier" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testModifier(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
