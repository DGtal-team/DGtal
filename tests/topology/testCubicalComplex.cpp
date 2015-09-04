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
 * @file testCubicalComplex.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/08/28
 *
 * Functions for testing class CubicalComplex.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <map>
#include <unordered_map>
#include "DGtal/base/Common.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/CubicalComplex.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CubicalComplex.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testCubicalComplexWithMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef KhalimskySpaceND<3>              KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Cell                     Cell;
  typedef CubicalComplex< KSpace, std::map<Cell,CubicalCellData> > CC;

  trace.beginBlock ( "Testing Cubical complex creation" );
  KSpace K;
  K.init( Point( 0,0,0 ), Point( 10,10,10 ), true );
  CC complex( K );
  for ( int n = 0; n < 1000000; ++n )
    {
      Point p( rand() % 512, rand() % 512, rand() % 512 );
      Cell cell = K.uCell( p );
      complex.insertCell( cell );
    }
  trace.info() << complex << std::endl;
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

namespace std {
  template < DGtal::Dimension dim,
             typename TInteger >
  struct hash< DGtal::KhalimskyCell<dim, TInteger> >{
    typedef DGtal::KhalimskyCell<dim, TInteger> Key;
    typedef Key argument_type;
    typedef std::size_t result_type;
    inline hash() {}
    inline result_type operator()( const argument_type& cell ) const
    {
      result_type h = cell.myCoordinates[ 0 ];
      static const result_type mult[ 8 ] = { 1, 1733, 517237, 935783132, 305, 43791, 12846764, 56238719 };
      // static const result_type shift[ 8 ] = { 0, 13, 23, 7, 19, 11, 25, 4 };
      for ( DGtal::Dimension i = 1; i < dim; ++i )
        h += cell.myCoordinates[ i ] * mult[ i & 0x7 ];
      // h += cell.myCoordinates[ i ] << shift[ i & 0x7 ];
      return h;
    }
  };
}

bool testCubicalComplexWithHashMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef KhalimskySpaceND<3>              KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Cell                     Cell;
  typedef CubicalComplex< KSpace, std::unordered_map<Cell,CubicalCellData> > CC;

  trace.beginBlock ( "Testing Cubical complex creation" );
  KSpace K;
  K.init( Point( 0,0,0 ), Point( 10,10,10 ), true );
  CC complex( K );
  for ( int n = 0; n < 1000000; ++n )
    {
      Point p( rand() % 512, rand() % 512, rand() % 512 );
      Cell cell = K.uCell( p );
      complex.insertCell( cell );
    }
  trace.info() << complex << std::endl;
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CubicalComplex" );
  bool res = 
    testCubicalComplexWithMap()
    && testCubicalComplexWithHashMap();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
