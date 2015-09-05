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
  template < typename TInteger >
  struct hash< DGtal::KhalimskyCell<2, TInteger> >{
    typedef DGtal::KhalimskyCell<3, TInteger> Key;
    typedef Key argument_type;
    typedef std::size_t result_type;
    inline hash() {}
    inline result_type operator()( const argument_type& cell ) const
    {
      result_type h = cell.myCoordinates[ 0 ];
      h += cell.myCoordinates[ 1 ] * 1733;
      return h;
    }
  };
  template < typename TInteger >
  struct hash< DGtal::KhalimskyCell<3, TInteger> >{
    typedef DGtal::KhalimskyCell<3, TInteger> Key;
    typedef Key argument_type;
    typedef std::size_t result_type;
    inline hash() {}
    inline result_type operator()( const argument_type& cell ) const
    {
      result_type h = cell.myCoordinates[ 0 ];
      h += cell.myCoordinates[ 1 ] * 1733;
      h += cell.myCoordinates[ 2 ] * 517237;
      return h;
    }
  };
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CubicalComplex.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename KSpace, typename Map>
bool testCubicalComplexWithMap( const std::string& str )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef typename KSpace::Point         Point;
  typedef typename KSpace::Cell             Cell;
  typedef CubicalComplex< KSpace, Map >     CC;
  typedef typename CC::CellMapConstIterator CellMapConstIterator;

  srand( 0 );
  std::string s1 = "[" + str + "]" + "Testing Cubical complex creation";
  trace.beginBlock ( s1.c_str() );
  KSpace K;
  K.init( Point( 0,0,0 ), Point( 512,512,512 ), true );
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
  
  std::string s2 = "[" + str + "]" + "Testing faces computation";
  trace.beginBlock ( s2.c_str() );
  std::vector<Cell> faces;
  std::back_insert_iterator< std::vector<Cell> > outIt( faces );
  for ( CellMapConstIterator it = complex.begin( 3 ), itE = complex.end( 3 );
        it != itE; ++it )
    {
      complex.faces( outIt, it->first );
    }
  trace.info() << "#Faces of maximal cells = " << faces.size() << std::endl;
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();

  std::string s3 = "[" + str + "]" + "Testing close operation";
  trace.beginBlock ( s3.c_str() );
  complex.close();
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
  typedef KhalimskySpaceND<3> K3;
  typedef K3::Cell Cell;
  bool res = 
    testCubicalComplexWithMap< K3, std::map<Cell, CubicalCellData> >( "3D, std::map" )
    && 
    testCubicalComplexWithMap< K3, std::unordered_map<Cell, CubicalCellData> >( "3D, std::unordered_map" );
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
