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
 * @file testChordNaivePlaneComputer-benchmark.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class ChordNaivePlaneComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/Statistic.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/geometry/surfaces/ChordNaivePlaneComputer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ChordNaivePlaneComputer.
///////////////////////////////////////////////////////////////////////////////

template <typename Integer>
Integer getRandomInteger( const Integer & first, const Integer & after_last )
{
  Integer r = (Integer) rand();
  return ( r % (after_last - first) ) + first;
}

/**
 * Checks the naive plane d <= ax+by+cz <= d + max(|a|,|b|,|c|)-1
 */
template <typename Integer, typename NaivePlaneComputer>
bool
checkPlane( Integer a, Integer b, Integer c, Integer d, 
            int diameter, unsigned int nbpoints )
{
  typedef typename NaivePlaneComputer::Point Point;
  typedef typename Point::Component PointInteger;
  IntegerComputer<Integer> ic;
  Integer absA = ic.abs( a );
  Integer absB = ic.abs( b );
  Integer absC = ic.abs( c );
  Integer x, y, z;
  Dimension axis;
  if ( ( absA >= absB ) && ( absA >= absC ) )
    axis = 0;
  else if ( ( absB >= absA ) && ( absB >= absC ) )
    axis = 1;
  else
    axis = 2;
  Point p;
  NaivePlaneComputer plane;
  plane.init( axis, 1, 1 );
  // Checks that points within the naive plane are correctly recognized.
  unsigned int nb = 0;
  unsigned int nbok = 0;
  while ( nb != nbpoints )
    {
      p[ 0 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 1 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 2 ] = getRandomInteger<PointInteger>( -diameter+1, diameter );
      x = (Integer) p[ 0 ];
      y = (Integer) p[ 1 ];
      z = (Integer) p[ 2 ];
      switch ( axis ) {
      case 0: p[ 0 ] = (PointInteger)NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - b * y - c * z, a ) ); break;
      case 1: p[ 1 ] = (PointInteger)NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - c * z, b ) ); break;
      case 2: p[ 2 ] = (PointInteger)NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - b * y, c ) ); break;
      } 
      bool ok = plane.extend( p ); // should be ok
      ++nb; nbok += ok ? 1 : 0;
      if ( ! ok )
        {
          std::cerr << "[ERROR] p=" << p << " NOT IN plane=" << plane << std::endl;
          break;
        }
    }
  return nb == nbok;
}

template <typename NaivePlaneComputer>
bool
checkPlanes( unsigned int nbplanes, unsigned int diameter, unsigned int nbpoints )
{
  //using namespace Z3i;
  typedef typename NaivePlaneComputer::InternalScalar Integer;
  unsigned int nb = 0;
  unsigned int nbok = 0;
  for ( unsigned int nbp = 0; nbp < nbplanes; ++nbp )
    {
      Integer a = getRandomInteger<Integer>( (Integer) 0, (Integer) diameter / 2 ); 
      Integer b = getRandomInteger<Integer>( (Integer) 0, (Integer) diameter / 2 ); 
      Integer c = getRandomInteger<Integer>( (Integer) 0, (Integer) diameter / 2 ); 
      Integer d = getRandomInteger<Integer>( (Integer) 0, (Integer) diameter / 2 ); 
      if ( ( a != 0 ) || ( b != 0 ) || ( c != 0 ) )
        {
          ++nb; nbok += checkPlane<Integer, NaivePlaneComputer>( a, b, c, d, diameter, nbpoints ) ? 1 : 0;
          if ( nb != nbok )
            {
              std::cerr << "[ERROR] for plane " << a << " * x + " 
                        << b << " * y + " << c << " * z = " << d << std::endl;
              break;
            }
        }
    }
  return nb == nbok;
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  using namespace Z3i;
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) : 100;
  unsigned int nbpoints = ( argc > 2 ) ? atoi( argv[ 2 ] ) : 100;
  unsigned int diameter = ( argc > 3 ) ? atoi( argv[ 3 ] ) : 100;
  std::cout << "# Usage: " << argv[0] << " <nbtries> <nbpoints> <diameter>." << std::endl;
  std::cout << "# Test class ChordNaivePlaneComputer. Points are randomly chosen in [-diameter,diameter]^3." << std::endl;
  std::cout << "# Integer nbtries nbpoints diameter time/plane(ms)" << std::endl;
  
  trace.beginBlock ( "Testing class ChordNaivePlaneComputer" );
  bool res = true 
    && checkPlanes<ChordNaivePlaneComputer<Space, Point, DGtal::int64_t> >( nbtries, diameter, nbpoints );
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  long t = trace.endBlock();
  std::cout << "int64_t" << " " << nbtries
            << " " << nbpoints
            << " " << diameter 
            << " " << ( (double) t / (double) nbtries )
            << std::endl;
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
