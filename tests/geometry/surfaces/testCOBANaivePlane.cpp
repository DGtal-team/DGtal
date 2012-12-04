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
 * @file testCOBANaivePlane.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class COBANaivePlane.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/geometry/surfaces/COBANaivePlane.h"
#include "DGtal/geometry/surfaces/COBAGenericNaivePlane.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class COBANaivePlane.
///////////////////////////////////////////////////////////////////////////////

template <typename Integer>
Integer getRandomInteger( const Integer & first, const Integer & after_last )
{
  Integer r = (Integer) random();
  return ( r % (after_last - first) ) + first;
}

/**
 * Checks the naive plane d <= ax+by+cz <= d + max(|a|,|b|,|c|)-1
 */
template <typename Integer, typename NaivePlane>
bool
checkPlane( Integer a, Integer b, Integer c, Integer d, 
            int diameter, unsigned int nbtries )
{
  typedef typename NaivePlane::Point Point;
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
  NaivePlane plane;
  plane.init( axis, diameter, 1, 1 );
  // Checks that points within the naive plane are correctly recognized.
  unsigned int nb = 0;
  unsigned int nbok = 0;
  while ( nb != nbtries )
    {
      p[ 0 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 1 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 2 ] = getRandomInteger<PointInteger>( -diameter+1, diameter );
      x = (Integer) p[ 0 ];
      y = (Integer) p[ 1 ];
      z = (Integer) p[ 2 ];
      switch ( axis ) {
      case 0: p[ 0 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - b * y - c * z, a ) ); break;
      case 1: p[ 1 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - c * z, b ) ); break;
      case 2: p[ 2 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - b * y, c ) ); break;
      } 
      bool ok_ext = plane.isExtendable( p ); // should be ok
      bool ok = plane.extend( p ); // should be ok
      ++nb, nbok += ok_ext ? 1 : 0;
      ++nb, nbok += ok ? 1 : 0;
      if ( ! ok )
        {
          std::cerr << "[ERROR] p=" << p << " NOT IN plane=" << plane << std::endl;
          break;
        }
      if ( ! ok_ext )
        {
          std::cerr << "[ERROR] p=" << p << " was NOT extendable IN plane=" << plane << std::endl;
          break;
        }
      // else
      //   std::cerr << "[OK] p=" << p << " IN plane=" << plane << std::endl;
    }

  // Checks that points outside the naive plane are correctly recognized as outliers.
  while ( nb != (nbtries * 11 ) / 10 )
    {
      p[ 0 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 1 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 2 ] = getRandomInteger<PointInteger>( -diameter+1, diameter );
      x = (Integer) p[ 0 ];
      y = (Integer) p[ 1 ];
      z = (Integer) p[ 2 ];
      switch ( axis ) {
      case 0: p[ 0 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - b * y - c * z, a ) ); break;
      case 1: p[ 1 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - c * z, b ) ); break;
      case 2: p[ 2 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - b * y, c ) ); break;
      } 
      PointInteger tmp = getRandomInteger<PointInteger>( 2, 5 ) 
        * (2*getRandomInteger<PointInteger>( 0, 2 ) - 1 );
      p[ axis ] += tmp;
      bool ok_ext = ! plane.isExtendable( p ); // should *not* be ok
      bool ok = ! plane.extend( p ); // should *not* be ok
      ++nb, nbok += ok ? 1 : 0;
      ++nb, nbok += ok_ext ? 1 : 0;
      if ( ! ok )
        {
          std::cerr << "[ERROR] p=" << p << " IN plane=" << plane << std::endl;
          break;
        }
      if ( ! ok_ext )
        {
          std::cerr << "[ERROR] p=" << p << " was extendable IN plane=" << plane << std::endl;
          break;
        }
      // else
      //   std::cerr << "[OK] p=" << p << " IN plane=" << plane << std::endl;
    }
  return nb == nbok;
}


/**
 * Checks the naive plane d <= ax+by+cz <= d + max(|a|,|b|,|c|)-1
 */
template <typename Integer, typename GenericNaivePlane>
bool
checkGenericPlane( Integer a, Integer b, Integer c, Integer d, 
                   int diameter, unsigned int nbtries )
{
  typedef typename GenericNaivePlane::Point Point;
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
  GenericNaivePlane plane;
  plane.init( diameter, 1, 1 );
  // Checks that points within the naive plane are correctly recognized.
  unsigned int nb = 0;
  unsigned int nbok = 0;
  while ( nb != nbtries )
    {
      p[ 0 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 1 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 2 ] = getRandomInteger<PointInteger>( -diameter+1, diameter );
      x = (Integer) p[ 0 ];
      y = (Integer) p[ 1 ];
      z = (Integer) p[ 2 ];
      switch ( axis ) {
      case 0: p[ 0 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - b * y - c * z, a ) ); break;
      case 1: p[ 1 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - c * z, b ) ); break;
      case 2: p[ 2 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - b * y, c ) ); break;
      } 
      bool ok_ext = plane.isExtendable( p ); // should be ok
      bool ok = plane.extend( p ); // should be ok
      ++nb, nbok += ok_ext ? 1 : 0;
      ++nb, nbok += ok ? 1 : 0;
      if ( ! ok )
        {
          std::cerr << "[ERROR] p=" << p << " NOT IN plane=" << plane << std::endl;
          break;
        }
      if ( ! ok_ext )
        {
          std::cerr << "[ERROR] p=" << p << " was NOT extendable IN plane=" << plane << std::endl;
          break;
        }
      // else
      //   std::cerr << "[OK] p=" << p << " IN plane=" << plane << std::endl;
    }

  // Checks that points outside the naive plane are correctly recognized as outliers.
  while ( nb != (nbtries * 11 ) / 10 )
    {
      p[ 0 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 1 ] = getRandomInteger<PointInteger>( -diameter+1, diameter ); 
      p[ 2 ] = getRandomInteger<PointInteger>( -diameter+1, diameter );
      x = (Integer) p[ 0 ];
      y = (Integer) p[ 1 ];
      z = (Integer) p[ 2 ];
      switch ( axis ) {
      case 0: p[ 0 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - b * y - c * z, a ) ); break;
      case 1: p[ 1 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - c * z, b ) ); break;
      case 2: p[ 2 ] = NumberTraits<Integer>::castToInt64_t( ic.ceilDiv( d - a * x - b * y, c ) ); break;
      } 
      PointInteger tmp = getRandomInteger<PointInteger>( 2, 5 ) 
        * (2*getRandomInteger<PointInteger>( 0, 2 ) - 1 );
      p[ axis ] += tmp;
      bool ok_ext = ! plane.isExtendable( p ); // should *not* be ok
      bool ok = ! plane.extend( p ); // should *not* be ok
      ++nb, nbok += ok ? 1 : 0;
      ++nb, nbok += ok_ext ? 1 : 0;
      if ( ! ok )
        {
          std::cerr << "[ERROR] p=" << p << " IN plane=" << plane << std::endl;
          break;
        }
      if ( ! ok_ext )
        {
          std::cerr << "[ERROR] p=" << p << " was extendable IN plane=" << plane << std::endl;
          break;
        }
      // else
      //   std::cerr << "[OK] p=" << p << " IN plane=" << plane << std::endl;
    }
  std::cerr << "plane = " << plane << std::endl;
  return nb == nbok;
}


template <typename Integer, typename NaivePlane>
bool
checkPlanes( unsigned int nbplanes, int diameter, unsigned int nbtries )
{
  //using namespace Z3i;
  //typedef COBANaivePlane<Z3, Integer> NaivePlane;
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
          ++nb, nbok += checkPlane<Integer, NaivePlane>( a, b, c, d, diameter, nbtries ) ? 1 : 0;
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

/**
 * Example of a test. To be completed.
 *
 */
bool testCOBANaivePlane()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  using namespace Z3i;
  typedef BigInteger Integer;
  typedef COBANaivePlane<Z3, BigInteger> NaivePlane;
  typedef COBAGenericNaivePlane<Z3, BigInteger> GenericNaivePlane;

  BOOST_CONCEPT_ASSERT(( CPointPredicate< NaivePlane > ));
  BOOST_CONCEPT_ASSERT(( boost::ForwardContainer< NaivePlane > ));
  BOOST_CONCEPT_ASSERT(( CPointPredicate< GenericNaivePlane > ));
  BOOST_CONCEPT_ASSERT(( boost::ForwardContainer< GenericNaivePlane > ));

  trace.beginBlock ( "Testing block: COBANaivePlane instantiation." );
  NaivePlane plane;
  Point pt0( 0, 0, 0 );
  plane.init( 2, 100, 3, 2 );
  bool pt0_inside = plane.extend( pt0 );
  trace.info() << "(" << nbok << "/" << nb << ") Plane=" << plane
               << std::endl;
  Point pt1( Point( 8, 1, 3 ) );
  bool pt1_inside = plane.extend( pt1 );
  ++nb, nbok += pt1_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt1 
               << " Plane=" << plane << std::endl;
  Point pt2( Point( 2, 7, 1 ) );
  bool pt2_inside = plane.extend( pt2 );
  ++nb, nbok += pt2_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt2 
               << " Plane=" << plane << std::endl;

  Point pt3( Point( 0, 5, 17 ) );
  bool pt3_inside = plane.extend( pt3 );
  ++nb, nbok += pt3_inside == false ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt3
               << " Plane=" << plane << std::endl;

  Point pt4( Point( -10, -10, 10 ) );
  bool pt4_inside = plane.extend( pt4 );
  ++nb, nbok += pt4_inside == false ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt4
               << " Plane=" << plane << std::endl;

  Point pt5 = pt0 + pt1 + pt2 + Point( 0, 0, 2 );
  bool pt5_inside = plane.extend( pt5 );
  ++nb, nbok += pt5_inside == true ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") add " << pt5
               << " Plane=" << plane << std::endl;

  NaivePlane plane2;
  plane2.init( 2, 100, 1, 1 );
  plane2.extend( Point( 10, 0, 0 ) );
  plane2.extend( Point( 0, 8, 0 ) );
  plane2.extend( Point( 0, 0, 6 ) );
  trace.info() << "(" << nbok << "/" << nb << ") "
               << " Plane2=" << plane2 << std::endl;

  ++nb, nbok += checkPlane<Integer,NaivePlane>( 11, 5, 19, 20, 100, 100 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkPlane<Integer,NaivePlane>( 11, 5, 19, 20, 100, 100 )"
               << std::endl;

  ++nb, nbok += checkGenericPlane<Integer,GenericNaivePlane>( 11, 5, 19, 20, 100, 100 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkGenericPlane<Integer,GenericNaivePlane>( 11, 5, 19, 20, 100, 100 )"
               << std::endl;
  ++nb, nbok += checkGenericPlane<Integer,GenericNaivePlane>( 17, 33, 7, 10, 100, 100 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkGenericPlane<Integer,GenericNaivePlane>( 17, 33, 7, 10, 100, 100 )"
               << std::endl;
  ++nb, nbok += checkPlane<Integer,NaivePlane>( 15, 8, 13, 15, 100, 100 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkPlane<Integer,NaivePlane>( 15, 8, 13, 15, 100, 100 )"
               << std::endl;
  ++nb, nbok += checkGenericPlane<Integer,GenericNaivePlane>( 15, 8, 13, 15, 100, 100 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkGenericPlane<Integer,GenericNaivePlane>( 15, 8, 13, 15, 100, 100 )"
               << std::endl;
  trace.endBlock();
  return nbok == nb;
}

template <typename NaivePlane>
bool 
checkManyPlanes( unsigned int diameter,
                 unsigned int nbplanes, 
                 unsigned int nbpoints )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef typename NaivePlane::InternalInteger Integer;
  stringstream ss (stringstream::out);
  ss << "Testing block: Diameter is " << diameter << ". Check " << nbplanes << " planes with " << nbpoints << " points each.";
  trace.beginBlock ( ss.str() );
  ++nb, nbok += checkPlanes<Integer,NaivePlane>( nbplanes, diameter, nbpoints ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb 
               << ") checkPlanes<Integer,NaivePlane>()"
               << std::endl;
  trace.endBlock();
  return nbok == nb;
}

/**
   NB (JOL): Unreliable.
*/
template <typename NaivePlane>
unsigned int maxDiameter( unsigned int min, unsigned int max )
{
  while ( min < max )
    {
      unsigned int middle = (min+max)/2;
      bool ok =  checkManyPlanes<NaivePlane>( middle, 2, 2000 );
      if ( ok ) min = middle+1;
      else      max = middle;
    }
  return min-1;
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int /*argc*/, char** /*argv*/ )
{
  using namespace Z3i;
  // Max diameter is ~20 for int32_t, ~500 for int64_t, any with BigInteger.
  trace.beginBlock ( "Testing class COBANaivePlane" );
  bool res = true 
    && testCOBANaivePlane()
    && checkManyPlanes<COBANaivePlane<Z3, DGtal::int32_t> >( 20, 100, 200 )
    && checkManyPlanes<COBANaivePlane<Z3, DGtal::int64_t> >( 500, 100, 200 )
    && checkManyPlanes<COBANaivePlane<Z3, DGtal::BigInteger> >( 10000, 10, 200 );
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  // trace.beginBlock ( "Max diameter for COBANaivePlane<Z3, int32_t>" );
  // unsigned int maxd = maxDiameter<COBANaivePlane<Z3, DGtal::int32_t> >( 10, 1000 );
  // trace.emphase() << maxd << endl;
  // trace.endBlock();
  // trace.beginBlock ( "Max diameter for COBANaivePlane<Z3, int64_t>" );
  // unsigned int maxd2 = maxDiameter<COBANaivePlane<Z3, DGtal::int32_t> >( 100, 100000 );
  // trace.emphase() << maxd2 << endl;
  // trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
