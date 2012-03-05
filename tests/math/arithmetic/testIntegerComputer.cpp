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
 * @file testIntegerComputer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class IntegerComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegerComputer.
///////////////////////////////////////////////////////////////////////////////

template <typename Integer>
bool testGCD( const IntegerComputer<Integer> & ic )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Integer a = random();
  Integer b = random();
  Integer g = ic.gcd( a, b );
  trace.info() << "GCD(" << a << "," << b << ")" 
               << " = " << g << std::endl;
  Integer ra = a % g;
  Integer rb = b % g;
  nbok += ic.isZero( ra ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << a << " % " << g << " == 0" << std::endl;
  nbok += ic.isZero( rb ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << b << " % " << g << " == 0" << std::endl;
  a /= g; b /= g;
  g = ic.gcd( a, b );
  nbok += g == Integer( 1 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "GCD(" << a << "," << b << ") == 1" << std::endl;
  Integer c = random(); 
  ++c; // avoids zero.
  a *= c; b *= c;
  ic.getGcd( g, a, b );
  nbok += g == c ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "GCD(" << a << "," << b << ") == " << c << std::endl;
  return nbok == nb;
}

template <typename Integer>
bool testCFrac( const IntegerComputer<Integer> & ic )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Integer a = random();
  Integer b = random();
  Integer g = ic.gcd( a, b );
  trace.info() << "a / b = " << a << " / " << b << std::endl;
  std::vector<Integer> quotients;
  Integer g2 = ic.getCFrac( quotients, a, b );
  nbok += g == g2 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << g << " == " << g2 << std::endl;
  trace.info() << a << " / " << b << " = ";
  for ( typename std::vector<Integer>::const_iterator it = quotients.begin(),
          it_end = quotients.end(); it != it_end; ++it )
    trace.info() << *it;
  trace.info() << std::endl;
  double da = NumberTraits<Integer>::castToDouble( a );
  double db = NumberTraits<Integer>::castToDouble( b );
  double q = floor( da / db );
  nbok += Integer( (int) q ) == quotients[ 0 ] ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << q << " == " << quotients[ 0 ] << std::endl;
  typedef typename IntegerComputer<Integer>::Point2I Point2I;
  Point2I p = ic.convergent( quotients, quotients.size() );
  nbok += p[ 0 ] == ( a / g ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "convergent p[ 0 ] " << p[ 0 ] << " == a / g " << std::endl;
  nbok += p[ 1 ] == ( b / g ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "convergent p[ 1 ] " << p[ 1 ] << " == b / g " << std::endl;
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testIntegerComputer()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef BigInteger Integer;
  IntegerComputer<Integer> ic;
  trace.beginBlock ( "Testing block: multiple random gcd." );
  for ( unsigned int i = 0; i < 100; ++i )
    {
      nbok += testGCD<Integer>( ic ) ? 1 : 0;
      nb++;
    }
  trace.info() << "(" << nbok << "/" << nb << ") gcd tests." << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing block: multiple random cfrac." );
  for ( unsigned int i = 0; i < 100; ++i )
    {
      nbok += testCFrac<Integer>( ic ) ? 1 : 0;
      nb++;
    }
  trace.info() << "(" << nbok << "/" << nb << ") cfrac tests." << std::endl;
  trace.endBlock();
  // Integer a = 123456;
  // Integer b = 6543210;
  // Integer g = ic.gcd( a, b );
  // trace.info() << "GCD(" << a << "," << b << ")" 
  //              << " = " << g << std::endl;
  // a /= g; b /= g;
  // g = ic.gcd( a, b );
  // nbok += g == Integer( 1 ) ? 1 : 0; 
  // nb++;
  // trace.info() << "(" << nbok << "/" << nb << ") "
  //              << "GCD(" << a << "," << b << ") == 1" << std::endl;
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class IntegerComputer" );
  bool res = testIntegerComputer(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
