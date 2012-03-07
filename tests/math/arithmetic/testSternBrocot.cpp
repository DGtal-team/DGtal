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
 * @file testSternBrocot.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class SternBrocot.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
#include "DGtal/math/arithmetic/SternBrocot.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SternBrocot.
///////////////////////////////////////////////////////////////////////////////

template <typename SB>
bool testInitFraction()
{
  typedef typename SB::Integer Integer;
  typedef typename SB::Fraction Fraction;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Integer p = random() / 10000;
  Integer q = random() / 10000;
  trace.beginBlock ( "Testing block: init fraction." );
  IntegerComputer<Integer> ic;
  Integer g = ic.gcd( p, q );
  p /= g;
  q /= g;
  Fraction f1 = SB::fraction( p, q );
  trace.info() << "p / q = " << p << " / " << q << std::endl;
  trace.info() << "f1 = ";
  SB::display( trace.info(), f1 );
  trace.info() << std::endl;
  nbok += ( ( p == f1.p() ) && ( q == f1.q() ) ) ? 1 : 0;
  ++nb;
  trace.info() << "(" << nbok << "/" << nb << ") " 
               << "( ( p == f1.p() ) && ( q == f1.q() ) )"
               << std::endl;
  trace.info() << "- nbFractions = " << SB::nbFractions << std::endl;
  trace.endBlock();

  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testSternBrocot()
{
  unsigned int nbtests = 100;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef BigInteger Integer;
  typedef SternBrocot<Integer,int32_t> SB;
  typedef typename SB::Fraction Fraction;
  trace.beginBlock ( "Testing block: init fractions." );
  for ( unsigned int i = 0; i < nbtests; ++i )
    {
      nbok += testInitFraction<SB>() ? 1 : 0;
      nb++;
    }
  trace.info() << "(" << nbok << "/" << nb << ") init fractions." << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing block: number of fractions." );
  trace.info() << "- nbFractions = " << SB::nbFractions << std::endl;
  trace.endBlock();

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SternBrocot" );
  bool res = testSternBrocot(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
