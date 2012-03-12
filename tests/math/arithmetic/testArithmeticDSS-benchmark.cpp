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
 * @file testArithmeticDSS.cpp
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
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
#include "DGtal/math/arithmetic/SternBrocot.h"
#include "DGtal/math/arithmetic/Pattern.h"
#include "DGtal/math/arithmetic/StandardDSLQ0.h"
#include "DGtal/geometry/curves/representation/ArithmeticalDSS.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SternBrocot.
///////////////////////////////////////////////////////////////////////////////

template <typename DSL>
bool checkSubArithmeticDSS( const DSL & D,
                            const typename DSL::Point & A, 
                            const typename DSL::Point & B ) 
{
  typedef typename DSL::Fraction Fraction;
  typedef typename DSL::Integer Integer;
  typedef typename DSL::Size Size;
  typedef typename DSL::Point Point;
  typedef typename DSL::ConstIterator ConstIterator;
  typedef typename DSL::Point2I Point2I;
  typedef typename DSL::Vector2I Vector2I;
  typedef ArithmeticalDSS<ConstIterator, Integer, 4> ADSS;

  ConstIterator it = D.begin( A );
  ConstIterator it_end = D.end( B );
  ADSS dss;
  dss.init( it );
  while ( ( dss.end() != it_end )
          && ( dss.extendForward() ) ) {}
  std::cout << D.a() << " " << D.b() << " " << D.mu() << " "
            << dss.getA() << " " << dss.getB() << " " << dss.getMu() 
            << std::endl;

  return true;
}

template <typename Fraction>
bool testSubArithmeticDSS( unsigned int nbtries )
{
  typedef StandardDSLQ0<Fraction> DSL;
  typedef typename Fraction::Integer Integer;
  typedef typename Fraction::Size Size;
  typedef typename DSL::Point Point;
  typedef typename DSL::ConstIterator ConstIterator;
  typedef typename DSL::Point2I Point2I;
  typedef typename DSL::Vector2I Vector2I;
  typedef ArithmeticalDSS<ConstIterator, Integer, 4> ADSS;
  IntegerComputer<Integer> ic;

  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      Integer a( random() % 12000 + 1 );
      Integer b( random() % 12000 + 1 );
      if ( ic.gcd( a, b ) == 1 )
        {
          for ( Integer mu = 0; mu < 5; ++mu )
            {
              DSL D( a, b, random() % 10000 );
              for ( Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % 1000;
                  Integer x2 = x1 + 1 + ( random() % 1000 );
                  Point A = D.lowestY( x1 );
                  Point B = D.lowestY( x2 );
                  checkSubArithmeticDSS<DSL>( D, A, B );
                }
            }
        }
    }
  return true;
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int , char** )
{
  typedef SternBrocot<DGtal::int64_t,DGtal::int32_t> SB;
  typedef SB::Fraction Fraction;
  testSubArithmeticDSS<Fraction>( 10000 );
  return true;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
