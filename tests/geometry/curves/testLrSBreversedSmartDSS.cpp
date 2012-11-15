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
 * @file testDSLSubsegment.cpp
 * @ingroup Tests
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/07/17
 *
 * Functions for testing class DSLSubsegment.
 *
  */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

#include <map>
#include "DGtal/geometry/curves/DSLSubsegment.h"
#include "DGtal/arithmetic/StandardDSLQ0.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/arithmetic/SternBrocot.h"
#include "DGtal/arithmetic/LighterSternBrocot.h"
#include "DGtal/arithmetic/LightSternBrocot.h"
#include "DGtal/arithmetic/Pattern.h"


using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DSLSubsegment.
///////////////////////////////////////////////////////////////////////////////


template <typename Fraction>
bool testDSLSubsegment( unsigned int nbtries, typename Fraction::Integer moda, typename Fraction::Integer modb, typename Fraction::Integer modx)
{
  typedef StandardDSLQ0<Fraction> DSL;
  typedef typename Fraction::Integer Integer;
  typedef typename DSL::Point Point;
  
  typedef typename Fraction::Quotient Quotient;
  typedef typename DSL::ConstIterator ConstIterator;
  typedef typename DSL::Point2I Point2I;
  typedef typename DSL::Vector2I Vector2I;
  
  
  DGtal::IntegerComputer<Integer> ic;
  
  // std::cout << "# a b mu a1 b1 mu1 Ax Ay Bx By" << std::endl;
  
  clock_t timeBegin, timeEnd;
  timeBegin = clock();
  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      Integer b( random() % modb + 1 );
      Integer a( random() % b +1);
   
      if ( ic.gcd( a, b ) == 1 )
        {
          for ( unsigned int j = 0; j < 5; ++j )
            {
	      Integer mu = random() % (moda+modb);
              DSL D( a, b, mu );
	      
	      for (Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % modx;
                  Integer x2 = x1 + 1 + ( random() % modx );
		
		  //std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;
		  
		  Point A = D.lowestY( x1 );
		  Point B = D.lowestY( x2 );
		  D.reversedSmartDSS(A,B);
		  
		}
            }
        }
    }
  
  timeEnd = clock();
  long double CPUTime;
  CPUTime =  ((double)timeEnd-(double)timeBegin)/((double)CLOCKS_PER_SEC)*1000;  
  
  std::cout << " " << (long double) CPUTime/(nbtries*5*10) ;
 

  return true;
}




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  
  typedef LighterSternBrocot<DGtal::int64_t,DGtal::int32_t, StdMapRebinder> LrSB;
  
  typedef LrSB::Fraction Fraction;
  typedef Fraction::Integer Integer;
  
  Integer modb = 1000000000;
  Integer moda = modb;
  
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) : 1000;
  
  for(Integer modx = 10; modx < modb;modx*=2)
    {
      moda = modb;
      std::cout << modb << " " << modx << " ";
      testDSLSubsegment<Fraction>( nbtries, moda, modb, modx);
      std::cout << std::endl;
    }
  
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
