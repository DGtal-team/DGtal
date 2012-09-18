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
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

#include "DGtal/geometry/curves/DSLSubsegment.h"
#include "DGtal/arithmetic/StandardDSLQ0.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/arithmetic/IntegerComputer.h"
#include "DGtal/arithmetic/SternBrocot.h"
#include "DGtal/arithmetic/Pattern.h"


using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DSLSubsegment.
///////////////////////////////////////////////////////////////////////////////


template <typename Fraction>
bool testDSLSubsegment( unsigned int nbtries, typename Fraction::Integer moda, typename Fraction::Integer modb, typename Fraction::Integer modx, int algo )
{
  typedef StandardDSLQ0<Fraction> DSL;
  typedef typename Fraction::Integer Integer;
  typedef DGtal::DSLSubsegment<Integer> DSLSubseg;
  typedef typename DSLSubseg::Point Point;


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
      //Integer a( random() % moda + 1 );
      Integer b( random() % modb + 1 );
      Integer a( random() % b +1);
      //assert(a<=b);
      if ( ic.gcd( a, b ) == 1 )
        {
          for ( unsigned int j = 0; j < 5; ++j )
            {
	      Integer mu = random() % (moda+modb);
              DSL D( a, b, mu );
	      //PointD param((CoordinateType) a/b, (CoordinateType) mu/b);
	      for (Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % modx;
                  Integer x2 = x1 + 1 + ( random() % modx );
		
		  if(algo == 1 || algo ==2)
		    {
		      Point A = D.lowestY( x1 );
		      Point B = D.lowestY( x2 );
		      if(algo == 1)
			D.smartDSS(A,B);
		      else
			D.reversedSmartDSS(A,B);
		      //checkSmartDSSDSLQ0<DSL>( D, A, B );
		    }
		  else
		    {
		      Integer y1 = ic.floorDiv(a*x1+mu,b);
		      Integer y2 = ic.floorDiv(a*x2+mu,b);
		      Point A = Point(x1,y1);
		      Point B = Point(x2,y2);
		      DSLSubseg DD(a,b,mu,A,B);
		    }
		  
		}
            }
        }
    }
  
  timeEnd = clock();
  long double CPUTime;
  CPUTime =  ((double)timeEnd-(double)timeBegin)/((double)CLOCKS_PER_SEC)*1000;  
  
  // std::cout << "DSLSubsegment: CPU Time ellapsed = " << CPUTime << " - Time/test = = " << (long double) CPUTime/(nbtries*5*10) << std::endl;
  //std::cout << "Use leaning points: " << (double) useLeaningPoints/(nbtries*5*10);
  
  
  std::cout << " " << (long double) CPUTime/(nbtries*5*10) ;
 

  return true;
}



// bool testDSLSubsegment()
// {
//   typedef DGtal::int64_t Integer;
//   typedef DGtal::DSLSubsegment<Integer> DSLSubseg;
//   typedef DSLSubseg::Point Point;
  
//   Point A(0,0);
//   Point B(7,1);
  
//   DSLSubseg D(3,16,7,A,B);
  
//   return true;
// }

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DSLSubsegment" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;
  
  typedef SternBrocot<DGtal::int64_t,DGtal::int32_t> SB;
  typedef SB::Fraction Fraction;
  typedef Fraction::Integer Integer;
  

  Integer modb = 1000000000;
  Integer moda = modb;
  
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) : 20000;
  
  // for(Integer modb = 100; modb < 1000000000000; modb*=10)
  //{
  //for(Integer modx = 10; modx < modb;modx*=2)
	{
	  moda = modb;
	  //std::cout << modb << " " << modx ;
	  Integer modx = 2*5242880;
	  testDSLSubsegment<Fraction>( nbtries, moda, modb, modx,0);
	  //testDSLSubsegment<Fraction>( nbtries, moda, modb, modx,1 );
	  testDSLSubsegment<Fraction>( nbtries, moda, modb, modx,2 );
	  std::cout << std::endl;
	}
      //std::cout << std::endl;
      //}
  
  //bool res = testDSLSubsegment(); // && ... other tests
  // trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  //trace.endBlock();
  //return res ? 0 : 1;
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
