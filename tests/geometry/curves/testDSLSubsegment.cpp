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
#include "DGtal/geometry/curves/ArithDSSIterator.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DSLSubsegment.
///////////////////////////////////////////////////////////////////////////////

#define CHECK_RES


template <typename Integer>
bool testDSLSubsegment( unsigned int nbtries, Integer moda, Integer modb, Integer modx)
{
  typedef DGtal::DSLSubsegment<Integer> DSLSubseg;
  typedef typename DSLSubseg::Point Point;
  
  
  typedef ArithDSSIterator<Integer,8> DSSIterator;
  typedef ArithmeticalDSS<DSSIterator,Integer,8> ArithDSS;
  
  
  DGtal::IntegerComputer<Integer> ic;
  
  // Point A(1,5);
  // Point B(6,9);
  // DSLSubseg DD(2,3,15,A,B);

  // std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;

  
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
              //DSL D( a, b, mu );
              
              for (Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % modx;
                  Integer x2 = x1 + 1 + ( random() % modx );
                  
                  //std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;
                  
                  Integer y1 = ic.floorDiv(a*x1+mu,b);
                  Integer y2 = ic.floorDiv(a*x2+mu,b);
                  Point A = Point(x1,y1);
                  Point B = Point(x2,y2);
                  DSLSubseg DD(a,b,mu,A,B);
                  
                  //std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
                  
#ifdef CHECK_RES
                  DSSIterator  it(a,b,-mu,A);
                  ArithDSS myDSS(it);
                  

                  while ( (*(myDSS.end()))[0] <=x2 && myDSS.extendForward())
                    {}
                  
                  //std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
                  if(DD.aa != myDSS.getA() || DD.bb != myDSS.getB() || DD.Nu != - myDSS.getMu())
		    {
		      std::cout << "ERROR " << std::endl;
		      std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;    
		      std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
		      std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		    }
		  #endif CHECK_RES

		  
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
  typedef DGtal::int64_t Integer;
  
  
  Integer modb = 10000000;
  Integer moda = modb;
  
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) :100;

 
  for(Integer modx = 10; modx <=  modb;modx*=2)
    //Integer  modx = 1000;
  {
  	  moda = modb;
  	  std::cout << modb << " " << modx << " ";
    	  testDSLSubsegment<Integer>( nbtries, moda, modb, modx);
  	  std::cout << std::endl;
  	}
    return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
