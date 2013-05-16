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

//#define CHECK_RES

template <typename Integer>
bool testDSLSubsegment( unsigned int nbtries, Integer m, Integer modx)
{
  typedef double Number;
  typedef DGtal::DSLSubsegment<Integer,Integer> DSLSubseg;
  typedef DGtal::DSLSubsegment<Integer,double> DSLSubsegD;
  
  typedef typename DSLSubseg::Point Point;
  
  DGtal::IntegerComputer<Integer> ic;
  
  // Point A(1,5);
  // Point B(6,9);
  // DSLSubseg DD(2,3,15,A,B);

  // std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;

  
  // std::cout << "# a b mu a1 b1 mu1 Ax Ay Bx By" << std::endl;
  
  long double timeTotalSubseg=0,timeTotalSubsegD=0;
  
  clock_t timeBeginSubseg, timeEndSubseg;
  clock_t timeBeginSubsegD, timeEndSubsegD;
  
  int nb = 0;
  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      // generate b as a power of 10
      // the parameters of the DSL can be expressed as (a,b,mu) with a,b,mu integers or (a/b,mu/b) as decimal numbers 
      Integer p(random() % m);
      
      Integer b = pow(10.0,p);
      
      Integer a( random() % b);
      
      std::cout << "p= " << p << " a=" <<  a << " b=" << b << std::endl; 

      Number alpha = (Number) a/(Number) b;
      
      Integer g = ic.gcd(a,b);
      a = a/g;
      b = b/g;
      
      if ( ic.gcd( a, b ) == 1 )
        {
	  nb ++;
          for ( unsigned int j = 0; j < 5; ++j )
            {
              //Integer mu = random() % (2*(Integer) pow(10.0,m));

	      Integer mu = random() % (2*b);
	      
	      Number beta = (Number) mu/(Number) b;
	      
	      
              for (Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % modx;
                  Integer x2 = x1 + 1+ random()%modx;
		  //Integer x2 = x1 + 1 + ( random() % modx );
                  
                  std::cout << "(" << a << "," << b << "," << mu << ") (" << alpha << "," << beta << ")" << std::endl;
		  
                  Integer y1 = ic.floorDiv(a*x1+mu,b);
                  Integer y2 = ic.floorDiv(a*x2+mu,b);
                  Point A = Point(x1,y1);
                  Point B = Point(x2,y2);
		  
		  trace.info() << A << " " << B << std::endl;
		  
		  // DSLSubsegment algorithm
		  
		  timeBeginSubseg = clock();
		  DSLSubseg D(a,b,mu,A,B);
		  timeEndSubseg = clock();
		  timeTotalSubseg += ((double)timeEndSubseg-(double)timeBeginSubseg)/(((double)CLOCKS_PER_SEC)/1000);
		  
		  std::cout << "res = " << "(" << D.aa << "," << D.bb << "," << D.Nu << ")" << std::endl;
		  // // DSLSubsegment algorithm using floating points
		  
		  // timeBeginSubsegD = clock();
		  // DSLSubsegD DD(alpha,beta,A,B);
		  // timeEndSubsegD = clock();
		  // timeTotalSubsegD += ((double)timeEndSubsegD-(double)timeBeginSubsegD)/(((double)CLOCKS_PER_SEC)/1000);
		  // std::cout << "res float = " << "(" << DD.aa << "," << DD.bb << "," << DD.Nu << ")" << std::endl;
		  
		  // assert(D.aa == DD.aa && D.bb == DD.bb && D.Nu == DD.Nu);
		 
		  
 
		  
		}
	      
	    }
	}
    }
  
  std::cout << " " << (long double) timeTotalSubseg/(nb*5*10);
  std::cout << " " << (long double) timeTotalSubsegD/(nb*5*10);
  
  
  return true;
}






///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::int32_t Integer;
  Integer m = 2; // b = 10^p with p <= m
  
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) :500;
  
  
  // for(Integer modx = 10; modx <=  modb;modx+=modx/4)
  //   //Integer  modx = 1000;
  //   {
  Integer modx = 10;
  std::cout << m << " " << modx << " ";
  testDSLSubsegment<Integer>( nbtries, m, modx);
  std::cout << std::endl;
  // }
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
