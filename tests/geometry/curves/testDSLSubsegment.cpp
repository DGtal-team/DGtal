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

//#include <gmpxx.h>

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DSLSubsegment.
///////////////////////////////////////////////////////////////////////////////

//#define CHECK_RES

template <typename Integer>
bool testDSLSubsegment( unsigned int nbtries, Integer modb, Integer modx)
{
  //typedef mpf_class Number;
  typedef long double Number;
  typedef DGtal::DSLSubsegment<Integer,Integer> DSLSubseg;
  typedef DGtal::DSLSubsegment<Integer,Number> DSLSubsegD;
  

  typedef ArithDSSIterator<Integer,8> DSSIterator;
  typedef ArithmeticalDSS<DSSIterator,Integer,8> ArithDSS;
  

  typedef typename DSLSubseg::Point Point;
  
  DGtal::IntegerComputer<Integer> ic;
  
  // Point A(1,5);
  // Point B(6,9);
  // DSLSubseg DD(2,3,15,A,B);

  // std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
  
  Integer b;
  
  // std::cout << "# a b mu a1 b1 mu1 Ax Ay Bx By" << std::endl;
  
  long double timeTotalSubseg=0,timeTotalSubsegD=0, timeTotalDSS = 0, timeTotalCH = 0;
  
  clock_t timeBeginSubseg, timeEndSubseg;
  clock_t timeBeginSubsegD, timeEndSubsegD;
  clock_t timeBeginDSS, timeEndDSS;
  clock_t timeBeginCH, timeEndCH;
  
  int nb = 0;
  int nberrors = 0;
  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      // generate b as a power of 10
      // the parameters of the DSL can be expressed as (a,b,mu) with a,b,mu integers or (a/b,mu/b) as decimal numbers 
      // SmallInteger p(random() % m);
      
      // Integer b = pow(10.0,p);
      
       Integer b( random() % modb + 1 );
       Integer a( random() % b +1);
       
       
       if ( ic.gcd( a, b ) == 1 )
        {
	  
          for ( unsigned int j = 0; j < 5; ++j )
            {
              //Integer mu = random() % (2*(Integer) pow(10.0,m));

	      Integer mu = random() % (2*modb);
	      
	      Number beta = (Number) mu/(Number) b;
	      //Number beta((double) mu/b,500);
	      
              for (Integer x = 0; x < 10; ++x )
                {
                  //nb ++;
		  Integer x1 = random() % modx;
                  //Integer x2 = x1 + 1+ random()%modx;
		  Integer x2 = x1 + 1+ modx;
		  //Integer x2 = x1 + 1 + ( random() % modx );
                  
		  // std::cout << "(" << a << "," << b << "," << mu << ")\n" ; //(" << alpha << "," << beta << ")" << std::endl;
		  
                  Integer y1 = ic.floorDiv(a*x1+mu,b);
                  Integer y2 = ic.floorDiv(a*x2+mu,b);
                  Point A = Point(x1,y1);
                  Point B = Point(x2,y2);
		  
		  //trace.info() << "Points " << A << " " << B << std::endl;
		  
		  // DSLSubsegment algorithm
		  
		  if(B[0]-A[0] < 2*b) // reject easy cases when the segment contains a period of the DSL
		    {
		      std::cout << "(" << a << "," << b << "," << mu << ")\n" ; //(" << alpha << "," << beta << ")" << std::endl;
		      trace.info() << "Points " << A << " " << B << std::endl;
		      nb++;
		      timeBeginSubseg = clock();
		      DSLSubseg D(a,b,mu,A,B);
		      timeEndSubseg = clock();
		      timeTotalSubseg += ((double)timeEndSubseg-(double)timeBeginSubseg)/(((double)CLOCKS_PER_SEC)/1000);
		      
		      std::cout << "res = " << "(" << D.aa << "," << D.bb << "," << D.Nu << ")" << std::endl;

		  // // DSLSubsegment algorithm using floating points
		  // timeBeginSubsegD = clock();
		  // Number precision = (double) 1/(2*b);
		  // DSLSubsegD DD(alpha,beta,A,B,precision);
		  // timeEndSubsegD = clock();
		  //timeTotalSubsegD += ((double)timeEndSubsegD-(double)timeBeginSubsegD)/(((double)CLOCKS_PER_SEC)/1000);
		  //std::cout << "res float = " << "(" << DD.aa << "," << DD.bb << "," << DD.Nu << ")" << std::endl;
		  
		  // Compare both results
		  
		  // assert(D.aa == DD.aa && D.bb == DD.bb && D.Nu == DD.Nu);
		  // if(D.aa == DD.aa && D.bb == DD.bb && D.Nu == DD.Nu)
		  //   timeTotalSubsegD += ((double)timeEndSubsegD-(double)timeBeginSubsegD)/(((double)CLOCKS_PER_SEC)/1000);
		  // else
		  //   nberrors++;
		  

		  // Computation of the reduced parameters of Charrier & Buzer
		      
		      timeBeginCH = clock();
		      DSLSubseg DD(a,b,mu,A,B,1);
		      assert(D.aa == DD.aa && D.bb == DD.bb && D.Nu == DD.Nu);
		      timeEndCH = clock();
		      timeTotalCH += ((double)timeEndCH-(double)timeBeginCH)/(((double)CLOCKS_PER_SEC)/1000);
		    }
		  
#ifdef CHECK_RES
		  // Check if the result is ok comparing with ArithmeticalDSS recognition algorithm
		  DSSIterator  it(a,b,-mu,A);
                  ArithDSS myDSS(it);
                  
		  timeBeginDSS = clock();
                  while ( (*(myDSS.end()))[0] <=x2 && myDSS.extendForward())
                    {}
		  timeEndDSS = clock();
		  
                  //std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		  
		  		  
                  if(D.aa != myDSS.getA() || D.bb != myDSS.getB() || D.Nu != - myDSS.getMu())
		    {
		      std::cout << "ERROR " << std::endl;
		      std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;    
		      std::cout << "aa=" << D.aa << " bb=" << D.bb << " Nu=" << D.Nu << std::endl;
		      std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		      assert(D.aa == myDSS.getA() && D.bb == myDSS.getB() && D.Nu == - myDSS.getMu());
		    }
		  
		  timeTotalDSS += ((double)timeEndDSS-(double)timeBeginDSS)/((double)CLOCKS_PER_SEC)*1000;
#endif CHECK_RES
		  
		  
		  
		  
		}
	      
	    }
	}
    }
  
  std::cout << nb ;
  std::cout << " " << (long double) timeTotalSubseg/(nb);
  std::cout << " " << (long double) timeTotalCH/(nb);
  //std::cout << " " << (long double) timeTotalSubsegD/((nb-nberrors));
  
  
  return true;
}






///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::int64_t Integer;
  

  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) :200;
  
  
  Integer modb = 10000000000;
  
  for(Integer i = 10; i<modb ;i*=10) 
    for(Integer modx = 10; modx <=  i;modx+=modx/5)
      //Integer  modx = 1000;
      {
	std::cout << i << " " << modx << " ";
	testDSLSubsegment<Integer>( nbtries,  i, modx);
	std::cout << std::endl;
      }
  
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
