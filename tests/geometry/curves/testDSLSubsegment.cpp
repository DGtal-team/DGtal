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


template <typename Fraction>
bool testDSLSubsegment( unsigned int nbtries, typename Fraction::Integer moda, typename Fraction::Integer modb, typename Fraction::Integer modx)
{
  typedef typename Fraction::Integer Integer;
  
  typedef DGtal::DSLSubsegment<Integer> DSLSubseg;
  typedef typename DSLSubseg::Point Point;
  
  
  typedef ArithDSSIterator<Integer,8> DSSIterator;
  typedef ArithmeticalDSS<DSSIterator,Integer,8> ArithDSS;
  
  
  typedef StandardDSLQ0<Fraction> DSL;
  typedef typename Fraction::Quotient Quotient;
  typedef typename DSL::Point PointDSL;
  typedef typename DSL::ConstIterator ConstIterator;
  typedef typename DSL::Point2I Point2I;
  typedef typename DSL::Vector2I Vector2I;


  
  DGtal::IntegerComputer<Integer> ic;
  
  // Point A(1,5);
  // Point B(6,9);
  // DSLSubseg DD(2,3,15,A,B);

  // std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;

  
  // std::cout << "# a b mu a1 b1 mu1 Ax Ay Bx By" << std::endl;
  
  long double timeTotalSubseg=0,timeTotalDSS=0,timeTotalSmartDSS=0,timeTotalReversedSmartDSS=0;
  
  clock_t timeBeginSubseg, timeEndSubseg;
  clock_t timeBeginSmartDSS, timeEndSmartDSS;
  clock_t timeBeginReversedSmartDSS, timeEndReversedSmartDSS;
  clock_t timeBeginDSS, timeEndDSS;
  
  int nb = 0;
  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      Integer b( random() % modb + 1 );
      Integer a( random() % b +1);
      
      if ( ic.gcd( a, b ) == 1 )
        {
	  nb ++;
          for ( unsigned int j = 0; j < 5; ++j )
            {
              Integer mu = random() % (moda+modb);
              DSL D( a, b, mu );
              
              for (Integer x = 0; x < 10; ++x )
                {
                  Integer x1 = random() % modx;
                  Integer x2 = x1 + 1+ modx;
		  //Integer x2 = x1 + 1 + ( random() % modx );
                  
                  //std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;
		  
                  Integer y1 = ic.floorDiv(a*x1+mu,b);
                  Integer y2 = ic.floorDiv(a*x2+mu,b);
                  Point A = Point(x1,y1);
                  Point B = Point(x2,y2);
		  
		  // DSLSubsegment algorithm

		  // timeBeginSubseg = clock();
		  // DSLSubseg DD(a,b,mu,A,B);
		  // timeEndSubseg = clock();
		  // timeTotalSubseg += ((double)timeEndSubseg-(double)timeBeginSubseg)/(((double)CLOCKS_PER_SEC)/1000);
		  
		  
		  PointDSL AA = D.lowestY( x1 );
		  PointDSL BB = D.lowestY( x2 );
		  
		  // SmartDSS algorithm
		  
		  // timeBeginSmartDSS = clock();
		  // D.smartDSS(AA,BB);
		  // timeEndSmartDSS = clock();
		  // timeTotalSmartDSS += ((double)timeEndSmartDSSDS-(double)timeBeginSmartDSS)/(((double)CLOCKS_PER_SEC)/1000);
		  
		  // ReversedSmartDSS algorithm
		  
		  timeBeginReversedSmartDSS = clock();
		  DSL S = D.reversedSmartDSS(AA,BB);
		  timeEndReversedSmartDSS = clock();
		  timeTotalReversedSmartDSS += ((double)timeEndReversedSmartDSS-(double)timeBeginReversedSmartDSS)/(((double)CLOCKS_PER_SEC)/1000);
		  
		  // DSLSubsegment algorithm for 4-connected DSL
		  
		  // Application of an horizontal shear transform
		  Point A2 = AA;
		  A2[0] += A2[1];
		  Point B2 = BB;
		  B2[0] += B2[1];
		  
	  
		  timeBeginSubseg = clock();
		  DSLSubseg D2(a,a+b,-mu,A2,B2); // DSL algorithm works with the definition 0 <= ab -by + mu < b whereas reversedSmartDSS uses mu <= ab-by < mu + b => -mu is introduced in order to compare the results
		  timeEndSubseg = clock();
		  timeTotalSubseg += ((double)timeEndSubseg-(double)timeBeginSubseg)/(((double)CLOCKS_PER_SEC)/1000);
		  
		  // The result is (aa,bb-aa, nu)
		  // std::cout << "DSLSubseg : a2=" << D2.aa << " b2=" << D2.bb-D2.aa << " Nu=" << D2.Nu << std::endl;
		  // std::cout << "Reversed  : a =" << S.a() << " b =" << S.b() << " Mu =" << S.mu() << std::endl;
	
		  assert(D2.aa==S.a() && (D2.bb-D2.aa)==S.b() && D2.Nu==-S.mu());
		  
		  /// std::cout << ((double)timeEndSubseg-(double)timeBeginSubseg) << " " << ((double)CLOCKS_PER_SEC)/1000 << " " << std::endl;
	  
		  
                  //Std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
                  
		  
#ifdef CHECK_RES
		  // Check if the result is ok comparing with ArithmeticalDSS recognition algorithm
		  DSSIterator  it(a,b,-mu,A);
                  ArithDSS myDSS(it);
                  
		  timeBeginDSS = clock();
                  while ( (*(myDSS.end()))[0] <=x2 && myDSS.extendForward())
                    {}
		  timeEndDSS = clock();

                  //std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
                  if(DD.aa != myDSS.getA() || DD.bb != myDSS.getB() || DD.Nu != - myDSS.getMu())
		    {
		      std::cout << "ERROR " << std::endl;
		      std::cout << a << " " << b << " " << mu << " " << x1 << " " << x2 << std::endl;    
		      std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
		      std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		      break;
		    }
		  timeTotalDSS += ((double)timeEndDSS-(double)timeBeginDSS)/((double)CLOCKS_PER_SEC)*1000;
#endif CHECK_RES
		  
		  
		  
		 
		  
		}
            }
        }
      
    }
  
  // timeEnd = clock();
  // long double CPUTime;
  // CPUTime =  ((double)timeEnd-(double)timeBegin)/((double)CLOCKS_PER_SEC)*1000;  
  
  //std::cout << " " << (long double) CPUTime/(nb*5*10) ;
  
  std::cout << " " << (long double) timeTotalSubseg/(nb*5*10);
  //std::cout << " " << (long double) timeTotalSmartDSS/(nb*5*10);
  std::cout << " " << (long double) timeTotalReversedSmartDSS/(nb*5*10);
  
#ifdef CHECK_RES
  std::cout << " " << (long double) timeTotalDSS/(nb*5*10);
#endif

  return true;
}






///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  //typedef DGtal::int64_t Integer;
  //typedef DGtal::BigInteger Integer;
  
  // typedef DGtal::DSLSubsegment<Integer> DSLSubseg;
  // typedef DSLSubseg::Point Point;
  
  // DGtal::IntegerComputer<Integer> ic;


  // Integer a = 139;
  // Integer b = 7919;
  // Integer mu = -5258;
  // Integer x1 = 2829;
  // Integer x2 = 5800;
  
  // Integer y1 = ic.floorDiv(a*x1+mu,b);
  // Integer y2 = ic.floorDiv(a*x2+mu,b);
  // Point A = Point(x1,y1);
  // Point B = Point(x2,y2);
  
  // clock_t timeBegin, timeEnd;
  // timeBegin = clock();

  // DSLSubseg DD(a,b,mu,A,B);

  // timeEnd = clock();
  // long double CPUTime;
  // CPUTime =  ((double)timeEnd-(double)timeBegin)/((double)CLOCKS_PER_SEC);  
  
  
  // std::cout << DD.aa << " " << DD.bb << " " << DD.Nu << "time = " << CPUTime << std::endl;
  
  // typedef ArithDSSIterator<Integer,8> DSSIterator;
  // typedef ArithmeticalDSS<DSSIterator,Integer,8> ArithDSS;

  // DSSIterator  it(a,b,-mu,A);
  // ArithDSS myDSS(it);
  
  
  // while ( (*(myDSS.end()))[0] <=x2 && myDSS.extendForward())
  //   {}
  
  // std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		     
  
  typedef LighterSternBrocot<DGtal::int64_t,DGtal::int64_t, StdMapRebinder> LrSB;
  typedef LightSternBrocot<DGtal::int64_t,DGtal::int64_t> LSB;
  typedef SternBrocot<DGtal::int64_t,DGtal::int64_t> SB;
  
  
  //typedef SternBrocot<DGtal::int64_t,DGtal::int32_t> SB;
  typedef LrSB::Fraction Fraction;
  typedef Fraction::Integer Integer;
  
  
  Integer modb = 1000000;
  Integer moda = modb;
  
  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) :500;
  
  
  for(Integer modx = 10; modx <=  modb;modx+=modx/4)
    //Integer  modx = 1000;
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
