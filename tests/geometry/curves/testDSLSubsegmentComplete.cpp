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

template <typename Integer,typename Fraction>
bool testDSLSubsegment( unsigned int nbtries, Integer modb, Integer modx)
{
  //typedef mpf_class Number;
  typedef long double Number;
  typedef DGtal::DSLSubsegment<Integer,Integer> DSLSubseg;
  typedef DGtal::DSLSubsegment<Integer,Number> DSLSubsegD;
  

  typedef ArithDSSIterator<Integer,8> DSSIterator;
  typedef ArithmeticalDSS<DSSIterator,Integer,8> ArithDSS;
 
  typedef typename DSLSubseg::Point Point;
  
  typedef StandardDSLQ0<Fraction> DSL;
  typedef typename DSL::Point PointDSL;
  
  DGtal::IntegerComputer<Integer> ic;
  
  // Point A(1,5);
  // Point B(6,9);
  // DSLSubseg DD(2,3,15,A,B);

  // std::cout << "aa=" << DD.aa << " bb=" << DD.bb << " Nu=" << DD.Nu << std::endl;
  
  Integer b;
  
  // std::cout << "# a b mu a1 b1 mu1 Ax Ay Bx By" << std::endl;
  
  long double timeTotalSubseg=0,timeTotalSubseg4=0,timeTotalSubsegD=0, timeTotalDSS = 0, timeTotalCH = 0, timeTotalSmartDSS=0,timeTotalReversedSmartDSS=0;
  
  clock_t timeBeginSubseg, timeEndSubseg;
  clock_t timeBeginSubseg4, timeEndSubseg4;
  clock_t timeBeginSubsegD, timeEndSubsegD;
  clock_t timeBeginDSS, timeEndDSS;
  clock_t timeBeginCH, timeEndCH;
  clock_t timeBeginSmartDSS, timeEndSmartDSS;
  clock_t timeBeginReversedSmartDSS, timeEndReversedSmartDSS;
  
  
  int nb = 0; int nbLocRay = 0;
  int nberrors = 0;
  for ( unsigned int i = 0; i < nbtries; ++i )
    {
      // generate b as a power of 10
      // the parameters of the DSL can be expressed as (a,b,mu) with a,b,mu integers or (a/b,mu/b) as decimal numbers 
      // SmallInteger p(random() % m);
      
      // Integer b = pow(10.0,p);
      
      // b between 0 and mod b
      //Integer b( random() % modb + 1 );
      
      Integer var(random()%(modb/2) + 1);
      
      Integer b;
      if(var%2==0)
	b = modb + var;
      else
	b = modb - var;
      
      // fixed b
      //Integer b(modb);
      
      Integer a( random() % b +1);
      
      while(ic.gcd(a,b) !=1)
	a = random() %b +1;
      
       
      //if ( ic.gcd( a, b ) == 1 )
        {
	  
          for ( unsigned int j = 0; j < 5; ++j )
            {
              //Integer mu = random() % (2*(Integer) pow(10.0,m));
	      
	      Integer mu = random() % (2*modb);
	      DSL D( a, b, mu );
	      //Number beta((double) mu/b,500);
	      
              for (Integer x = 0; x < 10; ++x )
                {
                  //nb ++;
		  Integer x1 = random() % modx;
                  //Integer x2 = x1 + 1+ random()%modx;
		  Integer x2 = x1 + 1+ modx;
		  //Integer x2 = x1 + 1 + ( random() % modx );
                  
		  // typedef typename DSL::Point PointDSL;std::cout << "(" << a << "," << b << "," << mu << ")\n" ; //(" << alpha << "," << beta << ")" << std::endl;
		  
                  Integer y1 = ic.floorDiv(a*x1+mu,b);
                  Integer y2 = ic.floorDiv(a*x2+mu,b);
                  Point A = Point(x1,y1);
                  Point B = Point(x2,y2);
		  
		  //trace.info() << "Points " << A << " " << B << std::endl;
		  
		  // DSLSubsegment algorithm
		  
		  if(B[0]-A[0] < 2*b) // reject easy cases when the segment contains a period of the DSL
		    {
		      //std::cout << "(" << a << "," << b << "," << mu << ")\n" ; //(" << alpha << "," << beta << ")" << std::endl;
		      //trace.info() << "Points " << A << " " << B << std::endl;
		      nb++;
		      
		      bool locRay;
		      
		      // DSLSubsegment with Farey Fan
		      timeBeginSubseg = clock();
		      DSLSubseg DSLsub(a,b,mu,A,B,&locRay);
		      timeEndSubseg = clock();
		      timeTotalSubseg += ((double)timeEndSubseg-(double)timeBeginSubseg)/(((double)CLOCKS_PER_SEC)/1000);
		      
		      
		      nbLocRay +=locRay;
		      //std::cout << "res = " << "(" << D.aa << "," << D.bb << "," << D.Nu << ")" << std::endl;
		      
		      PointDSL AA = D.lowestY( x1 );
		      PointDSL BB = D.lowestY( x2 );
		      
		      // // SmartDSS algorithm
		      
		      // timeBeginSmartDSS = clock();
		      // D.smartDSS(AA,BB);
		      // timeEndSmartDSS = clock();
		      // timeTotalSmartDSS += ((double)timeEndSmartDSS-(double)timeBeginSmartDSS)/(((double)CLOCKS_PER_SEC)/1000);
		  
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
		      
		      bool aBool;
		      
		      timeBeginSubseg4 = clock();
		      DSLSubseg D2(a,a+b,-mu,A2,B2,&aBool); // DSL algorithm works with the definition 0 <= ab -by + mu < b whereas reversedSmartDSS uses mu <= ab-by < mu + b => -mu is introduced in order to compare the results
		      timeEndSubseg4 = clock();
		      timeTotalSubseg4 += ((double)timeEndSubseg4-(double)timeBeginSubseg4)/(((double)CLOCKS_PER_SEC)/1000);
		      
		      // The result is (aa,bb-aa, nu)
		      // std::cout << "DSLSubseg : a2=" << D2.aa << " b2=" << D2.bb-D2.aa << " Nu=" << D2.Nu << std::endl;
		      // std::cout << "Reversed  : a =" << S.a() << " b =" << S.b() << " Mu =" << S.mu() << std::endl;
		      
		      // Compare results of DSLsubseg4 and reversedSmartDSS
		      assert(D2.aa==S.a() && (D2.bb-D2.aa)==S.b() && D2.Nu==-S.mu());
		      
		      
		      
		      // Computation of the local convex hulls
		      
		      timeBeginCH = clock();
		      DSLSubseg DCH(a,b,mu,A,B,1);
		      timeEndCH = clock();
		      timeTotalCH += ((double)timeEndCH-(double)timeBeginCH)/(((double)CLOCKS_PER_SEC)/1000);
		      // Compare results of local convex hull and DSLSubsegment
		      assert(DSLsub.aa == DCH.aa && DSLsub.bb == DCH.bb && DSLsub.Nu == DCH.Nu);
		      
		      
#ifdef CHECK_RES
		      // Check if the result is ok comparing with ArithmeticalDSS recognition algorithm
		      DSSIterator  it(a,b,-mu,A);
		      ArithDSS myDSS(it);
		      
		      timeBeginDSS = clock();
		      while ( (*(myDSS.end()))[0] <=x2 && myDSS.extendForward())
			{}
		      timeEndDSS = clock();
		      
                  //std::cout << "a =" << myDSS.getA() << " b =" << myDSS.getB() << " mu =" << myDSS.getMu() << std::endl << std::endl;
		  
		  		  
		      if(DSLsub.aa != myDSS.getA() || DSLsub.bb != myDSS.getB() || DSLsub.Nu != - myDSS.getMu())
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
    }
  
  
  std::cout << nb  << " " << nbLocRay ;
  if(nb!=0)
    {
      std::cout << " " << (long double) timeTotalSubseg/(nb);
      std::cout << " " << (long double) timeTotalReversedSmartDSS/(nb);
      std::cout << " " << (long double) timeTotalSubseg4/(nb);
      std::cout << " " << (long double) timeTotalCH/(nb);
    }
  else
    std::cout << " 0" << " 0" << " 0" << " 0" << std::endl;
  //std::cout << " " << (long double) timeTotalSubsegD/((nb-nberrors));
  
  
  return true;
}






///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  typedef DGtal::int64_t Integer;
  typedef LightSternBrocot<Integer,DGtal::int32_t> LSB;
  typedef LSB::Fraction Fraction;

  unsigned int nbtries = ( argc > 1 ) ? atoi( argv[ 1 ] ) :2000;
  
  
  Integer modb = 1000000000;
  
  //for(Integer i = 10; i<=modb ;i*=10)
  //for(Integer i = 10; i<modb ;i+=i/3) 
  Integer i = modb;
  for(Integer modx = 10; modx <=  2*i;modx+=modx/3)
    //Integer  modx = 1000;
    {
	std::cout << i << " " << modx << " ";
	testDSLSubsegment<Integer,Fraction>( nbtries,  i, modx);
	std::cout << std::endl;
      }
  
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
