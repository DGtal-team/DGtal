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
 * @file testBasicFunctors.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/01/27
 *
 * Functions for testing basic functors.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <functional>

#include "DGtal/base/Common.h"

#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/base/BasicFunctors.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

/**
 * Concept checking
 *
 */
template <typename TFunctor, typename TArg, typename TRes >
void basicFunctorsConceptChecking()
{
  BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, TArg, TRes > ));
}
/**
 * Simple test. 
 *
 */
bool testBasicFunctors()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing basic functors ..." );
  
  //default functor
  {
    DefaultFunctor f; 
    int a = 5; 
    nbok += ( f(a) == 5 ) ? 1 : 0; 
    nb++;
  }

  {//constant functor
    const int v = -1;
    ConstValueFunctor<int> f(v);
    char c = 'a'; 
    nbok += ( f(c) == v ) ? 1 : 0; 
    nb++;
    double d = 5.2; 
    nbok += ( f(d) == v ) ? 1 : 0; 
    nb++;
  }

  //cast functor
  {
    CastFunctor<int> f; 
    char c = 'a'; 
    nbok += ( f(c) == 97 ) ? 1 : 0; 
    nb++;
  }

  //composer quantizer
  {
    //need to explicitely specialized std::ptr_fun because there are several
    //overloaded versions of std::floor if used intead ctor of 
    //std::pointer_to_unary_function<double, double> 
    std::pointer_to_unary_function<double, double> f(std::floor); 
    std::pointer_to_unary_function<double, double> c(std::ceil); 
    CastFunctor<int> o; 

    //composer
    typedef Composer< std::pointer_to_unary_function<double, double>, 
      CastFunctor<int>, int > Quantizer;
    double d = 5.2; 

    Quantizer q(f, o); 
    nbok += ( q(d) == 5 ) ? 1 : 0; 
    nb++;

    Quantizer q2(c, o); 
    nbok += ( q2(d) == 6 ) ? 1 : 0; 
    nb++;
  }

  //binary to unary functor
  {
    int i = -5; 
    std::binder2nd<std::minus<int> > b(std::minus<int>(), 0); 
    //i - 0
    nbok += ( b(i) == -5 ) ? 1 : 0; 
    nb++;
    std::binder2nd<std::plus<int> > b2(std::plus<int>(), 2); 
    //i + 2
    nbok += ( b2(i) == -3 ) ? 1 : 0; 
    nb++;
  }

  {//thresholder
    int i = -3; 
    Thresholder< int > t;
    nbok += ( t(i) == true ) ? 1 : 0; 
    nb++;    
    Thresholder< int, true, true > t1;
    nbok += ( t1(i) == true ) ? 1 : 0; 
    nb++;    
    Thresholder< int, true, false > t2;
    nbok += ( t2(0) == false ) ? 1 : 0; 
    nb++;    
    Thresholder< int, false, true > t3;
    nbok += ( t3(i) == false ) ? 1 : 0; 
    nb++;    
    Thresholder< int, false, false > t4;
    nbok += ( t4(i) == false ) ? 1 : 0; 
    nb++;    
  }

  {//interval thresholder
    const int low = 1;
    const int up = 5; 
    IntervalThresholder< int > t(low, up);
    nbok += ( t(0) == false ) ? 1 : 0; 
    nb++;    
    for (int i = low; i <= up; ++i)
      {
	nbok += ( t(i) == true ) ? 1 : 0; 
	nb++;    
      }
    nbok += ( t(6) == false ) ? 1 : 0; 
    nb++;    
  }


  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing basic functors" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  //concept checking
  basicFunctorsConceptChecking<DefaultFunctor,int,int>(); 
  basicFunctorsConceptChecking<ConstValueFunctor<int>,int,int >(); 
  basicFunctorsConceptChecking<CastFunctor<int>,short,int >(); 
  basicFunctorsConceptChecking<Thresholder<int>,int,bool >(); 
  basicFunctorsConceptChecking<Composer<ConstValueFunctor<double>,CastFunctor<int>,int>,char,int >(); 


  //run-time tests
  bool res = testBasicFunctors();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
