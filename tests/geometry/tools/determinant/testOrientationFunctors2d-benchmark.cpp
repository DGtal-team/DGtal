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
 * @file testOrientationFunctors2d-benchmark.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/22
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/PointVector.h"

#include "DGtal/geometry/tools/determinant/C2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/Simple2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/SimpleIncremental2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/AvnaimEtAl2x2DetSignComputer.h"
#include "DGtal/geometry/tools/determinant/Filtered2x2DetComputer.h"

#include "DGtal/geometry/tools/determinant/COrientationFunctor.h"
#include "DGtal/geometry/tools/determinant/OrientationFunctor2dBy2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/OrientationFunctor2dBySimpleMatrix.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Random Functions 
///////////////////////////////////////////////////////////////////////////////
/**
 * @return a random integer of type DGtal::int32_t
 * lying in the range [0 ; 2^15[
 */
DGtal::int32_t randomInt15 ()
{
  return static_cast<DGtal::int32_t>(rand() % 32768); 
}

/**
 * @return a random integer of type DGtal::int32_t
 * lying in the range [0 ; 2^30[
 */
DGtal::int32_t randomInt30 ()
{
  return randomInt15() + 
    ( randomInt15() * 32768 ); 
}

/**
 * @return a random integer of type DGtal::int64_t
 * lying in the range [0 ; 2^52[
 */
DGtal::int64_t randomInt52 ()
{
  return
    static_cast<DGtal::int64_t>(rand() % 128) + 
    ( static_cast<DGtal::int64_t>(randomInt15()) +  
      ( static_cast<DGtal::int64_t>(randomInt30()) ) 
      * 32768 ) 
    * 128; 
}

/**
 * @return a random integer of type double
 * lying in the range [0 ; 2^52[
 */
double randomDouble52 ()
{
  return
    static_cast<double>(rand() % 128) + 
    ( static_cast<double>(randomInt15()) +  
      ( static_cast<double>(randomInt30()) ) 
      * 32768 ) 
    * 128; 
}

/**
 * @return a random (signed) integer of type DGtal::int32_t
 * lying in the range [-2^30 ; 2^30[
 */
DGtal::int32_t signedRandomInt30 ()
{
  DGtal::int32_t x = randomInt30(); 
  return ((rand() % 2) ? x : -x);
}

/**
 * @return a random (signed) integer of type DGtal::int64_t
 * lying in the range [-2^52 ; 2^52[
 */
DGtal::int64_t signedRandomInt52 ()
{
  DGtal::int64_t x = randomInt52(); 
    return ((rand() % 2) ? x : -x);
}

/**
 * @return a random (signed) integer of type double
 * lying in the range [-2^52 ; 2^52[
 */
double signedRandomDouble52 ()
{
  double x = randomDouble52(); 
  return ((rand() % 2) ? x : -x);
}

///////////////////////////////////////////////////////////////////////////////
// Tests Functions 
///////////////////////////////////////////////////////////////////////////////
/**
 * Function that traces to the standard output the running
 * time of a given functor @a f for @a n computations over 
 * points whose coordinates are randomly chosen by @a gen.
 * @param f a functor to run
 * @param gen a generator providing random numbers
 * @param n number of tries 
 * @tparam OrientationFunctor a model of COrientationFunctor
 */
template<typename OrientationFunctor, typename RandomFunctor>
bool randomTest(OrientationFunctor f, RandomFunctor gen, const DGtal::int32_t n = 1000000)
{
  BOOST_CONCEPT_ASSERT(( COrientationFunctor<OrientationFunctor> )); 

  typedef typename OrientationFunctor::Point Point; 
  Point P, Q, R; 

  clock_t timeBegin, timeEnd;
  timeBegin = clock();

  for (DGtal::int32_t i = 0; (i < n); ++i)
    {
      P[0] = gen(); 
      P[1] = gen(); 
      Q[0] = gen(); 
      Q[1] = gen(); 
      f.init(P, Q);
      R[0] = gen(); 
      R[1] = gen(); 
      f( R );
    }

  timeEnd = clock();
  long double time, CPUTime;
  time = ((double)timeEnd-(double)timeBegin); 
  CPUTime = time/((double)CLOCKS_PER_SEC);  
  std::cout.precision(5);
  std::cout << CPUTime << " " <<  std::endl; 

  return true; 
}


/**
 * Function that traces to the standard output the running
 * time of all available functors for random points whose 
 * coordinates are within [-2^30 ; 2^30[.
 * @param f a functor to run
 * @tparam OrientationFunctor a model of COrientationFunctor
 */
bool randomTest30All()
{
  typedef PointVector<2, DGtal::int32_t> Point; 

  std::cout << "# random integers within [-2^30 ; 2^30[" << std::endl; 
  std::cout << "# running times in s. for 1 million tries" << std::endl; 
  {
    std::cout << "3x3-int32-int64 "; 
    typedef OrientationFunctor2dBySimpleMatrix<Point, DGtal::int64_t> F; 
    randomTest( F(), signedRandomInt30 );
  }
#ifdef WITH_BIGINTEGER
  { 
    std::cout << "3x3-int32-BigInt "; 
    typedef OrientationFunctor2dBySimpleMatrix<Point, DGtal::BigInteger> F; 
    randomTest( F(), signedRandomInt30 );
  }
#endif
  { 
    std::cout << "2x2-int32-int64 "; 
    typedef Simple2x2DetComputer<DGtal::int32_t, DGtal::int64_t> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
#ifdef WITH_BIGINTEGER
  {
    std::cout << "2x2-int32-BigInt "; 
    typedef Simple2x2DetComputer<DGtal::int32_t, DGtal::BigInteger> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
#endif 
  { 
    std::cout << "2x2-inc-int32-int64 "; 
    typedef SimpleIncremental2x2DetComputer<DGtal::int32_t, DGtal::int64_t> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
#ifdef WITH_BIGINTEGER
  { 
    std::cout << "2x2-inc-int32-BigInt "; 
    typedef SimpleIncremental2x2DetComputer<DGtal::int32_t, DGtal::BigInteger> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
#endif
  {
    std::cout << "2x2-avnaim-int32-int32 "; 
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int32_t> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
  { 
    std::cout << "2x2-avnaim-int32-double "; 
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }
  {
    std::cout << "2x2-avnaim++-int32-double "; 
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef Filtered2x2DetComputer<DetComputer> FDetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, FDetComputer> F; 
    randomTest( F(), signedRandomInt30 );
  }

  return true; 
}

/**
 * Function that traces to the standard output the running
 * time of all available functors for random points whose 
 * coordinates are within [-2^52 ; 2^52[.
 * @param f a functor to run
 * @tparam OrientationFunctor a model of COrientationFunctor
 */
bool randomTest52All()
{
  std::cout << "# random integers within [-2^52 ; 2^52[" << std::endl; 
  std::cout << "# running times in s. for 1 million tries" << std::endl; 

#ifdef WITH_BIGINTEGER
  { //! BigInt cannot be constructed from a DGtal::int64_t
    std::cout << "3x3-double-BigInt ";  
    typedef PointVector<2, double> Point; 
    typedef OrientationFunctor2dBySimpleMatrix<Point, DGtal::BigInteger> F; 
    randomTest( F(), signedRandomDouble52 );
  }
  {
    std::cout << "2x2-double-BigInt "; 
    typedef PointVector<2, double> Point; 
    typedef Simple2x2DetComputer<double, DGtal::BigInteger> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomDouble52 );
  }
  { 
    std::cout << "2x2-inc-double-BigInt "; 
    typedef PointVector<2, double> Point; 
    typedef SimpleIncremental2x2DetComputer<double, DGtal::BigInteger> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomDouble52 );
  }
#endif
  {
    std::cout << "2x2-avnaim-int64-int64 "; 
    typedef PointVector<2, DGtal::int64_t> Point; 
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt52 );
  }
  {
    std::cout << "2x2-avnaim-double-int64 "; 
    typedef PointVector<2, double> Point; 
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomDouble52 );
  }
  {
    std::cout << "2x2-avnaim-int64-double "; 
    typedef PointVector<2, DGtal::int64_t> Point; 
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomInt52 );
  }
  { 
    std::cout << "2x2-avnaim-double-double "; 
    typedef PointVector<2, double> Point; 
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, DetComputer> F; 
    randomTest( F(), signedRandomDouble52 );
  }
  {
    std::cout << "2x2-avnaim++-int64-double ";
    typedef PointVector<2, DGtal::int64_t> Point;  
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef Filtered2x2DetComputer<DetComputer> FDetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, FDetComputer> F; 
    randomTest( F(), signedRandomInt52 );
  }
  {
    std::cout << "2x2-avnaim++-double-double ";
    typedef PointVector<2, double> Point;  
    typedef AvnaimEtAl2x2DetSignComputer<double> DetComputer; 
    typedef Filtered2x2DetComputer<DetComputer> FDetComputer; 
    typedef OrientationFunctor2dBy2x2DetComputer<Point, FDetComputer> F; 
    randomTest( F(), signedRandomDouble52 );
  }

  return true; 
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class OrientationFunctors-benchmark" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  randomTest30All(); 
  randomTest52All(); 

  bool res = true; 
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
