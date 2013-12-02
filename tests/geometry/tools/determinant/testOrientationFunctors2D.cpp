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
 * @file testOrientationFunctors2D.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/22
 *
 * Functions for testing models of COrientationFunctor2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"

#include "DGtal/geometry/tools/determinant/Simple2x2DetComputer.h"

#include "DGtal/geometry/tools/determinant/COrientationFunctor2D.h"
#include "DGtal/geometry/tools/determinant/OrientationFunctor2DBy2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/OrientationFunctor2DBySimpleMatrix.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class OrientationFunctors.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 * @param f any orientation functor
 * @tparam OrientationFunctor a model of COrientationFunctor2D
 */
template<typename OrientationFunctor>
bool testOrientationFunctors(OrientationFunctor f)
{
  BOOST_CONCEPT_ASSERT(( COrientationFunctor2D<OrientationFunctor> )); 

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  trace.info() << f << endl; 

  typedef typename OrientationFunctor::Value Value; 
  typedef typename OrientationFunctor::Point Point; 
  Point a(0,0); 
  Point b(5,2);
 
  //first quadrant
  f.init(a, b); 
  if (f( Point(2,1) ) == NumberTraits<Value>::ONE)
    nbok++;   //a, b, (2,1) are CCW oriented
  nb++; 
  trace.info() << "(" << nbok << "/" << nb << ") " << endl;

  if (f( Point(3,1) ) == -NumberTraits<Value>::ONE)
    nbok++;   //a, b, (3,1) are CW oriented
  nb++; 
  trace.info() << "(" << nbok << "/" << nb << ") " << endl;

  if (f( Point(10,4) ) == NumberTraits<Value>::ZERO)
    nbok++;   //a, b, (10,4) belong to the same line
  nb++; 
  trace.info() << "(" << nbok << "/" << nb << ") " << endl;

  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class OrientationFunctors" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  typedef PointVector<2, DGtal::int16_t> Point; 

  bool res = true; 

  typedef Simple2x2DetComputer<DGtal::int32_t, DGtal::int64_t> DetComputer; 
  typedef OrientationFunctor2DBy2x2DetComputer<Point, DetComputer> Functor1; 
  res = res && testOrientationFunctors( Functor1() );
 
  typedef OrientationFunctor2DBySimpleMatrix<Point, DGtal::int32_t> Functor2; 
  res = res && testOrientationFunctors( Functor2() );

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
