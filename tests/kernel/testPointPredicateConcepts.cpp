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
 * @file testPointPredicateConcepts.cpp
 * @ingroup Tests
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/07
 *
 * Functions for testing class PointPredicateConcepts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <functional>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/images/ImageSelector.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace DGtal::functors;
using namespace DGtal::concepts;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PointPredicateConcepts.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool
testPointPredicateConcepts()
{
    typedef ImageSelector<Z3i::Domain, int>::Type IntImage;
    using IntPredicate = std::function<bool(int)>;
    typedef ImageSelector<Z3i::Domain, float>::Type FloatImage;
    using FloatPredicate = std::function<bool(float)>;
    
    // PointFunctorPredicate
    typedef PointFunctorPredicate<IntImage, IntPredicate> PointPredicate1;
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< PointPredicate1 > );
    typedef PointFunctorPredicate<FloatImage, FloatPredicate> PointPredicate2;
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< PointPredicate2 > );

    // Binary PointPredicate
    typedef std::logical_and<bool> BinaryFunctor;
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< BinaryPointPredicate<PointPredicate1, PointPredicate2, BinaryFunctor> > );

    // NotPointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< NotPointPredicate<PointPredicate1> > );
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< NotPointPredicate<PointPredicate2> > );

    typedef typename IntImage::Point Point;
    // EqualPointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< EqualPointPredicate<Point> > );
    // IsWithinPointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< IsWithinPointPredicate<Point> > );
    // IsUpperPointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< IsUpperPointPredicate<Point> > );
    // IsLowerPointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< IsLowerPointPredicate<Point> > );
    // TruePointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< TruePointPredicate<Point> > );
    // FalsePointPredicate
    DGTAL_CONCEPT_CHECK( requires concepts::CPointPredicate< FalsePointPredicate<Point> > );

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing PointPredicate Concepts" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testPointPredicateConcepts();

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
