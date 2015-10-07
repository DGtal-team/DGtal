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
 * @file testCompleteVoronoiMap
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2015/10/07
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testCompleteVoronoiMapCatch' <p>
 * Aim: unit test of the complete voronoi map extraction.
 */

#include <iostream>
#include <limits>

#include "DGtalCatch.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/geometry/volumes/distance/CompleteVoronoiMap.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/kernel/BasicPointPredicates.h"

using namespace DGtal;

//Helper function to check the Voronoi map distances using
//brute-force approach
template <typename VMap, typename Set>
void checkDistance(const VMap & voronoi, const Set &pointset)
{
  typedef typename VMap::SeparableMetric::RawValue RawValue;
  const typename VMap::SeparableMetric d = *voronoi.metric();
  typename Set::Point site;
  bool ok = true;
  
  for(typename VMap::Domain::ConstIterator it = voronoi.domain().begin(), itend = voronoi.domain().end();
      it != itend ; ++it )
  {
    RawValue val = std::numeric_limits<RawValue>::max();
    
    //bruteforce closest point
    for(typename Set::ConstIterator itset = pointset.begin(), itsetend = pointset.end();
        itset != itsetend;
        ++itset)
    {
      if ( d.rawDistance( *it, *itset) < val )
      {
        val = d.rawDistance( *it, *itset);
        site = *itset;
      }
    }
    ok = (val == d.rawDistance( *it, voronoi(*it)));
    CAPTURE( *it );
    CAPTURE( voronoi(*it));
    CAPTURE( d.rawDistance(*it, voronoi(*it)));
    CAPTURE( val );
    CAPTURE( site );
    REQUIRE(( val == d.rawDistance(*it, voronoi(*it)) ));
  }
}

TEMPLATE_TEST_CASE_2("Testing Complete Voronoi Map in 2D",
                     "Templated test of CompleteVoronoiMap in 2D", T,
                     Z2i::L2Metric , Z2i::L1Metric)
{
  typedef Z2i::Space Space;
  typedef Z2i::Point Point;
  typedef Z2i::Domain Domain;
  typedef Z2i::DigitalSet Set;
  typedef T Metric;
  
  //Simple digitalset
  const Domain domain(Point(0,0), Point(32,32));
  const Metric metric;
  Set pointset( domain );
  pointset.insertNew( Point (0,16));
  pointset.insertNew( Point(8,20));
  pointset.insertNew( Point(24,20));
  Board2D board;
  functors::NotPointPredicate< Set > notInSet(pointset);

  //Instanciation
  typedef CompleteVoronoiMap<Space, functors::NotPointPredicate< Set >, Metric> CVMap;
  CVMap voronoi( domain, notInSet, metric);

  SECTION("   Init")
  {
    //Validity check
    BOOST_CONCEPT_ASSERT(( concepts::CConstImage<CVMap> ));
    REQUIRE(( voronoi.domain().isValid() ));
  }
  
  SECTION("   Check distances")
  {
    //We first check the closest point distance
    checkDistance<CVMap,Set>(voronoi,pointset);
  }

  SECTION("   Check assignments")
  {
    //We first check the closest point distance
    checkDistance<CVMap,Set>(voronoi,pointset);
  }

}


/** @ingroup Tests **/
