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

#pragma once

/**
 * @file CLMSTDSSFilter.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2018/08/25
 *
 * This file is part of the DGtal library.
 */

#if defined(CLMSTDSSFilter_RECURSES)
#error Recursive header files inclusion detected in CLMSTDSSFilter.h
#else // defined(CLMSTDSSFilter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLMSTDSSFilter_RECURSES

#if !defined CLMSTDSSFilter_h
/** Prevents repeated inclusion of headers. */
#define CLMSTDSSFilter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "boost/concept_check.hpp"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/base/CBidirectionalRange.h"
#include "DGtal/geometry/curves/CForwardSegmentComputer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace concepts
  {

    /////////////////////////////////////////////////////////////////////////////
    // class CLMSTDSSFilter
    /**
     * Description of \b concept '\b CLMSTDSSFilter' <p>
     * @ingroup Concepts
     * @brief Aim: Defines the concept describing a functor which decides which maximal
     * segments (DSSes) enter the pencil of a point in L-MST calculations.
     *
     * The pencil of a point is the set of maximal segments whose directions
     * are averaged, with lambda weights, into the tangent at that point. In
     * the plain lambda-MST it is the set of maximal segments that contain the
     * point. A filter changes that set in two ways, and a model answers one
     * question for each:
     *
     * - operator()(dss): is the segment left out of every pencil (typically
     *   because it is too short to carry a reliable direction)?
     * - reach(dss): up to how many points beyond either end of the segment,
     *   counted along the curve, does a kept segment still enter the pencil
     *   of points it does not contain? 0 means it enters only the pencils of
     *   the points it contains, which gives back the plain lambda-MST.
     *
     * With a positive reach, a point that lies d points before the first or
     * after the last point of a kept segment of L + 1 points receives that
     * segment with eccentricity (d + L + 1) / (d + L + 2): the further away,
     * the closer to 1, so the lower its lambda weight. Removing short segments
     * can leave a point with no containing segment; the reach is what keeps
     * its pencil non-empty. The eccentricities themselves are computed by the
     * estimator, not by the model.
     *
     * # Refinement of boost::DefaultConstructible<T>, boost::CopyConstructible<T>, boost::Assignable<T>
     *
     * # Associated types
     *    - TDSS a 3d digital straight segment recognition algorithm
     *
     * # Notation
     * - \e T : A type that is a model of CLMSTDSSFilter
     * - \e x object of type X
     *
     * # Definitions
     *
     * # Valid expressions and semantics
     *
     * | Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
     * |-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
     * | operator()|x()     |(const TDSS &)|bool|              | true if the DSS is left out of every pencil (e.g. it is too short). | |model dependent|
     * | reach     |x.reach()|(const TDSS &)|double|            | geodesic distance, in points, up to which a kept DSS enters the pencil of a point it does not cover; 0 for covering points only. | non-negative |model dependent|
     *
     * # Invariants
     *
     * # Models
     *
     * - DSSLengthLessEqualFilter, DSSMuteFilter
     *
     * # Notes
     *
     * @tparam T the type that should be a model of CLMSTDSSFilter.
     */
    template <typename T>
    struct CLMSTDSSFilter : boost::DefaultConstructible<T>, boost::CopyConstructible<T>, boost::Assignable<T>
    {
      // ----------------------- Types ------------------------------
      typedef typename T::DSSType TDSS;
      // ----------------------- Concept checks ------------------------------
      BOOST_CONCEPT_ASSERT(( CForwardSegmentComputer < TDSS > ));
    public:
      BOOST_CONCEPT_USAGE(CLMSTDSSFilter)
      {
         ConceptUtils::sameType( c, x.operator()( dss ) );
         ConceptUtils::sameType( r, x.reach ( dss ) );
      }
      void checkConstConstraints() const
      {
         ConceptUtils::sameType( c, x.operator()( dss ) );
         ConceptUtils::sameType( r, x.reach ( dss ) );
      }
      // ------------------------- Private Data --------------------------------
    private:
      bool c;
      double r;
      T x;
      TDSS dss;
    }; // end of concept CLMSTDSSFilter

  } // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLMSTDSSFilter_h

#undef CLMSTDSSFilter_RECURSES
#endif // else defined(CLMSTDSSFilter_RECURSES)
