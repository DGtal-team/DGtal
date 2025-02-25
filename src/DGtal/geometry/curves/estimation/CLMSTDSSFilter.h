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
     * @brief Aim: Defines the concept describing a functor which filters DSSes for L-MST calculations.
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
     * | operator()|x()     |(const TDSS &)|bool|              | Checks if a DSS fulfills a given condition. | |model dependant|
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
    requires concepts::CForwardSegmentComputer<typename T::DSSType>
    struct CLMSTDSSFilter : boost::DefaultConstructible<T>, boost::CopyConstructible<T>, boost::Assignable<T>
    {
      // ----------------------- Types ------------------------------
      typedef typename T::DSSType TDSS;
      // ----------------------- Concept checks ------------------------------
    public:
      BOOST_CONCEPT_USAGE(CLMSTDSSFilter)
      {
         ConceptUtils::sameType( c, x.operator()( dss ) );
         ConceptUtils::sameType( c, x.admissibility ( dss, p ) );
         ConceptUtils::sameType( i, x.position ( dss, p ) );
      }
      void checkConstConstraints() const
      {
         ConceptUtils::sameType( c, x.operator()( dss ) );
         ConceptUtils::sameType( c, x.admissibility ( dss, p ) );
         ConceptUtils::sameType( i, x.position ( dss, p ) );
      }
      // ------------------------- Private Datas --------------------------------
    private:
      bool c;
      long int i;
      T x;
      typename T::Point p;
      TDSS dss;
    }; // end of concept CLMSTDSSFilter
    
  } // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLMSTDSSFilter_h

#undef CLMSTDSSFilter_RECURSES
#endif // else defined(CLMSTDSSFilter_RECURSES)
