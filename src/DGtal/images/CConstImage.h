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
 * @file CConstImage.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CConstImageRECURSES)
#error Recursive header files inclusion detected in CConstImage.h
#else // defined(CConstImageRECURSES)
/** Prevents recursive inclusion of headers. */
#define CConstImageRECURSES

#if !defined CConstImage_h
/** Prevents repeated inclusion of headers. */
#define CConstImage_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/kernel/CPointFunctor.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/CConstBidirectionalRangeFromPoint.h"
#include "DGtal/base/CLabel.h"
#include "DGtal/images/CTrivialConstImage.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CConstImage
  /**
   * DescriptionDescription of \b concept '\b CConstImage' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing a read-only image,
   * which is a refinement of CPointFunctor.
   *
### Refinement of CTrivialConstImage
   *
###  Associated types :
   * - \t Domain: type of the image domain, model of concept CDomain
   * - \t ConstRange: type of range of image values,
   * model of concept CConstBidirectionalRangeFromPoint
   *
###  Notation
   * - \t X : A type that is a model of CConstImage
   * - \t x, \t y  : Object of type X
   *
###  Definitions
   *
###  Valid expressions and
      <table>
      <tr>
        <td class=CName> \b Name </td>
        <td class=CExpression> \b Expression </td>
        <td class=CRequirements> \b Type requirements </td>
        <td class=CReturnType> \b Return type </td>
        <td class=CPrecondition> \b Precondition </td>
        <td class=CSemantics> \b Semantics </td>
        <td class=CPostCondition> \b Postcondition </td>
        <td class=CComplexity> \b Complexity </td>
      </tr>

	<tr>
        <td class=CName> accessor to the domain            </td>
        <td class=CExpression>  x.domain()   </td>
        <td class=CRequirements>    </td>
        <td class=CReturnType>  const Domain &    </td>
        <td class=CPrecondition>    </td>
        <td class=CSemantics>  returns a const reference to the image domain     </td>
        <td class=CPostCondition>   </td>
        <td class=CComplexity> O(1)     </td>
      </tr>

	<tr>
        <td class=CName> accessor to the range of the image values            </td>
        <td class=CExpression>  x.constRange()   </td>
        <td class=CRequirements>    </td>
        <td class=CReturnType>  ConstRange    </td>
        <td class=CPrecondition>    </td>
        <td class=CSemantics>  returns a constant range     </td>
        <td class=CPostCondition>   </td>
        <td class=CComplexity> O(1)     </td>
      </tr>


    </table>

   *
###  Invariants
   *
###  Model
   * ImageContainerBySTLVector, ImageContainerBySTLMap, ImageContainerByITKImage, ImageContainerByHashTree
   *
###  Notes
   *
   */

  template <typename I>
  struct CConstImage: CTrivialConstImage<I>
  {

  public:

    //Inner types
    typedef typename I::Domain Domain;
    BOOST_CONCEPT_ASSERT((CDomain<Domain>));

    typedef typename I::ConstRange ConstRange;
    BOOST_CONCEPT_ASSERT((CConstBidirectionalRangeFromPoint<ConstRange>));

    BOOST_CONCEPT_USAGE(CConstImage)
    {
      ConceptUtils::sameType(i.domain(), d);
      ConceptUtils::sameType(i.constRange(), r);
    }

  private:
    I i;
    Domain d;
    ConstRange r;

  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CConstImage_h

#undef CConstImageRECURSES
#endif // else defined(CConstImageRECURSES)
