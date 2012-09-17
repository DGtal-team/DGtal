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
 * @file CTrivialConstImage.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/08
 *
 * This file is part of the DGtal library.
 */

#if defined(CTrivialConstImageRECURSES)
#error Recursive header files inclusion detected in CTrivialConstImage.h
#else // defined(CTrivialConstImageRECURSES)
/** Prevents recursive inclusion of headers. */
#define CTrivialConstImageRECURSES

#if !defined CTrivialConstImage_h
/** Prevents repeated inclusion of headers. */
#define CTrivialConstImage_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/kernel/CPointFunctor.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/CLabel.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CTrivialConstImage
  /**
   * Description of \b concept '\b CTrivialConstImage' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing a read-only image,
   * which is a refinement of CPointFunctor.
   *
###  Refinement of CPointFunctor
   *
###  Associated types :
   * - \t Domain: type of the image domain, model of concept CDomain
   *
###  Notation
   * - \t X : A type that is a model of CTrivialConstImage
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


    </table>

   *
###  Invariants
   *
###  Models
   * ImageContainerBySTLVector, ImageContainerBySTLMap, ImageContainerByITKImage, ImageContainerByHashTree
   *
   */

  template <typename I>
  struct CTrivialConstImage: CPointFunctor<I>
  {

  public:

    BOOST_CONCEPT_ASSERT((CLabel<typename I::Value>));
    //Inner types
    typedef typename I::Domain Domain;
    BOOST_CONCEPT_ASSERT((CDomain<Domain>));


    BOOST_CONCEPT_USAGE(CTrivialConstImage)
    {
      ConceptUtils::sameType(i.domain(), d);
    }

  private:
    I i;
    Domain d;

  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CTrivialConstImage_h

#undef CTrivialConstImageRECURSES
#endif // else defined(CTrivialConstImageRECURSES)
