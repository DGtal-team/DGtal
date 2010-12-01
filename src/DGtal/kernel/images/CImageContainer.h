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
 * @file CImageContainer.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/09
 *
 * This file is part of the DGtal library.
 */

#if defined(CImageContainerRECURSES)
#error Recursive header files inclusion detected in CImageContainer.h
#else // defined(CImageContainerRECURSES)
/** Prevents recursive inclusion of headers. */
#define CImageContainerRECURSES

#if !defined CImageContainer_h
/** Prevents repeated inclusion of headers. */
#define CImageContainer_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

#include "DGtal/kernel/images/CValueType.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CImageContainer
  /**
   * Description of \b concept '\b CImageContainer' <p>
   *
   * @ingroup Concepts
   * Aim:
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CImageContainer
   * - \t x, \t y	: Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td>
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr>
	 * <td> Image extent</td> <td>x.extent() </td> <td> </td> <td> X::Space::Vector</td>
   * <td> </td> <td> returns the extent of the image  </td> <td> </td> <td> O(dimension)</td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * <p> Notes <br>
   *
   * @todo Complete ImageContainer checking.
   */

  template <typename IContainer>
  struct CImageContainer
  {

  public:
    typedef typename IContainer::Iterator Iterator;
    typedef typename IContainer::ValueType ValueType;
    typedef typename IContainer::ConstIterator ConstIterator;
    typedef typename IContainer::Point Point;

    BOOST_CONCEPT_ASSERT((boost::BidirectionalIterator<Iterator>));
    BOOST_CONCEPT_ASSERT((CValueType<ValueType>));

    BOOST_CONCEPT_USAGE(CImageContainer)
    {
      //TContainer j(a,b);
      it = i.begin();  // require postincrement-dereference returning value_type
      same_type(i(a), v);

      //API
      same_type(i.extent(), a); //get the extent
      i.setValue(a, v);  //set a value at a Point
      i.setValue(it, v); //set a value at an Iterator
      same_type(i.operator()(itconst), v);       // get the value from a ConstIterator
      same_type(i.operator()(a), v);       //get the value from a point

    }

  private:
    IContainer i;
    Iterator it;
    ConstIterator itconst;
    ValueType v;
    Point a, b;

    // Type deduction will fail unless the arguments have the same type.
    template <typename T>
    void same_type(T const&, T const&);
  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImageContainer_h

#undef CImageContainerRECURSES
#endif // else defined(CImageContainerRECURSES)
