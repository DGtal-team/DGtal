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

#include "DGtal/kernel/images/CValue.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // struct CImageContainer
  /**
   * Description of \b concept '\b CImageContainer' <p>
   *
   * @ingroup Concepts
   * Aim: Defines the concept describing an image container.
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   * - \t Value: the type of values stored in the image.
   * - \t Iterator: an iterator in the image.
   * - \t ConstIterator: a const iterator in the image.
   * - \t SpanIterator: a 1D span iterator in the image (not yet tested).
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
   * <td> Image extent</td> <td>extent = x.extent() </td> <td> </td> <td> X::Space::Vector</td>
   * <td> </td> <td> returns the extent of the image  </td> <td> </td> <td> O(dimension)</td>
   * </tr>
   * <tr>
   * <td>Set a value at a position</td> <td>x.setValue(point, val)
   * </td> <td>point is of type X::Space::Point, val is of type
   * X::Value </td> <td> </td>
   * <td> the point is in the domain associted to
   * the image </td> <td> associate a value  to a point  </td> <td> </td> <td> Depends on the model</td>
   * </tr>
   * <tr>
   * <td>Set a value at a position</td> <td>x.setValue(it, val) </td>
   * <td>it is of type X::Iterator or X::ConstIterator, val is of type
   * X::Value </td> <td> </td>
   * <td> the iterator is in the image range</td> <td> associate a value  to a point given by an
   * iterator</td> <td> </td> <td> Depends on the model</td>
   * </tr>
   * <tr>
   * <td>Get the value at a position</td> <td>val = x(point) </td>
   * <td>point is of type X::Space::Point, val is of type X::Value
   * </td> <td> X::Value </td>
   * <td>  the point is in the domain associted to
   * the image</td> <td> get the value associated to a point  </td> <td> </td> <td> Depends on the model</td>
   * </tr>
     * <tr>
   * <td>Get the value at a position</td> <td>val = x(it) </td>
   * <td>it is of type X::Iterator, val is of type X::Value
   * </td> <td> X::Value </td>
   * <td>  the iterator is in the image range</td> <td> get the value associated to a point  </td> <td> </td> <td> Depends on the model</td>
   * </tr>
     * <tr>
   * <td>Get the value at a position</td> <td>val = x(constIt) </td>
   * <td>constIt is of type X::ConstIterator, val is of type X::Value
   * </td> <td> X::Value </td>
   * <td>  the const iterator is in the image range</td> <td> get the value associated to a point  </td> <td> </td> <td> Depends on the model</td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   * ImageContainerBySTLVector, ImageContainerByITKImage
   * <p> Notes <br>
   *
   * @todo Complete ImageContainer checking.
   */

  template <typename ImageContainer>
  struct CImageContainer
  {

  public:
    typedef typename ImageContainer::Value Value;
    typedef typename ImageContainer::Iterator Iterator;
    typedef typename ImageContainer::ConstIterator ConstIterator;
    typedef typename ImageContainer::Point Point;

 
    BOOST_CONCEPT_ASSERT((CValue<Value>));

    BOOST_CONCEPT_USAGE(CImageContainer)
    {
      //Iterators
      it = i.begin();
      itconst = i.begin();
      ++it;
      same_type(++itconst,itconst);
      --it;
      same_type(--itconst, itconst);

      it = i.end();
      itconst = i.end();
      
      
      //Accessors
      same_type(i(a), v);
      same_type(i(it), v);
      same_type(i(itconst), v);
      
      //API
      same_type(i.extent(), a); //get the extent
      i.setValue(a, v);  //set a value at a Point
      i.setValue(it, v); //set a value at an Iterator
      same_type(i.operator()(itconst), v);       // get the value from a ConstIterator
      same_type(i.operator()(it), v);       // get the value from a ConstIterator
      same_type(i.operator()(a), v);       //get the value from a point

    }

  private:
    ImageContainer i;
    Iterator it;
    ConstIterator itconst;
    Value v;
    Point a, b;

    //  deduction will fail unless the arguments have the same type.
    template <typename T>
    void same_type(T const&, T const&);
  };
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImageContainer_h

#undef CImageContainerRECURSES
#endif // else defined(CImageContainerRECURSES)
