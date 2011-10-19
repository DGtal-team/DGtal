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

#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/base/CBidirectionalRange.h"
#include "DGtal/images/CValue.h"

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
   * <p> Refinement of CBidirectionalRange
   *
   * <p> Associated types :
   * - \t Value: the type of values stored in the image, model of
   * concept CValue
   * - \t Domain: type of the image domain, model of concept CDomain
   * <p> Notation
   * - \t X : A type that is a model of CImageContainer
   * - \t x, \t y  : Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
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
        <td class=CName>  Constructor          </td> 
        <td class=CExpression> X x(@c aDomain)     </td>
        <td class=CRequirements> @c aDomain of type Domain    </td> 
        <td class=CReturnType> an instance of X     </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics> Create an image container on the domain
      @c aDomain for value type Value     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>   Container dependent   </td>
	</tr>
	

	<tr> 
        <td class=CName> Domain            </td> 
        <td class=CExpression>  x.domain()   </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType>  const Domain &    </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>  returns a const reference to the image domain     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity> O(1)     </td>
      </tr>
    

	<tr> 
        <td class=CName> Set a value           </td> 
        <td class=CExpression>  x.setValue(@c aPoint, @c aValue)    </td>
        <td class=CRequirements> @c aPoint of type Point and @c aValue of
        type Value   </td> 
        <td class=CReturnType>  void    </td>
        <td class=CPrecondition> @c aPoint must be inside the image domain  </td> 
        <td class=CSemantics>  associate the value @c aValue with the
        point @aPoint     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>

	<tr> 
        <td class=CName> Set a value           </td> 
        <td class=CExpression>  x.setValue(@c anIterator, @c aValue)    </td>
        <td class=CRequirements> @c anIterator of type Iterator and @c aValue of
        type Value   </td> 
        <td class=CReturnType>  void    </td>
        <td class=CPrecondition> @c anIterator must be valid (inside the image domain)  </td> 
        <td class=CSemantics>  associate the value @c aValue with the
        point  referenced by the itertor @anIterator     </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
     
    
      <tr> 
      <td class=CName> Set a value           </td> 
      <td class=CExpression>  x.setValue(@c aReverseIterator, @c aValue)    </td>
      <td class=CRequirements> @c aReverseIterator of type ReverseIterator and @c aValue of
      type Value   </td> 
      <td class=CReturnType>  void    </td>
      <td class=CPrecondition> @c aReverseIterator must be valide (inside the image domain)  </td> 
      <td class=CSemantics>  associate the value @c aValue with the
      point  referenced by the itertor @aReverseIterator     </td> 
      <td class=CPostCondition>   </td> 
      <td class=CComplexity>  Container dependent    </td>
      </tr>
      
      <tr> 
        <td class=CName> Accessor           </td> 
        <td class=CExpression>  x(@c aPoint)    </td>
        <td class=CRequirements> @c aPoint of type Point    </td> 
        <td class=CReturnType>  Value    </td>
        <td class=CPrecondition> @c aPoint must be inside the image domain  </td> 
        <td class=CSemantics> returns the value associated to the
        point @c aPoint</td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
   

     	<tr> 
        <td class=CName> Accessor           </td> 
        <td class=CExpression>  x(@c aIterator)    </td>
        <td class=CRequirements> @c aIterator of type Iterator    </td> 
        <td class=CReturnType>  Value    </td>
        <td class=CPrecondition> @c aIterator must be valide (inside the image domain)  </td> 
        <td class=CSemantics> returns the value associated to the
        point referenced by the iterator aIterator</td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
     	<tr> 
        <td class=CName> Accessor           </td> 
        <td class=CExpression>  x(@c aConstIterator)    </td>
        <td class=CRequirements> @c aConstIterator of type ConstIterator    </td> 
        <td class=CReturnType>  Value    </td>
        <td class=CPrecondition> @c aConstIterator must be valide (inside the image domain)  </td> 
        <td class=CSemantics> returns the value associated to the
        point referenced by the iterator aConstIterator</td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
     	<tr> 
        <td class=CName> Accessor           </td> 
        <td class=CExpression>  x(@c aReverseIterator)    </td>
        <td class=CRequirements> @c aReverseIterator of type ReverseIterator    </td> 
        <td class=CReturnType>  Value    </td>
        <td class=CPrecondition> @c aReverseIterator must be valide (inside the image domain)  </td> 
        <td class=CSemantics> returns the value associated to the
        point referenced by the iterator aReverseIterator</td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
     	<tr> 
        <td class=CName> Accessor           </td> 
        <td class=CExpression>  x(@c aConstReverseIterator)    </td>
        <td class=CRequirements> @c aConstReverseIterator of type ConstReverseIterator    </td> 
        <td class=CReturnType>  Value    </td>
        <td class=CPrecondition> @c aConstReverseIterator must be valide (inside the image domain)  </td> 
        <td class=CSemantics> returns the value associated to the
        point referenced by the iterator aConstReverseIterator</td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>  Container dependent    </td>
      </tr>
</table>   

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
  struct CImageContainer: CBidirectionalRange<ImageContainer>
  {

  public:
    
    //Inner types
    typedef typename ImageContainer::Domain Domain;
    typedef typename ImageContainer::Value Value;
    typedef typename ImageContainer::Point Point;

    //Iterators (already tested in CRange)
    typedef typename  ImageContainer::Iterator Iterator;
    typedef typename  ImageContainer::ConstIterator ConstIterator;
    typedef typename  ImageContainer::ReverseIterator ReverseIterator;
    typedef typename  ImageContainer::ConstReverseIterator ConstReverseIterator;
    

    BOOST_CONCEPT_ASSERT((CValue<Value>));
    BOOST_CONCEPT_ASSERT((CDomain<Domain>));

    
    BOOST_CONCEPT_USAGE(CImageContainer)
    {
      //Accessors
      same_type(image(a), v);
      same_type(image(it), v);
      same_type(image(itconst), v);
      same_type(image(itrev), v);
      same_type(image(itconstrev), v);
      
      //API
      same_type(image.domain(), d); 
      image.setValue(a, v);  //set a value at a Point
      image.setValue(it, v); //set a value at an Iterator
      image.setValue(a, v);  //set a value at a Point
      image.setValue(itrev, v); //set a value at an ConstIterator
      
      same_type(image.operator()(itconst), v);       // get the value from a ConstIterator
      same_type(image.operator()(it), v);       // get the value from a ConstIterator
      same_type(image.operator()(itconstrev), v);       // get the value from a ConstIterator
      same_type(image.operator()(itrev), v);       // get the value from a ConstIterator
      same_type(image.operator()(a), v);       //get the value from a point

    }

  private:
    ImageContainer image;
    Iterator it;
    ConstIterator itconst;
    ReverseIterator itrev;
    ConstReverseIterator itconstrev;
    Value v;
    Point a, b;
    Domain d;

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
