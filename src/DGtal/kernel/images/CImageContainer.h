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
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * <td> </td> <td> </td> <td> </td> <td> </td>
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

  template <typename TPoint, typename ValueType, typename TContainer>
    struct CImageContainer
    {

    public:
      typedef typename TContainer::Iterator Iterator;

      BOOST_CONCEPT_ASSERT((boost::BidirectionalIterator<Iterator>));
      BOOST_CONCEPT_ASSERT((CValueType<ValueType>));


      BOOST_CONCEPT_USAGE(CImageContainer)
      {
        //TContainer j(a,b);
        it=i.begin();  // require postincrement-dereference returning value_type	
        same_type(i(it),v);        // require preincrement returning X&
	same_type(i(a),v);
      }

    private:
      TContainer i;
      Iterator it;
      ValueType v;
      TPoint a,b;
 
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
