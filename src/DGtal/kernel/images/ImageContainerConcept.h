#pragma once

/**
 * @file ImageContainerConcept.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/06/09
*
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerConceptRECURSES)
#error Recursive header files inclusion detected in ImageContainerConcept.h
#else // defined(ImageContainerConceptRECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerConceptRECURSES

#if !defined ImageContainerConcept_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerConcept_h

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>

namespace DGtal
{

 /////////////////////////////////////////////////////////////////////////////
  // struct ImageContainerConcept
  /**
   * Description of struct 'ImageContainerConcept' <p>
   *
   *
   * @brief Implements the concept 'ImageContainer' using boost concept.
   * 
   * Usage: to check if the concept is consistent with the  template arguments TPoint (Point class), 
   * TValue, and TContainer, we have to use the following snippet:
   *
   * \code
   * ...
   *  BOOST_CONCEPT_ASSERT((DGtal::ImageContainerConcept<TPoint,TValue,TContainer>));
   * ...
   * \endcode
   * \see Image.h
   */

template <class TPoint, typename TValue, class TContainer>
struct ImageContainerConcept  : boost::Assignable<TValue>, boost::EqualityComparable<TValue>
{

 public:
    typedef typename TContainer::Iterator Iterator;

    BOOST_CONCEPT_ASSERT((boost::BidirectionalIterator<Iterator>));
    
    BOOST_CONCEPT_USAGE(ImageContainerConcept)
    {
        TContainer j(a,b);
        it=i.begin();  // require postincrement-dereference returning value_type	
        same_type(i(it),v);        // require preincrement returning X&
	same_type(i(a),v);
    }

 private:
    TContainer i;
    Iterator it;
    TValue v;
    TPoint a,b;
 
    // Type deduction will fail unless the arguments have the same type.
    template <typename T>
    void same_type(T const&, T const&);
};
} // namespace DGtal
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerConcept_h

#undef ImageContainerConceptRECURSES
#endif // else defined(ImageContainerConceptRECURSES)
