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
 * @file CImageCacheWritePolicy.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/02/22
 *
 * Header file for concept CImageCacheWritePolicy.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CImageCacheWritePolicy_RECURSES)
#error Recursive header files inclusion detected in CImageCacheWritePolicy.h
#else // defined(CImageCacheWritePolicy_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CImageCacheWritePolicy_RECURSES

#if !defined CImageCacheWritePolicy_h
/** Prevents repeated inclusion of headers. */
#define CImageCacheWritePolicy_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CImageCacheWritePolicy
/**
Description of \b concept '\b CImageCacheWritePolicy' <p>
@ingroup Concepts
@brief Aim: Defines the concept describing a cache write policy

### Refinement of

### Associated types :
 - \e ImageContainer : type of the image in the cache, model of concept CImage
 - \e Point : type of the image point
 - \e Value : type of the image value

### Notation
 - \e X : A type that is a model of CImageCacheWritePolicy
 - \e x : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

### Invariants

### Models
ImageCacheWritePolicyWT, ImageCacheWritePolicyWB

### Notes

@tparam T the type that should be a model of CImageCacheWritePolicy.
 */
template <typename T>
struct CImageCacheWritePolicy
            // Use derivation for coarser concepts, like
            // : CoarserConcept<T>
            // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
            // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
{
    // ----------------------- Concept checks ------------------------------
public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::ImageContainer ImageContainer;
    // possibly check these types so as to satisfy a concept with
//BOOST_CONCEPT_ASSERT(( CConcept< InnerType > ));
    // To test if two types A and X are equals, use
//BOOST_STATIC_ASSERT(( ConceptUtils::SameType<A,X>::value ));
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CImageCacheWritePolicy )
    {
        // Static members of type A can be tested with
//ConceptUtils::sameType( myA, T::staticMember );
        // non-const method dummy should take parameter myA of type A and return
        // something of type B
        myT.writeOnPage(myIC, myPoint, myValue);
        myT.flushPage(myIC);
        // look at CInteger.h for testing tags.
        // check const methods.
        checkConstConstraints();
    }
    void checkConstConstraints() const
    {
        // const method dummyConst should take parameter myA of type A and return
        // something of type B
//ConceptUtils::sameType( myB, myT.dummyConst( myA ) );
    }
    // ------------------------- Private Datas --------------------------------
private:
    T myT; // do not require T to be default constructible.
    ImageContainer * myIC;
    typename T::Point myPoint;
    typename T::Value myValue;
//    A myA;
//    B myB;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CImageCacheWritePolicy

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImageCacheWritePolicy_h

#undef CImageCacheWritePolicy_RECURSES
#endif // else defined(CImageCacheWritePolicy_RECURSES)
