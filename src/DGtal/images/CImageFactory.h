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
 * @file CImageFactory.h
 * @author Martial Tola (\c martial.tola@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/02/22
 *
 * Header file for concept CImageFactory.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CImageFactory_RECURSES)
#error Recursive header files inclusion detected in CImageFactory.h
#else // defined(CImageFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CImageFactory_RECURSES

#if !defined CImageFactory_h
/** Prevents repeated inclusion of headers. */
#define CImageFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CImageFactory
/**
Description of \b concept '\b CImageFactory' <p>
@ingroup Concepts
@brief Aim: Defines the concept describing an image factory

### Refinement of

### Associated types :
 - \e OutputImage : type of the image created by the factory, model of concept CImage
 - \e Domain : type of the image domain, model of concept CDomain

### Notation
 - \e X : A type that is a model of CImageFactory
 - \e x : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

### Invariants

### Models
ImageFactoryFromImage

### Notes

@tparam T the type that should be a model of CImageFactory.
 */
template <typename T>
struct CImageFactory
            // Use derivation for coarser concepts, like
            // : CoarserConcept<T>
            // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
            // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
{
    // ----------------------- Concept checks ------------------------------
public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::OutputImage OutputImage;
    // possibly check these types so as to satisfy a concept with
    BOOST_CONCEPT_ASSERT(( CImage< OutputImage > ));
    // To test if two types A and X are equals, use
//BOOST_STATIC_ASSERT(( ConceptUtils::SameType<A,X>::value ));
    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CImageFactory )
    {
        // Static members of type A can be tested with
//ConceptUtils::sameType( myA, T::staticMember );
        // non-const method dummy should take parameter myA of type A and return
        // something of type B
        ConceptUtils::sameType( myOI, myT.requestImage(myDomain) );
        myT.flushImage(myOI);
        myT.detachImage(myOI);
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
    OutputImage * myOI;
    typename T::Domain myDomain;
//    A myA;
//    B myB;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CImageFactory

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CImageFactory_h

#undef CImageFactory_RECURSES
#endif // else defined(CImageFactory_RECURSES)
