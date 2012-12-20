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
 * @file CDigitalSet.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et
 * Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/01
 *
 * Header file for concept CDigitalSet.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalSet_RECURSES)
#error Recursive header files inclusion detected in CDigitalSet.h
#else // defined(CDigitalSet_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalSet_RECURSES

#if !defined CDigitalSet_h
/** Prevents repeated inclusion of headers. */
#define CDigitalSet_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSet
  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSet
  /**
Description of \b concept '\b CDigitalSet' <p>
     @ingroup Concepts

     @brief Aim: Represents a set of points within the given
     domain. This set of points is modifiable by the user.
     
 ### Refinement of boost::CopyConstructible, boost::Assignable

     @todo add boost::Container ? Not for now, since coding style do
     not match with STL (e.g. Iterator instead of iterator).
    
 ### Associated types :
    
 ### Notation
     - \t X : A type that is a model of CDigitalSet
     - \t x, \t y : object of type X
    
 ### Definitions
    
 ### Valid expressions and 
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
        <td class=CName>            </td> 
        <td class=CExpression>      </td>
        <td class=CRequirements>    </td> 
        <td class=CReturnType>      </td>
        <td class=CPrecondition>    </td> 
        <td class=CSemantics>       </td> 
        <td class=CPostCondition>   </td> 
        <td class=CComplexity>      </td>
      </tr>
    
     </table>
    
 ### Invariants###
    
 ### Models###
    
 ### Notes###

@tparam T the type that should be a model of CDigitalSet.
   */
  template <typename T> 
  struct CDigitalSet :
    boost::CopyConstructible< T >, 
    boost::Assignable< T >
  {
    // ----------------------- Concept checks ------------------------------
  public:
    // 1. define first provided types (i.e. inner types), like
    typedef typename T::Domain Domain;
    typedef typename T::Point Point;
    typedef typename T::Size Size;
    typedef typename T::Iterator Iterator;
    typedef typename T::ConstIterator ConstIterator;

    //BOOST_CONCEPT_ASSERT(( boost::BidirectionalIterator< Iterator > ));
    //    BOOST_CONCEPT_ASSERT(( boost_concepts::LvalueIteratorConcept<Iterator > ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<Iterator > ));
//DigitalSetFromMap cannot be a model of CDigitalSet if lvalue is required because 
//in STL maps, in pairs <const key, value>, key is const... 
    BOOST_CONCEPT_ASSERT(( boost_concepts::BidirectionalTraversalConcept<Iterator > ));

    //BOOST_CONCEPT_ASSERT(( boost::BidirectionalIterator<ConstIterator > ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<ConstIterator > ));
    BOOST_CONCEPT_ASSERT(( boost_concepts::BidirectionalTraversalConcept<ConstIterator > ));

    // To test if two types A and Y are equals, use
    // BOOST_STATIC_ASSERT( ConceptUtils::sameType<A,X>::value );    

    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CDigitalSet )
    {
      ConceptUtils::sameType( myDomain, myX.domain() );
      ConceptUtils::sameType( mySize, myX.size() );
      ConceptUtils::sameType( myBool, myX.empty() );
      myX.insert( myPoint );
      // template <typename PointInputIterator>
      //   BOOST_CONCEPT_REQUIRES
      //   ( ((boost::InputIterator<PointInputIterator>)),
      //     (void) )
      //   myX.insert( PointInputIterator, PointInputIterator );
      myX.insertNew( myPoint );
      ConceptUtils::sameType( mySize, myX.erase( myPoint ) );
      myX.erase( myIterator );
      myX.erase( myIterator, myIterator );
      myX.clear();
      ConceptUtils::sameType( myIterator, myX.begin() );
      ConceptUtils::sameType( myIterator, myX.end() );
      ConceptUtils::sameType( myX, myX.operator+=( myX ) );
      myX.computeComplement( myOutputIt );
      myX.assignFromComplement( myX );
      myX.computeBoundingBox( myPoint, myPoint );
      checkConstConstraints();
      checkNonConstConstraints();
      // look at CInteger.h for testing tags.
    }
    /**
       This method checks const methods when a non-const version exist.
    */
    void checkConstConstraints() const
    {
      ConceptUtils::sameType( myConstIterator, myX.find( myPoint ) );
      ConceptUtils::sameType( myConstIterator, myX.begin() );
      ConceptUtils::sameType( myConstIterator, myX.end() );
    }

    /**
       This method checks non-const methods when a const version exist.
    */
    void checkNonConstConstraints()
    {
      ConceptUtils::sameType( myIterator, myX.find( myPoint ) );
      ConceptUtils::sameType( myIterator, myX.begin() );
      ConceptUtils::sameType( myIterator, myX.end() );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    Domain myDomain;
    Size mySize;
    bool myBool;
    Point myPoint;
    Iterator myIterator;
    ConstIterator myConstIterator;
    Point* myOutputIt; 
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDigitalSet
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/sets/CDigitalSet.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSet_h

#undef CDigitalSet_RECURSES
#endif // else defined(CDigitalSet_RECURSES)
