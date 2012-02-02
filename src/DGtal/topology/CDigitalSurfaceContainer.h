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
 * @file CDigitalSurfaceContainer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/02
 *
 * Header file for concept CDigitalSurfaceContainer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalSurfaceContainer_RECURSES)
#error Recursive header files inclusion detected in CDigitalSurfaceContainer.h
#else // defined(CDigitalSurfaceContainer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalSurfaceContainer_RECURSES

#if !defined CDigitalSurfaceContainer_h
/** Prevents repeated inclusion of headers. */
#define CDigitalSurfaceContainer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/Topology.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSurfaceContainer
  /**
     Description of \b concept '\b CDigitalSurfaceContainer' <p>
     @ingroup Concepts
     @brief Aim:
     
     <p> Refinement of boost_concepts::CopyConstructible
    
     <p> Associated types :
    
     <p> Notation
     - \t X : A type that is a model of CDigitalSurfaceContainer
     - \t x, \t y : object of type X
    
     <p> Definitions
    
     <p> Valid expressions and semantics <br>
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
    
     <p> Invariants <br>
    
     <p> Models <br>

     A dummy model (for concept checking) is CCDigitalSurfaceContainerArchetype.

     <p> Notes <br>

     @tparam T the type that should be a model of CDigitalSurfaceContainer.
   */
  template <typename T> 
  struct CDigitalSurfaceContainer : boost::CopyConstructible<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::KSpace KSpace;
    typedef typename T::Surfel Surfel;
    typedef typename T::SurfelConstIterator SurfelConstIterator;
    typedef typename T::DigitalSurfaceTracker DigitalSurfaceTracker;
    typedef typename T::Size Size;

    BOOST_CONCEPT_ASSERT(( boost_concepts::SinglePassIteratorConcept<SurfelConstIterator> ));

    // 2. then check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( CDigitalSurfaceContainer )
    {
      // check const methods.
      checkConstConstraints();
    }
    void checkConstConstraints() const
    {
      // x.space() const, returns a const KSpace &
      ConceptUtils::sameType( myKSpace, myX.space() );
      // x.isInside( Surfel ) const, returns bool.
      ConceptUtils::sameType( myBool, myX.isInside( mySurfel ) );
      // x.begin() const, returns SurfelConstIterator
      ConceptUtils::sameType( mySurfelCIt, myX.begin() );
      // x.end() const, returns SurfelConstIterator
      ConceptUtils::sameType( mySurfelCIt, myX.end() );
      // x.newTracker( Surfel ) const, returns DigitalSurfaceTracker*
      ConceptUtils::sameType( myPtrTracker, myX.newTracker( mySurfel ) );
      // x.connectedness() const, returns Connectedness
      ConceptUtils::sameType( myConnectedness, myX.connectedness() );
      // x.nbSurfels() const, returns Connectedness
      ConceptUtils::sameType( mySize, myX.nbSurfels() );
      // x.empty() const, returns bool
      ConceptUtils::sameType( myBool, myX.empty() );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // only if T is default constructible.
    KSpace myKSpace;
    Surfel mySurfel;
    bool myBool;
    SurfelConstIterator mySurfelCIt;
    DigitalSurfaceTracker* myPtrTracker;
    Connectedness myConnectedness;
    Size mySize;
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CDigitalSurfaceContainer
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSurfaceContainer_h

#undef CDigitalSurfaceContainer_RECURSES
#endif // else defined(CDigitalSurfaceContainer_RECURSES)
