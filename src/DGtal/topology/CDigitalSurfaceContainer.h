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
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/CDigitalSurfaceTracker.h"
//////////////////////////////////////////////////////////////////////////////

// @since 0.8 In DGtal::concepts
namespace DGtal {
  namespace concepts {

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSurfaceContainer
  /**
Description of \b concept '\b CDigitalSurfaceContainer' <p>
@ingroup Concepts

@brief Aim: The digital surface container concept describes a
minimal set of inner types and methods so as to describe the data
of digital surfaces. 

Digital surfaces arise in many different contexts:

- an explicit set of oriented surfels
- the boundary of an explicit set of spels
- the boundary of an explicit set of digital points
- the boundary of a set of digital points, defined implicitly by a
predicate: Point -> bool
- a set of oriented surfels, defined implicitly by a
predicate: Surfel -> bool
- a set of oriented surfels, implicitly by a
predicate: Oriented Surfel -> bool
- the boundary of a region in a labelled image
- the frontier between two regions in a labelled image
- ...

Since there are so many digital surfaces, it is necessary to
provide a mechanism to handle them generically. The class
DigitalSurface will be the common proxy to hide models of
CDigitalSurfaceContainer.

Hence CDigitalSurfaceContainer describes how to access the
data representing the digital surface in common way.

See @ref dgtal_digsurf_sec3_2

# Refinement of boost::CopyConstructible
    
# Associated types
- KSpace: the type of cellular grid space in which lies the digital surface.
- Surfel: the type of an oriented n-1-cell in this space.
- SurfelConstIterator: the type for iterating over the of surfels of the digital surface, must be a model of boost_concepts::SinglePassIteratorConcept, boost_concepts::ReadableIteratorConcept
- DigitalSurfaceTracker: the type for tracking surfels over the digital surface
- Size: the integral type for counting elements.

# Notation
- \c X : A type that is a model of CDigitalSurfaceContainer
- \c x : object of type X
- \c s : object of type Surfel
    
# Definitions
    
# Valid expressions and semantics

| Name          | Expression | Type requirements   | Return type | Precondition     | Semantics | Post condition | Complexity |
|---------------|------------|---------------------|-------------|------------------|-----------|----------------|------------|
| space accessor| \e x.space()|                    | const \e KSpace & |            | returns a reference to the cellular grid space in which lies the digital surface. | | |
| inside test   | \e x.isInside( s )|              | \e bool     |                  | returns \c true iff the surfel \c s belongs to this digital surface. | | |
| begin of range| \e x.begin()|                    | \e SurfelConstIterator |       | returns a const iterator pointing to the first element in the digital surface, seen as a collection of surfels. | | |
| end of range  | \e x.end()|                      | \e SurfelConstIterator |       | returns an iterator pointing past the last element in the digital surface, seen as a collection of surfels. | | |
| tracker instanciation | \e x.newTracker( s )|    | \e DigitalSurfaceTracker* |    | returns a dynamically allocated instance of tracker initialized at the surfel \e s. | | |
| connectedness test | \e x.connectedness()|       | \c enum \e Connectedness |     | returns either DISCONNECTED, CONNECTED, UNKNOWN depending on the surface. | | |
| number of surfels | \e x.nbSurfels()|            | \e Size     |                  | returns the number of surfels of this surface. | | |
| empty container test | \e x.empty()|             | \e bool     |                  | returns \c true iff the digital surface contains no surfel. | | |
    
# Invariants
    
# Models
- DigitalSetBoundary, SetOfSurfels, ImplicitDigitalSurface, LightImplicitDigitalSurface, ExplicitDigitalSurface, LightExplicitDigitalSurface

# Notes
@tparam T the type that should be a model of CDigitalSurfaceContainer.
   */
  template <typename T>
  concept CDigitalSurfaceContainer = 
    std::is_copy_constructible_v<T> &&  
    CCellularGridSpaceND<typename T::KSpace> &&
    CDigitalSurfaceTracker<typename T::DigitalSurfaceTracker> &&
    ConceptUtils::SinglePassIterator<typename T::SurfelConstIterator> && 
    ConceptUtils::ReadableIterator<typename T::SurfelConstIterator> &&
    requires(const T myX, typename T::Surfel mySurfel)
    {
        { myX.space() } -> std::same_as<const typename T::KSpace&>;
        { myX.isInside(mySurfel) } -> std::same_as<bool>;
        { myX.begin() } -> std::same_as<typename T::SurfelConstIterator>;
        { myX.end() } -> std::same_as<typename T::SurfelConstIterator>;
        { myX.newTracker(mySurfel) } -> std::same_as<typename T::DigitalSurfaceTracker*>;
        { myX.connectedness() } -> std::same_as<Connectedness>;
        { myX.nbSurfels() } -> std::same_as<typename T::Size>;
        { myX.empty() } -> std::same_as<bool>;
    };
} // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSurfaceContainer_h

#undef CDigitalSurfaceContainer_RECURSES
#endif // else defined(CDigitalSurfaceContainer_RECURSES)
