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
 * @file CDigitalSurfaceTracker.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/02
 *
 * Header file for concept CDigitalSurfaceTracker.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDigitalSurfaceTracker_RECURSES)
#error Recursive header files inclusion detected in CDigitalSurfaceTracker.h
#else // defined(CDigitalSurfaceTracker_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDigitalSurfaceTracker_RECURSES

#if !defined CDigitalSurfaceTracker_h
/** Prevents repeated inclusion of headers. */
#define CDigitalSurfaceTracker_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal {
  namespace concepts {

  /////////////////////////////////////////////////////////////////////////////
  // class CDigitalSurfaceTracker
  /**
  Description of \b concept '\b CDigitalSurfaceTracker' <p>
  @ingroup Concepts
  @brief Aim:
  @todo Complete documentation
     
  # Refinement of 
  boost::CopyConstructible
    
  # Associated types
    
  # Notation
  - \a X : A type that is a model of CDigitalSurfaceTracker
  - \a x, \a y : object of type X
    
  # Definitions
    
  # Valid expressions and semantics
    
  | Name          | Expression | Type requirements   | Return type | Precondition     | Semantics | Post condition | Complexity |
  |---------------|------------|---------------------|-------------|------------------|-----------|----------------|------------|
  | | | | | | | | | 
     
  # Invariants
    
  # Models
  A dummy model (for concept checking) is CCDigitalSurfaceTrackerArchetype.

  # Notes

  @tparam T the type that should be a model of CDigitalSurfaceTracker.
  */
  template <typename T>
  concept CDigitalSurfaceTracker = 
    std::is_copy_constructible_v<T> && 
    requires(T myX, typename T::Surfel mySurfel, Dimension myDim, bool myBool) 
    {
        myX.move( mySurfel );
        { myX.surface() } -> std::same_as<const typename T::DigitalSurfaceContainer&>;
        { myX.current() } -> std::same_as<const typename T::Surfel&>;
        { myX.orthDir() }  -> std::same_as<Dimension>;
        { myX.adjacent(mySurfel, myDim, myBool) } -> std::same_as<uint8_t>;
    };
  
} // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDigitalSurfaceTracker_h

#undef CDigitalSurfaceTracker_RECURSES
#endif // else defined(CDigitalSurfaceTracker_RECURSES)
