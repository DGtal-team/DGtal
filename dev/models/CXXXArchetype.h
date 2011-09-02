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
#pragma GCC diagnostic ignored "-Wunused-parameter"

/**
 * @file XXX.h
 * @author AUTHOR (\c EMAIL )
 * INSTITUTION
 *
 * @date 2000/??/??
 *
 * Definition of Archetype CXXXArchetype
 *
 * This file is part of the DGtal library.
 */

#if defined(CXXXArchetype_RECURSES)
#error Recursive header files inclusion detected in XXX.h
#else // defined(CXXXArchetype_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CXXXArchetype_RECURSES

#if !defined CXXXArchetype_h
/** Prevents repeated inclusion of headers. */
#define CXXXArchetype_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "YYY/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace YYY
{

  /////////////////////////////////////////////////////////////////////////////
  // class XXX
  /**
    Description of template class 'CXXXArchetype' <p>
    @ingroup Archetypes 

    @brief Aim: The archetype for the concept CXXX. A dummy model that
    represents ...
  */
  template <typename TSpace>
  class CXXXArchetype
  {
    // ----------------------- associated types -------------------------------
  public:
    // ----------------------- methods ------------------------------
    // Prototypes of methods should be written with a dummy code {}.
    // For methods/functions returning a value of type X use
    // { return DummyObject<X>::get(); }
  public:
  }; // end of concept XXX
  
} // namespace YYY

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CXXXArchetype_h

#undef CXXXArchetype_RECURSES
#endif // else defined(CXXXArchetype_RECURSES)
