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
 * @file CSCellValueFunctor.h
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/26
 *
 * Header file for concept CSCellValueFunctor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSCellValueFunctor_RECURSES)
#error Recursive header files inclusion detected in CSCellValueFunctor.h
#else // defined(CSCellValueFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSCellValueFunctor_RECURSES

#if !defined CSCellValueFunctor_h
/** Prevents repeated inclusion of headers. */
#define CSCellValueFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSCellValueFunctor
  /**
     Description of \b concept '\b CSCellValueFunctor' <p>
     @ingroup Concepts
     @brief Aim: defines the concept of functors on signed cells.
     
     <p> Refinement of boost::UnaryFunction
    
     <p> Associated types :
      - Value: functor return type
      - SCell: signed cell type
    
     <p> Notation
     - \t X : A type that is a model of CSCellValueFunctor
     - \t x, \t y : object of type X
    
     <p> Definitions
    
     <p> Valid expressions and semantics <br>
  
     <p> Invariants <br>
    
     <p> Models <br>

     <p> Notes <br>

     @tparam T the type that should be a model of CSCellValueFunctor.
   */
  template <typename T> 
  struct CSCellValueFunctor 
  {
  // ----------------------- Concept checks ------------------------------
  public:
    typedef typename T::Value Value;
    typedef typename T::SCell SCell;
    
    BOOST_CONCEPT_ASSERT(( boost::UnaryFunction<T, Value, SCell> ));
         
  // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of concept CSCellValueFunctor
  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSCellValueFunctor_h

#undef CSCellValueFunctor_RECURSES
#endif // else defined(CSCellValueFunctor_RECURSES)
