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
 * @file COrientationFunctor2d.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/22
 *
 * Header file for concept COrientationFunctor2d.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(COrientationFunctor2d_RECURSES)
#error Recursive header files inclusion detected in COrientationFunctor2d.h
#else // defined(COrientationFunctor2d_RECURSES)
/** Prevents recursive inclusion of headers. */
#define COrientationFunctor2d_RECURSES

#if !defined COrientationFunctor2d_h
/** Prevents repeated inclusion of headers. */
#define COrientationFunctor2d_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/CPointFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class COrientationFunctor2d
  /**
     Description of \b concept '\b COrientationFunctor2d' <p>
     @ingroup Concepts
     @brief Aim: This concept gathers models used to perform
     an orientation test given three points. These three points
     are provided in two steps so that this concept is a refinement
     of CPointFunctor: 
      - First, we set the first two points by method 'init'. 
      - Then, we look at the position of the third point with respect
      to the first two ones: 'operator()' takes an input point and 
      returns a signed value.  

     The returned value is guaranteed to be: 
     - zero if the three points belong to the same line
     - strictly positive if the three points are counter-clockwise oriented
     - striclty negative if the three points are clockwise oriented
     
     ### Refinement of CPointFunctor

     ### Associated types
     As a refinement of CPointFunctor, it has the following nested types: 
     - Point type of input points
     - Value type of the result, at least a model of CSignedNumber

     ### Notation
     - \e X : A type that is a model of COrientationFunctor2d
     - \e x : object of type X
     - \e a , \e b : objects of type Point

     ### Valid expressions and semantics

     | Name           | Expression  | Type requirements  | Return type | Precondition | Semantics         | Post condition | Complexity |
     |----------------+-------------+--------------------+-------------+--------------+-------------------+----------------+------------|
     | initialization | x.init(a,b) | a and b are points |             |              | memorizes a and b |                | constant   |

     ### Models
  
     OrientationFunctor2dBy2x2DetComputer

     @tparam T the type that should be a model of COrientationFunctor2d.
  */
  template <typename T>
  struct COrientationFunctor2d : CPointFunctor<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:

    BOOST_CONCEPT_USAGE( COrientationFunctor2d )
    {
      myX.init( myA, myB );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; 
    typename T::Point myA, myB;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept COrientationFunctor2d

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined COrientationFunctor2d_h

#undef COrientationFunctor2d_RECURSES
#endif // else defined(COrientationFunctor2d_RECURSES)
