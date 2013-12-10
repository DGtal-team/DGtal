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
 * @file COrientationPredicate2.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/11/22
 *
 * Header file for concept COrientationPredicate2.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(COrientationPredicate2_RECURSES)
#error Recursive header files inclusion detected in COrientationPredicate2.h
#else // defined(COrientationPredicate2_RECURSES)
/** Prevents recursive inclusion of headers. */
#define COrientationPredicate2_RECURSES

#if !defined COrientationPredicate2_h
/** Prevents repeated inclusion of headers. */
#define COrientationPredicate2_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/kernel/CSignedNumber.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class COrientationPredicate2
  /**
     Description of \b concept '\b COrientationPredicate2' <p>
     @ingroup Concepts
     @brief Aim: This concept gathers models used to perform
     an orientation test given three points. These three points
     are provided in two steps so that this concept is a refinement
     of CPointPredicate: 
      - First, we set the first two points by method 'init'. 
      - Then, we look at the position of the third point with respect
      to the first two ones: 'operator()' takes an input point and 
      returns 'true' or 'false'.  
     
     ### Refinement of CPointPredicate

     ### Associated types
     As a refinement of CPointPredicate, it has the following nested types: 
     - Point type of input points

     ### Notation
     - \e X : A type that is a model of COrientationPredicate2
     - \e x : object of type X
     - \e a , \e b : objects of type Point

     ### Valid expressions and semantics

     | Name           | Expression  | Type requirements  | Return type | Precondition | Semantics         | Post condition | Complexity |
     |----------------+-------------+--------------------+-------------+--------------+-------------------+----------------+------------|
     | initialization | x.init(a,b) | a and b are points |             |              | memorizes a and b |                | constant   |

     ### Models
  
     OrientationPredicate2By2x2DetComputer

     @tparam T the type that should be a model of COrientationPredicate2.
  */
  template <typename T>
  struct COrientationPredicate2 : CPointPredicate<T>
  {
    // ----------------------- Concept checks ------------------------------
  public:

    BOOST_CONCEPT_ASSERT(( CSignedNumber< typename T::Value > ));

    BOOST_CONCEPT_USAGE( COrientationPredicate2 )
    {
      myX.init( myA, myB );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; 
    typename T::Point myA, myB;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept COrientationPredicate2

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined COrientationPredicate2_h

#undef COrientationPredicate2_RECURSES
#endif // else defined(COrientationPredicate2_RECURSES)
