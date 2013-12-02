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
 * @file PredicateFromOrientationFunctor2D.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/12/02
 *
 * Header file for module PredicateFromOrientationFunctor2D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PredicateFromOrientationFunctor2D_RECURSES)
#error Recursive header files inclusion detected in PredicateFromOrientationFunctor2D.h
#else // defined(PredicateFromOrientationFunctor2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PredicateFromOrientationFunctor2D_RECURSES

#if !defined PredicateFromOrientationFunctor2D_h
/** Prevents repeated inclusion of headers. */
#define PredicateFromOrientationFunctor2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/base/Alias.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/geometry/tools/determinant/COrientationFunctor2D.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PredicateFromOrientationFunctor2D
  /**
   * Description of template class 'PredicateFromOrientationFunctor2D' <p>
   * \brief Aim: Small adapter to models of COrientationFunctor2D. 
   * It is a model of CPointPredicate. It is also a ternary predicate on points, 
   * useful for basic geometric tasks such as convex hull computation. 
   *
   * Once a orientation functor is defined, you can adapt it as follows: 
   * @snippet geometry/tools/determinant/exampleOrientationFunctors2D.cpp PredicateDefinition
   *
   * The test can be done in one or two separate steps as follows: 
   * @snippet geometry/tools/determinant/exampleOrientationFunctors2D.cpp FunctorInitialization
   * @snippet geometry/tools/determinant/exampleOrientationFunctors2D.cpp PredicateUsage
   *
   * The default behavior is to return 'true' for strictly positive functor values.
   * You can however custom this behavior with the last two template parameters. 
   *
   * @tparam TOrientationFunctor a model of COrientationFunctor2D
   * @tparam acceptNeg bool equal to 'true' to get a predicate returning 'true' 
   * for strictly negative values of the functor, but equal to 'false' to get a predicate 
   * returning 'true' for strictly positive values (default). 
   * @tparam acceptZero bool equal to 'true' to get a predicate returning 'true' 
   * for null values, but equal to 'false' for strictly positive or negative 
   * values (default).  
   */
  template <typename TOrientationFunctor, bool acceptNeg = false, bool acceptZero = false >
  class PredicateFromOrientationFunctor2D
  {
    // ----------------------- Types ------------------------------------------
  public: 
    /**
     * Type of the adapter orientation functor
     */
    typedef TOrientationFunctor Functor; 
    BOOST_CONCEPT_ASSERT(( COrientationFunctor2D<Functor> )); 
    
    /**
     * Type of input points
     */
    typedef typename Functor::Point Point; 

    /**
     * Type of the thresholder, which compares the result
     * of the functor to zero
     */
    typedef Thresholder<typename Functor::Value, acceptNeg, acceptZero> MyThresholder; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor
     * @param aFunctor any orientation functor to alias
     */
    PredicateFromOrientationFunctor2D( Alias<Functor> aFunctor );

    /**
     * Initialization of @a myFunctor
     * @param aPoint1 a first point
     * @param aPoint2 a second point
     */
    void init( const Point& aPoint1, const Point& aPoint2 );

    /**
     * unary function operator
     * @param aPoint any point
     */
    bool operator()( const Point& aPoint ) const;

    /**
     * ternary function operator
     * @param aPoint any point
     */
    bool operator()(const Point& aPoint1, const Point& aPoint2, const Point& aPoint3 ) const;

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Private Datas --------------------------------
  private:

    /**
     * a pointer to an orientation functor 
     */
    Functor* const myFunctorPtr; 

    /**
     * object that thresholds the result of the functor
     */
    const MyThresholder myThresholder; 


  }; // end of class PredicateFromOrientationFunctor2D


  /**
   * Overloads 'operator<<' for displaying objects of class 'PredicateFromOrientationFunctor2D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PredicateFromOrientationFunctor2D' to write.
   * @return the output stream after the writing.
   */
  template <typename TOrientationFunctor, bool acceptNeg, bool acceptZero>
  std::ostream&
  operator<< ( std::ostream & out, const PredicateFromOrientationFunctor2D<TOrientationFunctor, acceptNeg, acceptZero > & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/tools/determinant/PredicateFromOrientationFunctor2D.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PredicateFromOrientationFunctor2D_h

#undef PredicateFromOrientationFunctor2D_RECURSES
#endif // else defined(PredicateFromOrientationFunctor2D_RECURSES)
