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
 * @file PointPredicateToPointFunctor.h
 * @brief Convert a point predicate to a point functor.
 *
 * @author Jeremy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2013/02/20
 *
 * Header file for module PointPredicateToPointFunctor.ih
 *
 * This file is part of the DGtal library.
 */

#if defined(PointPredicateToPointFunctor_RECURSES)
#error Recursive header files inclusion detected in PointPredicateToPointFunctor.h
#else // defined(PointPredicateToPointFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointPredicateToPointFunctor_RECURSES

#if !defined PointPredicateToPointFunctor_h
/** Prevents repeated inclusion of headers. */
#define PointPredicateToPointFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"

#include "DGtal/kernel/CPointPredicate.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PointPredicateToPointFunctor
  /**
   * Description of template class 'PointPredicateToPointFunctor' <p>
   *
   * \brief Aim: Convert a predicate on Digital Point to a Functor on Digital Point
   *
   * @tparam TPointPredicate a model of predicate on Digital Points.
   */
  template <typename TPointPredicate>
  class PointPredicateToPointFunctor
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef TPointPredicate PointPredicate;
    typedef typename PointPredicate::Point Point;
    typedef int Value;

    BOOST_CONCEPT_ASSERT(( CPointPredicate< PointPredicate > ));

    /**
      * Constructor.
      *
      * @param[in] predicate a predicate on digital points.
      * @param[in] reverseIsInside in some case, the value return by the functor in inverse depending is dimension (we have this case in 2D and 3D).
      */
    PointPredicateToPointFunctor ( ConstAlias< PointPredicate > predicate, bool reverseIsInside = false)
      : myPredicate(predicate),
        reverse(reverseIsInside)

    {}

    /**
     * Destructor.
     */
    ~PointPredicateToPointFunctor();


    // ----------------------- Interface --------------------------------------
  public:
    /**
     * Call the functor on Point by converting the Cell to a Point.
     *
     * @param[in] aCell any cell in the Khalimsky space.
     *
     * @return 'ONE' if the point is inside the shape, 'ZERO' if it is strictly outside.
     */
    Value operator()( const Point & aPoint ) const
    {
      if (!reverse)
      {
        return myPredicate(aPoint) ? NumberTraits<Value>::ONE : NumberTraits<Value>::ZERO;
      }
      else
      {
        return myPredicate(aPoint) ? NumberTraits<Value>::ZERO : NumberTraits<Value>::ONE;
      }
    }


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

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    PointPredicateToPointFunctor & operator= ( const PointPredicateToPointFunctor & other ){return *this;}

    // ------------------------- Private Datas --------------------------------
  private:
    /// Const ref on predicate on Points. Used on operator() to get the return value
    const PointPredicate & myPredicate;

    /// If true, reverse all value of operator()
    const bool reverse;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    PointPredicateToPointFunctor();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    PointPredicateToPointFunctor ( const PointPredicateToPointFunctor & other );




    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class PointPredicateToPointFunctor


  /**
   * Overloads 'operator<<' for displaying objects of class 'PointPredicateToPointFunctor'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PointPredicateToPointFunctor' to write.
   * @return the output stream after the writing.
   */
  template <typename TP>
  std::ostream&
  operator<< ( std::ostream & out, const PointPredicateToPointFunctor<TP> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/PointPredicateToPointFunctor.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PointPredicateToPointFunctor_h

#undef FunctorOnCells_RECURSES
#endif // else defined(PointPredicateToPointFunctor_RECURSES)
