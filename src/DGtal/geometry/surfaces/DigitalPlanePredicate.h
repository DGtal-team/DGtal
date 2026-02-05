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
 * @file
 * @author Jocelyn Meyron (\c jocelyn.meyron@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2020/12/03
 *
 * Header file for module DigitalPlanePredicate.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalPlanePredicate_RECURSES)
#error Recursive header files inclusion detected in DigitalPlanePredicate.h
#else // defined(DigitalPlanePredicate_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalPlanePredicate_RECURSES

#if !defined DigitalPlanePredicate_h
/** Prevents repeated inclusion of headers. */
#define DigitalPlanePredicate_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalPlanePredicate
  /**
   * Description of template class 'DigitalPlanePredicate' <p>
   * \brief Aim: Representing digital planes, which are digitizations of Euclidean planes,
   * as point predicates.
   *
   * A digital plane is a digitization of a Euclidean plane, it can
   * be seen as the set of digital points comprised between two parallel Euclidean planes.
   *
   * If N is a normal vector, \f$ \mu \f$ the lower bound and \f$ \nu \f$ the thickness then the corresponding
   * digital plane is the set of digital points \f$ x \f$ such that \f$ \mu \leq x \cdot N < \mu + \nu \f$.
   *
   * When \f$ \nu = \| N \|_1 \f$, then it represents a standard digital plane,
   * and when \f$ \nu = \| N \|_\infty \f$, then it represents a naive digital plane.
   *
   * @tparam TSpace any digital space, i.e., a model of CSpace.
   *
   * \b Models: A DigitalPlanePredicate is a model of concepts::CPointPredicate.
   */
  template <typename TSpace>
  class DigitalPlanePredicate
  {
    BOOST_CONCEPT_ASSERT((concepts::CSpace<TSpace>));

    // ----------------------- public types ------------------------------
  public:
    using Self    = DigitalPlanePredicate<TSpace>;
    using Space   = TSpace;
    using Integer = typename Space::Integer;
    using Point   = typename Space::Point;
    using Vector  = typename Space::Vector;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    DigitalPlanePredicate() = default;

    /**
     * Constructor from normal, lower bound and thickness.
     * @param aN vector that defines the normal of the plane.
     * @param aMu constant that defines the lower bound.
     * @param aNu constant that defines the thickness.
     */
    DigitalPlanePredicate (Vector const& aN, Integer const& aMu, Integer const& aNu);

    /**
     * Destructor.
     */
    ~DigitalPlanePredicate() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    DigitalPlanePredicate ( const DigitalPlanePredicate & other );

    /**
     * Move constructor.
     * @param other the object to move.
     */
    DigitalPlanePredicate ( DigitalPlanePredicate && other );

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DigitalPlanePredicate & operator= ( const DigitalPlanePredicate & other );

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    DigitalPlanePredicate & operator= ( DigitalPlanePredicate && other );

    //-------------------- plane services -----------------------------
  public:
    /**
     * @return the normal vector to the digital plane.
     */
    Vector const& normal () const;

    /**
     * @return the lower bound defining the digital plane.
     */
    Integer mu () const;

    /**
     * @return the thickness of the digital plane.
     */
    Integer nu () const;

    //-------------------- model of concepts::CPointPredicate -----------------------------
  public:
    /**
     * Checks whether a point \a aPoint belongs to the digital plane.
     *
     * @param aPoint any digital point.
     * @return 'true' if p belongs to the digital plane, 'false' otherwise.
     */
    bool operator() (Point const& aPoint) const;

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

    // ------------------------- Protected Data ------------------------------
  protected:

    // ------------------------- Private Data --------------------------------
  private:
    Vector myNormal; /**< The normal vector. */
    Integer myMu; /**< The lower bound. */
    Integer myNu; /**< The thickness. */

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalPlanePredicate


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalPlanePredicate'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalPlanePredicate' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalPlanePredicate<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/DigitalPlanePredicate.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalPlanePredicate_h

#undef DigitalPlanePredicate_RECURSES
#endif // else defined(DigitalPlanePredicate_RECURSES)
