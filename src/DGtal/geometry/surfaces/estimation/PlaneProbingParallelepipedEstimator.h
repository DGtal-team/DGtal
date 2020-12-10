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
 * @date 2020/12/07
 *
 * Header file for module PlaneProbingParallelepipedEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingParallelepipedEstimator_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingParallelepipedEstimator.h
#else // defined(PlaneProbingParallelepipedEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingParallelepipedEstimator_RECURSES

#if !defined PlaneProbingParallelepipedEstimator_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingParallelepipedEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingEstimatorCommon.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingParallelepipedEstimator
  /**
   * Description of template class 'PlaneProbingParallelepipedEstimator' <p>
   * \brief Aim:
   *
   * @tparam TPredicate the InPlane predicate.
   */
  template <typename TPredicate, ProbingMode mode>
  class PlaneProbingParallelepipedEstimator
  {
    // ----------------------- Public types ------------------------------
  public:
      class NotAbovePredicate;

      using Self                 = PlaneProbingParallelepipedEstimator<TPredicate, mode>;
      using Predicate            = TPredicate;
      using Point                = typename Predicate::Point;
      using Integer              = typename Predicate::Integer;
      using TetrahedronEstimator = PlaneProbingTetrahedronEstimatorParallelepiped<NotAbovePredicate, mode>;
      using Triangle             = typename TetrahedronEstimator::Triangle;
      using ProbingRay           = typename TetrahedronEstimator::ProbingRay;
      using Quantity             = typename TetrahedronEstimator::Quantity;

      class NotAbovePredicate
      {
      public:
          using Point = Self::Point;
          using Integer = Self::Integer;

      public:
          NotAbovePredicate (Predicate const& aPredicate, Integer const& aBound, Self* aParallelepipedEstimator);

          bool InPlane (Point const& aPoint) const;

          bool operator() (Point const& aPoint) const;

      private:
          Predicate const& myPredicate;
          Integer myBound;
          Self* myParallelpipedEstimator = nullptr;

          Point q () const;
      };


    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingParallelepipedEstimator() = delete;

    PlaneProbingParallelepipedEstimator(Point const& aPoint, Triangle const& aM,
                                        Predicate const& aPredicate, Integer const& aBound);

    /**
     * Destructor.
     */
    ~PlaneProbingParallelepipedEstimator() = default;

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingParallelepipedEstimator ( const PlaneProbingParallelepipedEstimator & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingParallelepipedEstimator ( PlaneProbingParallelepipedEstimator && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingParallelepipedEstimator & operator= ( const PlaneProbingParallelepipedEstimator & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingParallelepipedEstimator & operator= ( PlaneProbingParallelepipedEstimator && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    Point q () const;

    Point m (int aIndex) const;

    Point getOrigin () const;

    int state () const;

    bool isSeparating () const;

    bool advance (std::vector<ProbingRay> const& aNeighbors);

    bool advance ();

    Quantity compute ();

    Point getNormal() const;

    bool isInReverseState () const;

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

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:
    NotAbovePredicate myNotAbovePredicate;
    TetrahedronEstimator myTetrahedronEstimator;
    bool myIsInReverseState;

    void reverseIf ();
  }; // end of class PlaneProbingParallelepipedEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingParallelepipedEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingParallelepipedEstimator' to write.
   * @return the output stream after the writing.
   */
  template <typename TPredicate, ProbingMode mode>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingParallelepipedEstimator<TPredicate, mode> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingParallelepipedEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingParallelepipedEstimator_h

#undef PlaneProbingParallelepipedEstimator_RECURSES
#endif // else defined(PlaneProbingParallelepipedEstimator_RECURSES)
