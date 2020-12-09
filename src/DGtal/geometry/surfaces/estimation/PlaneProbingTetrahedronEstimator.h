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
 * @date 2020/12/04
 *
 * Header file for module PlaneProbingTetrahedronEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingTetrahedronEstimator_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingTetrahedronEstimator.h
#else // defined(PlaneProbingTetrahedronEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingTetrahedronEstimator_RECURSES

#if !defined PlaneProbingTetrahedronEstimator_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingTetrahedronEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingEstimatorCommon.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingNeighborhood.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingTetrahedronEstimator
  /**
   * Description of template class 'PlaneProbingTetrahedronEstimator' <p>
   * \brief Aim:
   *
   * @tparam TPredicate the InPlane predicate.
   * @tparam mode the probing mode.
   */
  template <typename TPredicate, ProbingMode mode = ProbingMode::H>
  class PlaneProbingTetrahedronEstimator
  {
    // ----------------------- Public types ------------------------------
  public:
      using Self         = PlaneProbingTetrahedronEstimator<TPredicate, mode>;
      using Predicate    = TPredicate;
      using Point        = typename Predicate::Point;
      using Neighborhood = PlaneProbingNeighborhood<TPredicate>;
      using Triangle     = typename Neighborhood::Triangle;
      using Integer      = typename Point::Coordinate;
      using ProbingRay   = typename Neighborhood::ProbingRay;
      using Permutation  = typename ProbingRay::Permutation;
      using Quantity     = Point;
      using HexagonState = typename Neighborhood::HexagonState;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingTetrahedronEstimator() = delete;

    PlaneProbingTetrahedronEstimator (Point const& aPoint, Triangle const& aM, Predicate const& aPredicate);

    /**
     * Destructor.
     */
    ~PlaneProbingTetrahedronEstimator();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingTetrahedronEstimator ( const PlaneProbingTetrahedronEstimator & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingTetrahedronEstimator ( PlaneProbingTetrahedronEstimator && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingTetrahedronEstimator & operator= ( const PlaneProbingTetrahedronEstimator & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingTetrahedronEstimator & operator= ( PlaneProbingTetrahedronEstimator && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    Point m (int aIndex) const;

    Point q () const;

    Point shift () const;

    Triangle vertices () const;

    std::pair<Point, Point> getBasis() const;

    bool isReduced () const;

    Point getNormal () const;

    bool advance (std::vector<ProbingRay> const& aNeighbors);

    bool advance ();

    void compute ();

    HexagonState hexagonState () const;

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
    Triangle myM;
    Predicate myPredicate;
    Point myS, myQ;
    Neighborhood* myNeighborhood = nullptr;

    struct Operation {
        Point oldM;
        Permutation sigma;
        Integer lambda;
    };

    std::vector<Operation> myOperations;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:
    bool isProjectedInside (Triangle const& aTriangle) const;

    void update (ProbingRay const& aRay);

  }; // end of class PlaneProbingTetrahedronEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingTetrahedronEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingTetrahedronEstimator' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingTetrahedronEstimator<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingTetrahedronEstimator_h

#undef PlaneProbingTetrahedronEstimator_RECURSES
#endif // else defined(PlaneProbingTetrahedronEstimator_RECURSES)
