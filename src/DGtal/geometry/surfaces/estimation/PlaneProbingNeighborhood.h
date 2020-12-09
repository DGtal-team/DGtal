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
 * Header file for module PlaneProbingNeighborhood.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingNeighborhood_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingNeighborhood.h
#else // defined(PlaneProbingNeighborhood_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingNeighborhood_RECURSES

#if !defined PlaneProbingNeighborhood_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingNeighborhood_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cassert>
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingEstimatorCommon.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingNeighborhood
  /**
   * Description of template class 'PlaneProbingNeighborhood' <p>
   * \brief Aim:
   *
   * \tparam TPredicate the InPlane predicate.
   */
  template <typename TPredicate>
  class PlaneProbingNeighborhood
  {
    // ----------------------- Public types ------------------------------
  public:
      using Predicate  = TPredicate;
      using Point      = typename Predicate::Point;
      using Integer    = typename Point::Coordinate;
      using Triangle   = std::array<Point, 3>;
      using ProbingRay = detail::ProbingRay;

      enum class HexagonState
      {
          Empty,
          Planar,
          NonPlanar,
          NonConvex,
      };

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingNeighborhood() = delete;

    PlaneProbingNeighborhood(Predicate const& aPredicate, Point const& aQ, Triangle const& aM);

    /**
     * Destructor.
     */
    virtual ~PlaneProbingNeighborhood();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingNeighborhood ( const PlaneProbingNeighborhood & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingNeighborhood ( PlaneProbingNeighborhood && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingNeighborhood & operator= ( const PlaneProbingNeighborhood & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingNeighborhood & operator= ( PlaneProbingNeighborhood && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    bool closestCandidate (std::vector<ProbingRay> const& neighbors, ProbingRay& closest);

    virtual HexagonState hexagonState () = 0;

    HexagonState classify (std::array<bool, 6> const& aState) const;

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
    Predicate const& myPredicate;
    Point const& myQ;
    Triangle const& myM;
    std::vector<ProbingRay> myCandidates;

    static const ProbingRay myNeighborhood[6];
    std::vector<ProbingRay> myNeighbors;

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:
    ProbingRay closestPointInList (std::vector<ProbingRay> const& aPoints) const;

    bool isNeighbor (ProbingRay const& r) const;

    /**
     * Computes the relative position of a point with respect to a sphere passing through 4 points.
     *
     * @param aX 1 more input point.
     * @param aY the test point.
     * @return 'true' if \a aY lies inside the sphere passing through \a vertices and \a aX.
     */
    bool isSmallest (Point const& aX, Point const& aY) const;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class PlaneProbingNeighborhood


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingNeighborhood'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingNeighborhood' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingNeighborhood<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingNeighborhood.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingNeighborhood_h

#undef PlaneProbingNeighborhood_RECURSES
#endif // else defined(PlaneProbingNeighborhood_RECURSES)
