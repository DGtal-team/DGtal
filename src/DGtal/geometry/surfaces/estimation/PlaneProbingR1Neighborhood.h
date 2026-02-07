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
 * Header file for module PlaneProbingR1Neighborhood.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingR1Neighborhood_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingR1Neighborhood.h
#else // defined(PlaneProbingR1Neighborhood_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingR1Neighborhood_RECURSES

#if !defined PlaneProbingR1Neighborhood_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingR1Neighborhood_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingRNeighborhood.h"
#include "DGtal/kernel/CPointPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingR1Neighborhood
  /**
   * Description of template class 'PlaneProbingR1Neighborhood' <p>
   * \brief Aim: Represent a way to probe the R-neighborhood, with the R1 optimization,
   * see \cite RLDGCI2019 for details.
   *
   * \tparam TPredicate the probing predicate, a model of concepts::CPointPredicate.
   */
  template <typename TPredicate>
  class PlaneProbingR1Neighborhood : public PlaneProbingRNeighborhood<TPredicate>
  {
    BOOST_CONCEPT_ASSERT((concepts::CPointPredicate<TPredicate>));

    // ----------------------- Public types ------------------------------
  public:
      using Predicate           = TPredicate;
      using Point               = typename TPredicate::Point;
      using Triangle            = typename PlaneProbingRNeighborhood<TPredicate>::Triangle;
      using PointOnProbingRay   = typename PlaneProbingRNeighborhood<TPredicate>::PointOnProbingRay;
      using Integer             = typename Point::Coordinate;
      using HexagonState        = typename PlaneProbingRNeighborhood<TPredicate>::HexagonState;

      using Index               = typename PlaneProbingRNeighborhood<TPredicate>::Index;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingR1Neighborhood() = delete;

    /**
     * Constructor.
     *
     * @param aPredicate a probing predicate.
     * @param aQ the fixed point 'q'.
     * @param aM a frame composed of the three vectors.
     */
    PlaneProbingR1Neighborhood(Predicate const& aPredicate, Point const& aQ, Triangle const& aM);

    /**
     * Destructor.
     */
    ~PlaneProbingR1Neighborhood();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingR1Neighborhood ( const PlaneProbingR1Neighborhood & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingR1Neighborhood ( PlaneProbingR1Neighborhood && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingR1Neighborhood & operator= ( const PlaneProbingR1Neighborhood & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingR1Neighborhood & operator= ( PlaneProbingR1Neighborhood && other ) = delete;

    // ----------------------- Plane Probing services ------------------------------
  public:
    virtual HexagonState hexagonState () override;

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
    mutable std::array<bool, 6> myState; /**< The current state of the H-neighborhood. */

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:
    /**
     * @return an integer coding the current configuration of the H-neighborhood.
     */
    int getNeighborhoodCode () const;

    /**
     * @param index an integer between 0 and 2.
     * @return a pair of a point on a ray and a ray.
     */
    std::pair<PointOnProbingRay, PointOnProbingRay> candidateRay (Index const& index) const;

    /**
     * @param aPoint a point on a ray.
     * @param aRay a ray.
     * @return extremal points of aRay included in the sphere passing through aPoint and the current triangle.
     */
    std::vector<PointOnProbingRay> intersectSphereRay (PointOnProbingRay const& aPoint, PointOnProbingRay const& aRay) const;

    /**
     * Sanity check for point returned by intersectSphereRay.
     *
     * @param aPoint a point on a ray.
     * @param aRay a ray.
     * @param aLst a list of points on a ray.
     * @return 'true' if the points of aLst are indeed in the sphere passing through aPoint and the current triangle, 'false' otherwise.
     */
    bool isValidIntersectSphereRay (PointOnProbingRay const& aPoint, PointOnProbingRay const& aRay,
                                    std::vector<PointOnProbingRay> const& aLst) const;

    /**
     * @param aRay a ray.
     * @return the closest point on a ray using \cite RLDGCI2019 Algorithm 4.
     */
    PointOnProbingRay closestPointOnRayConstant (PointOnProbingRay const& aRay) const;

    /**
     * @param aRay a ray.
     * @return the closest point on the ray using a linear search.
     */
    PointOnProbingRay closestPointOnRayLinear (PointOnProbingRay const& aRay) const;

    /**
     * @param aRayPoint a pair describing a ray and a point on a ray.
     * @return the closest point among the points of a ray and a point on another ray.
     */
    PointOnProbingRay closestRayPoint (std::pair<PointOnProbingRay, PointOnProbingRay> const& aRayPoint) const;

  }; // end of class PlaneProbingR1Neighborhood


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingR1Neighborhood'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingR1Neighborhood' to write.
   * @return the output stream after the writing.
   */
  template <typename TPredicate>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingR1Neighborhood<TPredicate> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingR1Neighborhood.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingR1Neighborhood_h

#undef PlaneProbingR1Neighborhood_RECURSES
#endif // else defined(PlaneProbingR1Neighborhood_RECURSES)
