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
 * @author Tristan Roussilllon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2024/09/16
 *
 * Header file for module PlaneProbingLNeighborhood.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingLNeighborhood_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingLNeighborhood.h
#else // defined(PlaneProbingLNeighborhood_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingLNeighborhood_RECURSES

#if !defined PlaneProbingLNeighborhood_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingLNeighborhood_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cassert>
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingRNeighborhood.h"
#include "DGtal/kernel/CPointPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingLNeighborhood
  /**
   * Description of template class 'PlaneProbingLNeighborhood' <p>
   * \brief Aim: Represents a way to probe the L-neighborhood, 
   * see \cite Lu2022 for details.
   *
   * \tparam TPredicate the probing predicate, a model of concepts::CPointPredicate.
   */
  template <typename TPredicate>
  class PlaneProbingLNeighborhood: public DGtal::PlaneProbingRNeighborhood<TPredicate>
  {
    BOOST_CONCEPT_ASSERT((DGtal::concepts::CPointPredicate<TPredicate>));

    // ----------------------- Public types ------------------------------
  public:
    using Predicate         = TPredicate;
    using Point             = typename Predicate::Point;
    using Vector            = Point;
    using Integer           = typename Point::Coordinate;
    using Triangle          = std::array<Vector, 3>;

    using HexagonState        = typename PlaneProbingNeighborhood<TPredicate>::HexagonState;
    using UpdateOperation     = typename PlaneProbingNeighborhood<TPredicate>::UpdateOperation;

    using Index                 = typename PlaneProbingNeighborhood<TPredicate>::Index;  
    using PointOnProbingRay     = typename PlaneProbingNeighborhood<TPredicate>::PointOnProbingRay;
    using GridPoint             = typename detail::GridPoint<Integer, Index>;
    using GridPointOnProbingRay = typename detail::GridPointOnProbingRay<Integer, Index>;
    
    // ----------------------- Internal type -------------------------------
  private: 
    /**
     * Description of data structure 'ClosestGridPoint' <p>
     * \brief Aim: Used to store the closest grid point associated 
     * to a vertex of the triangle and two extra boolean values
     * about the local configuration at that vertex. 
     * 
     * More precisely, given a triplet of vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ 
     * and a point \f$ q \f$, let us denote \f$ v \f$ the vertex equal to 
     * \f$ q - m_k \f$. The first boolean value is 'true' iff
     * the predicate returns 'true' on \f$ v - m_{k+1} \f$ and,  
     * similarly, the second boolean value is 'true' iff
     * the predicate returns 'true' on \f$ v - m_{k+2} \f$
     * (the indices are taken modulo 3). 
     */    
    struct ClosestGridPoint
    {
      /** 
       * Default constructor.
       */
      ClosestGridPoint () = default;

      /**
       * Constructor
       *
       * @param aGridPoint the closest grid point (possibly invalid)
       * @param aFirst the first boolean value
       * @param aSecond the second boolean value
       */
      ClosestGridPoint (const GridPoint& aGridPoint,
			const bool& aFirst, const bool& aSecond )
	: myGridPoint(aGridPoint), myPair(std::make_pair(aFirst,aSecond)) {}

      GridPoint myGridPoint; /**< a grid point, which can be invalid 
				 if no grid point belong to the underlying surface */

      std::pair<bool,bool> myPair; /**< pair of boolean values that encode 
				       the local configuration */
    };
    
    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingLNeighborhood() = delete;

    /**
     * Constructor.
     *
     * @param aPredicate a probing predicate.
     * @param aQ the fixed point 'q'.
     * @param aM a frame composed of the three vectors.
     */
    PlaneProbingLNeighborhood(Predicate const& aPredicate, Point const& aQ, Triangle const& aM);

    /**
     * Destructor.
     */
    ~PlaneProbingLNeighborhood();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    PlaneProbingLNeighborhood ( const PlaneProbingLNeighborhood & other ) = delete;

    /**
     * Move constructor.
     * @param other the object to move.
     */
    PlaneProbingLNeighborhood ( PlaneProbingLNeighborhood && other ) = delete;

    /**
     * Copy assignment operator.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    PlaneProbingLNeighborhood & operator= ( const PlaneProbingLNeighborhood & other ) = delete;

    /**
     * Move assignment operator.
     * @param other the object to move.
     * @return a reference on 'this'.
     */
    PlaneProbingLNeighborhood & operator= ( PlaneProbingLNeighborhood && other ) = delete;

    // ----------------------- Plane-Probing services ------------------------------
  public:
    
    /**
     * Computes the current state of the neighborhood.
     * This is the function that is overloaded for the different probing modes.
     *
     * @return the hexagon state, see HexagonState.
     */
    HexagonState hexagonState () override;

    /**
     * Computes the closest candidate point, used for updating a frame in a plane probing based estimator.
     *
     * @return the update operation to apply.
     */
    UpdateOperation closestCandidate () override;
    
    // ------------------------- Protected Datas ------------------------------
  protected:

    std::vector<ClosestGridPoint> myGrids; /**< closest point and additional useful data stored at each vertex. */
    
    // ------------------------- Helpers to find a closest point --------------
  protected:
    
    /**
     * Computes the closest candidate point in a given grid identified by the 
     * index of the associated vertex.
     *
     * @param aIdx
     * @return an instance of ClosestGridPoint.
     */
    ClosestGridPoint closestInGrid (const Index& aIdx) const;

    /**
     * Computes the candidate grid points lying in a cone given by two grid points.
     *
     * @param y1 a first grid point
     * @param y2 a second grid point
     * @param out an output iterator on grid points
     */
    void candidatesInGrid (const GridPoint& y1, const GridPoint& y2,
			   std::back_insert_iterator<std::vector<GridPoint> > out) const;

    /**
     * Finds a closest point on a given ray using a linear search.
     *
     * @param aRay a ray.
     * @param aBound a bound that limits the search range. 
     * @return a closest point on the ray.
     */
    GridPointOnProbingRay closestOnBoundedRayLinearWithPredicate (GridPointOnProbingRay const& aRay, Integer const& aBound) const;
    
    /**
     * Finds a closest point on a given ray using a binary search.
     *
     * @param aRay a ray.
     * @param aBound a bound that limits the search range. 
     * @return a closest point on the ray.
     */
    GridPointOnProbingRay closestOnBoundedRayLogWithPredicate (GridPointOnProbingRay const& aRay, Integer const& aBound) const; 
   
    /**
     * Constructs an update operation from the closest candidate point.
     *
     * @param aClosest the closest candidate point.
     * @return the update operation.
     */
    UpdateOperation getOperationFromGridPoint (GridPoint const& aClosest) const;
   
    /**
     * Update a grid after a triangle update. This procedure is called at the 
     * beginning of every call to hexagonState, which must prepare the computations. 
     *
     * @param aIdx
     */
    void updateGrid (const Index& aIdx);

    /**
     * Returns the vector from the base to a grid point.
     *
     * @param aP a point on a grid.
     * @return the vector.
     */
    Point direction (GridPoint const& aP) const;

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

  }; // end of class PlaneProbingLNeighborhood


  /**
   * Overloads 'operator<<' for displaying objects of class 'PlaneProbingLNeighborhood'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'PlaneProbingLNeighborhood' to write.
   * @return the output stream after the writing.
   */
  template <typename TPredicate>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingLNeighborhood<TPredicate> & object );

} // namespace DGtal


  ///////////////////////////////////////////////////////////////////////////////
  // Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingLNeighborhood.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingLNeighborhood_h

#undef PlaneProbingLNeighborhood_RECURSES
#endif // else defined(PlaneProbingLNeighborhood_RECURSES)
