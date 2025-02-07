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
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h"
#include "DGtal/kernel/CPointPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingParallelepipedEstimator
  /**
   * Description of template class 'PlaneProbingParallelepipedEstimator' <p>
   * \brief Aim:
   *
   * @tparam TPredicate the probing predicate, a model of concepts::CPointPredicate.
   */
  template <concepts::CPointPredicate TPredicate, ProbingMode mode>
  class PlaneProbingParallelepipedEstimator
  {
    // ----------------------- Public types ------------------------------
  public:
      class NotAbovePredicate;
      using Self                        = PlaneProbingParallelepipedEstimator<TPredicate, mode>;
      using Predicate                   = TPredicate;
      using Point                       = typename Predicate::Point;
      using Vector                      = Point;
      using Integer                     = typename Predicate::Integer;
      using TetrahedronEstimator        = PlaneProbingTetrahedronEstimator<NotAbovePredicate, mode>;
      using Triangle                    = typename TetrahedronEstimator::Triangle;
      using PointOnProbingRay           = typename TetrahedronEstimator::PointOnProbingRay;
      using Quantity                    = typename TetrahedronEstimator::Quantity;
      using UpdateOperation             = typename TetrahedronEstimator::UpdateOperation;
      using HexagonState                = typename TetrahedronEstimator::HexagonState;

      class NotAbovePredicate
      {
      public:

      using Point   = typename Self::Point;
      using Integer = typename Self::Integer;

          /**
           * Constructs the NotAbove predicate.
           *
           * @param aPredicate the InPlane predicate.
           * @param aBound the bound used in the NotAbve predicate.
           * @param aParallelepipedEstimator a pointer on a plane-probing parallelepiped estimator (to access the fixed point 'q').
           */
          NotAbovePredicate (Predicate const& aPredicate, Integer const& aBound, Self* aParallelepipedEstimator);

          /**
           * Copy assignment operator.
           * @param other the object to copy.
           * @return a reference on 'this'.
           */
          NotAbovePredicate& operator= (const NotAbovePredicate & other);

          /**
           * A wrapper around the "is a point inside the plane?" predicate.
           *
           * @param aPoint any digital point.
           * @return true if the point is inside, false otherwise.
           */
          bool inPlane (Point const& aPoint) const;

          /**
           * The NotAbove predicate, see @cite LMRJMIV2020.
           *
           * @param aPoint any digital point.
           * @return true if the point is not above the fixed point, false otherwise.
           */
          bool operator() (Point const& aPoint) const;

      private:
          const Predicate* myPredicate = nullptr; /**< A pointer to the InPlane predicate */
          Integer myBound; /**< The bound. */
          Self* myParallelpipedEstimator = nullptr; /**< A pointer to a plane-probing parallelepiped estimator. */

          /**
           * Returns the fixed point 'q'.
           */
          Point q () const;
      };


    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingParallelepipedEstimator() = delete;

    /**
     * Constructs a plane probing parallelepiped estimator from an initial frame and a probing predicate.
     *
     * @param aPoint the base point of the initial frame.
     * @param aM the three vectors of the initial frame.
     * @param aPredicate the probing predicate.
     * @param aBound the bound used in the NotAbove predicate.
     */
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
    /**
     * @param aIndex the index of the vector (between 0 and 2).
     * @return the i-th height vector \f$ m_i \f$.
     */
    Vector m (int aIndex) const;

    /**
     * @return the fixed point \f$ q \f$.
     */
    Point q () const;

    /**
     * @return the point 'p' that is the base point of the parallelepiped.
     */
    Point getOrigin () const;

    /**
     * @return the current state of the parallelepiped i.e. the number of points of the parallelepiped that are inside the plane.
     */
    int getState () const;

    /**
     * @return whether the current parallelepiped is separating or not (q and getOrigin are inside/outside or outside/inside the plane).
     */
    bool isSeparating () const;

    /**
     * Do one step of the estimation, but considering only a subset of candidate rays.
     * Particularly useful on digital surfaces when some directions must be discarded because
     * of 'flat' zones.
     *
     * @param aNeighbors the list of candidates ray to consider.
     * @return false if the algorithm has terminated, true otherwise.
     */
    bool advance (std::vector<PointOnProbingRay> const& aNeighbors);

    /**
     * Do one step of the estimation.
     *
     * @return false if the algorithm has terminated, true otherwise.
     */
    bool advance ();

    /**
     * Estimate the normal using a plane-probing approach, calls \a advance repeatedly.
     *
     * @param aNeighbors the list of candidates ray to consider.
     * @return the estimated normal.
     */
    Quantity compute (std::vector<PointOnProbingRay> const& aNeighbors);

    /**
     * Estimate the normal using a plane-probing approach, calls \a advance repeatedly.
     *
     * @return the estimated normal.
     */
    Quantity compute ();

    /**
     * @return the current local configuration, see HexagonState.
     */
    HexagonState hexagonState () const;

    /**
     * @return the current estimated normal.
     */
    Vector getNormal() const;

    /**
     * @return the two shortest edges of the triangle that is a basis of the lattice generated by the current estimated normal.
     */
    std::pair<Vector, Vector> getBasis () const;

    /**
     * @return 'true' if the current basis is reduced or not, 'false' otherwise.
     */
    bool isReduced () const;

    /**
     * @return whether we are in a reversed state or not (strictly less than 4 points of the parallelepiped satisfy the predicate).
     */
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
     *
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
    NotAbovePredicate myNotAbovePredicate; /**< The NotAbove predicate. */
    TetrahedronEstimator myTetrahedronEstimator; /**< Internally, we use a plane probing tetrahedron estimator. */
    bool myIsInReverseState; /**< We store whether we are in a reversed state or not */

    /**
     * Decompose a non-elementary operations into an array of elementary ones.
     *
     * @return a list of elementary operations.
     */
    std::vector<UpdateOperation> geometricalDecomposition (UpdateOperation const& aOp) const;

    /**
     * Translates (reverses) the current parallelepiped if needed (depending on the state).
     *
     * @param aOp the operation to apply.
     * @return 'true' if the parallelepiped is still separating, 'false' otherwise.
     */
    bool translateIf (UpdateOperation const& aOp);

    /**
     * A shortcut to the inPlane function of 'myNotAbovePredicate'
     * @param aPoint any digital point.
     * @return true if the point is inside, false otherwise.
     */
    bool inPlane (Point const& aPoint) const;

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
