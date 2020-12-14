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
   * \brief Aim: A class that locally estimates a normal on a digital set using only
   * a predicate "does a point x belong to the digital set or not?".
   *
   * @tparam TPredicate the probing predicate.
   * @tparam mode the probing mode, see DGtal::ProbingMode.
   */
  template <typename TPredicate, ProbingMode mode = ProbingMode::H>
  class PlaneProbingTetrahedronEstimator
  {
    // ----------------------- Public types ------------------------------
  public:
      using Self            = PlaneProbingTetrahedronEstimator<TPredicate, mode>;
      using Predicate       = TPredicate;
      using Point           = typename Predicate::Point;
      using Neighborhood    = PlaneProbingNeighborhood<TPredicate>;
      using Triangle        = typename Neighborhood::Triangle;
      using Integer         = typename Point::Coordinate;
      using ProbingRay      = typename Neighborhood::ProbingRay;
      using Permutation     = typename ProbingRay::Permutation;
      using Quantity        = Point;
      using HexagonState    = typename Neighborhood::HexagonState;
      using UpdateOperation = typename Neighborhood::UpdateOperation;

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Default constructor.
     */
    PlaneProbingTetrahedronEstimator() = delete;

    /**
     * Constructs a plane probing tetrahedron estimator from an initial frame and a probing predicate.
     *
     * @param aPoint the base point of the initial frame.
     * @param aM the three vectors of the initial frame.
     * @param aPredicate the probing predicate.
     */
    PlaneProbingTetrahedronEstimator (Point const& aPoint, Triangle const& aM, Predicate const& aPredicate);

    /**
     * Destructor.
     */
    virtual ~PlaneProbingTetrahedronEstimator();

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
    /**
     * Get the i⁻th height vector.
     *
     * @param aIndex the index of the vector (between 0 and 2).
     */
    Point const& m (int aIndex) const;

    /**
     * Get the fixed point.
     */
    Point const& q () const;

    /**
     * Get the shift vector i.e. the vector indicating the orthant the estimated normal will be in.
     */
    Point shift () const;

    /**
     * Get the three vertices of the base of the tetrahedron that is updatej.
     */
    Triangle vertices () const;

    /**
     * Get a basis of the lattice generated by the estimated normal.
     */
    std::pair<Point, Point> getBasis() const;

    /**
     * Tells if the current basis is reduced or not.
     */
    bool isReduced () const;

    /**
     * Get the estimated normal.
     */
    Point getNormal () const;

    /**
     * Do one step of the estimation, but considering only a subset of candidate rays.
     * Particularly useful on digital surfaces when some directions must be discarded because
     * of 'flat' zones.
     *
     * @param aNeighbors the list of candidates ray to consider.
     * @return false if the algorithm has terminated, true otherwise.
     */
    bool advance (std::vector<ProbingRay> const& aNeighbors);

    /**
     * Do one step of the estimation.
     *
     * @return false if the algorithm has terminated, true otherwise.
     */
    bool advance ();

    /**
     * Estimate the normal using a plane-probing approach, calls \a advance repeatedly.
     *
     * @return the estimated normal.
     */
    Quantity compute ();

    /**
     * Returns the current local configuration (planar, non-convex, non-planar).
     */
    HexagonState hexagonState () const;

    /**
     * Translaes the fixed point \f$ q \f$, this is used by the parallelepiped version of the estimator,
     * see PlaneProbingParallelepipedEstimator.
     */
    void translateQ ();

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
    virtual bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  protected:

    // ------------------------- Private Datas --------------------------------
  private:
    Triangle myM; /**< The three height vectors, representing the current frame. */
    Predicate const& myPredicate; /**< The probing predicate. */
    Point myS; /**< The shift vector. */
    Point myQ; /**< The fixed point 'q'. */
    Neighborhood* myNeighborhood = nullptr; /**< Describes what kind of probing method is used. */
    std::vector<UpdateOperation> myOperations; /**< The list of all operations. */

    // ------------------------- Hidden services ------------------------------
  protected:
    /**
     * Tests whether the fixed point 'q' projects the given triangle, along the current estimated normal.
     *
     * @param aTriangle the 3 points of the triangle.
     */
    bool isProjectedInside (Triangle const& aTriangle) const;

    // ------------------------- Internals ------------------------------------
  private:
    /**
     * Updates one height vector.
     *
     * @param aOp operation describing the update step.
     */
    void update (UpdateOperation const& aOp);

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

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingTetrahedronEstimatorParallelepiped
  /*
   * Helper class that overrides the isValid check to be compatible with the parallelepiped-based estimators
   * where the 3 vertices of the base frame \f$ v_i  \f$ are not necessarily in the plane.
   *
   * @tparam TPredicate the probing predicate.
   * @tparam mode the probing mode, see ProbingMode.
   */
  template <typename TPredicate, ProbingMode mode = ProbingMode::H>
  class PlaneProbingTetrahedronEstimatorParallelepiped : public PlaneProbingTetrahedronEstimator<TPredicate, mode>
  {
    // ----------------------- Public types ------------------------------
    public:
      using Predicate = TPredicate;
      using Point     = typename PlaneProbingTetrahedronEstimator<TPredicate, mode>::Point;
      using Triangle  = typename PlaneProbingTetrahedronEstimator<TPredicate, mode>::Triangle;

    // ----------------------- Standard services ------------------------------
    public:
    /**
     * Constructs a plane probing tetrahedron estimator from an initial frame and a probing predicate.
     *
     * @param aPoint the base point of the initial frame.
     * @param aM the three vectors of the initial frame.
     * @param aPredicate the probing predicate.
     */
      PlaneProbingTetrahedronEstimatorParallelepiped (Point const& aPoint, Triangle const& aM, Predicate const& aPredicate);

    // ----------------------- Interface --------------------------------------
    public:
    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
      bool isValid() const override;
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingTetrahedronEstimator_h

#undef PlaneProbingTetrahedronEstimator_RECURSES
#endif // else defined(PlaneProbingTetrahedronEstimator_RECURSES)
