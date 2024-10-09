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
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.h"
#include "DGtal/geometry/surfaces/estimation/PlaneProbingNeighborhood.h"
#include "DGtal/kernel/CPointPredicate.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
    /**
     * Probing mode for plane-probing estimators.
     * This mode allows to select the good \ref PlaneProbingNeighborhood subclass when
     * constructing a \ref PlaneProbingTetrahedronEstimator.
     */
    enum class ProbingMode
    {
        H, /**< The H-neighborhood composed of 6 points on an hexagon, see \ref PlaneProbingHNeighborhood.*/
        R, /**< The R-neighborhood composed of 6 rays, see \ref PlaneProbingRNeighborhood. */
        R1, /**< The R-neighborhood but with an optimization to reduce the number of calls to the predicate, see \ref PlaneProbingR1Neighborhood. */
        L, /**< The L-neighborhood composed of three lattices, see \ref PlaneProbingLNeighborhood. */
    };

    /**
     * Display a mode on the standard output.
     *
     * @param aOs the output stream.
     * @param aMode the mode to display.
     * @return the output stream after the writing.
     */
    inline
    std::ostream& operator<< (std::ostream& aOs, ProbingMode const& aMode)
    {
        switch (aMode)
        {
            case ProbingMode::H:
                aOs << "H";
                break;

            case ProbingMode::R:
                aOs << "R";
                break;

            case ProbingMode::R1:
                aOs << "R1";
                break;

            case ProbingMode::L:
                aOs << "L";
                break;
        }

        return aOs;
    }

  /////////////////////////////////////////////////////////////////////////////
  // template class PlaneProbingTetrahedronEstimator
  /**
   * Description of template class 'PlaneProbingTetrahedronEstimator' <p>
   * \brief Aim: A class that locally estimates a normal on a digital set using only
   * a predicate "does a point x belong to the digital set or not?".
   *
   * @tparam TPredicate the probing predicate, a model of concepts::CPointPredicate.
   * @tparam mode the probing mode, see DGtal::ProbingMode.
   *
   * @todo Add PlaneProbingAlgorithm concept.
   */
  template <typename TPredicate, ProbingMode mode = ProbingMode::H>
  class PlaneProbingTetrahedronEstimator
  {
    BOOST_CONCEPT_ASSERT((concepts::CPointPredicate<TPredicate>));

    // ----------------------- Public types ------------------------------
  public:
      using Self                   = PlaneProbingTetrahedronEstimator<TPredicate, mode>;
      using Predicate              = TPredicate;
      using Point                  = typename Predicate::Point;
      using Vector                 = Point;
      using Neighborhood           = PlaneProbingNeighborhood<TPredicate>;
      using Triangle               = typename Neighborhood::Triangle;
      using Integer                = typename Point::Coordinate;
      using PointOnProbingRay      = typename Neighborhood::PointOnProbingRay;
      using Permutation            = typename PointOnProbingRay::Permutation;
      using Quantity               = Vector;
      using HexagonState           = typename Neighborhood::HexagonState;
      using UpdateOperation        = typename Neighborhood::UpdateOperation;

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
    /**
     * @param aIndex the index of the vector (between 0 and 2).
     * @return the i-th height vector \f$ m_i \f$.
     */
    Vector const& m (int aIndex) const;

    /**
     * @return the fixed point \f$ q \f$.
     */
    Point const& q () const;

    /**
     * @return the point 'p' that is the base point of the tetrahedron.
     */
    Point getOrigin () const;

    /**
     * @return the three vertices of the base of the tetrahedron.
     */
    Triangle vertices () const;

    /**
     * @return the two shortest edges of the triangle that is a basis of the lattice generated by the current estimated normal.
     */
    std::pair<Vector, Vector> getBasis() const;

    /**
     * @return 'true' if the current basis is reduced or not, 'false' otherwise.
     */
    bool isReduced () const;

    /**
     * @return the estimated normal.
     */
    Vector getNormal () const;

    /**
     * Do one step of the estimation, but considering only a subset of candidate rays.
     * Particularly useful on digital surfaces when some directions must be discarded because
     * of 'flat' zones.
     *
     * @param aNeighbors the list of candidates ray to consider.
     * @return a pair (op, b) where b = false if the algorithm has terminated, true otherwise; and op is the
     * operation used to update the current tetrahedron.
     */
    std::pair<bool, UpdateOperation> advance (std::vector<PointOnProbingRay> const& aNeighbors);

    /**
     * Do one step of the estimation.
     *
     * @return false if the algorithm has terminated, true otherwise.
     */
    std::pair<bool, UpdateOperation> advance ();

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
     * Translates the fixed point \f$ q \f$ by a given vector.
     *
     * @param aTranslation a translation vector.
     */
    void translateQ (Vector const& aTranslation);

    /**
     * Translates the fixed point \f$ q \f$, this is used by the parallelepiped version of the estimator,
     * see PlaneProbingParallelepipedEstimator.
     */
    void translateQ ();

    /**
     * Applies an operation.
     *
     * @param aOp the operation to apply.
     */
    void applyOperation (UpdateOperation const& aOp);

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
    /**
     * Tests whether the fixed point 'q' projects the given triangle, along the current estimated normal.
     *
     * @param aTriangle the 3 points of the triangle.
     */
    bool isProjectedInside (Triangle const& aTriangle) const;
    /**
     * Tests whether the fixed point 'q' projects into the base, along the current estimated normal.
     */
    bool isProjectedInside () const;
    /**
     * Checks whether the predicate is true for all vertices of 
     * the base triangle or not.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isInside() const; 
    /**
     * Checks whether the three vectors stored in 'myM'
     * are the columns of a unimodular matrix or not.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isUnimodular() const;

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
  template <typename TPredicate, ProbingMode mode>
  std::ostream&
  operator<< ( std::ostream & out, const PlaneProbingTetrahedronEstimator<TPredicate, mode> & object );
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingTetrahedronEstimator_h

#undef PlaneProbingTetrahedronEstimator_RECURSES
#endif // else defined(PlaneProbingTetrahedronEstimator_RECURSES)
