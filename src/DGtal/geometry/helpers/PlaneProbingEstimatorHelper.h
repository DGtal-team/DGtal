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
 * Helper functions for plane-probing algorithms.
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingEstimatorHelper_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingEstimatorHelper.h
#else // defined(PlaneProbingEstimatorHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingEstimatorHelper_RECURSES

#if !defined PlaneProbingEstimatorHelper_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingEstimatorHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <array>
#include <cassert>
#include "DGtal/math/linalg/SimpleMatrix.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
    namespace detail
    {
        /**
         * Version of DGtal::PointVector::squaredNorm with no conversion to double.
         *
         * @param aPoint an input digital point.
         * @return the squared norm of the input point.
         */
        template < typename Point >
        typename Point::Coordinate squaredNorm (Point const& aPoint);

        /**
         * Determinant of a NxN matrix represented by a two-dimensional static array.
         *
         * @param aMatrix the static array representing the matrix.
         * @return the determinant of the input matrix.
         */
        template < int N, typename T >
        T determinant (const T aMatrix[N][N]);

        /**
         * Computes the distance of a point to a sphere passing through 4 given points.
         *
         * @param aPoints 5 points where the first 4 define the sphere and the last one is the point to which we want to compute the distance.
         * @return the distance of the last point to the sphere defined by the first 4 points.
        */
        template < typename Point >
        inline
        typename Point::Coordinate distToSphere (std::array<Point, 5> const& aPoints);

        /**
         * Test if a pair of vectors form a reduced basis.
         *
         * @param aU the first vector.
         * @param aV the second vector.
         * @return 'true' if (\a aU, \a aV) is a reduced basis, 'false' otherwise.
         */
        template < typename Point >
        inline
        bool isBasisReduced (Point const& aU, Point const& aV);

        /////////////////////////////////////////////////////////////////////////////
        // template class PointOnProbingRay
        /**
         * Description of template class 'PointOnProbingRay' <p>
         * A ray consists of a permutation \f$ \sigma \f$ and an integer index \f$ \lambda \f$ (position on the ray).
         * For a triplet of vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ and a point \f$ q \f$, a point on the ray is defined as:
         * \f$ q - m_{\sigma(0)} + m_{\sigma(1)} + \lambda m_{\sigma(2)} \f$. \f$ q - m_{\sigma(0)} + m_{\sigma(1)} \f$ is called the \e base point.
         *
         * This class is used to represent points on rays for a plane-probing estimator, so in practice the point \f$ q \f$ is the fixed point
         * and the three vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ are the vectors defining the current probing frame.
         *
         * @tparam Integer the integer type, model of concepts::CInteger.
         */
        template < typename Integer = int >
        class PointOnProbingRay
        {
            BOOST_CONCEPT_ASSERT(( concepts::CInteger<Integer> ) );

            // ----------------------- Public types ------------------------------
            public:
                using Permutation = std::array<int, 3>;

            public:
                /**
                 * Default constructor.
                 */
                PointOnProbingRay () = default;

                /**
                 * Constructs a ray with a permutation and an index.
                 *
                 * @param aSigma a permutation.
                 * @param aIndex an index.
                 */
                PointOnProbingRay (Permutation const& aSigma, Integer const& aIndex = Integer(0));

                /**
                 * @return the base point of the ray (with index 0).
                 */
                PointOnProbingRay getBase () const;

                /**
                 * @return the permutation that defines the ray.
                 */
                Permutation const& sigma () const;

                /**
                 * @param aIndex an index between 0 and 2.
                 * @return the i-th element of the permutation that defines the ray.
                 */
                int sigma (int aIndex) const;

                /**
                 * @return index of the current point on the ray.
                 */
                Integer const& index () const;

                /**
                 * Equality test between two rays: the internal permutations and
                 * indices must be the same.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are the same, false otherwise.
                 */
                bool operator== (PointOnProbingRay const& aRay) const;

                /**
                 * Inequality test between two rays.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are different, false otherwise.
                 */
                bool operator!= (PointOnProbingRay const& aRay) const;

                /**
                 * Comparison operator between two rays: one ray is less than another if they have
                 * the same internal permutation and the first one has a smaller index than the second
                 * one.
                 *
                 * @param aRay an other ray.
                 * @return true if *this <= aRay, false otherwise.
                 *
                 */
                bool operator<= (PointOnProbingRay const& aRay) const;

                /**
                 * @param aInc an increment.
                 * @return a new point on a ray, with index the current index incremented by aInc.
                 */
                PointOnProbingRay next (Integer const& aInc) const;

                /**
                 * @param aDec a decrement.
                 * @return a new point on a ray, with index the current index decremented by aInc.
                 */
                PointOnProbingRay previous (Integer const& aDec) const;

            private:
                Permutation mySigma; /**< The permutation. */
                Integer myIndex; /**< The index. */
        }; // end of class PointOnProbingRay

        /**
         * Display a probing ray on the standard output.
         *
         * @param aOs the output stream where the object is written.
         * @param aRay the probing ray to display.
         * @return the output stream after the writing.
         */
        template < typename Integer >
        std::ostream& operator<< (std::ostream& aOs, PointOnProbingRay<Integer> const& aRay);
    } // namespace detail
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/helpers/PlaneProbingEstimatorHelper.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingEstimatorHelper_h

#undef PlaneProbingEstimatorHelper_RECURSES
#endif // else defined(PlaneProbingEstimatorHelper_RECURSES)
