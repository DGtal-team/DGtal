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
 * Header file for module PlaneProbingEstimatorHelper.cpp
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
        inline
        typename Point::Coordinate squaredNorm (Point const& aPoint)
        {
            using Integer = typename Point::Coordinate;
            Integer res = DGtal::NumberTraits<Integer>::ZERO;

            for (typename Point::Dimension i = 0; i < aPoint.size(); ++i)
            {
                res += aPoint[i] * aPoint[i];
            }

            return res;
        }

        /**
         * Determinant of a NxN matrix represented by a two-dimensional static array.
         *
         * @param aMatrix the static array representing the matrix.
         * @return the determinant of the input matrix.
         */
        template < int N, typename T >
        inline
        T determinant (const T aMatrix[N][N])
        {
            DGtal::SimpleMatrix<T, N, N> m;

            for (int i = 0; i < N; ++i)
            {
                for (int j = 0; j < N; ++j)
                {
                    m.setComponent(i, j, aMatrix[i][j]);
                }
            }

            return m.determinant();
        }

        /**
         * Computes the distance of a point to a sphere passing through 4 given points.
         *
         * @param aPoints 5 points where the fist 4 define the sphere and the last one is the point to which we want to compute the distance.
         * @return the distance of the last point to the sphere defined by the first 4 points.
        */
        template < typename Point >
        inline
        typename Point::Coordinate distToSphere (std::array<Point, 5> const& aPoints)
        {
            using Integer = typename Point::Coordinate;
            Integer one = DGtal::NumberTraits<Integer>::ONE,
                    zero = DGtal::NumberTraits<Integer>::ZERO;

            Integer M0[4][4] = { { aPoints[0][0], aPoints[0][1], aPoints[0][2], one },
                                 { aPoints[1][0], aPoints[1][1], aPoints[1][2], one },
                                 { aPoints[2][0], aPoints[2][1], aPoints[2][2], one },
                                 { aPoints[3][0], aPoints[3][1], aPoints[3][2], one } };

            if (determinant(M0) == zero)
            {
                throw std::runtime_error("4 coplanar points in distToSphere");
            }

            Integer M[5][5] = { { aPoints[0][0], aPoints[0][1], aPoints[0][2], squaredNorm(aPoints[0]), one },
                                { aPoints[1][0], aPoints[1][1], aPoints[1][2], squaredNorm(aPoints[1]), one },
                                { aPoints[2][0], aPoints[2][1], aPoints[2][2], squaredNorm(aPoints[2]), one },
                                { aPoints[3][0], aPoints[3][1], aPoints[3][2], squaredNorm(aPoints[3]), one },
                                { aPoints[4][0], aPoints[4][1], aPoints[4][2], squaredNorm(aPoints[4]), one } };

            return determinant(M);
        }

        /**
         * Test if a pair of vectors form a reduced basis.
         *
         * @param aU the first vector.
         * @param aV the second vector.
         * @return 'true' if (\a aU, \a aV) is a reduced basis, 'false' otherwise.
         */
        template < typename Point >
        inline
        bool isBasisReduced (Point const& aU, Point const& aV)
        {
            Point w = aU + aV, x = aU - aV;
            return (squaredNorm(aU) <= squaredNorm(w)) &&
                   (squaredNorm(aU) <= squaredNorm(x)) &&
                   (squaredNorm(aV) <= squaredNorm(w)) &&
                   (squaredNorm(aV) <= squaredNorm(x));
        }

        /////////////////////////////////////////////////////////////////////////////
        // template class ProbingRay
        /**
         * Description of template class 'ProbingRay' <p>
         * A ray consists of a permutation \f$ \sigma \f$ and an integer index \f$ \lambda \f$ (position on the ray).
         * For a triplet of vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ and a point \f$ q \f$, a point on the ray is defined as:
         * \f$ q - m_{\sigma(0)} + m_{\sigma(1)} + \lambda m_{\sigma(2)} \f$.
         *
         * This class is used to represent rays for a plane-probing estimator, so in practice the point \f$ q \f$ is the fixed point
         * and the three vectors \f$ (m_k)_{0 \leq k \leq 2} \f$ are the vectors defining the current probing frame.
         *
         * @tparam Integer the integer type.
         */
        template < typename Integer = int >
        class ProbingRay
        {
            public:
                using Permutation = std::array<int, 3>;

            public:
                /**
                 * Default constructor.
                 */
                ProbingRay () = default;

                /**
                 * Constructs a ray with a permutation and an index.
                 *
                 * @param aSigma a permutation.
                 * @param aIndex an index.
                 */
                ProbingRay (Permutation const& aSigma, Integer const& aIndex = Integer(0))
                    : mySigma(aSigma), myIndex(aIndex)
                {}

                /**
                 * @return the first point on the ray (with index 0).
                 */
                ProbingRay getBase () const
                {
                    return ProbingRay(mySigma, 0);
                }

                /**
                 * @return the permutation that defines the ray.
                 */
                Permutation const& sigma () const
                {
                    return mySigma;
                }

                /**
                 * @param aIndex an index between 0 and 2.
                 * @return the i-th element of the permutation that defines the ray.
                 */
                int sigma (int aIndex) const
                {
                    assert(aIndex >= 0 && aIndex <= 2);
                    return mySigma[aIndex];
                }

                /**
                 * @return index of the current point on the ray.
                 */
                Integer const& index () const
                {
                    return myIndex;
                }

                /**
                 * Equality test between two rays.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are the same, false otherwise.
                 */
                bool operator== (ProbingRay const& aRay) const
                {
                    return (mySigma == aRay.mySigma) && (myIndex == aRay.index());
                }

                /**
                 * Inequality test between two rays.
                 *
                 * @param aRay an other ray.
                 * @return true if the two rays are different, false otherwise.
                 */
                bool operator!= (ProbingRay const& aRay) const
                {
                    return !(*this == aRay);
                }

                /**
                 * Comparison operator between two rays (lexicographic order on the indices).
                 *
                 * @param aRay an other ray.
                 * @return true if *this <= aRay, false otherwise.
                 *
                 */
                bool operator<= (ProbingRay const& aRay) const
                {
                    return (mySigma == aRay.mySigma) && (myIndex <= aRay.index());
                }

                /**
                 * @param aInc an increment.
                 * @return a new point on a ray, with index the current index incremented by aInc.
                 */
                ProbingRay next (Integer const& aInc) const
                {
                    return ProbingRay(mySigma, myIndex + aInc);
                }

                /**
                 * @param aDec a decrement.
                 * @return a new point on a ray, with index the current index decremented by aInc.
                 */
                ProbingRay previous (Integer const& aDec) const
                {
                    return ProbingRay(mySigma, myIndex - aDec);
                }

            private:
                Permutation mySigma; /**< The permutation. */
                Integer myIndex; /**< The index. */
        }; // end of class ProbingRay

        /**
         * Display a probing ray on the standard output.
         *
         * @param aOs the output stream where the object is written.
         * @param aRay the probing ray to display.
         * @return the output stream after the writing.
         */
        template < typename Integer >
        inline
        std::ostream& operator<< (std::ostream& aOs, ProbingRay<Integer> const& aRay)
        {
            aOs << "sigma=(" <<
                aRay.sigma(0) << ", " <<
                aRay.sigma(1) << ", " <<
                aRay.sigma(2) << "); i=" << aRay.index();
            return aOs;
        }
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
