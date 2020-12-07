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
 * Header file for module PlaneProbingEstimatorCommon.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(PlaneProbingEstimatorCommon_RECURSES)
#error Recursive header files inclusion detected in PlaneProbingEstimatorCommon.h
#else // defined(PlaneProbingEstimatorCommon_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PlaneProbingEstimatorCommon_RECURSES

#if !defined PlaneProbingEstimatorCommon_h
/** Prevents repeated inclusion of headers. */
#define PlaneProbingEstimatorCommon_h

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
         * @param aMatrix the matrix.
         * @return the determinant of the input matrix.
         */
        template < int N, typename T >
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
        bool isBasisReduced (Point const& aU, Point const& aV)
        {
            Point w = aU + aV, x = aU - aV;
            return (squaredNorm(aU) <= squaredNorm(w)) &&
                   (squaredNorm(aU) <= squaredNorm(x)) &&
                   (squaredNorm(aV) <= squaredNorm(w)) &&
                   (squaredNorm(aV) <= squaredNorm(x));
        }

        /**
         * A ray consists of a permutation 'sigma' and an index (position on the ray).
         */
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
                 * @param aSigma a permutation.
                 * @param aIndex an index.
                 */
                ProbingRay (Permutation const& aSigma, int aIndex = 0)
                    : mySigma(aSigma), myIndex(aIndex)
                {}

                /**
                 * Returns the first point on the ray.
                 */
                ProbingRay getBase () const
                {
                    return ProbingRay(mySigma, 0);
                }

                /**
                 * Returns the permutation that defines the ray.
                 */
                Permutation sigma () const
                {
                    return mySigma;
                }

                /**
                 * Returns the i-th element of the permutation that defines the ray.
                 */
                int sigma (int aIndex) const
                {
                    assert(aIndex >= 0 && aIndex <= 2);
                    return mySigma[aIndex];
                }

                /**
                 * Returns index of the current point on the ray.
                 */
                int index () const
                {
                    return myIndex;
                }

                template < typename Point >
                Point getRelPt (std::array<Point, 3> const& aM) const
                {
                    return aM[mySigma[0]] - aM[mySigma[1]] - aM[mySigma[2]] * myIndex;
                }

                template < typename Point >
                Point getAbsPt (std::array<Point, 3> const& aM, Point const& aQ) const
                {
                    return aQ - getRelPt(aM);
                }

                bool operator== (ProbingRay const& aRay) const
                {
                    return (mySigma == aRay.mySigma) && (myIndex == aRay.index());
                }

                bool operator!= (ProbingRay const& aRay) const
                {
                    return !(*this == aRay);
                }

                bool operator<= (ProbingRay const& aRay) const
                {
                    return (mySigma == aRay.mySigma) && (myIndex <= aRay.index());
                }

                ProbingRay next (int aInc) const
                {
                    return ProbingRay(mySigma, myIndex + aInc);
                }

                ProbingRay previous (int aDec) const
                {
                    return ProbingRay(mySigma, myIndex - aDec);
                }

            private:
                Permutation mySigma;
                int myIndex;
        };

        /**
         * Display a probing ray on the standard output.
         *
         * @param aOs an ouput stream.
         * @param aRay the probing ray to display.
         */
        std::ostream& operator<< (std::ostream& aOs, ProbingRay const& aRay)
        {
            aOs << "sigma=(" <<
                aRay.sigma(0) << ", " <<
                aRay.sigma(1) << ", " <<
                aRay.sigma(2) << "); i=" << aRay.index();
            return aOs;
        }

        /**
         * Compute a reduced basis from a triplet of points.
         *
         * @param aM three points that define a basis of the lattice \f$ \mathbb{Z}^3 \f$.
         * @return the reduced basis.
         */
        template < typename Point >
        std::pair<Point, Point> computeReducedBasis (std::array<Point, 3> const& aM)
        {
            Point u = aM[1] - aM[0],
                  v = aM[2] - aM[1],
                  w = aM[0] - aM[2];

            assert(w == -u - v);

            if (squaredNorm(u) < squaredNorm(v))
            {
                if (squaredNorm(u) < squaredNorm(w))
                {
                    if (squaredNorm(-w) < squaredNorm(v))
                    {
                        return std::make_pair(u, -w);
                    }
                    else
                    {
                        return std::make_pair(u, v);
                    }
                }
                else
                {
                    if (squaredNorm(-v) < squaredNorm(u))
                    {
                        return std::make_pair(w, -v);
                    }
                    else
                    {
                        return std::make_pair(w, u);
                    }
                }
            }
            else
            {
                if (squaredNorm(v) < squaredNorm(w))
                {
                    if (squaredNorm(-u) < squaredNorm(w))
                    {
                        return std::make_pair(v, -u);
                    }
                    else
                    {
                        return std::make_pair(v, w);
                    }
                }
                else
                {
                    if (squaredNorm(-v) < squaredNorm(u))
                    {
                        return std::make_pair(w, -v);
                    }
                    else
                    {
                        return std::make_pair(w, u);
                    }
                }
            }
        }

    } // namespace detail

    /**
     * Probing mode for plane-probing estimators.
     */
    enum class ProbingMode
    {
        H,
        R,
        R1,
    };

    /**
     * Display a mode on the standard output.
     *
     * @param aOs the output stream.
     * @param aMode the mode to display.
     */
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
        }

        return aOs;
    }
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/PlaneProbingEstimatorCommon.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PlaneProbingEstimatorCommon_h

#undef PlaneProbingEstimatorCommon_RECURSES
#endif // else defined(PlaneProbingEstimatorCommon_RECURSES)
