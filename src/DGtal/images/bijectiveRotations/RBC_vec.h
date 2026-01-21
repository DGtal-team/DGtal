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
* @file RBC_vec.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(RBC_vec_RECURSES)
#error Recursive header files inclusion detected in RBC_vec.h
#else // defined(RBC_vec_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RBC_vec_RECURSES

#if !defined RBC_vec_h
/** Prevents repeated inclusion of headers. */
#define RBC_vec_h

#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include "PointUtils.h"
#include <DGtal/kernel/CSpace.h>
#include <DGtal/io/readers/GenericReader.h>
#include <DGtal/images/ImageSelector.h>

namespace DGtal {
    /**
     * Description of template struct RBC_vec
     * \brief RBC : Bijective Rotation through Circles
     * @tparam TSpace a 2 dimensional space.
     * @tparam TInputValue type of the input point e.g., TSpace::RealPoint.
     * @tparam TOutputValue type of the output point e.g., TSpace::Point
    */
    template<typename TSpace, typename TInputValue = typename TSpace::RealPoint, typename TOutputValue = typename TSpace::Point>
struct RBC_vec {
        typedef std::vector< TOutputValue > Circle;

        /// the array of circles, where index is the integer radius of the
        /// circle.
        std::vector< Circle >          circles;
        /// the map associating to a lattice point its polar coordinates
        /// (r,idx) in the circles.
        std::map< TOutputValue, TOutputValue > point2circle;

        /// Constructor
        /// @param max_radius the maximal distance of a point to
        /// the center of rotation.
        /// @param smart when 'true', tries to regularize the number of
        /// points of each circle, when 'false' each circle contains the
        /// point between distance r (included) and r+1 (excluded).
        RBC_vec( int max_radius, bool smart = false ) {
            init( max_radius, smart );
            my_angle = 0.0;
            my_center = TOutputValue( 0, 0 );
        }

        /// Initialization
        /// @param R the maximal distance of a point to
        /// the center of rotation.
        ///
        /// @param smart when 'true', tries to regularize the number of
        /// points of each circle, when 'false' each circle contains the
        /// point between distance r (included) and r+1 (excluded).
        void init( int R, bool smart ) {
            TOutputValue c( 0, 0 );
            circles.clear();
            circles.resize( R + 1 );
            circles[ 0 ].push_back( c );
            point2circle[ c ] = TOutputValue( 0, 0 );
            std::size_t N = 1;
            std::vector< TOutputValue > points;
            for ( int r = 1; r <= R; r++ )
            {
                circles[ r ] = computeCircle( r );
                for ( int i = 0; i < circles[ r ].size(); i++, N++ )
                {
                    point2circle[ circles[ r ][ i ] ] = TOutputValue( r, i );
                    points.push_back( circles[ r ][ i ] );
                }
            }
            if ( ! smart ) return;
            struct DistanceComparator {
                bool operator()( TOutputValue p, TOutputValue q ) const
                {
                    return p.squaredNorm() < q.squaredNorm();
                }
            };
            struct AngleComparator {
                bool operator()( TOutputValue p, TOutputValue q ) const
                {
                    double ap = atan2( p[ 1 ], p[ 0 ] );
                    double aq = atan2( q[ 1 ], q[ 0 ] );
                    return ap < aq;
                }
            };

            // Try to improve circles.
            // target number of points for circle r is
            // n(r) = a * r, with a = 2( N - 1 ) / (R(R+1))
            double a = 2.0 * double( N - 1 ) / double(R*(R+1));
            DistanceComparator dcomp;
            AngleComparator    acomp;
            std::sort( points.begin(), points.end(), dcomp );
            int current = 1;
            for ( int r = 1; r <= R; r++ )
            {
                int n = int( round( a * r / 8.0 ) * 8.0 );
                circles[ r ].resize( n );
                for ( int i = 0; i < n; i++ )
                    circles[ r ][ i ] = points[ current++ ];
                std::sort( circles[ r ].begin(), circles[ r ].end(), acomp );
                // std::cout << "r=" << r << " #C=" << circles[ r ].size() << std::endl;
            }
        }

        /// @return the angle of rotation.
        double    angle() const {
            return my_angle;
        }
        /// @return a reference to the angle of rotation.
        double&   setAngle() {
            return my_angle;
        }
        /// @return the centre of rotation
        TOutputValue  center() const {
            return my_center;
        }
        /// @return a reference to the centre of rotation
        TOutputValue& center() {
            return my_center;
        }

        /// @param p a lattice point
        /// @return the rotation of the point \a p according to the current
        /// angle and center.
        TOutputValue rotate( TInputValue p ) const {
            auto   cp   = static_cast<TOutputValue>((p - my_center));
            const auto it   = point2circle.find( cp );
            if ( it == point2circle.end() ) return static_cast<TOutputValue>(p);
            TOutputValue  polar = it->second;
            int       r     = polar[ 0 ];
            int       idx   = polar[ 1 ];
            const Circle& C = circle( r );
            const int     N = C.size();
            int       shift = int( round( -my_angle * N / ( 2.0 * M_PI ) ) );
            int       jdx   = ( N + idx - shift ) % N;
            return my_center + C[ jdx ];
        }

        TOutputValue operator()( const TInputValue & aInput ) const
        {
            return this->rotate(aInput);
        }

        /// @return the number of circles
        std::size_t size() const {
            return circles.size();
        }

        /// @param r an integer smaller than `nbCircles()`.
        /// @return the number of circles
        const Circle& circle( int r ) const {
            return circles[ r ];
        }

        /// @param r an integer specifying the digital radius of the circle.
        /// @return the points lying in the circle between radius r
        /// (included) and r+1 (excluded)
        static Circle computeCircle( int r ) {
            int d2_lo = r*r;
            int d2_up = (r+1)*(r+1);
            TOutputValue start( r, 0 );
            Circle circle;
            TOutputValue current = start;
            do {
                circle.push_back( current );
                auto V = nextNeighbors<TOutputValue>( current );
                std::vector<TOutputValue> P;
                for ( auto p : V ) {
                    int d2 = p.squaredNorm();
                    // std::cout << "p=" << p << " d2=" << d2 << std::endl;
                    if ( d2_lo <= d2 && d2 < d2_up ) P.push_back( p );
                }
                if ( P.empty() ) std::cerr << "Error ! " << std::endl;
                TOutputValue b = P[ 0 ];
                for ( int i = 1; i < P.size(); i++ )
                    if ( less<TOutputValue>( P[ i ], b ) ) b = P[ i ];
                current = b;
            } while ( current != start );
            return circle;
        }



    protected:
        /// The angle of rotation.
        double   my_angle;
        /// The center of rotation.
        TOutputValue my_center;
    };
}


#endif //RBC_vec

#undef RBC_vec_RECURSES
#endif // else defined(RBC_vec_RECURSES)
