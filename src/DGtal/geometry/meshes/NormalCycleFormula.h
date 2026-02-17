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
 * @file NormalCycleFormula.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/01/01
 *
 * Header file for module NormalCycleFormula.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NormalCycleFormula_RECURSES)
#error Recursive header files inclusion detected in NormalCycleFormula.h
#else // defined(NormalCycleFormula_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NormalCycleFormula_RECURSES

#if !defined NormalCycleFormula_h
/** Prevents repeated inclusion of headers. */
#define NormalCycleFormula_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/SimpleMatrix.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class NormalCycleFormula
  /**
     Description of class 'NormalCycleFormula' <p> \brief
     Aim: A helper class that provides static methods to compute
     normal cycle formulas of curvatures.

     @tparam TRealPoint any model of 3D RealPoint.
     @tparam TRealVector any model of 3D RealVector.
  */
  template < typename TRealPoint, typename TRealVector >
  struct NormalCycleFormula
  {
    typedef TRealPoint                     RealPoint;
    typedef TRealVector                    RealVector;
    typedef typename RealVector::Component Scalar;
    typedef std::vector< RealPoint >       RealPoints;
    typedef std::vector< RealVector >      RealVectors;
    typedef SimpleMatrix< Scalar, 3, 3 >   RealTensor;
    typedef std::size_t                    Size;
    typedef std::size_t                    Index;
    static const Dimension dimension = RealPoint::dimension;

    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for curvature
    /// @{

    /// Computes area of polygonal face \a pts.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @return the area of the given polygonal face.
    static
    Scalar area( const RealPoints& pts )
    {
      if ( pts.size() <  3 ) return 0.0;
      if ( pts.size() == 3 )
	return area( pts[ 0 ], pts[ 1 ], pts[ 2 ] );
      const RealPoint b = barycenter( pts );
      Scalar          a = 0.0;
      for ( Index i = 0; i < pts.size(); i++ )
	a += area( b, pts[ i ], pts[ (i+1)%pts.size() ] );
      return a;
    }

    /// Computes twice the mean curvature on edge ab
    /// given normal vectors \a right, \a left.
    ///
    /// @param a any point
    /// @param b any point
    /// @param right the normal vector at face xba where x is some vertex(ices)
    /// @param left the normal vector at face yab where y is some vertex(ices)
    /// @return twice the mean curvature according to Normal Cycle formula.
    static
    Scalar twiceMeanCurvature
    ( const RealPoint& a, const RealPoint& b,
      const RealVector& right, const RealVector& left )
    {
      const RealVector diedre = right.crossProduct( left );
      const Scalar n = std::min( 1.0, std::max( diedre.norm(), 0.0 ) );
      const Scalar angle = ( diedre.dot( b - a) < 0.0 )
	? asin( n ) : - asin( n );
      return ( b - a ).norm() * angle;
    }

    /// Computes the Gaussian curvature at point \a a with incident
    /// vertices \a vtcs.
    ///
    /// @param a any point
    /// @param vtcs a range of points
    /// @return the Gaussian curvature according to Normal Cycle formula.
    static
    Scalar gaussianCurvature
    ( const RealPoint& a, const RealPoints& vtcs )
    {
      Scalar angle_sum = 0.0;
      for ( Size i = 0; i < vtcs.size(); i++ )
	angle_sum += acos( (vtcs[i] - a).getNormalized()
			   .dot( ( vtcs[(i+1)%vtcs.size()] - a ).getNormalized() ) );
      return 2.0 * M_PI - angle_sum;
    }

    /// Computes the Gaussian curvature at point \a a with incident
    /// pairs of points \a pairs.
    ///
    /// @param a any point
    /// @param pairs a range of points [x_0, y_0, x_1, y_1, etc] such
    /// that (a,x_i,y_i) is an incident face to a.
    /// @return the Gaussian curvature according to Normal Cycle formula.
    static
    Scalar gaussianCurvatureWithPairs
    ( const RealPoint& a, const RealPoints& pairs )
    {
      Scalar angle_sum = 0.0;
      for ( Size i = 0; i < pairs.size(); i += 2 )
	angle_sum += acos( ( pairs[i] - a ).getNormalized()
			   .dot( ( pairs[i+1] - a ).getNormalized() ) );
      return 2.0 * M_PI - angle_sum;
    }

    /// Computes the anisotropic measure \f$ \bar{H} \f$ at edge ab.
    /// @param a any point
    /// @param b any point
    /// @param right the normal vector at face xba where x is some vertex(ices)
    /// @param left the normal vector at face yab where y is some vertex(ices)
    /// @return the anisotropic curvature measure \f$ \bar{H} \f$ at edge ab.
    static
    RealTensor anisotropicCurvatureH1
    ( const RealPoint& a, const RealPoint& b,
      const RealVector& right, const RealVector& left )
    {
      const RealVector diedre = right.crossProduct( left );
      const Scalar     length = std::max( 0.0, std::min( 1.0, diedre.norm() ) );
      const Scalar      angle = ( diedre.dot( b - a) > 0.0 )
	? asin( length ) : - asin( length );
      RealVector          e_p = right + left;
      RealVector          e_m = right - left;
      const Scalar norm_e_p   = e_p.norm();
      const Scalar norm_e_m   = e_m.norm();
      e_p = norm_e_p > 1e-10 ? e_p / norm_e_p : RealVector::zero;
      e_m = norm_e_m > 1e-10 ? e_m / norm_e_m : RealVector::zero;
      const RealTensor T_p  =
	{ e_p[ 0 ] * e_p[ 0 ], e_p[ 0 ] * e_p[ 1 ], e_p[ 0 ] * e_p[ 2 ],
	  e_p[ 1 ] * e_p[ 0 ], e_p[ 1 ] * e_p[ 1 ], e_p[ 1 ] * e_p[ 2 ],
	  e_p[ 2 ] * e_p[ 0 ], e_p[ 2 ] * e_p[ 1 ], e_p[ 2 ] * e_p[ 2 ] };
      const RealTensor T_m  =
	{ e_m[ 0 ] * e_m[ 0 ], e_m[ 0 ] * e_m[ 1 ], e_m[ 0 ] * e_m[ 2 ],
	  e_m[ 1 ] * e_m[ 0 ], e_m[ 1 ] * e_m[ 1 ], e_m[ 1 ] * e_m[ 2 ],
	  e_m[ 2 ] * e_m[ 0 ], e_m[ 2 ] * e_m[ 1 ], e_m[ 2 ] * e_m[ 2 ] };
      return 0.5 * ( b - a ).norm()
	* ( ( angle - sin( angle ) ) * T_p + ( angle + sin( angle ) ) * T_m );
    }

    /// Computes the anisotropic measure \f$ \bar{\tilde{H}} \f$ at edge ab.
    /// @param a any point
    /// @param b any point
    /// @param right the normal vector at face xba where x is some vertex(ices)
    /// @param left the normal vector at face yab where y is some vertex(ices)
    /// @return the anisotropic curvature measure \f$ \bar{\tilde{H}} \f$ at edge ab.
    static
    RealTensor anisotropicCurvatureH2
    ( const RealPoint& a, const RealPoint& b,
      const RealVector& right, const RealVector& left )
    {
      const RealVector diedre = right.crossProduct( left );
      const Scalar     length = std::max( 0.0, std::min( 1.0, diedre.norm() ) );
      const Scalar      angle = ( diedre.dot( b - a) > 0.0 )
	? asin( length ) : - asin( length );
      const Scalar norm_ab = (b - a).norm();
      const RealVector   e = norm_ab > 1e-10 ? (b - a) / norm_ab : RealVector::zero;
      const RealTensor   T =
	{ e[ 0 ] * e[ 0 ], e[ 0 ] * e[ 1 ], e[ 0 ] * e[ 2 ],
	  e[ 1 ] * e[ 0 ], e[ 1 ] * e[ 1 ], e[ 1 ] * e[ 2 ],
	  e[ 2 ] * e[ 0 ], e[ 2 ] * e[ 1 ], e[ 2 ] * e[ 2 ] };
      return ( 0.5 * norm_ab * angle ) * T; // JOL * 0.5
    }


    //-------------------------------------------------------------------------
  public:
    /// @name Other geometric services
    /// @{

    /// Given a vector of points, returns its barycenter.
    /// @param pts any vector of points
    /// @return the barycenter of these points.
    static
    RealPoint barycenter( const RealPoints& pts )
    {
      RealPoint b;
      for ( auto p : pts ) b += p;
      b /= pts.size();
      return b;
    }

    /// Computes a unit normal vector to triangle abc
    ///
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @return the unit normal vector to abc, ( ab x ac ) / || ab x ac ||.
    static
    RealVector normal( const RealPoint& a, const RealPoint& b, const RealPoint& c )
    {
      return ( ( b - a ).crossProduct( c - a ) ).getNormalized();
    }

    /// Computes triangle area
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @return the area of triangle abc
    static
    Scalar area( const RealPoint& a, const RealPoint& b, const RealPoint& c )
    {
      return 0.5 * ( ( b - a ).crossProduct( c - a ) ).norm();
    }

    /// Given a vector of unit vectors, returns their average unit vector.
    /// @param vecs any vector of vectors.
    /// @return the average unit vector.
    static
    RealVector averageUnitVector( const RealVectors& vecs )
    {
      RealVector avg;
      for ( auto v : vecs ) avg += v;
      auto avg_norm = avg.norm();
      return avg_norm != 0.0 ? avg / avg_norm : avg;
    }


    /// @}

  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "NormalCycleFormula.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NormalCycleFormula_h

#undef NormalCycleFormula_RECURSES
#endif // else defined(NormalCycleFormula_RECURSES)
