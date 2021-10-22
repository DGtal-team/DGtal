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
 * @file CorrectedNormalCurrentFormula.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/01/01
 *
 * Header file for module CorrectedNormalCurrentFormula.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CorrectedNormalCurrentFormula_RECURSES)
#error Recursive header files inclusion detected in CorrectedNormalCurrentFormula.h
#else // defined(CorrectedNormalCurrentFormula_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CorrectedNormalCurrentFormula_RECURSES

#if !defined CorrectedNormalCurrentFormula_h
/** Prevents repeated inclusion of headers. */
#define CorrectedNormalCurrentFormula_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/SimpleMatrix.h"
#include "SphericalTriangle.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CorrectedNormalCurrentFormula
  /**
     Description of class 'CorrectedNormalCurrentFormula' <p> \brief
     Aim: A helper class that provides static methods to compute
     corrected normal current formulas of curvatures.

     @tparam TRealPoint any model of 3D RealPoint.
     @tparam TRealVector any model of 3D RealVector.

     Formula for interpolated measures:
     
     MU0=-1/6*((uAz + uBz + uCz)*Bx - (uAz + uBz + uCz)*Cx)*Ay + 1/6*((uAz + uBz + uCz)*Ax - (uAz + uBz + uCz)*Cx)*By - 1/6*((uAz + uBz + uCz)*Ax - (uAz + uBz + uCz)*Bx)*Cy + 1/6*((uAy + uBy + uCy)*Bx - (uAy + uBy + uCy)*Cx - (uAx + uBx + uCx)*By + (uAx + uBx + uCx)*Cy)*Az - 1/6*((uAy + uBy + uCy)*Ax - (uAy + uBy + uCy)*Cx - (uAx + uBx + uCx)*Ay + (uAx + uBx + uCx)*Cy)*Bz + 1/6*((uAy + uBy + uCy)*Ax - (uAy + uBy + uCy)*Bx - (uAx + uBx + uCx)*Ay + (uAx + uBx + uCx)*By)*Cz
     Let UM=uA+uB+uC.
     MU0=-1/6*(uMz*Bx - uMz*Cx)*Ay + 1/6*(uMz*Ax - uMz*Cx)*By - 1/6*(uMz*Ax - uMz*Bx)*Cy + 1/6*(uMy*Bx - uMy*Cx - uMx*By + uMx*Cy)*Az - 1/6*(uMy*Ax - uMy*Cx - uMx*Ay + uMx*Cy)*Bz + 1/6*(uMy*Ax - uMy*Bx - uMx*Ay + uMx*By)*Cz
     We see by simple computations that MU0 can be written as (uM = UM/3)
     MU0=1/2*det( uM, B-A, C-A )

     MU1=1/6*((uBy - uCy)*uAz - (uAy + 2*uCy)*uBz + (uAy + 2*uBy)*uCz)*Ax + 1/6*((uBy + 2*uCy)*uAz - (uAy - uCy)*uBz - (2*uAy + uBy)*uCz)*Bx - 1/6*((2*uBy + uCy)*uAz - (2*uAy + uCy)*uBz - (uAy - uBy)*uCz)*Cx - 1/6*((uBx - uCx)*uAz - (uAx + 2*uCx)*uBz + (uAx + 2*uBx)*uCz)*Ay - 1/6*((uBx + 2*uCx)*uAz - (uAx - uCx)*uBz - (2*uAx + uBx)*uCz)*By + 1/6*((2*uBx + uCx)*uAz - (2*uAx + uCx)*uBz - (uAx - uBx)*uCz)*Cy + 1/6*((uBx - uCx)*uAy - (uAx + 2*uCx)*uBy + (uAx + 2*uBx)*uCy)*Az + 1/6*((uBx + 2*uCx)*uAy - (uAx - uCx)*uBy - (2*uAx + uBx)*uCy)*Bz - 1/6*((2*uBx + uCx)*uAy - (2*uAx + uCx)*uBy - (uAx - uBx)*uCy)*Cz

     This formula can also be written in a clearer form
     6*MU1 = | u_A+u_B+u_C u_C-u_B A | + | u_A+u_B+u_C u_A-u_C B | + | u_A+u_B+u_C u_B-u_A C |
     It follows that 
     MU1=1/2( | uM u_C-u_B A | + | uM u_A-u_C B | + | uM u_B-u_A C |

     Gaussian curvature measure is
     MU2=-1/2*uCx*uBy*uAz + 1/2*uBx*uCy*uAz + 1/2*uCx*uAy*uBz - 1/2*uAx*uCy*uBz - 1/2*uBx*uAy*uCz + 1/2*uAx*uBy*uCz

     which is simply
     MU2=1/2*det( uA, uB, uC )

     Anisotropic curvature measure is written as
     MUXY = 1/2 < uM | < Y | uc-ua > X x (b-a) - < Y | ub-ua > X x (c-a) >
  */
  template < typename TRealPoint, typename TRealVector >
  struct CorrectedNormalCurrentFormula
  {
    typedef TRealPoint                     RealPoint;
    typedef TRealVector                    RealVector;
    typedef typename RealVector::Component Scalar;
    typedef std::vector< RealPoint >       RealPoints;
    typedef std::vector< RealVector >      RealVectors;
    typedef SimpleMatrix< Scalar, 3, 3 >   RealTensor;
    typedef std::size_t                    Index;
    static const Dimension dimension = RealPoint::dimension;
    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for mu0 measure
    /// @{
    
    /// Computes mu0 measure (area) of triangle abc given a constant
    /// corrected normal vector \a u.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param u the constant corrected normal vector to triangle abc
    /// @return the mu0-measure of triangle abc, i.e. its area.
    static
    Scalar mu0ConstantU
    ( const RealPoint& a, const RealPoint& b, const RealPoint& c,
      const RealVector& u )
    {
      return 0.5 * ( b - a ).crossProduct( c - a ).dotProduct( u );
    }

    /// Computes mu0 measure (area) of triangle abc given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param ua the corrected normal vector at point a
    /// @param ub the corrected normal vector at point b
    /// @param uc the corrected normal vector at point c
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu0-measure of triangle abc, i.e. its area.
    static
    Scalar mu0InterpolatedU
    ( const RealPoint& a, const RealPoint& b, const RealPoint& c,
      const RealVector& ua, const RealVector& ub, const RealVector& uc,
      bool unit_u = false )
    {
      // MU0=1/2*det( uM, B-A, C-A )
      //    =  1/2 < ( (u_A + u_B + u_C)/3.0 ) | (AB x AC ) >
      RealVector uM = ( ua+ub+uc ) / 3.0;
      if ( unit_u )
        {
          auto uM_norm = uM.norm();
          uM = uM_norm == 0.0 ? uM : uM / uM_norm;
        }
      return 0.5 * ( b - a ).crossProduct( c - a ).dot( uM );
    }
    
    /// Computes mu0 measure (area) of polygonal face \a pts given a
    /// constant corrected normal vector \a u.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the constant corrected normal vector to this polygonal face.
    /// @return the mu0-measure of the given polygonal face, i.e. its area.
    static
    Scalar mu0ConstantU( const RealPoints& pts, const RealVector& u )
    {
      if ( pts.size() <  3 ) return 0.0;
      if ( pts.size() == 3 )
	return mu0ConstantU( pts[ 0 ], pts[ 1 ], pts[ 2 ], u );
      const RealPoint b = barycenter( pts );
      Scalar          a = 0.0;
      for ( Index i = 0; i < pts.size(); i++ )
	a += mu0ConstantU( b, pts[ i ], pts[ (i+1)%pts.size() ], u );
      return a;
    }

    /// Computes area of polygonal face \a pts given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the (ccw ordered) normal vectors at the corresponding vertices in \a pts.
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu0-measure of the given polygonal face, i.e. its area.
    static
    Scalar mu0InterpolatedU( const RealPoints& pts, const RealVectors& u,
			     bool unit_u = false )
    {
      ASSERT( pts.size() == u.size() );
      if ( pts.size() <  3 ) return 0.0;
      if ( pts.size() == 3 )
	return mu0InterpolatedU( pts[ 0 ], pts[ 1 ], pts[ 2 ],
				 u[ 0 ], u[ 1 ], u[ 2 ], unit_u );
      const RealPoint   b = barycenter( pts );
      const RealVector ub = averageUnitVector( u );
      Scalar          a = 0.0;
      for ( Index i = 0; i < pts.size(); i++ )
	a += mu0InterpolatedU( b,  pts[ i ], pts[ (i+1)%pts.size() ],
			       ub,   u[ i ],   u[ (i+1)%pts.size() ], unit_u );
      return a;
    }

    /// @}
    
    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for mu1 measure
    /// @{

    /// Computes mu1 measure (mean curvature) of triangle abc given a constant
    /// corrected normal vector \a u.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param u the constant corrected normal vector to triangle abc
    /// @return the mu1-measure of triangle abc, i.e. its mean curvature, always 0.0.
    static
    Scalar mu1ConstantU
    ( const RealPoint& /* a */, const RealPoint& /* b */, const RealPoint& /* c */,
      const RealVector& /* u */ )
    {
      return 0.0;
    }

    /// Computes mu1 measure (mean curvature) of triangle abc given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param ua the corrected normal vector at point a
    /// @param ub the corrected normal vector at point b
    /// @param uc the corrected normal vector at point c
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu1-measure of triangle abc, i.e. its mean curvature.
    static
    Scalar mu1InterpolatedU
    ( const RealPoint& a, const RealPoint& b, const RealPoint& c,
      const RealVector& ua, const RealVector& ub, const RealVector& uc,
      bool unit_u = false )
    {
      // MU1=1/2( | uM u_C-u_B A | + | uM u_A-u_C B | + | uM u_B-u_A C |
      RealVector uM = ( ua+ub+uc ) / 3.0;
      if ( unit_u ) uM /= uM.norm();
      // JOL: if I put 0.5, we obtain twice the mean curvature.
      return 0.25 * ( uM.crossProduct( uc - ub ).dot( a )
		     + uM.crossProduct( ua - uc ).dot( b )
		     + uM.crossProduct( ub - ua ).dot( c ) );
    }
    
    /// Computes mu1 measure (mean curvature) of polygonal face \a pts given a
    /// constant corrected normal vector \a u.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the constant corrected normal vector to this polygonal face.
    /// @return the mu1-measure of the given polygonal face, i.e. its mean curvature, always 0.0.
    static
    Scalar mu1ConstantU( const RealPoints& pts, const RealVector& u )
    {
      return 0.0;
    }

    /// Computes mean curvature of polygonal face \a pts given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the (ccw ordered) normal vectors at the corresponding vertices in \a pts.
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu1-measure of the given polygonal face, i.e. its mean curvature.
    static
    Scalar mu1InterpolatedU( const RealPoints& pts, const RealVectors& u,
			     bool unit_u = false )
    {
      ASSERT( pts.size() == u.size() );
      if ( pts.size() <  3 ) return 0.0;
      if ( pts.size() == 3 )
	return mu1InterpolatedU( pts[ 0 ], pts[ 1 ], pts[ 2 ],
				 u[ 0 ], u[ 1 ], u[ 2 ], unit_u );
      const RealPoint   b = barycenter( pts );
      const RealVector ub = averageUnitVector( u );
      Scalar          a = 0.0;
      for ( Index i = 0; i < pts.size(); i++ )
	a += mu1InterpolatedU( b,  pts[ i ], pts[ (i+1)%pts.size() ],
			       ub,   u[ i ],   u[ (i+1)%pts.size() ], unit_u );
      return a;
    }

    /// @}

    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for mu2 measure
    /// @{

    /// Computes mu2 measure (Gaussian curvature) of triangle abc given a constant
    /// corrected normal vector \a u.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param u the constant corrected normal vector to triangle abc
    /// @return the mu2-measure of triangle abc, i.e. its Gaussian curvature, always 0.0.
    static
    Scalar mu2ConstantU
    ( const RealPoint& /* a */, const RealPoint& /* b */, const RealPoint& /* c */,
      const RealVector& /* u */ )
    {
      return 0.0;
    }

    /// Computes mu2 measure (Gaussian curvature) of triangle abc given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param ua the corrected normal vector at point a
    /// @param ub the corrected normal vector at point b
    /// @param uc the corrected normal vector at point c
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu2-measure of triangle abc, i.e. its Gaussian curvature.
    static
    Scalar mu2InterpolatedU
    ( const RealPoint& a, const RealPoint& b, const RealPoint& c,
      const RealVector& ua, const RealVector& ub, const RealVector& uc,
      bool unit_u = false )
    {
      // Using non unitary interpolated normals give
      // MU2=1/2*det( uA, uB, uC )
      // When normals are unitary, it is the area of a spherical triangle.
      if ( unit_u )
	{
	  typedef SpaceND< dimension > Space;
	  SphericalTriangle<Space> ST( ua, ub, uc ); // check order.
	  return ST.algebraicArea();
	}
      else
	return 0.5 * ( ua.crossProduct( ub ).dot( uc ) );
    }
    
    /// Computes mu2 measure (Gaussian curvature) of polygonal face \a pts given a
    /// constant corrected normal vector \a u.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the constant corrected normal vector to this polygonal face.
    /// @return the mu2-measure of the given polygonal face, i.e. its Gaussian curvature, always 0.0.
    static
    Scalar mu2ConstantU( const RealPoints& pts, const RealVector& u )
    {
      return 0.0;
    }
    
    /// Computes Gaussian curvature of polygonal face \a pts given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the (ccw ordered) normal vectors at the corresponding vertices in \a pts.
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the mu2-measure of the given polygonal face, i.e. its Gaussian curvature.
    static
    Scalar mu2InterpolatedU( const RealPoints& pts, const RealVectors& u,
			     bool unit_u = false )
    {
      ASSERT( pts.size() == u.size() );
      if ( pts.size() <  3 ) return 0.0;
      if ( pts.size() == 3 )
	return mu2InterpolatedU( pts[ 0 ], pts[ 1 ], pts[ 2 ],
				 u[ 0 ], u[ 1 ], u[ 2 ], unit_u );
      const RealPoint   b = barycenter( pts );
      const RealVector ub = averageUnitVector( u );
      Scalar          a = 0.0;
      for ( Index i = 0; i < pts.size(); i++ )
	a += mu2InterpolatedU( b,  pts[ i ], pts[ (i+1)%pts.size() ],
			       ub,   u[ i ],   u[ (i+1)%pts.size() ], unit_u );
      return a;
    }
    
    /// @}
    
    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for muXY measure
    /// @{
    
    /// Computes muXY measure (anisotropic curvature) of triangle abc given a constant
    /// corrected normal vector \a u.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param u the constant corrected normal vector to triangle abc
    /// @return the muXY-measure of triangle abc, i.e. its anisotropic curvature, always 0.0.
    static
    RealTensor muXYConstantU
    ( const RealPoint& /* a */, const RealPoint& /* b */, const RealPoint& /* c */,
      const RealVector& /* u */ )
    {
      return RealTensor { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    }
    
    /// Computes muXY measure (anisotropic curvature) of triangle abc given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param a any point
    /// @param b any point
    /// @param c any point
    /// @param ua the corrected normal vector at point a
    /// @param ub the corrected normal vector at point b
    /// @param uc the corrected normal vector at point c
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the muXY-measure of triangle abc, i.e. its anisotropic curvature.
    static
    RealTensor muXYInterpolatedU
    ( const RealPoint& a, const RealPoint& b, const RealPoint& c,
      const RealVector& ua, const RealVector& ub, const RealVector& uc,
      bool unit_u = false )
    {
      RealTensor T;
      //  MUXY = 1/2 < uM | < Y | uc-ua > X x (b-a) - < Y | ub-ua > X x (c-a) >
      //  MUXY = 1/2 ( < Y | ub-ua > | X uM (c-a) | - < Y | uc-ua > | X uM (b-a) | )
      RealVector uM = ( ua+ub+uc ) / 3.0;
      if ( unit_u ) uM /= uM.norm();
      const RealVector uac = uc - ua;
      const RealVector uab = ub - ua;
      const RealVector  ab = b - a;
      const RealVector  ac = c - a;
      for ( Dimension i = 0; i < 3; ++i ) {
	RealVector X = RealVector::base( i, 1.0 );
	for ( Dimension j = 0; j < 3; ++j ) {
	  // Since RealVector Y = RealVector::base( j, 1.0 );
	  // < Y | uac > = uac[ j ]
	  const Scalar tij =
	    0.5 * uM.dot( uac[ j ] * X.crossProduct( ab )
                          - uab[ j ] * X.crossProduct( ac ) );
	  T.setComponent( i, j, tij );
	}
      }
      return T;
    }
    
    /// Computes muXY measure (anisotropic curvature) of polygonal face \a pts given a
    /// constant corrected normal vector \a u.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the constant corrected normal vector to this polygonal face.
    /// @return the muXY-measure of the given polygonal face, i.e. its anisotropic curvature, always 0.0.
    static
    RealTensor muXYConstantU( const RealPoints& pts, const RealVector& u )
    {
      return RealTensor { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    }

    /// Computes anisotropic curvature of polygonal face \a pts given an interpolated
    /// corrected normal vector \a ua, \a \ub, \a uc.
    /// @param pts the (ccw ordered) points forming the vertices of a polygonal face.
    /// @param u the (ccw ordered) normal vectors at the corresponding vertices in \a pts.
    /// @param unit_u when 'true' considers that interpolated
    /// corrected normals should be made unitary, otherwise
    /// interpolated corrected normals may have smaller norms.
    /// @return the muXY-measure of the given polygonal face, i.e. its anisotropic curvature.
    static
    RealTensor muXYInterpolatedU( const RealPoints& pts, const RealVectors& u,
				  bool unit_u = false )
    {
      ASSERT( pts.size() == u.size() );
      if ( pts.size() <  3 ) return RealTensor { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
      if ( pts.size() == 3 )
	return muXYInterpolatedU( pts[ 0 ], pts[ 1 ], pts[ 2 ],
				 u[ 0 ], u[ 1 ], u[ 2 ], unit_u );
      const RealPoint   b = barycenter( pts );
      const RealVector ub = averageUnitVector( u );
      RealTensor        T = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
      for ( Index i = 0; i < pts.size(); i++ )
	T += muXYInterpolatedU( b,  pts[ i ], pts[ (i+1)%pts.size() ],
				ub,   u[ i ],   u[ (i+1)%pts.size() ], unit_u );
      return T;
    }
    
    /// @}

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

    /// Given a vector of unit vectors, returns their average unit vector.
    /// @param pts any vector of vectors.
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
//#include "CorrectedNormalCurrentFormula.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CorrectedNormalCurrentFormula_h

#undef CorrectedNormalCurrentFormula_RECURSES
#endif // else defined(CorrectedNormalCurrentFormula_RECURSES)
  
