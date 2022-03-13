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
 * @file SphericalTriangle.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2017/06/19
 *
 * Header file for module SphericalTriangle.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SphericalTriangle_RECURSES)
#error Recursive header files inclusion detected in SphericalTriangle.h
#else // defined(SphericalTriangle_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SphericalTriangle_RECURSES

#if !defined SphericalTriangle_h
/** Prevents repeated inclusion of headers. */
#define SphericalTriangle_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <map>
#include "DGtal/base/Common.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class SphericalTriangle
  /**
     Description of class 'SphericalTriangle' <p> \brief Aim:
     Represent a triangle drawn onto a sphere of radius 1.
     
     @tparam TSpace any type of 3-dimensional digital space.
  */
  template <typename TSpace>
  class SphericalTriangle
  {
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));
    typedef TSpace                         Space;
    typedef SphericalTriangle<Space>       Self;
    typedef typename Space::RealPoint      RealPoint;
    typedef typename Space::RealVector     RealVector;
    typedef typename RealVector::Component Scalar;
    
    // Checks that dimension is 3.
    BOOST_STATIC_ASSERT(( Space::dimension == 3 ));
    
    // ----------------------- Standard services ------------------------------
  public:
    
    /**
     * Destructor.
     */
    ~SphericalTriangle() {}
    
    /// Default constructor. The object is invalid.
    SphericalTriangle( const RealVector& va, const RealVector& vb, const RealVector& vc,
		       bool normalize = true )
    {
      setA( va, normalize );
      setB( vb, normalize );
      setC( vc, normalize );
    }
    
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    SphericalTriangle ( const SphericalTriangle & other ) = default;
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    SphericalTriangle & operator= ( const SphericalTriangle & other ) = default;

    /// @return the point A of the spherical triangle.
    const RealVector& A() const { return myA; }
    /// @return the point B of the spherical triangle.
    const RealVector& B() const { return myB; }
    /// @return the point C of the spherical triangle.
    const RealVector& C() const { return myC; }

    /// Sets the point A of the triangle.
    /// @param va the new point A
    ///
    /// @param normalize if true, force normalization, otherwise va
    /// should be of unit length.
    void setA( const RealVector& va, bool normalize = true )
    {
      myA = va;
      if ( normalize )
	{
	  Scalar n = myA.norm();
	  if ( fabs( n ) > 1e-8 ) myA /= n;
	  else myA = RealVector::zero;
	}
    }

    /// Sets the point B of the triangle.
    /// @param vb the new point B
    ///
    /// @param normalize if true, force normalization, otherwise vb
    /// should be of unit length.
    void setB( const RealVector& vb, bool normalize = true )
    {
      myB = vb;
      if ( normalize )
	{
	  Scalar n = myB.norm();
	  if ( fabs( n ) > 1e-8 ) myB /= n;
	  else myB = RealVector::zero;
	}
    }
    /// Sets the point C of the triangle.
    /// @param vc the new point C
    ///
    /// @param normalize if true, force normalization, otherwise vc
    /// should be of unit length.
    void setC( const RealVector& vc, bool normalize = true )
    {
      myC = vc;
      if ( normalize )
	{
	  Scalar n = myC.norm();
	  if ( fabs( n ) > 1e-8 ) myC /= n;
	  else myC = RealVector::zero;
	}
    }

    /// @return 'true' if the spherical triangle is too small or too thin.
    bool isDegenerate() const
    {
      Scalar d[ 3 ] = { ( myA - myB ).norm(),
			( myA - myC ).norm(),
			( myB - myC ).norm() };
      // Checks that the spherical triangle is small or thin.
      if ( ( d[ 0 ] < 1e-8 ) || ( d[ 1 ] < 1e-8 ) || ( d[ 2 ] < 1e-8 ) )
	return true;
      // Checks that the spherical triangle is flat.
      Dimension m = 0;
      if ( d[ 1 ] > d[ m ] ) m = 1;
      if ( d[ 2 ] > d[ m ] ) m = 2;
      return ( fabs( d[ m ] - d[ (m+1)%3 ] - d[ (m+2)%3 ] ) < 1e-8 );
    }
    
    /// @return the polar triangle associated with this triangle.
    Self polarTriangle() const
    {
      auto Ap = myB.crossProduct(myC);
      auto Bp = myC.crossProduct(myA);
      auto Cp = myA.crossProduct(myB);
      // Reorient points.
      if ( Ap.dot( myA ) < 0.0 ) Ap = -Ap;
      if ( Bp.dot( myB ) < 0.0 ) Bp = -Bp;
      if ( Cp.dot( myC ) < 0.0 ) Cp = -Cp;
      return Self( Ap, Bp, Cp, true );
    }

    /// Returns the interior angles of the spherical triangle ABC.
    /// @param[out] alpha the interior angle at vertex A.
    /// @param[out] beta  the interior angle at vertex B.
    /// @param[out] gamma the interior angle at vertex C.
    void interiorAngles( Scalar& alpha, Scalar& beta, Scalar& gamma ) const
    {
      Self    T = polarTriangle();
      if ( T.A() == RealVector::zero || T.B() == RealVector::zero || T.C() == RealVector::zero )
	alpha = beta = gamma = 0.0;
      else
	{
	  Scalar ca = std::max( -1.0, std::min( 1.0, T.B().dot( T.C() ) ) );
	  Scalar cb = std::max( -1.0, std::min( 1.0, T.C().dot( T.A() ) ) );
	  Scalar cc = std::max( -1.0, std::min( 1.0, T.A().dot( T.B() ) ) );
	  alpha     = acos( ca );
	  beta      = acos( cb );
	  gamma     = acos( cc );
	}
    }

    /// @return the (unsigned) area of the spherical triangle (below 2pi).
    Scalar area() const
    {
      Scalar alpha, beta, gamma;
      if ( isDegenerate() ) return 0.0;
      interiorAngles( alpha, beta, gamma );
      return ( (alpha == 0.0) || (beta == 0.0) || (gamma == 0.0) )
	       ? 0.0 : 2.0*M_PI - alpha - beta - gamma;
    }

    /// @return the (signed) area of the spherical triangle (below 2pi).
    Scalar algebraicArea() const
    {
      Scalar     S = area();
      RealVector M = myA + myB + myC;
      RealVector X = ( myB - myA ).crossProduct( myC - myA );
      if ( M.norm1() <= 1e-8 || X.norm1() <= 1e-8 ) return 0.0;
      return M.dot( X ) < 0.0 ? -S : S;
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    // ------------------------- Hidden services ------------------------------
  protected:
    /// The point A of the triangle ABC, of unit length
    RealVector myA;
    /// The point B of the triangle ABC, of unit length
    RealVector myB;
    /// The point C of the triangle ABC, of unit length
    RealVector myC;
    
    // ------------------------- Internals ------------------------------------
  private:
    
  }; // end of class SphericalTriangle
  

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SphericalTriangle_h

#undef SphericalTriangle_RECURSES
#endif // else defined(SphericalTriangle_RECURSES)
