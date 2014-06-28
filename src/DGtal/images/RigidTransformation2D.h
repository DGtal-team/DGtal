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
 * @file RigidTransformation2D.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, France
 *
 * @date 2014/06/26
 *
 * This file is part of the DGtal library.
 */

#if defined(RigidTransformation2D_RECURSES)
#error Recursive header files inclusion detected in RigidTransformation2D.h
#else // defined(RigidTransformation2D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RigidTransformation2D_RECURSES

#if !defined RigidTransformation2D_h
/** Prevents repeated inclusion of headers. */
#define RigidTransformation2D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include <climits>
#include <utility>
#include <functional>
#include "DGtal/base/Common.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/kernel/domains/CDomain.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace functors
  {
    template <typename TPoint, typename TRealVector>
    class ForwardRigidTransformation2D : std::unary_function <TPoint,TPoint>
    {
      BOOST_STATIC_ASSERT(( TPoint::dimension == 2 ));
      BOOST_STATIC_ASSERT(( TRealVector::dimension == 2 ));
    public:
      /**
       * Constructor.
       * @param aOrigin  the center of rotation.
       */
      ForwardRigidTransformation2D ( const TPoint & aOrigin, const double & angle, const TRealVector & aTranslate )
      :origin(aOrigin), translation(aTranslate) 
      {
	t_sin = std::sin ( angle );
	t_cos = std::cos ( angle );
      }
      
      /**
       * Operator
       *
       * @return the transformed point.
       */
      inline
      TPoint operator()( const TPoint& aInput ) const
      {
	TPoint p;
	p[0] = std::floor ( ( ( t_cos * ( aInput[0] - origin[0] ) -
	t_sin * ( aInput[1] - origin[1] ) ) + translation[0] ) + 0.5 );
	
	p[1] = std::floor ( ( ( t_sin * ( aInput[0] - origin[0] ) +
	t_cos * ( aInput[1] - origin[1] ) ) + translation[1] ) + 0.5 );
	return p + origin;
      }
      
    private:
      /**
       * value
       */
      TPoint origin;
      double t_sin;
      double t_cos;
      TRealVector translation;
    };
    
    template <typename TPoint, typename TRealVector>
    class BackwardRigidTransformation2D : std::unary_function <TPoint,TPoint>
    {
      BOOST_STATIC_ASSERT(( TPoint::dimension == 2 ));
      BOOST_STATIC_ASSERT(( TRealVector::dimension == 2 ));
    public:
      /**
       * Constructor.
       * @param aOrigin the center of rotation.
       */
      BackwardRigidTransformation2D ( const TPoint& aOrigin, const double & angle, const TRealVector & aTranslate )
      :origin(aOrigin), translation(aTranslate) 
      {
	t_sin = std::sin ( angle );
	t_cos = std::cos ( angle );
      }
      
      /**
       * Operator
       *
       * @return the transformed point.
       */
      inline
      TPoint operator()( const TPoint& aInput ) const
      {
	TPoint p;
	p[0] = std::floor ( ( t_cos * (aInput[0] - translation[0] - origin[0] ) +
	t_sin * ( aInput[1] - translation[1] - origin[1] ) ) + 0.5 );
	
	p[1] = std::floor ( ( -t_sin * ( aInput[0] - translation[0] - origin[0] ) 
	+ t_cos * ( aInput[1] - translation[1] - origin[1] ) ) + 0.5 );
	return p + origin;
      }
      
    private:
      /**
       * value
       */
      TPoint origin;
      double t_sin;
      double t_cos;
      TRealVector translation;
    };
    
    template <typename TDomain, typename TRigidTransformFunctor >
    class DomainRigidTransformation2D : 
    std::unary_function < std::pair < typename TDomain::Point, typename TDomain::Point >, TDomain>
    {
      BOOST_STATIC_ASSERT(( TDomain::dimension == 2 ));
      BOOST_CONCEPT_ASSERT(( CDomain<TDomain> ));
      
    // ----------------------- Types ------------------------------
    public:
     typedef std::pair < typename TDomain::Space::Point, typename TDomain::Space::Point > Bounds;
     
    public:
      /**
       * Constructor.
       * @param aRigidFunctor  - functor to rigid transformation.
       */
      DomainRigidTransformation2D ( TRigidTransformFunctor & aRigidFunctor ) : transform ( aRigidFunctor ) {}
      
      /**
       * Operator
       *
       * @return the transformed domain.
       */
      inline
      Bounds operator()( const TDomain & aInput ) const
      {
	typedef typename TDomain::Point Point;
	Point points[4];
	points[0] = transform ( aInput.lowerBound() );
	points[1] = transform ( aInput.upperBound() );
	points[2] = transform ( Point ( aInput.upperBound()[0], aInput.lowerBound()[1] ) );
	points[3] = transform ( Point ( aInput.lowerBound()[0], aInput.upperBound()[1] ) );
	
	Point t_min ( INT_MAX, INT_MAX ), t_max ( INT_MIN, INT_MIN );
	for ( unsigned int i = 0; i < 4 ; i++ )
	{
	  if ( points[i][0] < t_min[0] )
	    t_min[0] = points[i][0]; 
	  if ( points[i][1] < t_min[1] )
	    t_min[1] = points[i][1];
	  
	  if ( points[i][0] > t_max[0] )
	    t_max[0] = points[i][0]; 
	  if ( points[i][1] > t_max[1] )
	    t_max[1] = points[i][1]; 
	}
	
	Bounds bounds;
	bounds.first = t_min;
	bounds.second = t_max;
	return bounds;
      }
      
    private:
      /**
       * value
       */
      TRigidTransformFunctor & transform;
    }; 
    
  }// namespace DGtal::functors
}// namespace DGtal

#endif // !defined RigidTransformation2D_h

#undef RigidTransformation2D_RECURSES
#endif // else defined(RigidTransformation2D_RECURSES)

