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
 * @file SeparableMetricHelper.h
 * @brief Basic functions associated to metrics used by separable volumetric algorithms.
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/08/01
 *
 * Header file for module SeparableMetricHelper.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testDistanceTransformationND.cpp
 */

#if defined(SeparableMetricHelper_RECURSES)
#error Recursive header files inclusion detected in SeparableMetricHelper.h
#else // defined(SeparableMetricHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SeparableMetricHelper_RECURSES

#if !defined SeparableMetricHelper_h
/** Prevents repeated inclusion of headers. */
#define SeparableMetricHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CBoundedInteger.h"
#include "DGtal/base/BasicFunctors.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SeparableMetricHelper
  /**
   * Description of template class 'SeparableMetricHelper' <p>
   * \brief Aim: Implements basic functions associated to metrics used
   * by separable volumetric algorithms.
   *
   * @tparam TAbscissa Type used to store the coordinaites of the Domain (model of CBoundedInteger).
   * @tparam TInternalValue the type used to store the internal
   * numbers for exact computations. More precisely,
   * TInternalValueType must be able to represent numbers of type
   * TAbscissa to the power tp (model of CBoundedInteger).
   * @tparam tp the order p of the L_p metric.
   *
   * @warning this  code is node GMP compliant
   *
   */
  template <typename TPoint, typename TInternalValue, DGtal::uint32_t tp>
  struct SeparableMetricHelper
  {
    // ----------------------- Standard services ------------------------------

    typedef TInternalValue InternalValue;
    typedef typename TPoint::Coordinate Abscissa;
    typedef TPoint Point;
    
    
    BOOST_CONCEPT_ASSERT(( CBoundedInteger<Abscissa> ));
    BOOST_CONCEPT_ASSERT(( CBoundedInteger<TInternalValue> ));

    /**
     * Static constants containing the power p of the Lp-metric.
     *
     */
    static const DGtal::uint32_t p = tp;


    /**
     * Returns an approximation (double) of the InternalValues
     * associated to the metric. 
     *
     * @param aInternalValue the internal value to convert
     *
     * @return the converted value.
     */
    double getApproxValue( const InternalValue & aInternalValue ) const
    {
      return std::pow( NumberTraits<InternalValue>::castToDouble(aInternalValue),
		       (double) 1.0 / p);
    }

    /**
     * Returns the height at a point  pos of a Lp-parabola with
     * center  ci and height hi.
     *
     * @param pos an abscissa.
     * @param ci center of the Lp-parabola.
     * @param hi height of the Lp-parabola.
     *
     * @return the height of the parabola (ci,hi) at pos.
     */
    InternalValue F ( const Abscissa pos, const Abscissa ci, const InternalValue hi ) const
    {
      return std::pow( abs(NumberTraits<Abscissa>::castToDouble(pos - ci)),
		       (double)p) + hi;
    }

    /**
     * Returns the height at a point  pos of a reversed Lp-parabola with
     * center  ci and height hi.
     *
     * @param pos an abscissa.
     * @param ci center of the Lp-parabola.
     * @param hi height of the Lp-parabola.
     *
     * @return the height of the reversed parabola (ci,hi) at pos.
     */
    InternalValue reversedF ( const Abscissa pos, const Abscissa ci, const InternalValue hi ) const
    {
      return hi - std::pow( abs((double )pos - ci) , (double)p);
    }


    /**
     * Returns the InternalValue value of order p for a given
     * position. Basically, its computes @paramp pos^p.
     *
     * @param pos the value of type Abscissa
     *
     * @return the InternaValue value.
     */
    InternalValue power ( const Abscissa pos ) const
    {
      return ( InternalValue ) std::pow ( (double)pos, (double)p );
    }


    /**
     * Returns the abscissa of the intersection point between two reversed
     * Lp-parabolas (ci,hi) and (cj,hj).
     *
     * @param ci center of the first Lp-parabola.
     * @param hi height of the first Lp-parabola power p (hi = real height^p)
     * @param cj center of the first Lp-parabola.
     * @param hj height of the first Lp-parabola power p (hj = real height^p).
     *
     * @return
     */
    Abscissa reversedSep ( const Abscissa i, const InternalValue hi, 
			   const Abscissa j, const InternalValue hj ) const
    {
      ASSERT(false && "Not-Yet-Implemented");
    }
    
    /**
     * Returns the abscissa of the intersection point between two
     * Lp-parabolas (ci,hi) and (cj,hj).
     *
     * @param ci center of the first Lp-parabola.
     * @param hi height of the first Lp-parabola power p (hi = real height^p)
     * @param cj center of the first Lp-parabola.
     * @param hj height of the first Lp-parabola power p (hj = real height^p).
     *
     * @return
     */
    Abscissa Sep ( const Abscissa i, const InternalValue hi, 
		   const Abscissa j, const InternalValue hj ) const
    {
      ASSERT(false && "Not-Yet-Implemented");
    }

    
    enum Closest { FIRST=0, SECOND=1, BOTH=2};
    
    
    /** 
     * Given an origin and two points, this method decides which one
     * is closest to the origin. This method should be faster than
     * comparing distance values.
     * 
     * @param origin the origin
     * @param first  the first point
     * @param second the second point
     * 
     * @return a Closest enum: FIRST, SECOND or BOTH.
     */
    Closest closest(const Point &origin, 
		    const Point &first,
		    const Point &second) const
    {
      InternalValue a=NumberTraits<InternalValue>::ZERO,
	b=NumberTraits<InternalValue>::ZERO;
      
      for(typename Point::Dimension i=0; i <  Point::dimension; i++)
	{
	  a += power(abs (origin[i] - first[i]));
	  b += power(abs (origin[i] - second[i]));
	}
      if (a<b)
	return FIRST;
      else
	if (a>b)
	  return SECOND;
	else
	  return BOTH;
    }


    /** 
     * Perform a binary search on the interval [lower,upper] to
     * detect the mid-point between u and v according to the l_p
     * distance.
     * 
     * @param udim coordinate of u along dimension dim
     * @param vdim coordinate of v along dimension dim
     * @param nu  partial distance of u (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param nv partial distance of v (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param lower interval lower bound 
     * @param upper interval upper bound
     * 
     * @return the Voronoi boundary point coordinates along dimension dim.
     */
    Abscissa binarySearchHidden(const Abscissa &udim, 
                                const Abscissa &vdim,
                                const InternalValue &nu,
                                const InternalValue &nv,
                                const Abscissa &lower,
                                const Abscissa &upper) const
    {   
      ASSERT(  (nu + (InternalValue) std::pow( (double)abs( udim - lower), (double) p)) <=
               (nv + (InternalValue) std::pow( (double)abs( vdim - lower), (double)p)));
      
      //Recurrence stop 
      if ( (upper - lower) <= NumberTraits<Abscissa>::ONE)
        return lower;
      
      Abscissa mid = (lower + upper)/2;
      InternalValue nuUpdated = nu + (InternalValue) std::pow( (double)abs( udim - mid ), (double)p);
      InternalValue nvUpdated = nv + (InternalValue) std::pow( (double)abs( vdim - mid ), (double)p);
      
      //Recursive call
      if ( nuUpdated < nvUpdated)
        return binarySearchHidden(udim,vdim,nu,nv,mid,upper);
      else
        return binarySearchHidden(udim,vdim,nu,nv,lower,mid);
      
    }

    /** 
     * Given three sites (a,b,c) and a straight segment
     * [startingPoint,endPoint] along dimension dim, we detect if the
     * voronoi cells of a and c @e hide the voronoi cell of c on the
     * straight line.
     *
     * @pre both voronoi cells associated with @a a and @a b must
     * intersect the straight line. 
     * 
     * @param u a site
     * @param v a site
     * @param w a site
     * @param startingPoint starting point of the segment
     * @param endPoint end point of the segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */
    bool hiddenBy(const Point &u, 
                  const Point &v,
                  const Point &w, 
                  const Point &startingPoint,
                  const Point &endPoint,
                  const typename Point::UnsignedComponent dim) const
    {
      
      //Interval bound for the binary search
      Abscissa lower = startingPoint[dim];
      Abscissa upper = endPoint[dim];
      
      //Partial norm computation
      // (sum_{i!=dim}  |u_i-v_i|^p
      InternalValue nu = NumberTraits<InternalValue>::ZERO;
      InternalValue nv = NumberTraits<InternalValue>::ZERO;
      InternalValue nw = NumberTraits<InternalValue>::ZERO;
      for(Dimension i  = 0 ; i < Point::dimension ; i++)
	if (i != dim)
	  {
	    nu += ( InternalValue ) std::pow ( (double)abs(u[i] - startingPoint[i] ) , (double)p);
	    nv += ( InternalValue ) std::pow ( (double)abs(v[i] - startingPoint[i] ) , (double)p);
	    nw += ( InternalValue ) std::pow ( (double)abs(w[i] - startingPoint[i] ) , (double)p);
	  }
 
      //Intersection of voronoi boundary
     

      //Optimization if vw lies before starting
      if ((nv + (InternalValue) std::pow( (double)abs( v[dim] - lower), (double) p)) >
          (nw + (InternalValue) std::pow( (double)abs( w[dim] - lower), (double)p)))
        { //trace.endBlock();
          return true;
        }
      //Optimization if vw lies before starting   
      if ((nu + (InternalValue) std::pow( (double)abs( u[dim] - lower), (double) p)) >
          (nv + (InternalValue) std::pow( (double)abs( v[dim] - lower), (double)p)))
        { //trace.endBlock();
          return false;
        }
  
      //Binary search
      Abscissa uv = binarySearchHidden(u[dim],v[dim],nu,nv,lower,upper); 
      Abscissa vw = binarySearchHidden(v[dim],w[dim],nv,nw,lower,upper);
#ifdef DEBUG_VERBOSE
      trace.info() << "Midpoint (u,v) ="<< uv<< " Midpoint (v,w) ="<<vw<<std::endl;
#endif      

      //trace.endBlock();
      if ( uv > vw )
        return true;
      else
        return false;
      
    }

  }; // end of class SeparableMetricHelper

  // ------------------------------------------------------------------------
  // -----------------------  Specializations   ------------------------------
  // ------------------------------------------------------------------------

  /**
   * L_2 specialization
   *
   */
  template <typename TPoint, typename TInternalValue>
  struct SeparableMetricHelper<TPoint, TInternalValue, 2>
  {
    typedef TInternalValue InternalValue;
    typedef typename TPoint::Coordinate Abscissa;
    typedef TPoint Point;
 
    static const DGtal::uint32_t p = 2;

    inline double getApproxValue ( const InternalValue & aInternalValue ) const
    {
      return ( double ) sqrt ( NumberTraits<InternalValue>::castToDouble(aInternalValue) );
    }

    inline InternalValue F ( const Abscissa pos, 
			     const Abscissa ci, 
			     const InternalValue hi ) const
    {
      return ( pos - ci ) * ( pos - ci ) + hi;
    }

    inline InternalValue reversedF ( const Abscissa pos, 
				     const Abscissa ci, 
				     const InternalValue hi ) const
    {
      return hi - ( pos - ci ) * ( pos - ci ) ;
    }


    inline Abscissa Sep ( const Abscissa i, const InternalValue hi, 
			  const Abscissa j, const InternalValue hj ) const
    {
      if (   ( ( j*j - i*i ) + hj - hi )  / ( 2* ( j - i ) )  >= 0)
	return (Abscissa)( ( j*j - i*i ) + hj - hi )  / ( 2* ( j - i ) );
      else
	return (Abscissa)( ( j*j - i*i ) + hj - hi )  / ( 2* ( j - i ) ) -1;
  
    }

    inline Abscissa reversedSep ( const Abscissa i, const InternalValue hi, 
				  const Abscissa j, const InternalValue hj ) const
    {
      return ( ( i*i -j*j ) + hj - hi )  / ( 2* ( i - j ) );
    }

    inline InternalValue power ( const Abscissa i ) const
    {
      return (InternalValue) (i*i);
    }
    enum Closest { FIRST=0, SECOND=1, BOTH=2};
    
    
    Closest closest(const Point &origin, 
		    const Point &first,
		    const Point &second) const
    {
      InternalValue a=NumberTraits<InternalValue>::ZERO,
	b=NumberTraits<InternalValue>::ZERO;
      
      for(typename Point::Dimension i=0; i <  Point::dimension; i++)
	{
	  a += (origin[i] - first[i])*(origin[i] - first[i]);
	  b += (origin[i] - second[i])*(origin[i] - second[i]);
	}
      if (a<b)
	return FIRST;
      else
	if (a>b)
	  return SECOND;
	else
	  return BOTH;
    }

    /** 
     * Given three sites (a,b,c) and a straight line (startingPoint,
     * dim), we detect if the voronoi cells of a and c @e hide the
     * voronoi cell of c on the straight line.
     * 
     * @param a a site
     * @param b a site
     * @param c a site
     * @param startingPoint starting point of the straight line
     * @param endPoint end point of the straight segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */
    bool hiddenBy(const Point &u, 
                  const Point &v,
                  const Point &w, 
                  const Point &startingPoint,
                  const Point &/*endPoint*/,
                  const typename Point::UnsignedComponent dim) const
    {
      //decide if (a,c) hide b in the lines (startingPoint, dim)

      Abscissa a,b, c;

      a = v[dim] - u[dim];
      b = w[dim] - v[dim];
      c = a + b;  
  
      Abscissa d2_v=0, d2_u=0 ,d2_w=0;

      for(Dimension i  = 0 ; i < Point::dimension ; i++)
	if (i != dim)
	  {
	    d2_u += (u[i] - startingPoint[i] ) *(u[i] - startingPoint[i] );
	    d2_v += (v[i] - startingPoint[i] ) *(v[i] - startingPoint[i] );
	    d2_w += (w[i] - startingPoint[i] ) *(w[i] - startingPoint[i] );
	  }
 
      return (c * d2_v -  b*d2_u - a*d2_w - a*b*c) > 0 ; 
    }
  
  };

  /**
   * L_1 specialization
   *
   */
  template <typename TPoint, typename TInternalValue>
  struct SeparableMetricHelper<TPoint, TInternalValue, 1>
  {
    
    typedef TInternalValue InternalValue;
    static const DGtal::uint32_t p = 1;
    typedef typename TPoint::Coordinate Abscissa;
    typedef TPoint Point;
    

    inline double getApproxValue ( const InternalValue & aInternalValue ) const
    {
      return ( double ) aInternalValue;
    }
 
    inline InternalValue F ( const Abscissa pos, 
			     const Abscissa ci, 
			     const InternalValue hi ) const
    {
      return ( InternalValue ) ( ((long int) pos - ci)>=0 ? ((long int) pos - ci) : - ((long int) pos - ci) ) + hi;
    }

    inline InternalValue reversedF ( const Abscissa pos, 
				     const Abscissa ci, 
				     const InternalValue hi ) const
    {
      return ( InternalValue ) hi - abs ( pos - ci );
    }


    inline Abscissa Sep ( const Abscissa i, const InternalValue hi, 
			  const Abscissa j, const InternalValue hj ) const
    {
      if (hj >= hi + j - i)
        return NumberTraits<Abscissa>::max();
      if (hi > hj + j - i)
        return NumberTraits<Abscissa>::min();
      return (int)((hj - hi + j + i) / 2);
    }

    inline Abscissa reversedSep ( const Abscissa i, const InternalValue hi, 
				  const Abscissa j, const InternalValue hj ) const
    {
      if (hj <= hi - j + i)
	return NumberTraits<Abscissa>::max();
      if (hi < hj - j + i)
        return NumberTraits<Abscissa>::min();
      return (hi + i - hj + j ) / 2;
    }
    

    inline InternalValue power ( const Abscissa i ) const
    {
      return (InternalValue) abs(i);
    }

    enum Closest { FIRST=0, SECOND=1, BOTH=2};
    
    
    Closest closest(const Point &origin, 
		    const Point &first,
		    const Point &second) const
    {
      InternalValue a=(origin-first).norm(Point::L_1),
	b=(origin-second).norm(Point::L_1);
      
      if (a<b)
	return FIRST;
      else
	if (a>b)
	  return SECOND;
	else
	  return BOTH;
    }

    /** 
     * Given three sites (a,b,c) and a straight line (startingPoint,
     * dim), we detect if the voronoi cells of a and c @e hide the
     * voronoi cell of c on the straight line.
     * 
     * @param a a site
     * @param b a site
     * @param c a site
     * @param startingPoint starting point of the straight line
     * @param endPoint end point of the straight segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */
    bool hiddenBy(const Point &u, 
                  const Point &v,
                  const Point &w, 
                  const Point &startingPoint,
                  const Point &endPoint,
                  const typename Point::UnsignedComponent dim) const
    {
      ASSERT(false && "Not-Yet-Implemented");
      Abscissa uv, vw;
      
      
    }

  }; // end of class SeparableMetricHelper

  /**
   * L_infinity specialization
   *
   */
  template <typename TPoint, typename TInternalValue>
  struct SeparableMetricHelper<TPoint, TInternalValue, 0>
  {
    typedef TInternalValue InternalValue;
    static const DGtal::uint32_t p = 0;
    typedef typename TPoint::Coordinate Abscissa;
    typedef TPoint Point;
 

    inline double getApproxValue ( const InternalValue & aInternalValue ) const
    {
      return ( double ) aInternalValue;
    }

    inline InternalValue F ( const Abscissa pos, const Abscissa ci, 
			     const InternalValue hi ) const
    {
      return ( InternalValue ) 
	max( (Abscissa) (((long int)pos - ci) >= 0 ? ((long int)pos - ci) :
			 -((long int)pos - ci)), (Abscissa) hi);
    }
    
    inline InternalValue reversedF ( const Abscissa pos, 
				     const Abscissa ci, 
				     const InternalValue hi ) const
    {
      ASSERT(false && "Not-Implemented");
    }


    inline Abscissa Sep ( const Abscissa i, const InternalValue hi,
			  const Abscissa j, const InternalValue hj ) const
    {
      if (hi <= hj)
        return max ((Abscissa)(i + hj), (Abscissa)(i + j) / 2);
      else
	return min ((Abscissa)(j - hi), (Abscissa)(i + j) / 2);
    }

    inline Abscissa reversedSep ( const Abscissa i, const InternalValue hi,
				  const Abscissa j, const InternalValue hj ) const
    {
      ASSERT(false && "Not-Implemented");
    }

    

    inline InternalValue power ( const Abscissa i ) const
    {
      return (InternalValue) abs(i);
    }

    enum Closest { FIRST=0, SECOND=1, BOTH=2};
    
    
    Closest closest(const Point &origin, 
		    const Point &first,
		    const Point &second) const
    {
      InternalValue a=(origin-first).norm(Point::L_infty),
	b=(origin-second).norm(Point::L_infty);
      
      if (a<b)
	return FIRST;
      else
	if (a>b)
	  return SECOND;
	else
	  return BOTH;
    }

    /** 
     * Given three sites (a,b,c) and a straight line (startingPoint,
     * dim), we detect if the voronoi cells of a and c @e hide the
     * voronoi cell of c on the straight line.
     * 
     * @param a a site
     * @param b a site
     * @param c a site
     * @param startingPoint starting point of the straight line
     * @param endPoint end point of the straight segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */
    bool hiddenBy(const Point &a, 
                  const Point &b,
                  const Point &c, 
                  const Point &startingPoint,
                  const Point &endPoint,
                  const typename Point::UnsignedComponent dim) const
    {
      ASSERT(false && "Not-Yet-Implemented");
    }
  }; // end of class SeparableMetricHelper


} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SeparableMetricHelper_h

#undef SeparableMetricHelper_RECURSES
#endif // else defined(SeparableMetricHelper_RECURSES)
