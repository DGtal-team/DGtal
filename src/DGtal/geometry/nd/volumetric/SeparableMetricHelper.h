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
   * @todo Fix the integer type problems.
   */
  template <typename TAbscissa, typename TInternalValue, DGtal::uint32_t tp>
  struct SeparableMetricHelper
  {
    // ----------------------- Standard services ------------------------------

    typedef TInternalValue InternalValue;
    typedef TAbscissa Abscissa;
    
    
    BOOST_CONCEPT_ASSERT(( CBoundedInteger<TAbscissa> ));
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
      return ( InternalValue ) std::pow ( pos, p );
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


  }; // end of class SeparableMetricHelper

  // ------------------------------------------------------------------------
  // -----------------------  Specializations   ------------------------------
  // ------------------------------------------------------------------------

  /**
   * L_2 specialization
   *
   */
  template <typename TAbscissa, typename TInternalValue>
  struct SeparableMetricHelper<TAbscissa, TInternalValue, 2>
  {
    typedef TInternalValue InternalValue;
    typedef TAbscissa Abscissa;
    
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
  return ( ( j*j - i*i ) + hj - hi )  / ( 2* ( j - i ) );
      else
  return ( ( j*j - i*i ) + hj - hi )  / ( 2* ( j - i ) ) -1;
  
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
  };

  /**
   * L_1 specialization
   *
   */
  template <typename TAbscissa, typename TInternalValue>
  struct SeparableMetricHelper<TAbscissa, TInternalValue, 1>
  {

    typedef TInternalValue InternalValue;
    static const DGtal::uint32_t p = 1;
    typedef TAbscissa Abscissa;
    

    inline double getApproxValue ( const InternalValue & aInternalValue ) const
    {
      return ( double ) aInternalValue;
    }
 
    inline InternalValue F ( const Abscissa pos, 
           const Abscissa ci, 
           const InternalValue hi ) const
    {
      return ( InternalValue ) ( ((long int) pos - ci)>=0 ? ((long int) pos - ci) : - ((long int) pos - ci) ) + hi;
      //std::abs ( (long int) pos - ci ) + hi;
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
      return (hj - hi + j + i) / 2;
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

  }; // end of class SeparableMetricHelper

  /**
   * L_infinity specialization
   *
   */
  template <typename TAbscissa, typename TInternalValue>
  struct SeparableMetricHelper<TAbscissa, TInternalValue, 0>
  {
    typedef TAbscissa Abscissa;
    typedef TInternalValue InternalValue;
    static const DGtal::uint32_t p = 0;
        

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

  }; // end of class SeparableMetricHelper


} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SeparableMetricHelper_h

#undef SeparableMetricHelper_RECURSES
#endif // else defined(SeparableMetricHelper_RECURSES)
