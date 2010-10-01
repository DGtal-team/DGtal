#pragma once

/**
 * @file SeparableMetricTraits.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/08/01
 *
 * Header file for module SeparableMetricTraits.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SeparableMetricTraits_RECURSES)
#error Recursive header files inclusion detected in SeparableMetricTraits.h
#else // defined(SeparableMetricTraits_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SeparableMetricTraits_RECURSES

#if !defined SeparableMetricTraits_h
/** Prevents repeated inclusion of headers. */
#define SeparableMetricTraits_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SeparableMetricTraits
  /**
   * Description of template class 'SeparableMetricTraits' <p>
   * \brief Aim: Implements basic functions associated to metrics used
   * by separable volumetric algorithms.
   *
   * @todo fix the choice of types  Integer (extent of the image)
   * LongInteger (size of the values).
   * @todo replace Integer -> ExtentType
   *
   * @tparam Integer Type used to store the Domain extent.
   * @tparam TValueType the type of the input map.
   * @tparam tp the order p of the L_p metric.
   *
   */
  template <typename Integer, typename TValueType, unsigned int tp>
  struct SeparableMetricTraits
  {
    // ----------------------- Standard services ------------------------------

    typedef TValueType ValueType;

    /**
     * Static constants containing the power p of the Lp-metric.
     *
     */
    static const double p = tp;
    

    /**
     * Default InternalType.
     */
    typedef double InternalValueType;

    
    /** 
     * Operator () in order to return the correct value from the
     * InternalValuetype used to ensure exact computations.
     * 
     * @param aInternalValueType the internal value to convert
     * 
     * @return the converted value.
     */
    ValueType operator()(const InternalValueType & aInternalValueType) const;

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
    InternalValueType F(const Integer pos, const Integer ci, const InternalValueType hi) const;

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
    Integer Sep(const Integer i, const InternalValueType hi, const Integer j, const InternalValueType hj) const;    


  }; // end of class SeparableMetricTraits

  // ------------------------------------------------------------------------
  // -----------------------  Specializations   ------------------------------
  // ------------------------------------------------------------------------

  /**
   * L_2 specialization
   *
   */
  template <typename Integer,typename TValueType>
  struct SeparableMetricTraits<Integer, TValueType, 2>
  {
    typedef TValueType ValueType;
    
    static const double p = 2;

    //Check if ValueType sizeof() > capacité max
    typedef ValueType InternalValueType;

    inline ValueType operator()(const InternalValueType & aInternalValue) const
    {
      return (ValueType)sqrt(aInternalValue);
    }
    
    inline InternalValueType F(const Integer pos, const Integer ci, const InternalValueType hi) const
    {
      return (pos - ci)*(pos - ci) + hi;
    }
    
    inline Integer Sep(const int i, const long hi, const int j, const long hj) const
    {
      return ((j*j - i*i) + hj + hi)  / (2*(j-i));
    }
  };
   
    /**
     * L_1 specialization
     *
     */
    template <typename Integer, typename TValueType>
    struct SeparableMetricTraits<Integer, TValueType, 1>
    {
    
      typedef TValueType ValueType; 
      static const double p = 1;
      typedef ValueType InternalValueType;

      inline ValueType operator()(const InternalValueType & aInternalValue) const
      {
	return (ValueType)aInternalValue;
      }
    
      inline InternalValueType F(const Integer pos, const Integer ci, const InternalValueType hi) const
      {
	return (InternalValueType) abs(pos - ci) + hi;
      }
    
      inline Integer Sep(const int i, const long hi, const int j, const long hj) const  
      {
	///@todo
	return 0;
      }
   

    }; // end of class SeparableMetricTraits

  } // namespace DGtal


  ///////////////////////////////////////////////////////////////////////////////
  // Includes inline functions.
#include "DGtal/geometry/nd/volumetric/SeparableMetricTraits.ih"

  //                                                                           //
  ///////////////////////////////////////////////////////////////////////////////

#endif // !defined SeparableMetricTraits_h

#undef SeparableMetricTraits_RECURSES
#endif // else defined(SeparableMetricTraits_RECURSES)
