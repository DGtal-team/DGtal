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
   * @todo fix the choice of types  Integer (size of the image)
   * LongInteger (size of the values).
   *
   * @tparam Integer the interger type used 
   *
   */
  template <typename Integer, double tp>
  struct SeparableMetricTraits
  {
    // ----------------------- Standard services ------------------------------

    /**
     * Static constants containing the power p of the Lp-metric.
     *
     */
    static const double p = tp;
    
    /**
     * Default ResultType.
     */
    typedef double ResultType;

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
    ResultType F(const Integer pos, const Integer ci, const ResultType hi) const;

    /** 
     * Returns the abscissa of the intersection point between two
     * Lp-parabolas (ci,hi) and (cj,hj).
     * 
     * @param ci center of the first Lp-parabola.
     * @param hi height of the first Lp-parabola.
     * @param cj center of the first Lp-parabola. 
     * @param hj height of the first Lp-parabola.
     * 
     * @return 
     */ 
    long Sep(const int i, const long hi, const int j, const long hj) const;    

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
