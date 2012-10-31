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
 * @file DistanceTransformation.h
 * @brief Linear in time distance transformation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/30
 *
 * Header file for module DistanceTransformation.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testDistanceTransformation.cpp, testDistanceTransformationND.cpp, testReverseDT.cpp
 */

#if defined(DistanceTransformation_RECURSES)
#error Recursive header files inclusion detected in DistanceTransformation.h
#else // defined(DistanceTransformation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DistanceTransformation_RECURSES

#if !defined DistanceTransformation_h
/** Prevents repeated inclusion of headers. */
#define DistanceTransformation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CSignedInteger.h"
#include "DGtal/images/CImage.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/images/imagesSetsUtils/ImageFromSet.h"

#include "DGtal/geometry/volumes/distance/SeparableMetricHelper.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DistanceTransformation
  /**
   * Description of template class 'DistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time distance
   * transformation for separable metrics.
   *  
   * Given a point predicate and a domain, the compute() method
   * returns for each point of the domain, the closest distance to a
   * point in the domain for which the predicate is false. The result
   * is given as a map point<->values implemented as an image
   * model OutputImage.
   *
   * The point predicate could be:
   *  - the result of the thresholding of an image (for example using SimpleThresholdForegroundPredicate)
   *  - a predicate constructed from a digital set (for example using SetPredicate)
   *  - ...
   *
   * @tparam TSpace type of Digital Space (model of CSpace).
   * @tparam TPointPredicate point predicate returning true for points
   * from which we compute the distance (model of CPointPredicate)
   * @tparam p the static integer value to define the l_p metric.
   * @tparam IntegerLong (optional) type used to represent exact
   * distance value according to p (default: DGtal::uint64_t)
   *
   * @see distancetransform2D.cpp
   * @see distancetransform3D.cpp
   */
  template < typename TSpace,
             typename TPointPredicate,
             typename TSeparableMetric>
  class DistanceTransformation: public VoronoiMap<TSpace,TPointPredicate,TSeparableMetric>
  {

  public:
    
    BOOST_CONCEPT_ASSERT(( CUnsignedSignedInteger<IntegerLong> ));
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));
 
    typedef TSeparableMetric SeparableMetric;
  
    ///Definition of the image value type.
    typedef  typename SeparableMetric::PowerValue Value;
    
    ///Definition of the image value type.
    typedef typename VoronoiMap<TSpace,TPointPredicate,TSeparableMetic>::ConstRange  ConstRange;
    
    ///Definition of the image value type.
    typedef typename VoronoiMap<TSpace,TPointPredicate,TSeparableMetic>::Domain  Domain;
    


    /**
     *  Constructor
     */
    DistanceTransformation(const Domain & aDomain,
                           const PointPredicate & predicate): ;
    /**
     * Default destructor
     */
    ~DistanceTransformation();
        
    // ------------------- Private functions ------------------------
  public:
    
     /**
     * Returns a const range on the DistanceMap values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return myImagePtr->constRange();
    }
        
    /**
     * Access to a DistanceMap value (a.k.a. the norm of the
     * associated Voronoi vector) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return this->myMetric.norm(myImagePtr->operator()(aPoint));
    }    
     

    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    DistanceTransformation();
   
    
    // ------------------- Private members ------------------------
  private:

  }; // end of class DistanceTransformation

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/DistanceTransformation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DistanceTransformation_h

#undef DistanceTransformation_RECURSES
#endif // else defined(DistanceTransformation_RECURSES)
