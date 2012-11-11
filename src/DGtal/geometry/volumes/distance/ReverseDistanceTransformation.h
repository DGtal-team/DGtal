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
 * @file ReverseDistanceTransformation.h
 * @brief Linear in time distance transformation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/09
 *
 * Header file for module ReverseDistanceTransformation.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testReverseDistanceTransformation.cpp, testReverseDistanceTransformationND.cpp, testReverseDT.cpp
 */

#if defined(ReverseDistanceTransformation_RECURSES)
#error Recursive header files inclusion detected in ReverseDistanceTransformation.h
#else // defined(ReverseDistanceTransformation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ReverseDistanceTransformation_RECURSES

#if !defined ReverseDistanceTransformation_h
/** Prevents repeated inclusion of headers. */
#define ReverseDistanceTransformation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/geometry/volumes/distance/CPowerSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/PowerMap.h"
#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ReverseDistanceTransformation
  /**
   * Description of template class 'ReverseDistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time reverse distance
   * transformation for separable metrics.
   *  
   *
   * @todo documentation
   *
   */
  template < typename TWeightImage,
             typename TPSeparableMetric>
  class ReverseDistanceTransformation: public PowerMap<TWeightImage,TPSeparableMetric>
  {

  public:

    ///Separable Metric type
    typedef TWeightImage WeightImage;

    ///Separable Metric type
    typedef TPSeparableMetric PowerSeparableMetric;

    ///Separable Metric type value type
    typedef typename PowerSeparableMetric::Value Value;

    ///Point type
    typedef typename TWeightImage::Domain::Space::Point Point;

    ///Vector type
    typedef typename TWeightImage::Domain::Space::Vector Vector;

    ///Separable Metric type weight type
    typedef typename PowerSeparableMetric::Weight Weight;
 
    //BOOST_STATIC_ASSERT((boost::is_same< typename TWeightImage::Value, 
    //                    typename PowerSeparableMetric::Point>::value));
    
    ///Definition of the image.
    typedef  ReverseDistanceTransformation<TWeightImage,TPSeparableMetric> Self;
    
    typedef PowerMap<TWeightImage,TPSeparableMetric> Parent;
   
    ///Definition of the image constRange
    typedef  DefaultConstImageRange<Self> ConstRange;


    ///Definition of the image value type.
    typedef typename PowerMap<TWeightImage,TPSeparableMetric>::Domain  Domain;
    

    /**
     *  Constructor
     */
    ReverseDistanceTransformation(const Domain & aDomain,
                                  const WeightImage & aWeightImage,
                                  const PowerSeparableMetric & aMetric):
      PowerMap<TWeightImage,TPSeparableMetric>(aDomain,aWeightImage,aMetric)
    {}
    
    /**
     * Default destructor
     */
    ~ReverseDistanceTransformation() {};
        
    // ------------------- Private functions ------------------------
  public:
    
     /**
     * Returns a const range on the ReverseDistanceMap values.
     *  @return a const range
     */
    Domain domain() const
    {
      return Parent::domain();
    }
    
     /**
     * Returns a const range on the ReverseDistanceMap values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return ConstRange(*this);
    }
        
    /**
     * Access to a ReverseDistanceMap value (a.k.a. the norm of the
     * associated Voronoi vector) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return this->myMetricPtr->powerDistance(aPoint, 
                                              this->myImagePtr->operator()(aPoint),
                                              this->myWeightImagePtr->operator()(aPoint));
    }    
          
    /**
     * Access to a ReverseDistanceMap value (a.k.a. the norm of the
     * associated Voronoi vector) at a point.
     *
     * @param aPoint the point to probe.
     */
    Vector getPowerVector(const Point &aPoint) const
    {
      return this->myImagePtr->operator()(aPoint);
    }    
     
    /** 
     * @return  Returns the underlying metric.
     */
    const PowerSeparableMetric* metricPtr() const
    {
      return Parent::metricPtr();
    }

    /** 
     * Self Display method.
     * 
     * @param out 
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[ReverseDistanceTransformation] underlying PowerMap={";
      Parent::selfDisplay(out);
      out << "}";
    }
    
    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    ReverseDistanceTransformation();
   
    
    // ------------------- Private members ------------------------
  private:

  }; // end of class ReverseDistanceTransformation


// //                                                                           //
// ///////////////////////////////////////////////////////////////////////////////
  
  template <typename W,typename TSep>
  inline
  std::ostream&
  operator<< ( std::ostream & out, 
               const ReverseDistanceTransformation<W,TSep> & object )
  {
    object.selfDisplay( out );
    return out;
  }
  

  
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ReverseDistanceTransformation_h

#undef ReverseDistanceTransformation_RECURSES
#endif // else defined(ReverseDistanceTransformation_RECURSES)
