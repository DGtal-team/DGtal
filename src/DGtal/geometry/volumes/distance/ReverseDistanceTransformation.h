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
 * @brief Linear in time power map computation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/10/24
 *
 *
 * This file is part of the DGtal library.
 *
 * @see testReverseDistanceTransformation.cpp
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
#include <utility>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/kernel/CPointPredicate.h"

#include "DGtal/geometry/volumes/distance/SeparableMetricHelper.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ReverseDistanceTransformation
  /**
   * Description of template class 'ReverseDistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time Power map
   * construction.

 
   *
   * @tparam TSpace type of Digital Space (model of CSpace).
   * @tparam TPointPredicate point predicate returning true for points
   * from which we compute the distance (model of CPointPredicate)
   * @tparam p the static integer value to define the l_p metric.
   * @tparam IntegerLong (optional) type used to represent exact
   * distance value according to p (default: DGtal::uint64_t)
   *
   */
  template < typename TWeightImage,
             typename TValue,
	     DGtal::uint32_t p>
  class ReverseDistanceTransformation
  {

  public:

    BOOST_CONCEPT_ASSERT(( CConstImage< TWeightImage > ));

    ///Copy of the distance image types
    typedef TWeightImage WeightImage;
    typedef typename TWeightImage::Value Weight;
    typedef typename WeightImage::Domain::Space Space;
    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;

    ///Definition of the underlying domain type.
    typedef HyperRectDomain<Space> Domain;
   
    ///Definition of the image model value type.
    typedef TValue Value;

    ///Self type
    typedef ReverseDistanceTransformation<TWeightImage, TValue, p> Self;


    ///Definition of the image value type.
    typedef typename DefaultConstImageRange< Self >  ConstRange;

    
    ///Type of underlying power map
    typedef PowerMap<TWeightImage, p> PowerMap;

    /**
     * Constructor.
     * 
     * This constructor computes the Power Map of a set of point
     * sites using a SeparableMetric metric.  The method associates to
     * each point satisfying the foreground predicate, the closest
     * site for which the predicate is false. This algorithm is
     * O(d.|domain size|).
     *
     * @param aDomain defines the (hyperrectangular) domain on which the computation is performed. 
     * @param @todo 
     */
    ReverseDistanceTransformation(const Domain & aDomain,
				  const WeightImage & aWeightImage
				  const Value & aDefaultValue);
    
    /**
     * Default destructor
     */
    ~ReverseDistanceTransformation();

  public:
    // ------------------- ConstImage model ------------------------

    /**
     * Assignment operator from another Power map.
     *
     *  @param aOtherReverseDistanceTransformation another instance of Self
     *  @return a reference to Self
     */
    Self &  operator=(const Self &aOtherReverseDistanceTransformation );
    
    /**
     * Returns a reference (const) to the Power map domain.
     *  @return a domain
     */
    const Domain &  domain() const
    {
      return *myDomainPtr;
    }

    /**
     * Returns a const range on the Power map values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return ConstRange(*this);
    }
        
    /**
     * Access to a Power value (a.k.a. vector to the closest site) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      PowerMap::Weight dist;
      dist = myPowerMapPtr->metric()->distance(aPoint, 0,  
					       myPowerMapPtr->operator()(aPoint) ,
					       myWeightImagePtr->operator()(myPowerMapPtr->operator()(aPoint)));
      if (dist >= NumberTraits<Weight>::ZERO)
	return NumberTraits<Value>::ZERO;
      else
	return myDefaultValue;
    }    
     
    // ------------------- Private functions ------------------------
  private:    
    
    /**
     * Compute the Power Map of a set of point sites using a
     * SeparableMetric metric.  The method associates to each point
     * satisfying the foreground predicate, the closest site for which
     * the predicate is false. This algorithm is O(d.|domain size|).
     */
    void compute ( ) ;


    /** 
     *  Compute the other steps of the separable Power map.
     * 
     * @param output the output map
     * @param dim the dimension to process
     */    
    void computeOtherSteps(const Dimension dim) const;
    /** 
     * Given  a voronoi map valid at dimension @a dim-1, this method
     * updates the map to make it consistent at dimension @a dim along
     * the 1D span starting at @a row along the dimension @a
     * dim.
     * 
     * @param output the Power map to update.
     * @param row starting point of the 1D process.
     * @param dim dimension of the update.
     * @param Sites stack of sites (pass as an argument for
     * performance purposes).
     */
    void computeOtherStep1D (const Point &row, 
			     const Size dim,
			     std::vector<Point> &Sites) const;
    
    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    ReverseDistanceTransformation();
   
    
    // ------------------- Private members ------------------------
  private:

    ///The separable metric instance
    SeparableMetric myMetric;

    ///Pointer to the computation domain
    const Domain * myDomainPtr;
    
    ///Pointer to the point predicate
    const WeightImage * myWeightImagePtr;

    ///Pointer to the point predicate
    const PowerMap * myPowerMapPtr;
    
    ///Copy of the image lower bound
    Point myLowerBoundCopy;
    
    ///Copy of the image lower bound
    Point myUpperBoundCopy;
    
    ///Value to act as a +infinity value
    Point myInfinity;

  }; // end of class ReverseDistanceTransformation

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/ReverseDistanceTransformation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ReverseDistanceTransformation_h

#undef ReverseDistanceTransformation_RECURSES
#endif // else defined(ReverseDistanceTransformation_RECURSES)
