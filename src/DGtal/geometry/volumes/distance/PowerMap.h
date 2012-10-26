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
 * @file PowerMap.h
 * @brief Linear in time power map computation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/10/24
 *
 *
 * This file is part of the DGtal library.
 *
 * @see testPowerMap.cpp
 */

#if defined(PowerMap_RECURSES)
#error Recursive header files inclusion detected in PowerMap.h
#else // defined(PowerMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PowerMap_RECURSES

#if !defined PowerMap_h
/** Prevents repeated inclusion of headers. */
#define PowerMap_h

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
  // template class PowerMap
  /**
   * Description of template class 'PowerMap' <p>
   * \brief Aim: Implementation of the linear in time Power map
   * construction.

   * The algorithm uses a sperable process to construct
   * partial Power maps which has been described in:
   *
   *     ﻿Maurer, C., Qi, R., & Raghavan, V. (2003). A Linear Time Algorithm
   *      for Computing Exact Euclidean Distance Transforms of Binary Images in
   *      Arbitrary Dimensions. IEEE Trans. Pattern Analysis and Machine
   *      Intelligence, 25pp265-270.
   *
   * and 
   *   
   *      Coeurjolly, D. (2002). Algorithmique et géométrie discrète pour
   *      la caractérisation des courbes et des surfaces. PhD Thesis,
   *      Université Lumière Lyon 2.
   *
   *
   * Given a domain and a point predicate, an instance returns,
   * for each point in the domain, the closest point for
   * which the predicate if false. Following Computational Geometry
   * terminoliogy, points for which the predicate is false are "sites"
   * for the Power map construction.
   *
   * This class is a model of CConstImage.
   *
   * The metric is specified by the @a p template parameter which
   * defines a l_p separable metric.
   *
   * If a point is equi-distant to two sites (e.g. if the digital
   * point belong to a Power cell boundary in the Euclidean space),
   * this Power map construction will only keep one of them.
   *
   * Hence, the result is given as a map (point from the domain,
   * closest site) implemented using an ImageContainerBySTLVector
   * whose value type is the type of Point of the predicate.
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
             DGtal::uint32_t p>
  class PowerMap
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
   
 
    ///We construct the type associated to the separable metric @todo
    typedef SeparableMetricHelper<  Point ,  Weight , p > SeparableMetric;
  
    ///Type of resulting image
    typedef ImageContainerBySTLVector<  Domain,
                                        Vector > OutputImage;
    
    ///Definition of the image model value type.
    typedef Vector Value;
    ///Definition of the image value type.
    typedef typename OutputImage::ConstRange  ConstRange;
    ///Self type
    typedef PowerMap<TWeightImage, p> Self;
    

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
    PowerMap(const Domain & aDomain,
	     const WeightImage & aWeightImage);

    /**
     * Default destructor
     */
    ~PowerMap();

  public:
    // ------------------- ConstImage model ------------------------

    /**
     * Assignment operator from another Power map.
     *
     *  @param aOtherPowerMap another instance of Self
     *  @return a reference to Self
     */
    Self &  operator=(const Self &aOtherPowerMap );
    
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
      return myImagePtr->constRange();
    }
        
    /**
     * Access to a Power value (a.k.a. vector to the closest site) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return myImagePtr->operator()(aPoint);
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
     */
    void computeOtherStep1D (const Point &row, 
			     const Size dim) const;
    
    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    PowerMap();
   
    
    // ------------------- Private members ------------------------
  private:

    ///The separable metric instance
    SeparableMetric myMetric;

    ///Pointer to the computation domain
    const Domain * myDomainPtr;
    
    ///Pointer to the point predicate
    const WeightImage * myWeightImagePtr;
    
    ///Copy of the image lower bound
    Point myLowerBoundCopy;
    
    ///Copy of the image lower bound
    Point myUpperBoundCopy;
    
    ///Value to act as a +infinity value
    Point myInfinity;

    ///Power map image
    CountedPtr<OutputImage> myImagePtr;

  }; // end of class PowerMap

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/PowerMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PowerMap_h

#undef PowerMap_RECURSES
#endif // else defined(PowerMap_RECURSES)
