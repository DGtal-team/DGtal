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
 * @file VoronoiMap.h
 * @brief Linear in time distance transformation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/14
 *
 * Header file for module VoronoiMap.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testVoronoiMap.cpp
 */

#if defined(VoronoiMap_RECURSES)
#error Recursive header files inclusion detected in VoronoiMap.h
#else // defined(VoronoiMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VoronoiMap_RECURSES

#if !defined VoronoiMap_h
/** Prevents repeated inclusion of headers. */
#define VoronoiMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/geometry/volumes/distance/CSeparableMetric.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class VoronoiMap
  /**
   * Description of template class 'VoronoiMap' <p>
   * \brief Aim: Implementation of the linear in time Voronoi map
   * construction.

   * The algorithm uses a sperable process to construct
   * partial Voronoi maps which has been described in:
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
   * for the Voronoi map construction.
   *
   * This class is a model of CConstImage.
   *
   * The metric is specified by the @a p template parameter which
   * defines a l_p separable metric.
   *
   * If a point is equi-distant to two sites (e.g. if the digital
   * point belong to a Voronoi cell boundary in the Euclidean space),
   * this Voronoi map construction will only keep one of them.
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
  template < typename TSpace,
             typename TPointPredicate,
             typename TSeparableMetric>
  class VoronoiMap
  {

  public:
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));
    BOOST_CONCEPT_ASSERT(( CSeparableMetric<TSeparableMetric> ));

    ///Both Space points and PointPredicate points must be the same.
    BOOST_STATIC_ASSERT ((boost::is_same< typename TSpace::Point,
					  typename TPointPredicate::Point >::value )); 
    
    ///Copy of the space type.
    typedef TSpace Space;

    ///Copy of the point predicate type.
    typedef TPointPredicate PointPredicate;

    ///Definition of the underlying domain type.
    typedef HyperRectDomain<Space> Domain;

    ///Definition of the separable metric type
    typedef TSeparableMetric SeparableMetric;

    ///Large integer type for SeparableMetricHelper construction.
    typedef DGtal::int64_t IntegerLong;

    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;
 
    ///Type of resulting image
    typedef ImageContainerBySTLVector<  Domain,
                                        Vector > OutputImage;
    
    ///Definition of the image value type.
    typedef Vector Value;
    
    ///Definition of the image value type.
    typedef typename OutputImage::ConstRange  ConstRange;

    ///Self type
    typedef VoronoiMap<TSpace, TPointPredicate, TSeparableMetric> Self;
    

    /**
     * Constructor.
     * 
     * This constructor computes the Voronoi Map of a set of point
     * sites using a SeparableMetric metric.  The method associates to
     * each point satisfying the foreground predicate, the closest
     * site for which the predicate is false. This algorithm is
     * O(d.|domain size|).
     *
     * @param aDomain defines the (hyperrectangular) domain on which the computation is performed. 
     * @param predicate point predicate to define the Voronoi sites
     * (false points). 
     * @param aMetric a seprable metric instance.
     */
    VoronoiMap(const Domain * aDomain,
               const PointPredicate * predicate,
               const SeparableMetric * aMetric);

    /**
     * Default destructor
     */
    ~VoronoiMap();

  public:
    // ------------------- ConstImage model ------------------------

    /**
     * Assignment operator from another Voronoi map.
     *
     *  @param aOtherVoronoiMap another instance of Self
     *  @return a reference to Self
     */
    Self &  operator=(const Self &aOtherVoronoiMap );
    
    /**
     * Returns a reference (const) to the Voronoi map domain.
     *  @return a domain
     */
    const Domain &  domain() const
    {
      return *myDomainPtr;
    }

    
    /**
     * Returns a const range on the Voronoi map values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return myImagePtr->constRange();
    }
        
    /**
     * Access to a Voronoi value (a.k.a. vector to the closest site) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return myImagePtr->operator()(aPoint);
    }    
     
    /** 
     * @return  Returns the underlying metric.
     */
    const SeparableMetric* metric() const
    {
      return myMetricPtr;
    }

    /**
     * Self Display method.
     * 
     * @param out output stream
     */
    void selfDisplay ( std::ostream & out ) const;    
   
    // ------------------- Private functions ------------------------
  private:    
    
    /**
     * Compute the Voronoi Map of a set of point sites using a
     * SeparableMetric metric.  The method associates to each point
     * satisfying the foreground predicate, the closest site for which
     * the predicate is false. This algorithm is O(d.|domain size|).
     */
    void compute ( ) ;


    /** 
     *  Compute the other steps of the separable Voronoi map.
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
     * @param output the Voronoi map to update.
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
    VoronoiMap();
   
    
    // ------------------- Private members ------------------------
  private:

    ///Pointer to the computation domain
    const Domain * myDomainPtr;
    
    ///Pointer to the point predicate
    const PointPredicate * myPointPredicatePtr;
    
    ///Copy of the image lower bound
    Point myLowerBoundCopy;
    
    ///Copy of the image lower bound
    Point myUpperBoundCopy;
    
    ///Value to act as a +infinity value
    Point myInfinity;

  protected:

    ///Pointer to the separable metric instance
    const SeparableMetric * myMetricPtr;

    ///Voronoi map image
    CountedPtr<OutputImage> myImagePtr;

  }; // end of class VoronoiMap

  /**
   * Overloads 'operator<<' for displaying objects of class 'ExactPredicateLpSeparableMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ExactPredicateLpSeparableMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename S, typename P,
            typename Sep>
  std::ostream&
  operator<< ( std::ostream & out, const VoronoiMap<S,P,Sep> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/VoronoiMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VoronoiMap_h

#undef VoronoiMap_RECURSES
#endif // else defined(VoronoiMap_RECURSES)
