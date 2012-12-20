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
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/CPointPredicate.h"

#include "DGtal/geometry/volumes/distance/SeparableMetricHelper.h"
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
   * Given a domain and a point predicate, the compute() method
   * returns, for each point in the domain, the closest point for
   * which the predicate if false. Following Computational Geometry
   * terminoliogy, points for which the predicate is false are "sites"
   * for the Voronoi map construction.
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
             DGtal::uint32_t p>
  class VoronoiMap
  {

  public:
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));

    ///Both Space points and PointPredicate points must be the same.
    BOOST_STATIC_ASSERT ((boost::is_same< typename TSpace::Point,
					  typename TPointPredicate::Point >::value )); 
    
    ///Copy of the space type.
    typedef TSpace Space;

    ///Copy of the point predicate type.
    typedef TPointPredicate PointPredicate;

    ///Definition of the underlying domain type.
    typedef HyperRectDomain<Space> Domain;
   
    ///Large integer type for SeparableMetricHelper construction.
    typedef DGtal::int64_t IntegerLong;

    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;
 
    ///We construct the type associated to the separable metric
    typedef SeparableMetricHelper<  Point ,  IntegerLong , p > SeparableMetric;
  
    ///Type of resulting image
    typedef ImageContainerBySTLVector<  Domain,
                                        Point > OutputImage;

    /**
     *  Constructor
     */
    VoronoiMap(const Domain & aDomain,
               const PointPredicate & predicate);

    /**
     * Default destructor
     */
    ~VoronoiMap();

  public:

     /**
     * Compute the Voronoi Map of a set of point sites using a
     * SeparableMetric metric.  The method associates to each point
     * satisfying the foreground predicate, the closest site for which
     * the predicate is false. This algorithm is O(d.|domain size|).
     *
     * @return the Voronoi map image.
     */
    OutputImage compute ( ) ;

    
    // ------------------- Private functions ------------------------
  private:    
    
    /** 
     *  Compute the other steps of the separable Voronoi map.
     * 
     * @param output the output map
     * @param dim the dimension to process
     */    
    void computeOtherSteps(OutputImage & output,
                           const Dimension dim) const;
    /** 
     * Given  a voronoi map valid at dimension @a dim-1, this method
     * updates the map to make it consistent at dimension @a dim along
     * the 1D span starting at @a row along the dimension @a
     * dim.
     * 
     * @param output the Voronoi map to update.
     * @param row starting point of the 1D process.
     * @param dim dimension of the update.
     * @param Sites stack of sites (pass as an argument for
     * performance purposes).
     */
    void computeOtherStep1D (OutputImage & output, 
			     const Point &row, 
			     const Size dim,
			     std::vector<Point> &Sites) const;
    
    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    VoronoiMap();
   
    
    // ------------------- Private members ------------------------
  private:

    ///The separable metric instance
    SeparableMetric myMetric;

    ///Copy of the computation domain
    const Domain & myDomain;
    
    ///Copy of the computation domain
    const PointPredicate  & myPointPredicate;
    
    ///Copy of the image lower bound
    Point myLowerBoundCopy;
    
    ///Copy of the image lower bound
    Point myUpperBoundCopy;
    
    ///Value to act as a +infinity value
    Point myInfinity;


  }; // end of class VoronoiMap

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/VoronoiMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VoronoiMap_h

#undef VoronoiMap_RECURSES
#endif // else defined(VoronoiMap_RECURSES)
