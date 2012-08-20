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
  
    
    ///Copy of the space type.
    typedef TSpace Space;

    ///Copy of the point predicate type.
    typedef TPointPredicate PointPredicate;

    ///Definition of the underlying domain type.
    typedef HyperRectDomain<Space> Domain;

   
    ///@todo comment here
    typedef DGtal::int64_t IntegerLong;

    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;
 
    ///We construct the type associated to the separable metric
    typedef SeparableMetricHelper<  Abscissa ,  IntegerLong , p > SeparableMetric;
  
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
     * Compute the Distance Transformation of an image with the
     * SeparableMetric metric.  The method associates to each point
     * with value satisfying the foreground predicate, its distance to
     * the closest background point.  This algorithm is
     * O(d.|domain size|).
     *
     * @pre the foreground point predicate @a predicate must be defined on the
     * domain @a aDomain
     *
     *
     * @return the distance transformation image.
     */
    OutputImage compute( ) ;

    
    // ------------------- Private functions ------------------------
  private:
    
    
    /** 
     * Compute the first step of the separable distance transformation.
     * 
     * @param output the output image with the first step DT values
     * @param predicate the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    void computeFirstStep(OutputImage & output) const;

    /** 
     * Compute the 1D DT associated to the first step.
     * 
     * @param output the output image  with the first step DT values
     * @param startingPoint a point to specify the starting point of the 1D row
     * @param predicate  the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    void computeFirstStep1D (OutputImage & output, 
			     const Point &startingPoint) const;
    
    /** 
     *  Compute the other steps of the separable distance transformation.
     * 
     * @param inputImage the image resulting of the first (or
     * intermediate) step 
     * @param output the output image 
     * @param dim the dimension to process
     */    
    void computeOtherSteps(OutputImage & output,
                           const Dimension dim) const;
    
    /** 
     */
     void computeOtherStep1D (OutputImage & output, 
                              const Point &row, 
                              const Size dim) const;
    
    bool hiddenBy(const Point &a, 
                  const Point &b,
                  const Point &c, 
                  const Point &startingPoint,
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
    
    ///Displacement vector to translate temporary images.
    Vector myDisplacementVector;

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
