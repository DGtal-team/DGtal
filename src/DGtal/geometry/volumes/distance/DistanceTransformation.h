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
             DGtal::uint32_t p, 
             typename IntegerLong = DGtal::int64_t>
  class DistanceTransformation
  {

  public:
    
    BOOST_CONCEPT_ASSERT(( CSignedInteger<IntegerLong> ));
    BOOST_CONCEPT_ASSERT(( CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));
  
    
    ///Copy of the space type.
    typedef TSpace Space;

    ///Copy of the point predicate type.
    typedef TPointPredicate PointPredicate;

    ///Definition of the underlying domain type.
    typedef HyperRectDomain<Space> Domain;

    ///Type of resulting image
    typedef ImageContainerBySTLVector<  Domain,
                                        IntegerLong > OutputImage;

    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;
 
    ///We construct the type associated to the separable metric
    typedef SeparableMetricHelper<  Point ,  IntegerLong , p > SeparableMetric;
  

    /**
     *  Constructor
     */
    DistanceTransformation(const Domain & aDomain,
                           const PointPredicate & predicate);

    /**
     * Default destructor
     */
    ~DistanceTransformation();

  public:

    /**
     * Compute the Distance Transformation of a set of point using a 
     * SeparableMetric metric.  The method associates to each point
     * with value satisfying the foreground predicate, its distance to
     * the closest background point.  This algorithm is
     * O(d.|domain size|).
     *
     * @pre the foreground point predicate @a predicate must be defined on the
     * domain @a aDomain
     *
     * @return the distance transformation image.
     */
    OutputImage compute( ) ;


    /**
     * Check the validity of the transformation. For instance, we
     * check that the output image pixel range is ok with respect to
     * the domain extent and the SeparableMetric.
     *
     * Warning and advices are printed in the trace system.
     *
     * @return true if a warning has been raised. 
     */
    bool checkTypesValidity () const;
    
    
    // ------------------- Private functions ------------------------
  private:
    
    
    /** 
     * Compute the first step of the separable distance transformation.
     * 
     * @param output the output image with the first step DT values
     */
    void computeFirstStep(OutputImage & output) const;

    /** 
     * Compute the 1D DT associated to the first step.
     * 
     * @param output the output image  with the first step DT values
     * @param startingPoint a point to specify the starting point of the 1D row
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
    void computeOtherSteps(const OutputImage & inputImage,
                           OutputImage & output,
                           const Dimension dim) const;

    /** 
     * Compute the 1D DT associated to the steps except the first one.
     * 
     * @param aImage the input image
     * @param output the output image  with the  DT values
     * @param row a point to specify the starting point of the 1D row
     * @param dim the dimension to process
     * @param predicate  the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    void computeOtherStep1D (const OutputImage & input, 
                             OutputImage & output, 
                             const Point &row, 
                             const Size dim, 
                             Abscissa s[], Abscissa t[]) const;
    

    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    DistanceTransformation();
   
    
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

    ///Copy of the domain extent
    Point myExtent;

    ///Value to act as a +infinity value
    IntegerLong myInfinity;


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
