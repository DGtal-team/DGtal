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
#include "DGtal/images/CImageContainer.h"
#include "DGtal/images/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/images/imagesSetsUtils/SimpleThresholdForegroundPredicate.h"

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
   * transformation.
   *  
   * @tparam Image an input image type.
   * @tparam p the static integer value to define the l_p metric.
   * @tparam IntegerLong (optional) type used to represent exact
   * distance value according to p (default: DGtal::uint64_t)
   *
   * Example:
   * @code
   * //Types definition
   * typedef ImageSelector<Domain, unsigned int>::Type Image; //image with "unsigned in value type
   *
   * //Distance transformation for the l_2 metric
   * typedef DistanceTransformation<Image, 2> DTl2; 
   * DTl2 dt; 
   *
   * // ...
   * //Construction of an instance "image" of Image (with an io reader for instance)
   * // ...
   *
   * //EDT computation
   * DTl2::OutputImage result = dt.compute(image);
   *
   * @endcode  
   */
  template <typename Image, DGtal::uint32_t p, typename IntegerLong = DGtal::int64_t >
  class DistanceTransformation
  {

  public:
    
    BOOST_CONCEPT_ASSERT(( CImageContainer<Image> ));
    BOOST_CONCEPT_ASSERT(( CSignedInteger<IntegerLong> ));
    

    ///Type of resulting image
    typedef ImageContainerBySTLVector<  HyperRectDomain<typename Image::Domain::Space> , IntegerLong > OutputImage;
  
    typedef typename Image::Value Value;
    typedef typename Image::Vector Vector;
    typedef typename Image::Point Point;
    typedef typename Image::Dimension Dimension;
    typedef typename Image::Size Size;
    typedef typename Image::Domain Domain;
    typedef typename Image::Domain::Space::Point::Coordinate Abscissa;
 
    ///We construct the type associated to the separable metric
    typedef SeparableMetricHelper<  Abscissa ,  IntegerLong , p > SeparableMetric;
  

    /**
     * Default Constructor
     */
    DistanceTransformation();

    /**
     * Default destructor
     */
    ~DistanceTransformation();

  public:

    /**
     * Check the validity of the transformation. For instance, we
     * check that the output image pixel range is ok with respect to
     * the input image range and the SeparableMetric.
     *
     * Warning and advices are printed in the trace system.
     *
     * @param aImage the image used to check the type consistency.
     * @return true if a warning has been raised. 
     */
    bool checkTypesValidity(const Image & aImage);

    /**
     * Compute the Distance Transformation of an image with the SeparableMetric metric.
     * The method associates to each point with value satisfying the
     * foreground predicate, its distance to the closest background point.
     * This algorithm is  O(d.|inputImage|).
     *
     * @pre the @a foregroundPredicate must have been constructed from
     * a reference to @a inputImage.
     *
     * @param inputImage the input image
     * @param foregroundPredicate a predicate to detect foreground
     * point from the image valuetype
     * @return the distance transformation image with the Internal format.
     */
    template <typename ForegroundPredicate>
    OutputImage compute(const Image & inputImage, 
			const ForegroundPredicate & predicate   );
    
    /**
     * Compute the Distance Transformation of an image with the SeparableMetric metric.
     * The method associates to each point with value satisfying the
     * foreground predicate (by default, values greater than 0), its
     * distance to the closest background point.
     * This algorithm is  O(d.|inputImage|).
     *
     * @param inputImage the input image
     * @return the distance transformation image with the Internal format.
     */
    OutputImage compute(const Image & inputImage )
    {
      return compute(inputImage, 
		     SimpleThresholdForegroundPredicate<Image>(inputImage, 0));
    };

    /**
     * Compute the Distance Transformation of a Set with the SeparableMetric metric.
     * This method first converts the digital set of an image and
     * compute the DT in O(d.N) where N is the number of grid points
     * of the bounding box of the set (and d the dimension). The
     * bounding box is enlarged by 1 point in each direction if the
     * addBoundary parameter is true (default value). 
     *
     * @param inputSet  the input set of grid points.
     * @param addBoundary if true (default value), we add a boundary
     * of thickness one to the bounding box (to make sure that all
     * grid points of aSet belonging to the boundary  have a DT of
     * value 1 (for the L2 and L1 case).
     *
     * @return the distance transformation image with the Internal format.
     */
    template<typename DigitalSet>
    OutputImage compute(const DigitalSet & inputSet, 
			const bool addBoundary=true );

   

    // ------------------- Private functions ------------------------
  private:

    /** 
     * Compute the first step of the separable distance transformation.
     * 
     * @param output the output image with the first step DT values
     * @param predicate the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    template <typename ForegroundPredicate>
    void computeFirstStep(OutputImage & output, 
			  const ForegroundPredicate &predicate) const;

    /** 
     * Compute the 1D DT associated to the first step.
     * 
     * @param output the output image  with the first step DT values
     * @param startingPoint a point to specify the starting point of the 1D row
     * @param predicate  the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    template <typename ForegroundPredicate>
    void computeFirstStep1D (OutputImage & output, 
			     const Point &startingPoint, 
			     const ForegroundPredicate &predicate) const;

    /** 
     *  Compute the other steps of the separable distance transformation.
     * 
     * @param inputImage the image resulting of the first (or
     * intermediate) step 
     * @param output the output image 
     * @param dim the dimension to process
     */    
    void computeOtherSteps(const OutputImage & inputImage, OutputImage & output, const Dimension dim)const;

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
    void computeOtherStep1D (const OutputImage & input, OutputImage & output, 
           const Point &row, const Size dim, 
           Abscissa s[], Abscissa t[]) const;


    // ------------------- Private members ------------------------
  private:

    ///The separable metric instance
    SeparableMetric myMetric;

    ///Copy of the image lower bound
    Point myLowerBoundCopy;

    ///Copy of the image lower bound
    Point myUpperBoundCopy;

    ///Displacement vector to translate temporary images.
    Vector myDisplacementVector;

    ///Copy of the image extent
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
