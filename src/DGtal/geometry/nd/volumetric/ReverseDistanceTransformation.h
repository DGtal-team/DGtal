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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/21
 *
 * Header file for module ReverseDistanceTransformation.cpp
 *
 * This file is part of the DGtal library.
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
#include "DGtal/kernel/IntegerTraits.h"
#include "DGtal/kernel/images/CImageContainer.h"
#include "DGtal/geometry/nd/volumetric/SeparableMetricTraits.h"
#include "DGtal/kernel/IntegerTraits.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ReverseDistanceTransformation
  /**
   * Description of template class 'ReverseDistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time distance
   * transformation.
   *  
   * @tparam Image an input image type.
   * @tparam p the static integer value to define the l_p metric.
   * @tparam IntegerLong (optional) type used to represent exact
   * distance value according to p (default: DGtal::uint64_t)
   *
   *
   * @endcode  
   */
  template <typename Image, DGtal::uint32_t p, typename IntegerShort = DGtal::int32_t >
  class ReverseDistanceTransformation
  {

  public:
    
    BOOST_CONCEPT_ASSERT(( CImageContainer<Image> ));
    BOOST_CONCEPT_ASSERT(( CInteger<IntegerShort> ));
    

    ///Type of resulting image
    typedef ImageContainerBySTLVector<  HyperRectDomain<typename Image::Domain::Space> , 
					IntegerShort > OutputImage;
  
    typedef typename Image::Value Value;
    typedef typename Image::Point Point;
    typedef typename Image::Dimension Dimension;
    typedef typename Image::Size Size;
    typedef typename Image::Integer Integer;
    typedef typename Image::Domain Domain;
    typedef typename Image::Domain::Space::Point::Coordinate Coordinate;
  
    ///We construct the type associated to the separable metric
    typedef SeparableMetricTraits<  IntegerShort ,  Integer , p > SeparableMetric;
  

    /**
     * Constructor.
     *
     * @param defaultForgroundValue (optional) default used to represent object
     * grid points 
     * @param defaultBackgroundValue (optional) default used to represent background
     * grid points
     */
    ReverseDistanceTransformation(const IntegerShort defaultForeground = IntegerTraits< IntegerShort >::ONE, 
				  const IntegerShort defaultBackground = IntegerTraits< IntegerShort >::ZERO);

    /**
     * Default destructor
     */
    ~ReverseDistanceTransformation();

  public:

    /**
     * Compute the Reverse Distance Transformation of an image with
     * the SeparableMetric metric.
     *
     * @param inputImage the input image with distance values
     * @return the distance transformation image with the Internal format.
     */
    OutputImage compute(const Image & inputImage);


    // ------------------- Private functions ------------------------
  private:

    /** 
     * Cast values in order to output an image of type
     * ImageOutput. Basically, in internal computations, 0 values are
     * associated to background points and '>0' to object point. This
     * method casts the value to match with myForegroundValue ad
     * myBackgroundValue specified in the constructor.
     * 
     * @param inputImage input internal image.
     * 
     * @return the filtered image.
     */
    OutputImage castValues(const Image &input) const;
  
    /** 
     *  Compute the other steps of the separable reverse distance
     *  transformation.
     * 
     * @param inputImage the image resulting of the first (or
     * intermediate) step 
     * @param output the output image 
     * @param dim the dimension to process
     */		
    void computeSteps(const Image & inputImage, 
		      Image & output, 
		      const Dimension dim)const;

    /** 
     * Compute the 1D DT associated to the steps except the first one.
     * 
     * @param aImage the input image
     * @param output the output image  with the  DT values
     * @param startingPoint a point to specify the starting point of the 1D row
     * @param dim the dimension to process
     * @param predicate  the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    void computeSteps1D (const Image & input, 
			 Image & output, 
			 const Point &startingPoint, 
			 const Size dim, 
			 Integer s[], Integer t[]) const;


    // ------------------- Private members ------------------------
  private:



    ///The separable metric instance
    SeparableMetric myMetric;

    ///Copy of the image lower bound
    Point myLowerBoundCopy;

    ///Copy of the image lower bound
    Point myUpperBoundCopy;

    ///Copy of the image extent
    Point myExtent;

    ///Value for foreground grid points.
    IntegerShort myForegroundValue;
    ///Value for background grid points.
    IntegerShort myBackgroundValue;
    


  }; // end of class ReverseDistanceTransformation

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/nd/volumetric/ReverseDistanceTransformation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ReverseDistanceTransformation_h

#undef ReverseDistanceTransformation_RECURSES
#endif // else defined(ReverseDistanceTransformation_RECURSES)
