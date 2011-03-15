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
 * @date 2011/03/03
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
#include "DGtal/geometry/nd/volumetric/CSeparableMetric.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ReverseDistanceTransformation
  /**
   * Description of template class 'ReverseDistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time reverse distance
   * transformation for a class of separable metrics (see
   * SeparableMetric).
   * 
   * Example:
   * @code
   * //Types definition
   * typedef ImageSelector<Domain, unsigned int>::Type Image; //image with "unsigned in value type
   * typedef ImageSelector<Domain, long int>::Type ImageLong; //output image with long int value type
   *
   * typedef SeparableMetricTraits<DGtal::int32_t, DGtal::uint32_t,2> L_2; //L_2 = Euclidean metric
   *
   * DistanceTransformation<Image, ImageLong, L_2> dt; 
   * ReverseDistanceTransformation<ImageLong,Image L_2> reverseDT;
   *
   * // ...
   * //Construction of an instance "image" of Image (with an io reader for instance)
   * // ...
   *
   * //EDT computation
   * ImageLong result = dt.compute(image);
   *
   * //ReverseEDT computation
   * Image reconstruction = reverseDT.compute(result);
   * //reconstruction is identical to image
   *
   * @endcode  
   */
  template <typename TImage, typename TImageOutput, typename TSeparableMetric >
  class ReverseDistanceTransformation
  {

  public:
    
    BOOST_CONCEPT_ASSERT(( CSeparableMetric<TSeparableMetric> ));
			
    ///@todo check image concept
    typedef TImage Image;
    typedef TImageOutput ImageOutput;
    typedef TSeparableMetric SeparableMetric;
    typedef typename TSeparableMetric::InternalValue InternalValue;
    typedef typename Image::Value Value;
    typedef typename Image::Point Point;
    typedef typename Point::Coordinate Coordinate;
    typedef typename Image::Dimension Dimension;
    typedef typename Image::Domain Domain;


    /**
     * Constructor.
     *
     * @param objectValue a default value for grid points belonging to
     * the object (default = 1)
     * @param backgroundValue a default value for grid points belonging to
     * the background (default = 0)
     */
    ReverseDistanceTransformation( typename ImageOutput::Value objectValue = 1, 
				   typename ImageOutput::Value backgroundValue = 0);

    /**
     * Default destructor
     */
    ~ReverseDistanceTransformation();

  public:

    /**
     * Compute the Reverse Distance Transformation of an image with
     * the SeparableMetric metric. Given a set of ball (i.e. grid
     * points where values indicate a radius -InternalType for the
     * metric-), the method associates to each point belonging to the
     * union of balls the objectValue.
     *
     * @param inputImage the input image
     * @return the reverse distance transformation image.
     */
    ImageOutput compute(const Image & inputImage );
    
    // ------------------- Private functions ------------------------
  private:

    /** 
     * Cast values in order to output an image of type
     * ImageOutput. Basically, in internal computations, 0 values are
     * associated to background points and '>0' to object point. This
     * method casts the value to match with myObjectValue ad
     * myBackgroundValue specified in the constructor.
     * 
     * @param inputImage input internal image.
     * 
     * @return the filtered image.
     */
    ImageOutput castValues(const Image & inputInternalImage) const;

    /** 
     *  Compute the other steps of the separable distance transformation.
     * 
     * @param inputImage the image resulting of the first (or
     * intermediate) step 
     * @param output the output image 
     * @param dim the dimension to process
     */		
    void computeSteps(const Image & inputImage, Image & output, const Dimension dim ) const;

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
    void computeStep1D (const Image & input, Image & output,
			const Point &startingPoint, const Dimension dim, 
			Coordinate s[], 
			Coordinate t[]) const;


    // ------------------- Private members ------------------------
  private:

    ///The separable metric instance
    SeparableMetric myMetric;

    ///Copy of the image lower bound
    Point myLowerBoundCopy;

    ///Copy of the image lower bound
    Point myUpperBoundCopy;

    ///Copy of the domain extent
    Point myExtent;

    ///Copy of the internal domain
    Domain myDomain;

    ///Default values for object and background
    typename ImageOutput::Value myObjectValue, myBackgroundValue;

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
