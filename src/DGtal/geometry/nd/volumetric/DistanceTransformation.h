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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/30
 *
 * Header file for module DistanceTransformation.cpp
 *
 * This file is part of the DGtal library.
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
#include "DGtal/kernel/IntegerTraits.h"
#include "DGtal/geometry/nd/volumetric/CSeparableMetric.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DistanceTransformation
  /**
   * Description of template class 'DistanceTransformation' <p>
   * \brief Aim: Implementation of the linear in time distance
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
   *
   * // ...
   * //Construction of an instance "image" of Image (with an io reader for instance)
   * // ...
   *
   * //EDT computation
   * ImageLong result = dt.compute(image);
   *
   * @endcode  
   */
  template <typename TImage, typename TImageOutput, typename TSeparableMetric >
  class DistanceTransformation
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
    typedef typename Image::Dimension Dimension;
    typedef typename Image::Size Size;
    typedef typename Image::Integer Integer;
    typedef typename Image::Domain Domain;


    /**
     * Default Constructor
     */
    DistanceTransformation();

    /**
     * Default destructor
     */
    ~DistanceTransformation();


    // ------- Private Functor to be used as a default template ----

  private:
    /**
     * Default foregroundPredicate : we return true if the value at a
     * point differs from zero.
     *
     * @todo Refactoring needed to generalize this class !
     */
    struct DefaultForegroundPredicate
    {
      bool operator()(const Image &aImage, const typename Image::Point &aPoint) const
      {
	return (aImage(aPoint) != 0);
      }

      bool operator()(const Image &aImage, const typename Image::Iterator &it) const
      {
	return (aImage(it) != 0);
      }

      bool operator()(const Image &aImage, const typename Image::ConstIterator &it) const
      {
	return (aImage(it) != 0);
      }

      bool operator()(const Image &aImage, const typename Image::SpanIterator &it) const
      {
	return (aImage(it) != 0);
      }

    };
  public:

    /**
     * Check the validity of the transformation. For instance, we
     * check that the output image pixel range is ok with respect to
     * the input image range and the SeparableMetric.
     *
     * Warning and advices are print in the trace system.
     *
     * @param aImage the image used to check the type consistency.
     */
    void checkTypesValidity(const Image & aImage);

    /**
     * Compute the Distance Transformation of an image with the SeparableMetric metric.
     * The method associates to each point with value satisfying the
     * foreground predicate, its distance to the closest background point.
     *
     * @param inputImage the input image
     * @param foregroundPredicate a predicate to detect foreground
     * point from the image valuetype
     * @return the distance transformation image with the Internal format.
     */
    template <typename ForegroundPredicate>
    ImageOutput compute(const Image & inputImage, const ForegroundPredicate & predicate  );

    /**
     * Compute the Distance Transformation of an image with the SeparableMetric metric.
     * The method associates to each point with value satisfying the
     * foreground predicate, its distance to the closest background point.
     *
     * @param inputImage the input image
     * @return the distance transformation image with the Internal format.
     */
    ImageOutput compute(const Image & inputImage )
    {
      return compute<DefaultForegroundPredicate>(inputImage, DefaultForegroundPredicate());
    };


    // ------------------- Private functions ------------------------
  private:

    /** 
     * Compute the first step of the separable distance transformation.
     * 
     * @param aImage the input image
     * @param output the output image with the first step DT values
     * @param predicate the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    template <typename ForegroundPredicate>
    void computeFirstStep(const Image & aImage, ImageOutput & output, const ForegroundPredicate &predicate) const;

    /** 
     * Compute the 1D DT associated to the first step.
     * 
     * @param aImage the input image
     * @param output the output image  with the first step DT values
     * @param starting a point to specify the starting point of the 1D row
     * @param predicate  the predicate to characterize the foreground
     * (e.g. !=0, see DefaultForegroundPredicate)
     */
    template <typename ForegroundPredicate>
    void computeFirstStep1D (const Image & aImage, ImageOutput & output, const Point &starting, const ForegroundPredicate &predicate) const;

    /** 
     *  Compute the other steps of the separable distance transformation.
     * 
     * @param inputImage the image resulting of the first (or
     * intermediate) step 
     * @param output the output image 
     * @param dim the dimension to process
     */		
    void computeOtherSteps(const ImageOutput & inputImage, ImageOutput & output, const Dimension dim)const;

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
    void computeOtherStep1D (const ImageOutput & input, ImageOutput & output,
			     const Point &startingPoint, const Dimension dim, 
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

    ///Value to act as a +infinity value
    InternalValue myInfinity;

    ///Copy of the internal domain
    Domain myDomain;

  }; // end of class DistanceTransformation

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/nd/volumetric/DistanceTransformation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DistanceTransformation_h

#undef DistanceTransformation_RECURSES
#endif // else defined(DistanceTransformation_RECURSES)
