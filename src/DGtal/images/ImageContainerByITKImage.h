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
 * @file ImageContainerByITKImage.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Pierre Gueth (\c pierre.gueth@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @author Pablo Hernandez-Cerdan (\c pablo.hernandez.cerdan@outlook.com)
 *
 * @date 2013/10/23
 *
 * Header file for module ImageContainerByITKImage.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerByITKImage_RECURSES)
#error Recursive header files inclusion detected in ImageContainerByITKImage.h
#else // defined(ImageContainerByITKImage_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerByITKImage_RECURSES

#if !defined ImageContainerByITKImage_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerByITKImage_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/base/CLabel.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/images/DefaultConstImageRange.h"
#include "DGtal/images/DefaultImageRange.h"

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#endif
#include <itkImage.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageRegionIterator.h>
#include <iostream>
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

    /////////////////////////////////////////////////////////////////////////////
    // template class ImageContainerByITKImage
    /**
     * Description of template class 'ImageContainerByITKImage' <p>
     * \brief Aim: implements a model of CImageContainer using a ITK Image.
     *
     * Using this container, you can switch from DGtal alogrithms to
     * ITK processing pipeline.
     * The Ownership of the underlying ITK image is shared between the wrapper
     * and the ITK pipeline.
     * If the ITK image region is modified, one should manually update the domain of the wrapper.
     * This is done by calling the updateDomain() method.
     *
     * \see testITKImage.cpp
     */
    template <concepts::CDomain TDomain, concepts::CLabel TValue>
    class ImageContainerByITKImage
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef TValue Value;
      typedef TDomain Domain;
      typedef ImageContainerByITKImage<TDomain, TValue> Self;

      // static constants
      static const typename Domain::Dimension dimension = Domain::dimension;

      typedef typename Domain::Point Point;
      typedef typename Domain::Vector Vector;
      typedef typename Domain::Dimension Dimension;
      typedef typename Domain::Integer Integer;
      typedef typename Domain::Size Size;
      typedef Point Vertex;
      typedef PointVector<dimension, double> RealPoint;
      typedef RealPoint PhysicalPoint;

      typedef typename itk::Image< TValue, dimension> ITKImage;
      typedef typename itk::ImageBase<dimension>::SpacingValueType ITKSpacingValueType;
      typedef RealPoint ImageSpacing;
      typedef typename ITKImage::Pointer ITKImagePointer;
      typedef typename ITKImage::PixelContainer Container;
      typedef typename itk::ImageRegionConstIterator< ITKImage > ConstIterator;
      typedef typename itk::ImageRegionIterator< ITKImage > Iterator;

      typedef DefaultConstImageRange<Self> ConstRange;
      typedef DefaultImageRange<Self> Range;

      /**
       * Constructor.
       *
       * @param aDomain the image domain.
       */
      ImageContainerByITKImage(const Domain& aDomain);

      /**
       * Constructor.
       *
       * @param aRef a reference to an ITKImage
       */
      ImageContainerByITKImage(const ITKImagePointer &aRef);

      /**
       * Copy constructor
       *
       * @param other the object to copy.
       *
       */
      ImageContainerByITKImage(const ImageContainerByITKImage& other);

      /**
       * Assignment.
       *
       * @param other the object to copy.
       * @return a reference on 'this'.
       */
      ImageContainerByITKImage & operator=(const ImageContainerByITKImage & other);

      /**
       * Destructor.
       */
      ~ImageContainerByITKImage();

      // ----------------------- Interface --------------------------------------
    public:

      /**
       *
       * update internal domain cache.
       * should be called after modifying underlying ITK image or to
       * set myDomainShift.
       *
       * @param inputDomainShift applies a domainShift to the lowerBound and
       * upperBound of myDomain. @sa myDomainShift
       *
       */
      void updateDomain(const Point & inputDomainShift = Point());

      /**
       * @return the range providing begin and end
       * iterators to scan the values of image.
       */
      ConstRange constRange() const
      {
          return ConstRange(*this);
      }

      /**
       * @return the range providing begin and end
       * iterators to scan the values of image.
       */
      Range range()
      {
          return Range(*this);
      }

      /**
       * Give access to the underlying container.
       * @return a (might be const) reference to the container.
      */
      const Container & container() const
      {
        return *(myITKImagePointer->GetPixelContainer());
      }
      /**
       * Give access to the underlying container.
       * @return a (might be const) reference to the container.
       */
      Container & container()
      {
        return *(myITKImagePointer->GetPixelContainer());
      }

      /**
       * Get the value of an image at a given position.
       *
       * @param domainPoint  position in the image.
       * @return the value at indexPoint.
       */
      Value operator()(const Point &domainPoint) const;

      /**
       * Get the value of an image at a given position.
       *
       * @param it  position in the image.
       * @return the value of the point pointed by the iterator.
       */
      Value operator()(const ConstIterator &it) const;

      /**
       * Get the value of an image at a given position.
       *
       * @param it  position in the image.
       * @return the value of the point pointed by the iterator.
       */
      Value operator()(const Iterator &it) const;

      /**
       * Set a value on an Image at domainPoint.
       *
       * @param domainPoint location of the point to associate with aValue.
       * @param aValue the value.
       */
      void setValue(const Point &domainPoint, const Value &aValue);

      /**
       * Set a value on an Image at a given position
       *
       * @param it location of the point (Iterator) to associate with aValue.
       * @param V the value.
       */
      void setValue(Iterator &it, const Value &V);

      // ------------------------- methods ------------------------------


      /**
       * @return the domain associated to the image.
       */
      const Domain& domain() const
      {
          return myDomain;
      }

      /**
       * Returns a copy of the itkImage smartPointer
       */
      inline
      ITKImagePointer getITKImagePointer() const
      {
          return myITKImagePointer;
      }

      inline
      const Point & getDomainShift() const
      {
          return myDomainShift;
      }

      // ------------------------- stream ------------------------------

      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay ( std::ostream & out ) const;

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const;

      // ------------------------- Iterators ------------------------------
      /**
       * begin() const iterator.
       *
       **/
      inline
      ConstIterator begin() const
      {
          ConstIterator iter = ConstIterator(myITKImagePointer, myITKImagePointer->GetLargestPossibleRegion());
          iter.GoToBegin();
          return iter;
      }

      /**
       * begin() const iterator.
       *
       **/
      inline
      Iterator begin()
      {
          Iterator iter = Iterator(myITKImagePointer, myITKImagePointer->GetLargestPossibleRegion());
          iter.GoToBegin();
          return iter;
      }

      /**
       * end() const iterator.
       *
       **/
      inline
      const ConstIterator end() const
      {
          ConstIterator iter = ConstIterator(myITKImagePointer, myITKImagePointer->GetLargestPossibleRegion());
          iter.GoToEnd();
          return iter;
      }

      /**
       * end()  iterator.
       *
       **/
      inline
      Iterator end()
      {
          Iterator iter = Iterator(myITKImagePointer, myITKImagePointer->GetLargestPossibleRegion());
          iter.GoToEnd();
          return iter;
      }

      // ----------------- DomainPoint to/from IndexPoint interface -------------

      /**
       * IndexPoint refers to a valid ITKImage::IndexPoint
       * DomainPoint refers to a Point between lowerBound and upperBound of myDomain.
       * They are different only when myDomainShift is different than Zero.
       *
       * @param indexPoint a point holding valid index coordinates of the ITK image.
       * @return domainPoint a point between lowerBound and upperBound
       */
      inline Point getDomainPointFromIndex(const Point &indexPoint) const;

      inline Point getIndexFromDomainPoint(const Point &domainPoint) const;

      /**
       * The same as @ref getDomainPointFromItkIndex and @ref getIndexFromDomainPoint
       * but using ITK types.
       *
       * @param itkIndexPoint an IndexType of ITK.
       * @return domainPoint a point between lowerBound and upperBound
       */
      inline Point getDomainPointFromItkIndex(const typename ITKImage::IndexType &itkIndexPoint) const;

      inline typename ITKImage::IndexType getItkIndexFromDomainPoint(const Point &domainPoint) const;


      // ------------------------- PhysicalPoint interface ----------------------

      /**
       * Get PhysicalPoints from a domain point in DGtal and viceversa.
       *
       * Remember that GetOrigin() in ITK is the physical location of
       * the index {0,0...}. Not the location of the start index of the region.
       *
       * @param domainPoint a point holding a point in the DGtal domain.
       * It will be converted to a valid index of the ITK image, taking into
       * account the value of myDomainShift.
       * @return physical point of the index.
       */
      inline PhysicalPoint getPhysicalPointFromDomainPoint(const Point &domainPoint) const;

      inline Point getDomainPointFromPhysicalPoint(const PhysicalPoint &physicalPoint) const;

      /**
       * Returns the lower and upper bounds as physical points.
       * Note that the location of pixels in ITK are in the center of
       * the square.
       * But in DGtal a lowerBound is in the south-west
       * corner of that square.
       * And an upperBound is the north-east corner of the last pixel.
       *
       * @return physical point of the location of the start index of the region.
       */
      inline PhysicalPoint getLowerBoundAsPhysicalPoint() const;
      inline PhysicalPoint getUpperBoundAsPhysicalPoint() const;
      /**
       * Get the image spacing specific to the ITK Image.
       * @return spacing values for each dimension of the image.
       */
      inline ImageSpacing getImageSpacing() const;
      /**
       * Set the image spacing specific to the ITK Image.
       * @param s an image spacing point representing the spacing values of each dimension of the space.
       */
      inline void setImageSpacing(const ImageSpacing& s) const;

      // ------------------------- Private Datas --------------------------------
    private:

      // ------------------------- Hidden services ------------------------------
    protected:

      /**
       * Constructor.
       * Forbidden by default (protected to avoid g++ warnings).
       */
      ImageContainerByITKImage();

      // ------------------------- Internals ------------------------------------
    private:

      ITKImagePointer myITKImagePointer;
      Domain myDomain; // cached from myITKImagePointer region. updated when calling update().

      /**
       * Apply a shift in the lower and upper bounds.
       * Useful to represent multiple images, or images with physical information,
       * i.e with non-default Origin (see ITK for Origin, Spacing and Direction metadata)
       *
       * Set it using the function updateDomain(inputDomainShift)
       *
       * Please note that this is a workaround, DGtal cannot fully work in the Physical Domain. The spacing in DGtal is fixed to 1.
       * This allows to work to represent multiple images with different Origins,
       * but they need to have the same Spacing and Direction for the visualization to be meaningful.
       *
       * Default to zero.
       *
       * When is not zero, use @ref getIndexFromDomainPoint and @ref getDomainPointFromIndex
       * to switch between points in the domain and indices in the itk image.
       */
      Point myDomainShift = Point();
    }; // end of class ImageContainerByITKImage

  /**
   * Overloads 'operator<<' for displaying objects of class 'ImageContainerByITKImage'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImageContainerByITKImage' to write.
   * @return the output stream after the writing.
   */
  template <typename T, typename TV>
  std::ostream&
  operator<< ( std::ostream & out, const ImageContainerByITKImage<T, TV> & object );

}
///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/images/ImageContainerByITKImage.ih"

//
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerByITKImage_h

#undef ImageContainerByITKImage_RECURSES
#endif // else defined(ImageContainerByITKImage_RECURSES)
