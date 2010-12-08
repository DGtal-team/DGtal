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
 * @file ImageContainerByVTKImageData.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/12/08
 *
 * Header file for module ImageContainerByVTKImageData.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ImageContainerByVTKImageData_RECURSES)
#error Recursive header files inclusion detected in ImageContainerByVTKImageData.h
#else // defined(ImageContainerByVTKImageData_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ImageContainerByVTKImageData_RECURSES

#if !defined ImageContainerByVTKImageData_h
/** Prevents repeated inclusion of headers. */
#define ImageContainerByVTKImageData_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/images/CValueType.h"
#include "DGtal/kernel/domains/CDomain.h"

#include <vtkImageData.h>
#include <vtkSmartPointer.h>

//////////////////////////////////////////////////////////////////////////////

//Forward declaration
class vtkImageData;

namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // template class ImageContainerByVTKImageData
  /**
   * Description of template class 'ImageContainerByVTKImageData' <p>
   * \brief Aim:
   */
  template <typename TDomain, typename TValueType>
  class ImageContainerByVTKImageData:  public vtkImageData
  {
    // ----------------------- Standard services ------------------------------
  public:
 
    BOOST_CONCEPT_ASSERT(( CValueType<TValueType> ));
    BOOST_CONCEPT_ASSERT(( CDomain<TDomain> ));
			
    typedef TValueType ValueType;
    typedef TDomain Domain;

    // static constants
    static const typename Domain::Dimension staticDimension = Domain::staticDimension;
    
    ///Only defined for 3D images
    BOOST_STATIC_ASSERT(staticDimension == 3);
    
    ///@todo IMPORTANT : Size must be int-compatible (see operator()(Point))

    typedef typename Domain::Point Point;
    typedef typename Domain::Vector Vector;
    typedef typename Domain::Dimension Dimension;
    typedef typename Domain::Integer Integer;
    typedef typename Domain::Size Size;
    //    typedef typename vector<ValueType>::iterator Iterator;
    //    typedef typename vector<ValueType>::const_iterator ConstIterator;

    /**
     * Constructor.
     */
    ImageContainerByVTKImageData(const Point &aPointA,
				 const Point &aPointB );

    
    /**
     * Destructor.
     */
    ~ImageContainerByVTKImageData();

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Get the value of an image at a given position.
     *
     * @param aPoint  position in the image.
     * @return the value at aPoint.
     */
    ValueType operator()(const Point &aPoint) const;
    
   
    /**
     * Set a value on an Image at aPoint.
     *
     * @param aPoint location of the point to associate with aValue.
     * @param aValue the value.
     */
    void setValue(const Point &aPoint, const ValueType &aValue);


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

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ImageContainerByVTKImageData();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ImageContainerByVTKImageData ( const ImageContainerByVTKImageData & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    ImageContainerByVTKImageData & operator= ( const ImageContainerByVTKImageData & other );

    // ------------------------- Internals ------------------------------------
  private:

    Point myLowerBound;
    Point myUpperBound;
    vtkSmartPointer<vtkImageData> myVTKImage;

  }; // end of class ImageContainerByVTKImageData


  /**
   * Overloads 'operator<<' for displaying objects of class 'ImageContainerByVTKImageData'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImageContainerByVTKImageData' to write.
   * @return the output stream after the writing.
   */
  template <typename TD, typename TV>
  std::ostream&
  operator<< ( std::ostream & out, const ImageContainerByVTKImageData<TD,TV> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/images//ImageContainerByVTKImageData.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ImageContainerByVTKImageData_h

#undef ImageContainerByVTKImageData_RECURSES
#endif // else defined(ImageContainerByVTKImageData_RECURSES)
