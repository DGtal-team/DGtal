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
 * @file ConstImageAdapter.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/07
 *
 * Header file for module ConstImageAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConstImageAdapter_RECURSES)
#error Recursive header files inclusion detected in ConstImageAdapter.h
#else // defined(ConstImageAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstImageAdapter_RECURSES

#if !defined ConstImageAdapter_h
/** Prevents repeated inclusion of headers. */
#define ConstImageAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/ConstRangeFromPointAdapter.h"
#include "DGtal/base/CLabel.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/kernel/domains/CDomain.h"
#include "DGtal/images/CConstImage.h"

#include <iostream>

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstImageAdapter
  /**
   * Description of template class 'ConstImageAdapter' <p>
   * \brief Aim: implements a model of CConstImage
   * that adapts an underlying image. 
   *
   * @tparam TImage a model of CConstImage
   *
   * The values associated to the points are adapted  
   * with a functor f given at construction so that 
   * operator() calls f(img(aPoint)), instead of calling directly 
   * operator() of the underlying image img
   *
   * @tparam TFunctor the type of functor that transforms
   * the value into another one
   *
   * @tparam TValue type of the value returned by the underlying functor. 
   *
   * Here is the construction of a simple image adapter that 
   * is a thresholded view of the initial scalar image: 
   *
   * @snippet images/exampleConstImageAdapter.cpp ConstImageAdapterConstruction 
   *
   * NB: the underlying image as well as the functor
   * are stored in the adapter as aliasing pointer
   * in order to avoid copies.  
   * The pointed objects must exist and must not be deleted 
   * during the use of the adapter
   *
   * @see exampleConstImageAdapter
   */
  template <typename TImage, typename TFunctor, typename TValue>
  class ConstImageAdapter
  {
    // ----------------------- Types definitions ------------------------------
  public:

    typedef TImage Image; 
    BOOST_CONCEPT_ASSERT(( CConstImage<Image> ));

    typedef typename Image::Domain Domain; 
    BOOST_CONCEPT_ASSERT(( CDomain<Domain> ));

    typedef TValue Value;
    BOOST_CONCEPT_ASSERT(( CLabel<TValue> ));

    BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor,typename Image::Value,TValue> )); 

    typedef typename Domain::Point Point;
    typedef typename Domain::Vector Vector;
    typedef typename Domain::Dimension Dimension;
    typedef typename Domain::Integer Integer;
    typedef typename Domain::Size Size;

    // static constants
    static const Dimension dimension = Domain::dimension;

    typedef typename Image::ConstRange ImageRange; 
    typedef ConstRangeFromPointAdapter<ImageRange, TFunctor, TValue> ConstRange; 

    // ----------------------- Standard services ------------------------------
  public:
    /**
     * Constructor.
     * @param aImg any image 
     * @param aF any functor
     */
    ConstImageAdapter(const Image &aImg, const TFunctor &aF)
      : myImg(&aImg), myF(&aF), myR( new ImageRange( aImg.constRange() ) ) {}

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ConstImageAdapter ( const ConstImageAdapter & other )
      : myImg(other.myImg), myF(other.myF), myR( new ImageRange(*other.myR) ) {}

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ConstImageAdapter & operator= ( const ConstImageAdapter & other )
    {
      if (this != &other)
	{
	  myImg = other.myImg; 
	  myF = other.myF;
	  delete myR; 
	  myR = new ImageRange( *other.myR ); 
	}
      return *this; 
    }


    /**
     * Destructor.
     */
    ~ConstImageAdapter() { delete myR; }

    // ----------------------- Interface --------------------------------------
  public:


    /**
     * Get the value of an image at a given position.
     *
     * @param aPoint  position in the image.
     * @return the value at aPoint.
     */
    Value operator()(const Point &aPoint) const
    {
      return myF->operator()(myImg->operator()(aPoint)); 
    }


    // ------------------------- methods ------------------------------

    /**
     * @return the domain associated to the image.
     */
    const Domain& domain() const
    {
      return myImg->domain();
    }

    /**
     * @return the range that can be used 
     * to iterate over the values of the image.
     */
    ConstRange constRange() const
    {
      return ConstRange(*myR, *myF);
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const 
    { 
      return true; 
    }

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const 
    {
      out << *myImg; 
    }

    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:


  private:


    // ------------------------- Internals ------------------------------------
  private:

    /**
     * Aliasing pointer on the underlying image
     */
    const Image* myImg; 

    /**
     * Aliasing pointer on the underlying functor
     */
    const TFunctor* myF; 

    /**
     * Owning pointer on the range of the image values
     * (stored to be able to use the range adapter, 
     * which is light and requires that the range 
     * to adapt exists during the life of the adapter)
     */
    const ImageRange* myR; 

  }; // end of class ConstImageAdapter

  /**
   * Overloads 'operator<<' for displaying objects of class 'ConstImageAdapter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ConstImageAdapter' to write.
   * @return the output stream after the writing.
   */
  template <typename TImage, typename TFunctor, typename TValue>
  std::ostream&
  operator<< ( std::ostream & out, const ConstImageAdapter<TImage, TFunctor, TValue> & object );

}
///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/images/ConstImageAdapter.ih"

//
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConstImageAdapter_h

#undef ConstImageAdapter_RECURSES
#endif // else defined(ConstImageAdapter_RECURSES)
