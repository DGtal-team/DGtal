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
 * @file ArrayImageView.h
 * @author Roland Denis (\c roland.denis@univ-smb.fr )
 * LAboratory of MAthematics - LAMA (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/06/19
 *
 * This file is part of the DGtal library.
 */

#if defined(ArrayImageView_RECURSES)
#error Recursive header files inclusion detected in ArrayImageView.h
#else // defined(ArrayImageView_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArrayImageView_RECURSES

#if !defined ArrayImageView_h
/** Prevents repeated inclusion of headers. */
#define ArrayImageView_h

#if __cplusplus >= 201103L

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <boost/assert.hpp>
#include <boost/static_assert.hpp>
#include <boost/concept/assert.hpp>
#include <boost/iterator/iterator_concepts.hpp>
#include <iterator>
#include <type_traits>

#include <DGtal/images/CConstImage.h>
#include "ImageViewIterator.h"
#include "IteratorCompletion.h"
#include "Linearizer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  /** 
   * @brief Aim: Image view for generic arrays.
   *
   * Description of template class 'ImageViewIterator' <p>
   * This creates an image (concepts::CImage compatible) given a random-access iterator 
   * that spans an array of data, and two domains:
   * - the definition (full) domain whose size is equal to the data size.
   * - the viewable domain, a subset of the full-domain, on which the image is accessible.
   *
   * The available iterators for this image can return the corresponding point and are
   * faster than using an iterator over the domain (see ImageViewIterator). 
   * Reverse iterators and ranges are defined in the inherited class IteratorCompletion.
   *
   * Some helpers are available (see make_ArrayImageViewFromIterator and make_ArrayImageViewFromImage)
   * for easy construction (with template deduction) from an iterator or an CConstImage model.
   *
   * The following code snippet demonstrates how to use ArrayImageView from converting a C-array to an image:
   * @code
   * using Space = SpaceND<2>;
   * using Domain = HyperRectDomain<Space>;
   * using Point = typename Space::Point;
   * using Value = double;
   *
   * const Domain domain{ {0, 1}, {4, 3} };
   *
   *
   * Value* data = new Value[ domain.size() ];
   *
   * // Convert this allocated memory to a CImage model. 
   * ArrayImageView< Value*, Domain > image( data, domain );
   * // Alternative syntax using the helpers:
   * // auto image = make_ArrayImageViewFromIterator( data, domain );
   * 
   * // Fill the image with first coordinate of the point
   * for ( auto it = image.begin(); it != image.end(); ++it )
   *   {
   *     *it = it.getPoint()[0];
   *   }
   *
   * // Get a constant view on a sub-domain.
   * const Domain sub_domain{ {1, 1}, {3, 2} };
   * ArrayImageView< Value const*, Domain > cst_image( data, domain, sub_domain );
   * // Alternative syntax using the helpers:
   * // auto cst_image = make_ArrayImageViewFromImage( image, sub_domain );
   *
   * // Display it.
   * for ( auto value : cst_image )
   *   {
   *     std::cout << value << " ";
   *   }
   * std::cout << std::endl;
   *
   * @endcode
   *
   * @remark The given random-access iterator can be either mutable or constant.
   *
   * @warning The array must be column-major ordered.
   * @warning The domain must be an HyperRectDomain.
   * @warning C++11 needs to be enable in order to use this class.
   *
   * @tparam TArrayIterator Type of a random-access iterator over the datas (can be a T* pointer).
   * @tparam TDomain  Type of the domain (must be an HyperRectDomain).
   *
   *
   */
  template < 
    typename TArrayIterator,
    typename TDomain
  >
  class ArrayImageView;

  template <
    typename TArrayIterator,
    typename TSpace
  >
  class ArrayImageView< TArrayIterator, HyperRectDomain<TSpace> >
      : public IteratorCompletion< ArrayImageView< TArrayIterator, HyperRectDomain<TSpace> > >
    {

    // Checks Random-access iterator concept on TArrayIterator
    BOOST_CONCEPT_ASSERT( (boost_concepts::RandomAccessTraversalConcept<TArrayIterator>) );

    public:
      // Typedefs
      using Self = ArrayImageView<TArrayIterator, HyperRectDomain<TSpace> >;
      using ArrayIterator = TArrayIterator;
      using Value = typename std::iterator_traits<ArrayIterator>::value_type;
      using Reference = typename std::iterator_traits<ArrayIterator>::reference;
      using ConstReference = const Reference;

      using Domain = HyperRectDomain<TSpace>;
      using Point = typename Domain::Point;
      using Linearizer = DGtal::Linearizer<Domain, ColMajorStorage>; ///< Linearization of the points.

      // Iterators & Ranges
      template <class> friend class ImageViewIterator;
      using Iterator = typename IteratorCompletionTraits<Self>::Iterator; ///< Mutable iterator.
      using ConstIterator = typename IteratorCompletionTraits<Self>::ConstIterator; ///< Constant iterator.

      /** Default constructor.
       *
       * Empty allocated memory on empty domains.
       */
      ArrayImageView()
        : myArrayIterator{nullptr}
        , myFullDomain{}
        , myViewDomain{}
        {}

      /** Constructor from iterator, full domain and viewable domain.
       *
       * @param anArrayIterator   A random-access iterator on the datas.
       * @param aFullDomain       The domain span by the given iterator.
       * @param aViewDomain       The viewable domain of this image.
       */
      ArrayImageView( ArrayIterator anArrayIterator, Domain const& aFullDomain, Domain const& aViewDomain )
          : myArrayIterator(anArrayIterator)
          , myFullDomain{ aFullDomain }
          , myViewDomain{ aViewDomain }
        {
          BOOST_ASSERT_MSG(
                 aFullDomain.lowerBound().isLower( aViewDomain.lowerBound() )
              && aFullDomain.upperBound().isUpper( aViewDomain.upperBound() ),
              "The viewable domain must be included into the full domain."
          );
        }

      /** Constructor from iterator and full domain.
       *
       * The viewable domain is then the full domain.
       *
       * @param anArrayIterator   A random-access iterator on the datas.
       * @param aFullDomain       The domain span by the given iterator.
       */
      ArrayImageView( ArrayIterator anArrayIterator, Domain const& aFullDomain )
          : ArrayImageView( anArrayIterator, aFullDomain, aFullDomain )
        {
        }

      /** Copy constructor with other viewable domain.
       *
       * @param other         An another ArrayImageView instance.
       * @param aViewDomain   A new viewable domain for this image.
       */
      ArrayImageView( Self const& other, Domain const& aViewDomain )
          : ArrayImageView( other.myArrayIterator, other.myFullDomain, aViewDomain )
        {}

      /**
       * @return the image viewable domain.
       */
      inline
      Domain domain() const
        {
          return myViewDomain;
        }

      /**
       * @return the full domain where the allocated memory is defined.
       */
      inline
      Domain fullDomain() const
        {
          return myFullDomain;
        }
      
      /** Reads a value given a point lying inside the full domain.
       *
       * @param[in] aPoint  The point.
       * @return a constant value.
       */
      inline
      Value getValue( Point const& aPoint ) const
        {
          BOOST_ASSERT_MSG(
              myFullDomain.isInside(aPoint),
              "The point is outside the full domain."
          );

          return myArrayIterator[ Linearizer::getIndex(aPoint, myFullDomain) ];
        }

      /** Sets a value given a point lying inside the full domain.
       *
       * @param[in] aPoint  The point.
       * @param[in] aValue  The value.
       */
      inline
      void setValue( Point const& aPoint, Value aValue )
        {
          BOOST_ASSERT_MSG(
              myFullDomain.isInside(aPoint),
              "The point is outside the full domain."
          );

          myArrayIterator[ Linearizer::getIndex(aPoint, myFullDomain) ] = aValue;
        }

      /** Reads a value given a point lying inside the full domain.
       *
       * @param[in] aPoint  The point.
       * @return a constant value.
       */
      inline
      Value operator() ( Point const& aPoint ) const
        {
          return getValue(aPoint);
        }

      /**
       * @return a mutable iterator pointing to the lower bound of the viewable domain.
       */
      inline
      Iterator begin()
        {
          return Iterator{ this, myFullDomain, myViewDomain };
        }
      
      /**
       * @return a constant iterator pointing to the lower bound of the viewable domain.
       */
      inline
      ConstIterator begin() const
        {
          return ConstIterator{ this, myFullDomain, myViewDomain };
        }
      
      /**
       * @return a constant iterator pointing to the lower bound of the viewable domain (C++11).
       */
      inline
      ConstIterator cbegin() const
        {
          return ConstIterator{ this, myFullDomain, myViewDomain };
        }

      /**
       * @return an mutable iterator pointing after the upper bound of the viewable domain.
       */
      inline
      Iterator end()
        {
          return Iterator{ this, myFullDomain, myViewDomain, true };
        }
      
      /**
       * @return a constant iterator pointing after the upper bound of the viewable domain.
       */
      inline
      ConstIterator end() const
        {
          return ConstIterator{ this, myFullDomain, myViewDomain, true };
        }
      
      /**
       * @return a constant iterator pointing after the upper bound of the viewable domain (C++11).
       */
      inline
      ConstIterator cend() const
        {
          return ConstIterator{ this, myFullDomain, myViewDomain, true };
        }
        

    public: // Should be private since ImageViewIterator is a friend but g++ 4.9.1 don't care ... (no prob with clang++ 3.5.0)

      /** Dereference of a mutable iterator.
       *
       * @param[in] aPoint      A point lying in the full domain (unused).
       * @param[in] aFullIndex  Linearized index of the point.
       * @return a mutable reference to the value associated to the point.
       */
      inline
      Reference dereference( Point const& /* aPoint */, typename Point::Coordinate aFullIndex )
        {
          BOOST_ASSERT_MSG(
              aFullIndex >= 0 && static_cast<typename Domain::Size>(aFullIndex) < myFullDomain.size(),
              "linearized index out of bounds !"
          );
          return myArrayIterator[aFullIndex];
        }

      /** Dereference of a constant iterator.
       *
       * @param[in] aPoint      A point lying in the full domain (unused).
       * @param[in] aFullIndex  Linearized index of the point.
       * @return a constant reference to the value associated to the point.
       */
      inline
      ConstReference dereference( Point const& /* aPoint */, typename Point::Coordinate aFullIndex ) const
        {
          BOOST_ASSERT_MSG(
              aFullIndex >= 0 && static_cast<typename Domain::Size>(aFullIndex) < myFullDomain.size(),
              "linearized index out of bounds !"
          );
          return myArrayIterator[aFullIndex];
        }

    // ----------------------- Interface --------------------------------------
    public:
      /**
       * Writes/Displays the object on an output stream.
       * @param out the output stream where the object is written.
       */
      void selfDisplay ( std::ostream & out ) const
        {
          out << "[ArrayImageView] with full domain " << myFullDomain << " and viewable domain " << myViewDomain;
        }

      /**
       * Checks the validity/consistency of the object.
       * @return 'true' if the object is valid, 'false' otherwise.
       */
      bool isValid() const
        {
          return true;
        }

    // ------------------------- Private Datas --------------------------------
    private:
      ArrayIterator myArrayIterator; ///< Pointer to the allocated memory.
      Domain myFullDomain;  ///< Definition (full) domain.
      Domain myViewDomain;  ///< Viewable domain.

    }; // end of class ArrayImageView

  /** Iterator traits specialized for ArrayImageView.
   *
   * \see IteratorCompletion
   */
  template <
    typename TArrayIterator,
    typename TDomain
  >
  class IteratorCompletionTraits< ArrayImageView<TArrayIterator, TDomain> >
    {
    public:
      using Self = ArrayImageView<TArrayIterator, TDomain>;
      using Iterator = ImageViewIterator<Self>; ///< Mutable iterator.
      using ConstIterator = ImageViewIterator<const Self>; ///< Constant iterator.

      /** Functor that returns the distance between the domain's lower bound and a given point.
       *
       * \see SimpleRandomAccessRangeFromPoint and SimpleRandomAccessConstRangeFromPoint.
       */
      class DistanceFunctor
        {
        public:
          using Domain = typename Self::Domain;
          using Point = typename Self::Point;
          using Difference = typename Self::Difference;

          DistanceFunctor( Self const* anImageView )
            : myDomain( anImageView->domain() )
            {}

          Difference operator() ( Point const& aPoint ) const
            {
              BOOST_ASSERT_MSG(
                  myDomain.isInside(aPoint),
                  "The point is outside the domain !"
              );
              return Linearizer<Domain, ColMajorStorage>::getIndex( aPoint, myDomain );
            }

        private:
          const Domain myDomain; ///< Stored domain to avoid iterator corruption if domain changesd.
        };

    }; // end of specialized class IteratorCompletionTraits

  /**
   * Overloads 'operator<<' for displaying objects of class 'ImageViewIterator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ImageViewIterator' to write.
   * @return the output stream after the writing.
   */
  template <
    typename TArrayIterator,
    typename TDomain
  >
  std::ostream&
  operator<< ( std::ostream & out, const ArrayImageView<TArrayIterator, TDomain> & object )
    {
      object.selfDisplay( out );
      return out;
    }
  
  
  // ------------------ ArrayImageView construction helpers ----------------
  
  /** Returns an ArrayImageView from an iterator, a full domain and a viewable domain.
   *
   * @param anArrayIterator   A random-access iterator on the datas.
   * @param aFullDomain       The domain span by the given iterator.
   * @param aViewDomain       The viewable domain of this image.
   * @return an ArrayImageView instance.
   */
  template <
    typename TArrayIterator,
    typename TDomain
  >
  ArrayImageView< TArrayIterator, TDomain >
  make_ArrayImageViewFromIterator( TArrayIterator anArrayIterator, TDomain const& aFullDomain, TDomain const& aViewDomain )
    {
      return { anArrayIterator, aFullDomain, aViewDomain };
    }
  
  /** Returns an ArrayImageView from an iterator and a full domain.
   *
   * The viewable domain will be the same as the full domain.
   *
   * @param anArrayIterator   A random-access iterator on the datas.
   * @param aFullDomain       The domain span by the given iterator.
   * @return an ArrayImageView instance.
   */
  template <
    typename TArrayIterator,
    typename TDomain
  >
  ArrayImageView< TArrayIterator, TDomain >
  make_ArrayImageViewFromIterator( TArrayIterator anArrayIterator, TDomain const& aFullDomain )
    {
      return { anArrayIterator, aFullDomain, aFullDomain };
    }

  /** Returns an ArrayImageView from an image and a viewable domain.
   *
   * @param anImage       The image that models the CConstImage concept.
   * @param aViewDomain   The viewable domain of this image.
   */
  template <
    typename TImage,
    typename TDomain = typename TImage::Domain
  >
  // We use decltype on begin() iterator because it returns the constant iterator
  //  if the image is constant while ::Iterator typedef returns the mutable iterator.
  ArrayImageView< decltype( ((TImage*)nullptr)->begin() ), TDomain >
  make_ArrayImageViewFromImage( TImage & anImage, TDomain const& aViewDomain )
    {
      // Remove constness because CConstImage requires assignability.
      BOOST_CONCEPT_ASSERT( (DGtal::concepts::CConstImage< typename std::remove_const<TImage>::type >) );
      
      return { anImage.begin(), anImage.domain(), aViewDomain };
    }
  
  /** Returns an ArrayImageView from an image.
   *
   * The viewable domain will be the same as the given image domain.
   *
   * @param anImage       The image that models the CConstImage concept.
   */
  template <
    typename TImage,
    typename TDomain = typename TImage::Domain
  >
  // We use decltype on begin() iterator because it returns the constant iterator
  //  if the image is constant while ::Iterator typedef returns the mutable iterator.
  ArrayImageView< decltype( ((TImage*)nullptr)->begin() ), TDomain >
  make_ArrayImageViewFromImage( TImage & anImage )
    {
      // Remove constness because CConstImage requires assignability.
      BOOST_CONCEPT_ASSERT( (DGtal::concepts::CConstImage< typename std::remove_const<TImage>::type >) ); 

      return { anImage.begin(), anImage.domain(), anImage.domain() };
    }

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////

#endif // __cplusplus >= 201103L

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArrayImageView_h

#undef ArrayImageView_RECURSES
#endif // else defined(ArrayImageView_RECURSES)

/* GNU coding style */
/* vim: set ts=2 sw=2 expandtab cindent cinoptions=>4,n-2,{2,^-2,:2,=2,g0,h2,p5,t0,+2,(0,u0,w1,m1 : */
