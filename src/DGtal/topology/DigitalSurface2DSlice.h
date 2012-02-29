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
 * @file DigitalSurface2DSlice.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/29
 *
 * Header file for module DigitalSurface2DSlice.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurface2DSlice_RECURSES)
#error Recursive header files inclusion detected in DigitalSurface2DSlice.h
#else // defined(DigitalSurface2DSlice_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurface2DSlice_RECURSES

#if !defined DigitalSurface2DSlice_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurface2DSlice_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <deque>
#include "DGtal/base/Common.h"
#include "DGtal/base/Circulator.h"
#include "DGtal/topology/CDigitalSurfaceContainer.h"
#include "DGtal/topology/CDigitalSurfaceContainer.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class DigitalSurface2DSlice
  /**
     Description of template class 'DigitalSurface2DSlice' <p>

     \brief Aim: Represents a 2-dimensional slice in a
     DigitalSurface. In a sense, it is a 4-connected contour, open or
     not. To be valide, it must be connected to some digital surface
     and a starting surfel.

     It provides the following inner types: 

     - ConstIterator
     - ConstReverseIterator
     - ConstCirculator
     - ConstReverseCirculator

     And the following (circular)iterator services: 

     - begin() : begin ConstIterator
     - end() : end ConstIterator
     - rbegin() : begin ConstReverseIterator
     - rend() : end ConstReverseIterator
     - c() : ConstCirculator
     - rc() : ConstReverseCirculator
     
     You can use these services to iterate over the elements. 

     @tparam TDigitalSurfaceContainer any model of
     CDigitalSurfaceContainer: the concrete representation chosen for
     the digital surface.

   */
  template <typename TDigitalSurfaceContainer>
  class DigitalSurface2DSlice
  {
  public:
    typedef TDigitalSurfaceContainer DigitalSurfaceContainer;
    BOOST_CONCEPT_ASSERT(( CDigitalSurfaceContainer<DigitalSurfaceContainer> ));

    typedef DigitalSurface2DSlice<DigitalSurfaceContainer> Self;
    typedef typename DigitalSurfaceContainer::KSpace KSpace;
    typedef typename DigitalSurfaceContainer::Cell Cell;
    typedef typename DigitalSurfaceContainer::SCell SCell;
    typedef typename DigitalSurfaceContainer::Surfel Surfel;
    typedef typename DigitalSurfaceContainer::Size Size;
    typedef DigitalSurface<DigitalSurfaceContainer> TheDigitalSurface;
    typedef typename DigitalSurfaceContainer::DigitalSurfaceTracker DigitalSurfaceTracker; 
    typedef std::deque<Surfel> Storage;
    typedef typename Storage::const_iterator ConstIterator;
    typedef typename Storage::const_reverse_iterator ConstReverseIterator;
    typedef Circulator<ConstIterator> ConstCirculator;
    typedef Circulator<ConstReverseIterator> ConstReverseCirculator;

    // struct ConstIterator {
    // public:
    //   /// The returned type for operator* on 'this'.
    //   typedef ConstIterator Self;
    //   typedef Surfel Value;

    //   // stl iterator types.
    //   typedef std::input_iterator_tag iterator_category;
    //   typedef Value value_type;
    //   typedef std::ptrdiff_t difference_type; 
    //   typedef const Value* pointer;
    //   typedef const Value& reference;

    // public:
    //   /// Destructor.
    //   inline ~ConstIterator() 
    //   {
    //     if ( myTracker ) delete myTracker;
    //   }

    // private:
    //   /// Constructor. Should not be used at any time.
    //   inline ConstIterator() 
    //     : myTracker( 0 )
    //   {}
      
    // public:
    //   /**
    //      Copy constructor. 
    //      @param other any other iterator.
    //   */
    //   inline ConstIterator( const Self & other ) 
    //     : myTracker( new DigitalSurfaceTracker( *other.myTracker ) ),
    //       myTrackingDir( other.myTrackingDir )
    //   {}

    //   /** 
    //       Assignment.
    //       @param other any other iterator (which may even point to a
    //       different surface).
    //       @return a reference to this.
    //   */
    //   inline Self & operator=( const Self & other ) 
    //   {
    //     if ( this != &other )
    //       {
    //         myTracker->move( other.myTracker->current() );
    //         myTrackingDir = other.myTrackingDir;
    //       }
    //     return *this;
    //   }

    //   /**
    //      Constructor.
    //      @param tracker any valid tracker on a digital surface (cloned
    //      in the iterator).
    //      @param i any direction different from the orthogonal
    //      direction to the current surfel.
    //   */
    //   inline ConstIterator( const DigitalSurfaceTracker & tracker, 
    //                         Dimension i )
    //     : myTracker( new DigitalSurfaceTracker( tracker ) ), myTrackingDir( i )
    //   {
    //     ASSERT( ( myTracker->surface().space().sOrthDir( myTracker->current() ) 
    //               != i ) && "[DGtal::DigitalSurface2DSlice<TDigitalSurfaceContainer>::ConstIterator::ConstIterator(...)] Tracking direction should not be the orthogonal direction of the current surfel." );
    //   }

    //   inline
    //   reference
    //   operator*() const
    //   {
    //     ASSERT( myTracker != 0 );
    //     ASSERT( ( myTrackingDir < KSpace::dimension )
    //             && "[DGtal::DigitalSurface2DSlice<TDigitalSurfaceContainer>::ConstIterator::operator*()]: you cannot dereferenced an end() iterator.");
    //     return myTracker->current();
    //   }

    //   inline
    //   pointer
    //   operator->() const
    //   { 
    //     ASSERT( myTracker != 0 );
    //     ASSERT( ( myTrackingDir < KSpace::dimension )
    //             && "[DGtal::DigitalSurface2DSlice<TDigitalSurfaceContainer>::ConstIterator::operator->()]: you cannot dereferenced an end() iterator.");
    //     return &( myTracker->current() );
    //   }

    //   /**
    //      Pre-increment operator. Moves to the next surfel or becomes end().
    //      @return a reference on 'this'.
    //   */
    //   inline Self& operator++()
    //   {
    //     ASSERT( myTracker != 0 );
    //     ASSERT( ( myTrackingDir < KSpace::dimension )
    //             && "[DGtal::DigitalSurface2DSlice<TDigitalSurfaceContainer>::ConstIterator::operator++()]: you cannot increment an end() iterator.");
    //     Surfel s;
    //     const KSpace & K = myTracker->surface().space();
    //     bool pos = K.sDirect( myTracker->current(), myTrackingDir );
    //     uint8_t code = myTracker->adjacent( s, myTrackingDir, pos );
    //     switch ( code )
    //       {
    //       case 0: // invalid move: iterator becomes end().
    //         myTrackingDir += KSpace::dimension;
    //         break;
    //       case 1: // 1 or 3, swap tracking dir and orthogonal dir.
    //       case 3: myTrackingDir = K.sOrthDir( myTracker->current() );
    //       case 2: myTracker->move( s ); // move in all 3 cases.
    //       }
    //     return *this;
    //   }

    //   inline
    //   Self
    //   operator++(int)
    //   {
    //     Self __tmp = *this;
    //     myVisitor->expand();
    //     return __tmp;
    //   }


    //   /**
    //      Equality operator.
    //      @param other any other iterator.
    //      @return 'true' whenever the iterators point on the same surfel or when both are invalid.
    //   */
    //   inline bool operator==( const Self & other ) const
    //   {
    //     if ( myTracker == 0 ) return other.myTracker == 0;
    //     return other.myTracker == 0 ? false
    //       : myTracker->current() == other.myTracker->current();
    //   }

    //   /// Current tracker or 0 if invalid.
    //   DigitalSurfaceTracker* myTracker;
    //   /// Tracking direction.
    //   Dimension myTrackingDir;
    // };
    // struct ConstCirculator {
    //   /// Current tracker or 0 if invalid.
    //   DigitalSurfaceTracker* myTracker;
    //   Dimension myTrackingDir;
    // };

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~DigitalSurface2DSlice();

    /**
       Constructor from tracker and Dimension. The current position of
       the tracker gives the starting surfel s. The 2D slice spans a
       2D-plane containing axes i and sOrthDir( s ).

       @param tracker a pointer on a tracker, which will be moved to
       build the slice. More precisely, if the contour is open, the
       tracker is moved to the first surfel when traversing in the
       direct orientation. If the contour is closed, the tracker is
       returned at the same position.
       
       @param i a direction different from the orthogonal direction of
       the current surfel of the tracker.

       @see init

       NB: O(n) complexity (at least) if n is the number of surfels of
       the slice.
     */
    DigitalSurface2DSlice( DigitalSurfaceTracker* tracker, Dimension i );

    /**
       Initializes the slice from a tracker \a tracker and a direction \a i.
      
       @param tracker a pointer on a tracker, which will be moved to
       build the slice. More precisely, if the contour is open, the
       tracker is moved to the first surfel when traversing in the
       direct orientation. If the contour is closed, the tracker is
       returned at the same position.
       
       @param i a direction different from the orthogonal direction of
       the current surfel of the tracker.

       @return 'true' if the initialization was ok, false otherwise
       (for instance if \a i is the orthogonal direction of the
       current surfel of the tracker.

       NB: O(n) complexity (at least) if n is the number of surfels of
       the slice.
    */
    bool init( DigitalSurfaceTracker* tracker, Dimension i );

    // The number of surfels of this slice.
    Size size() const;

    // ------------------------- iterator services ----------------------------
  public:
    
    /**
     * Iterator service.
     * @return begin iterator
     */
    ConstIterator begin() const;
    
    /**
     * Iterator service.
     * @return end iterator
     */
    ConstIterator end() const;
    
    /**
     * Iterator service.
     * @return rbegin iterator
     */
    ConstReverseIterator rbegin() const;
    
    /**
     * Iterator service.
     * @return rend iterator
     */
    ConstReverseIterator rend() const;
    
    /**
     * Circulator service.
     * @return a circulator
     */
    ConstCirculator c() const;
    
    /**
     * Circulator service.
     * @return a reverse circulator
     */
    ConstReverseCirculator rc() const;
    
    // ----------------------- Interface --------------------------------------
  public:

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
    /// The container that stores the surfels of this slice.
    std::deque<Surfel> mySurfels;
    /// Tells if the slice is closed (true) or open (false).
    bool myIsClosed;

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    DigitalSurface2DSlice();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    DigitalSurface2DSlice ( const DigitalSurface2DSlice & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    DigitalSurface2DSlice & operator= ( const DigitalSurface2DSlice & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class DigitalSurface2DSlice


  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurface2DSlice'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurface2DSlice' to write.
   * @return the output stream after the writing.
   */
  template <typename TDigitalSurfaceContainer>
  std::ostream&
  operator<< ( std::ostream & out, const DigitalSurface2DSlice<TDigitalSurfaceContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/DigitalSurface2DSlice.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurface2DSlice_h

#undef DigitalSurface2DSlice_RECURSES
#endif // else defined(DigitalSurface2DSlice_RECURSES)
