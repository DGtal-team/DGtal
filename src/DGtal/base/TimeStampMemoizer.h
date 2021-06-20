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
 * @file TimeStampMemoizer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2021/06/17
 *
 * Header file for class TimeStampMemoizer
 *
 * This file is part of the DGtal library.
 */

#if defined(TimeStampMemoizer_RECURSES)
#error Recursive header files inclusion detected in TimeStampMemoizer.h
#else // defined(TimeStampMemoizer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define TimeStampMemoizer_RECURSES

#if !defined TimeStampMemoizer_h
/** Prevents repeated inclusion of headers. */
#define TimeStampMemoizer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <unordered_map>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class TimeStampMemoizer
  /**
   * Description of template class 'TimeStampMemoizer' <p> \brief Aim:
   * A generic class to store costly computations. A computation is an
   * association betweena key and a value, together they form an item
   * to be memorized. A maximal number of memoized items is
   * given. Each time a query is made, if the item was memoized, the
   * result is returned while the timestamp of the item is
   * updated. User can also add/update a value in the memoizer. When
   * the maximal number of items is reached, at least the oldest half
   * (or a fraction) of the items are deleted.
   *
   * @tparam TKey the type used for keys, must be hashable.
   *
   * @tparam TValue the type used for values, must be
   * DefaultConstructible, CopyConstructible, Assignable.
   */
  template <typename TKey, typename TValue>
  class TimeStampMemoizer
  {
  public:
    typedef TKey   Key;
    typedef TValue Value;
    typedef std::size_t     Size;
    typedef DGtal::uint32_t TimeStamp;
    typedef std::pair< Value, TimeStamp > StoredValue;
    
    // ----------------------- Standard services ------------------------------
  public:
    
    /**
       Constructor.
     */
    TimeStampMemoizer( Size max_size = 0, double ratio = 0.5 )
      : myMaxSize( max_size ), myRatio( ratio ), myTimeStamp( 0 ),
        myMap( max_size ), myHits( 0 )
    {}

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    TimeStampMemoizer( const TimeStampMemoizer & other ) = default;
    /**
     * Move constructor.
     * @param other the object to clone.
     */
    TimeStampMemoizer( TimeStampMemoizer && other ) = default;
    /**
     * Assignment.
     * @param other the object to copy.
     */
    TimeStampMemoizer& operator=( const TimeStampMemoizer & other ) = default;
    /**
     * Move assignment.
     * @param other the object to copy.
     */
    TimeStampMemoizer& operator=( TimeStampMemoizer && other ) = default;

    /**
     * Destructor.
     */
    ~TimeStampMemoizer() = default;

    // ----------------------- Memoization services -----------------------------
  public:

    /// Given a \a key, return the associated pair <value, true> if it
    /// is found, or return <dummy, false> where dummy is an arbitrary value.
    std::pair< Value, bool > get( const Key& key )
    {
      auto it = myMap.find( key );
      if ( it == myMap.end() )
        return std::make_pair( Value(), false );
      it->second.second = myTimeStamp++;
      ++myHits;
      return std::make_pair( it->second.first, false );
    }
    
    /// Memoizes (or update) a pair \a key and \a value.
    void set( const Key& key, const Value& value )
    {
      if ( myMap.size() >= myMaxSize ) cleanUp();
      myMap[ key ] = std::make_pair( value, myTimeStamp++ );
    }

    /// Clean-up the memoizer by removing a fraction of its oldest elements.
    void cleanUp()
    {
      selfDisplay( std::cout );
      Size nb = 0;
      TimeStamp threshold =
        std::max( (Size) myTimeStamp
                  - (Size) round( (myMaxSize + 0.5 * myHits) * myRatio ),
                  (Size) 0 );
      for ( auto it = myMap.begin(), itE = myMap.end(); it != itE; )
        if ( it->second.second <= threshold )
          {
            it = myMap.erase( it );
            ++nb;
          }
        else
          ++it;
      std::cout << " " << nb << " erased." << std::endl;
      myHits   = 0;
    }
    
    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[TimeStampMemoizer " << myMap.size() << "/" << myMaxSize << " items"
          << " time=" << myTimeStamp << " ratio=" << myRatio
          << " hits=" << myHits 
          << "]";
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return ( myMaxSize > 0 );
    }

    // ------------------------- Protected Datas ------------------------------
  protected:
    /// The maximal number of memoized items
    Size      myMaxSize;
    /// The minimal ratio to remove if the maximal number of items  is reached.
    double    myRatio;
    /// Current time
    TimeStamp myTimeStamp;
    /// The map memoizing computations.
    std::unordered_map< Key, StoredValue > myMap;
    /// The number of hits up to the last clean-up.
    Size myHits;
    /// The number of misses up to the last clean-up.
    Size myMisses;

    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class TimeStampMemoizer


  /**
   * Overloads 'operator<<' for displaying objects of class 'TimeStampMemoizer'.
   * @tparam TKey the type used for keys, must be hashable.
   *
   * @tparam TValue the type used for values, must be
   * DefaultConstructible, CopyConstructible, Assignable.
   *
   * @param out the output stream where the object is written.
   * @param object the object of class 'TimeStampMemoizer' to write.
   * @return the output stream after the writing.
   */
  template <typename TKey, typename TValue>
  std::ostream&
  operator<< ( std::ostream & out, 
               const TimeStampMemoizer<TKey, TValue> & object )
  {
    object.selfDisplay( out );
    return out;
  }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined TimeStampMemoizer_h

#undef TimeStampMemoizer_RECURSES
#endif // else defined(TimeStampMemoizer_RECURSES)
