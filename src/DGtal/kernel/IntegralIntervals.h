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
 * @file IntegralIntervals.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/04/24
 *
 */

#if defined(IntegralIntervals_RECURSES)
#error Recursive header files inclusion detected in IntegralIntervals.h
#else // defined(IntegralIntervals_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IntegralIntervals_RECURSES

#if !defined IntegralIntervals_h
/** Prevents repeated inclusion of headers. */
#define IntegralIntervals_h

#include <vector>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CBoundedNumber.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class IntegralIntervals
  /**
     Description of template class 'IntegralIntervals' <p> \brief Aim:

     A class that represents a set of integers using intervals.  For
     instance the set X={-3,-2,0,1,2,4,7,8} is represented as the
     sorted vector ((-3,-2),(0,2),(4,4),(7,8)).

     Inserting -1 into X induced the sorted vector ((-3,2),(4,4),(7,8)).

     @tparam TInteger any model of concepts::CBoundedNumber, for instance int, long, etc

     @note Useful to represent points of (especially convex) lattice
     polytopes or points of digital sets.
  */
  template < typename TInteger >
  class IntegralIntervals
  {
  public:
    BOOST_CONCEPT_ASSERT(( concepts::CBoundedNumber< TInteger > ));

    typedef TInteger Integer;
    using Self      = IntegralIntervals< Integer >;
    using Interval  = std::pair<Integer,Integer>;
    using Container = std::vector< Interval >;
    using Size      = std::size_t;
    using CIterator = typename Container::iterator;
    using IntegerRange = std::vector< Integer >;

    /// Default Constructor
    IntegralIntervals() = default;

    /// Copy constructor
    /// @param other any other object.
    IntegralIntervals( const Self & other ) = default;

    /// Move constructor
    /// @param other any other object.
    IntegralIntervals( Self&& other ) = default;

    /// Assignment
    /// @param other any other object.
    /// @return a reference to this object.
    Self& operator=( const Self & other ) = default;

    /// Move assignment
    /// @param other any other object.
    /// @return a reference to this object.
    Self& operator=( Self&& other ) = default;

    /// Constructor from range
    /// @tparam InputIterator the type of forward iterator on a range of integer values.
    /// @param it,itE the range of integer values.
    template <typename InputIterator>
    IntegralIntervals( InputIterator it, InputIterator itE )
    {
      if ( it == itE ) return;
      Integer first = *it;
      Integer last  = *it;
      for ( ++it; it != itE; ++it )
        {
          Integer x = *it;
          if ( first <= x && x <= last ) continue;
          if ( x == last+1 )  { last = x;  continue; }
          if ( x == first-1 ) { first = x; continue; }
          insert( first, last );
          first = x;
          last  = x;
        }
      insert( first, last );
    }

    /// Clears the data structure.
    void clear() { myData.clear(); }

    /// @return a reference to the current data
    Container& data() { return myData; }
    /// @return a const reference to the current data
    const Container& data() const { return myData; }

    /// @return 'true' if the set contains no element
    bool empty() const { return myData.empty(); }

    /// @return the number of integers of the set.
    Size size() const
    {
      Size nb = 0;
      for ( const auto& I : myData ) nb += 1 + I.second - I.first;
      return nb;
    }

    /// @return the current allocated space in the object container.
    Size capacity() const
    {
      return myData.capacity();
    }

    /// @return 'true' if the set of integers is convex, i.e. empty or one interval.
    bool isConvex() const
    {
      return myData.size() <= 1;
    }

    /// @return the set of integers
    std::set<Integer> integerSet() const
    {
      std::set<Integer> S;
      for ( const auto& I : myData )
        for ( Integer x = I.first; x <= I.second; x++ )
          S.insert( x );
      return S;
    }
    /// @return the set of integers as a vector
    std::vector<Integer> integerVector() const
    {
      std::vector<Integer> S;
      for ( const auto& I : myData )
        for ( Integer x = I.first; x <= I.second; x++ )
          S.push_back( x );
      return S;
    }

    /// @param x any integer
    /// @return the number of times the element \a x is in the set (either 0 or 1).
    Size count( Integer x ) const
    {
      if ( empty() ) return 0;
      Size i = 0;
      Size j = myData.size() - 1;
      while ( i <= j )
        {
          const Size m = (i+j)/2;
          const Interval& I = myData[ m ]; // I = [a,...,b]
          if ( x < I.first ) // x < a
            {
              if ( m == 0 ) return 0;
              j = m - 1;
            }
          else if ( I.second < x ) // b < x
            i = m + 1;
          else // a <= x <= b
            return 1;
        }
      return 0;
    }

    /// Inserts the integer i into the sequence
    /// @param i any integer
    void insert( Integer i )
    {
      insert( Interval( i, i ) );
    }

    /// Inserts the interval of integers into the sequence
    /// @param f,l  any valid interval (f <= l)
    void insert( Integer f, Integer l )
    {
      insert( Interval( f, l ) );
    }

    /// Inserts the interval of integers into the sequence
    /// @param I any valid interval (I.first <= I.second)
    void insert( const Interval& I )
    {
      // Search position of first element.
      auto it = lowerBound( I.first );
      if ( it == myData.end() ) // if you reach the end, just add the interval
        {
          myData.push_back( I );
          if ( myData.size() >= 2 ) extend( myData.end() - 2 );
        }
      else if ( I.first < it->first )
        {
          // See if interval must merge with previous
          if ( it != myData.begin() )
            {
              auto it_prev = it; --it_prev;
              if ( I.first <= it_prev->second + 1 )
                {
                  it_prev->second = I.second;
                  extend( it_prev );
                  return;
                }
            }
          Size idx = it - myData.begin();
          // std::cout << "(Inserting " << idx << ")" << std::endl;;
          myData.insert( it, I );
          extend( myData.begin() + idx );
        }
      else // it->first <= I.first <= it->second
        {
          it->second = std::max( it->second, I.second );
          extend( it );
        }
    }

    /// Erases the integer i from the sequence
    /// @param i any integer
    void erase( Integer i )
    {
      erase( Interval( i, i ) );
    }
    /// Erases the interval of integers from the sequence
    /// @param f,l  any valid interval (f <= l)
    void erase( Integer f, Integer l )
    {
      erase( Interval( f, l ) );
    }

    /// Erases the interval of integers from the sequence
    /// @param I any valid interval (I.first <= I.second)
    void erase( const Interval& I )
    {
      for ( std::size_t i = 0; i < myData.size(); )
        {
          Interval& J = myData[ i ];
          // I=[a,b], J=[a',b'], a <= b, a' <= b'
          if ( I.second < J.first )
            { break; } // b < a' : no further intersection
          if ( J.second < I.first )
            { ++i; continue; } // b' < a : no further intersection
          // a' <= b and a <= b'
          //       a ----------  b
          // a' ...............  a'
          //       b' ................. b'
          //
          // a' ..................... b' => a'..a-1       b+1..b'
          Interval K1( J.first, I.first - 1 );
          Interval K2( I.second + 1, J.second );
          bool K1_exist = K1.second >= K1.first;
          bool K2_exist = K2.second >= K2.first;
          if ( K1_exist && K2_exist )
            {
              myData[ i ] = K2;
              myData.insert( myData.begin() + i, K1 );
              break; // no further intersection possible
            }
          else if ( K1_exist )
            {
              myData[ i ] = K1; i++;
            }
          else if ( K2_exist )
            {
              myData[ i ] = K2; break;
            }
          else
            {
              myData.erase( myData.begin() + i );
            }
        }
    }

    /// Performs the union of set \a other with this object.
    /// @param other any intervals
    /// @return a reference to this object
    Self& add( const Self& other )
    {
      for ( const auto& I : other.myData )
        insert( I );
      return *this;
    }

    /// Subtract set \a other from this object.
    /// @param other any intervals
    /// @return a reference to this object
    Self& subtract( const Self& other )
    {
      for ( const auto& I : other.myData )
        erase( I );
      return *this;
    }

    /// Performs the set union between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set union between this and other.
    Self set_union( const Self& other ) const
    {
      Self U = *this;
      U.add( other );
      return U;
    }

    /// Performs the set difference between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set difference between this and other.
    Self set_difference( const Self& other ) const
    {
      Self U = *this;
      U.subtract( other );
      return U;
    }

    /// Performs the set intersection between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set difference between this and other.
    Self set_intersection( const Self& other ) const
    {
      Self A_plus_B  = set_union( other );
      Self A_delta_B = set_symmetric_difference( other );
      return A_plus_B.subtract( A_delta_B );
    }

    /// Performs the set symmetric difference between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set symmetric difference between this and other.
    Self set_symmetric_difference( const Self& other ) const
    {
      Self A_minus_B = *this;
      A_minus_B.subtract( other );
      Self B_minus_A = other;
      B_minus_A.subtract( *this );
      return A_minus_B.add( B_minus_A );
    }

    /// Consider the set of integers as points, transform them into
    /// pointels inn Khalimsky coordinates and build their star. All
    /// integers are multiplied by two. All doubled integers are
    /// completed with their immediately inferior and superior value.
    ///
    /// @return the star of these points.
    Self starOfPoints() const
    {
      Self R( *this );
      for ( auto& I : R.myData )
        {
          I.first  = 2*I.first-1;
          I.second = 2*I.second+1;
        }
      return R;
    }

    /// Consider the set of integers as cells represented by their
    /// Khalimsky coordinates, and build their star.
    ///
    /// @return the star of these cells.
    Self starOfCells() const
    {
      Self R( *this );
      for ( size_t i = 0; i < R.myData.size(); )
        {
          auto& I = R.myData[ i ];
          if ( ( I.first  & 0x1 ) == 0 ) I.first  -= 1;
          if ( ( I.second & 0x1 ) == 0 ) I.second += 1;
          // We have to be careful since extending this interval may
          // have reached the next interval.
          // We have to merge them in this case.
          i += 1;
          if ( i < R.myData.size() )
            {
              auto& Inext = R.myData[ i ];
              if ( Inext.first <= I.second+1 )
                {
                  I.second = Inext.second;
                  R.myData.erase( R.myData.begin() + i );
                  i -= 1;
                }
            }
        }
      return R;
    }

    /// @return the range of points that contains the vertices of all
    /// the cells stored in this set. It is thus a range of integers.
    IntegerRange extremaOfCells() const
    {
      IntegerRange C;
      for ( auto I : myData )
        {
          if ( ( I.first  & 0x1 ) != 0 ) I.first  -= 1;
          if ( ( I.second & 0x1 ) != 0 ) I.second += 1;
          for ( auto x = I.first; x <= I.second; x += 2 )
            C.push_back( x >> 1 ); // here x / 2 == x >> 1 since x is even
        }
      auto last = std::unique( C.begin(), C.end() );
      C.erase( last, C.end() );
      return C;
    }

    /// @param other any other integral set represented by intervals
    /// @return 'true' iff this integer set includes the integer set \a other.
    bool includes( const Self& other ) const
    {
      auto it = myData.cbegin();
      for ( const auto& I : other.myData )
        {
          // Find possible interval
          while ( it != myData.cend() && it->second < I.second ) ++it;
          if ( it == myData.cend() )  return false;
          if ( I.first < it->first ) return false;
        }
      return true;
    }

    /// @param other any other integral set represented by intervals
    /// @return 'true' iff this integer set equals the integer set \a other.
    bool equals( const Self& other ) const
    {
      if ( myData.size() != other.myData.size() ) return false;
      auto it = myData.cbegin();
      for ( const auto& I : other.myData )
        {
          if ( it->first != I.first || it->second != I.second ) return false;
          ++it;
        }
      return true;
    }

    // ----------------------- Interface --------------------------------------
public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[";
      for ( const auto& I : myData )
        out << " (" << I.first << "," << I.second << ")";
      out << " ]";
    }

    /// @return 'true' iff the intervals are consistent and sorted.
    bool isValid() const
    {
      for ( const auto& I : myData )
        if ( I.first > I.second ) return false;
      for ( Size i = 1; i < myData.size(); i++ )
        {
          if ( myData[i-1].second >= myData[i].first - 1 )
            return false;
        }
      return true;
    }

    // ------------------- protected services -------------------------------
  protected:

    /// At the given iterator position the current interval may
    /// overlap with the following ones. Merge them.
    /// @param it any position
    void extend( CIterator it )
    {
      // std::cout  << "Extending" << std::endl;
      CIterator it_next = it; ++it_next;
      while ( it_next != myData.end() )
        {
          if ( it->second >= ( it_next->first - 1 ) )
            {
              it->second = std::max( it->second, it_next->second );
              ++it_next;
            }
          else break;
        }
      ++it;
      // std::cout  << "Erase from " << ( it - myData.begin() )
      //            << " to " << ( it_next - myData.begin() ) << std::endl;
      myData.erase( it, it_next );
    }

    /// @param x any integer
    ///
    /// @return the iterator on the interval which is not before x,
    /// i.e. the interval containing x or, if it does not exist, the
    /// interval after.
    CIterator lowerBound( Integer x )
    {
      // std::cout << "(lowerbound for " << x << ")" << std::endl;
      if ( empty() ) return myData.end();
      Size i = 0;
      Size j = myData.size() - 1;
      while ( i <= j )
        {
          const Size m = (i+j)/2;
          const Interval& I = myData[ m ]; // I = [a,...,b]
          if ( x < I.first ) // x < a
            {
              if ( m == 0 ) break;
              j = m - 1;
            }
          else if ( I.second < x ) // b < x
            i = m + 1;
          else // a <= x <= b
            return myData.begin() + m;
        }
      // std::cout << "(not found, return " << i <<  ")" << std::endl;
      return myData.begin() + i;
    }

    // ------------------- protected data -------------------------------
  protected:

    /// The sorted sequence of integral intervals
    Container myData;
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'IntegralIntervals'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'IntegralIntervals' to write.
   * @return the output stream after the writing.
   */
  template <typename TInteger>
  std::ostream&
  operator<< ( std::ostream & out, const IntegralIntervals<TInteger> & object )
  {
    object.selfDisplay( out );
    return out;
  }

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IntegralIntervals_h

#undef IntegralIntervals_RECURSES
#endif // else defined(IntegralIntervals_RECURSES)
