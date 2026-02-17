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
 * @file LatticeSetByIntervals.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/04/24
 *
 */

#if defined(LatticeSetByIntervals_RECURSES)
#error Recursive header files inclusion detected in LatticeSetByIntervals.h
#else // defined(LatticeSetByIntervals_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LatticeSetByIntervals_RECURSES

#if !defined LatticeSetByIntervals_h
/** Prevents repeated inclusion of headers. */
#define LatticeSetByIntervals_h

#include <unordered_map>
#include <boost/iterator/iterator_facade.hpp>
#include <climits>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CUnsignedNumber.h"
#include "DGtal/kernel/CBoundedNumber.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/IntegralIntervals.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class LatticeSetByIntervals
  /**
     Description of template class 'LatticeSetByIntervals' <p> \brief Aim:

     A class that represents a set of lattice points using intervals
     along a given axis.

     @tparam TSpace any model of concepts::CSpace, for instance any SpaceND like Z2i::Space, Z3i::Space.
  */
  template < typename TSpace >
  class LatticeSetByIntervals
  {
  public:
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));

    typedef TSpace Space;
    using Self      = LatticeSetByIntervals< Space >;
    using Point     = typename Space::Point;
    using Vector    = typename Space::Vector;
    using Integer   = typename Space::Integer;
    using PointRange= std::vector< Point >;
    using Intervals = IntegralIntervals< Integer >;
    using Interval  = typename Intervals::Interval;
    using Container = std::map< Point, Intervals >;
    using RowIterator      = typename Container::iterator;
    using RowConstIterator = typename Container::const_iterator;
    using LatticeSetByInterval = std::map< Point, Interval >;
    using Size      = std::size_t;
    using size_type = Size;
    static const Dimension dimension = Space::dimension;

    //------------------- standard services (construction, move) -------------------
  public:
    /// @name Standard services (construction, move, clear)
    /// @{

    /// Constructor from axis.
    /// @param axis the row axis chosen for stacking the points.
    LatticeSetByIntervals( Dimension axis = 0 )
      : myAxis( axis ), myData() {}

    /// Copy constructor
    /// @param other any other object.
    LatticeSetByIntervals( const Self & other ) = default;

    /// Move constructor
    /// @param other any other object.
    LatticeSetByIntervals( Self&& other ) = default;

    /// Assignment.
    /// @param other any other object.
    /// @return a reference to this object
    Self& operator=( const Self & other ) = default;

    /// Move Assignment.
    /// @param other any other object.
    /// @return a reference to this object
    Self& operator=( Self&& other ) = default;

    /// Constructor from range of points
    /// @tparam PointIterator any model of input iterator on points.
    /// @param it,itE the range of point
    /// @param axis the row axis chosen for stacking the points.
    template <typename PointIterator>
    LatticeSetByIntervals( PointIterator it, PointIterator itE, Dimension axis = 0 )
      : myAxis( axis )
    {
      for ( ; it != itE; ++it ) {
        Point   q = *it;
        Integer x = q[ axis ];
        q[ axis ] = 0;
        myData[ q ].insert( x );
      }
    }

    /// Constructor from lattice set of interval (often a lattice set
    /// representation for a polytope, since there is at most one
    /// interval per row).
    ///
    /// @param aSet any lattice set represented by one interval per row.
    /// @param axis the row axis chosen for stacking the points in \a aSet.
    LatticeSetByIntervals( const LatticeSetByInterval& aSet, Dimension axis )
      : myAxis( axis )
    {
      for ( const auto& aRow : aSet )
        myData[ aRow.first ].data().push_back( aRow.second );
    }

    /// Clears the data structure.
    void clear() { myData.clear(); }

    /// Change the main axis of projection. If the object is not
    /// empty, it empties the object.
    ///
    /// @param axis any valid integer between 0 and dimension
    /// (excluded)
    void setAxis( Dimension axis )
    {
      myData.clear();
      myAxis = axis;
    }

    /// @return the main axis of projection
    Dimension axis() const
    { return myAxis; }

    /// @return a reference to the container
    Container& data() { return myData; }

    /// @}

    //------------------- conversion services -----------------------------
  public:
    /// @name conversion services
    /// @{

    /// @return the range of points stored in this lattice set.
    PointRange toPointRange() const
    {
      PointRange X;
      X.reserve( size() );
      for ( const auto& pV : myData )
        {
          auto p    = pV.first;
          auto iVec = pV.second.integerVector();
          for ( auto x : iVec )
            {
              p[ myAxis ] = x;
              X.push_back( p );
            }
        }
      return X;
    }

    /// @}

    //------------------- capacity services -----------------------------
  public:
    /// @name capacity services
    /// @{

    /// @return 'true' iff this object represents the empty set.
    /// @return Constant time operation.
    bool empty() const
    {
      return myData.empty();
    }

    /// @return the number of lattice points represented in this object.
    ///
    /// @warning The complexity is linear in the number of stored intervals.
    Size size() const
    {
      Size nb = 0;
      for ( const auto& pV : myData )
        nb += pV.second.size();
      return nb;
    }

    /// @return the the maximum number of elements the container is
    /// able to hold due to system or library implementation
    /// limitations.
    Size max_size() const
    {
      return myData.max_size();
    }

    /// @}

    //------------------- modifier services -----------------------------
  public:
    /// @name modifier services
    /// @{

    /// Inserts the point into the set.
    /// @param p any point.
    void insert( Point p )
    {
      Integer x   = p[ myAxis ];
      p[ myAxis ] = 0;
      myData[ p ].insert( x );
    }

    /// Erases the point from the set.
    /// @param p any point.
    void erase( Point p )
    {
      Integer x   = p[ myAxis ];
      p[ myAxis ] = 0;
      auto it = myData.find( p );
      if ( it != myData.end() )
        {
          it->second.erase( x );
          if ( it->second.empty() )
            myData.erase( it ); // purge  element if it was the last in the row.
        }
    }


    /// Eliminates rows that contains no element.
    void purge()
    {
      for ( auto it = myData.begin(), itE = myData.end(); it != itE; )
        if ( it->second.empty() )
          it = myData.erase( it );
        else ++it;
    }

    /// @}

    //------------------- set operations --------------------------------
  public:
    /// @name set operations
    /// @{

    /// Performs the union of set \a other with this object.
    /// @param other any intervals
    /// @return a reference to this object
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self& add( const Self& other )
    {
      if ( other.axis() != axis() )
        {
          trace.error() << "[LatticeSetByInterval::add] "
                        << "Both lattice sets should share the same axis: "
                        << axis() << " != " << other.axis() << std::endl;
          return *this;
        }
      for ( const auto& pV : other.myData )
        {
          const Point& p = pV.first;
          auto it = myData.find( p );
          if ( it != myData.end() )
            it->second.add( pV.second );
          else
            myData[ p ] = pV.second;
        }
      return *this;
    }

    /// Subtract set \a other from this object.
    /// @param other any intervals
    /// @return a reference to this object
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self& subtract( const Self& other )
    {
      if ( other.axis() != axis() )
        {
          trace.error() << "[LatticeSetByInterval::subtract] "
                        << "Both lattice sets should share the same axis: "
                        << axis() << " != " << other.axis() << std::endl;
          return *this;
        }
      for ( const auto& pV : other.myData )
        {
          const Point& p = pV.first;
          auto it = myData.find( p );
          if ( it != myData.end() )
            it->second.subtract( pV.second );
        }
      return *this;
    }

    /// Performs the set union between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set union between this and other.
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self set_union( const Self& other ) const
    {
      Self U = *this;
      U.add( other );
      return U;
    }

    /// Performs the set difference between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set difference between this and other.
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self set_difference( const Self& other ) const
    {
      Self U = *this;
      U.subtract( other );
      return U;
    }

    /// Performs the set intersection between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set difference between this and other.
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self set_intersection( const Self& other ) const
    {
      Self A_plus_B  = set_union( other );
      Self A_delta_B = set_symmetric_difference( other );
      return A_plus_B.subtract( A_delta_B );
    }

    /// Performs the set symmetric difference between this and other.
    /// @param other any other integral set represented by intervals
    /// @return the set symmetric difference between this and other.
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    Self set_symmetric_difference( const Self& other ) const
    {
      Self A_minus_B = *this;
      A_minus_B.subtract( other );
      Self B_minus_A = other;
      B_minus_A.subtract( *this );
      return A_minus_B.add( B_minus_A );
    }

    /// @param other any other lattice set represented by intervals
    /// @return 'true' iff this lattice set includes the lattice set \a other.
    ///
    /// @pre \a other and 'this' should share the same axis:
    /// `this->axis() == other.axis()`
    bool includes( const Self& other ) const
    {
      if ( other.axis() != axis() )
        {
          trace.error() << "[LatticeSetByInterval::subtract] "
                        << "Both lattice sets should share the same axis: "
                        << axis() << " != " << other.axis() << std::endl;
          return false;
        }
      for ( const auto& pV : other.myData )
        {
          const Point& p = pV.first;
          const auto  it = myData.find( p );
          if ( it == myData.cend() )                return false;
          if ( ! it->second.includes( pV.second ) ) return false;
        }
      return true;
    }

    /// @param other any other integral set represented by intervals
    /// @return 'true' iff this integer set equals the integer set \a other.
    bool equals( const Self& other ) const
    {
      if ( other.axis() != axis() )
        {
          trace.error() << "[LatticeSetByInterval::subtract] "
                        << "Both lattice sets should share the same axis: "
                        << axis() << " != " << other.axis() << std::endl;
          return false;
        }
      if ( myData.size() != other.myData.size() ) return false;
      auto it = myData.cbegin();
      for ( const auto& I : other.myData )
        {
          if ( it->first != I.first )            return false;
          if ( ! it->second.equals( I.second ) ) return false;
          ++it;
        }
      return true;
    }

    /// @}

    //------------------- topology operations --------------------------------
  public:
    /// @name topology operations
    /// @{

    /// Consider the set of integers as points, transform them into
    /// pointels in Khalimsky coordinates and build their
    /// star. Afterwards points represent cells with their Khalimsky
    /// coordinates (i.e. even along an axis means closed, odd along
    /// an axis means open). Concretely, points coordinates are
    /// multiplied by two, and all the incident points are added to
    /// this set.
    ///
    /// @return the star of this set of points transformed to
    /// pointels, i.e. the smallest open cell complex containing it.
    Self starOfPoints() const
    {
      Self C( myAxis );
      // First step, place points as pointels and insert their star along
      // dimension a.
      for ( auto& pV : myData )
        {
          const Point q = 2 * pV.first;
          C.myData[ q ] = pV.second.starOfPoints();
        }
      // Second step, dilate along remaining directions
      for ( Dimension k = 0; k < dimension; k++ )
        {
          if ( k == myAxis ) continue;
          for ( const auto& value : C.myData )
            {
              Point    q = value.first;
              if ( q[ k ] & 0x1 ) continue;
              q[ k ]    -= 1;
              C.myData[ q ].add( value.second );
              q[ k ]    += 2;
              C.myData[ q ].add( value.second );
            }
        }
      return C;
    }

    /// Consider the set of integers as cells represented by their
    /// Khalimsky coordinates, and build their star.
    ///
    /// @return the star of this set of cells, i.e. the smallest open
    /// cell complex containing it.
    Self starOfCells() const
    {
      Self C( *this );
      // First step, compute star along dimension a.
      for ( auto& pV : C.myData )
        pV.second = pV.second.starOfCells();
      // Second step, dilate along remaining directions
      for ( Dimension k = 0; k < dimension; k++ )
        {
          if ( k == myAxis ) continue;
          for ( const auto& value : C.myData )
            {
              Point    q = value.first;
              if ( q[ k ] & 0x1 ) continue;
              q[ k ]    -= 1;
              C.myData[ q ].add( value.second );
              q[ k ]    += 2;
              C.myData[ q ].add( value.second );
            }
        }
      return C;
    }

    /// Consider the set of integers as cells represented by their
    /// Khalimsky coordinates, and build their skeleton.
    ///
    /// @return the skeleton of this set of cells, i.e. the smallest
    /// set of cells such that its star covers it.
    Self skeletonOfCells() const
    {
      Self S( *this );
      // Now extracting implicitly its Skel
      for ( const auto& pV : myData )
        {
          const Point&   p = pV.first;
          const auto&    V = pV.second;
          for ( Dimension k = 0; k < dimension; k++ )
            {
              if ( k == myAxis ) continue;
              if ( ( p[ k ] & 0x1 ) != 0 ) continue; // if open along axis continue
              // if closed, check upper incident cells along direction k
              Point q = p;  q[ k ] -= 1;
              Point r = p;  r[ k ] += 1;
              auto itq = S.myData.find( q );
              if ( itq != S.myData.end() )
                {
                  auto& W = itq->second;
                  W.subtract( V );
                }
              auto itr = S.myData.find( r );
              if ( itr != S.myData.end() )
                {
                  auto& W = itr->second;
                  W.subtract( V );
                }
            }
        }
      // Extract skel along main axis
      for ( auto& value : S.myData )
        {
          auto & V = value.second;
          Intervals sub_to_V;
          for ( auto I : V.data() )
            {
              if ( ( I.first & 0x1 )  != 0 )
                {
                  if ( I.first != I.second )
                    sub_to_V.data().push_back( Interval{ I.first, I.first } );
                  I.first  += 1;
                }
              if ( ( I.second & 0x1 ) != 0 ) I.second -= 1;
              for ( auto x = I.first; x <= I.second; x += 2 )
                sub_to_V.data().push_back( Interval{ x+1, x+1 } );
            }
          V.subtract( sub_to_V );
        }
      // Erase empty stacks
      S.purge();
      return S;
    }

    /// @return the range of points that contains the vertices of all
    /// the cells stored in this set.
    PointRange extremaOfCells() const
    {
      typedef std::vector< Integer > Coordinates;
      std::map< Point, Coordinates > E;
      // Now extracting vertices along axis direction.
      for ( const auto& pV : myData )
        {
          const Point&   p = pV.first;
          const auto&    V = pV.second;
          E[ p ] = V.extremaOfCells();
        }
      std::map< Point, Coordinates > next_E;
      for ( Dimension k = 0; k != dimension; k++ )
        {
          if ( k == myAxis ) continue;
          for ( const auto& pC : E )
            {
              const auto& p = pC.first;
              Point       q = p;
              q[ k ]        = q[ k ] >> 1;
              bool odd      = ( p[ k ] & 0x1 ) != 0;
              { // odd/even always copy
                auto it = next_E.find( q );
                if ( it == next_E.end() ) next_E[ q ] = pC.second;
                else
                  {
                    Coordinates F;
                    std::set_union( pC.second.cbegin(),  pC.second.cend(),
                                    it->second.cbegin(), it->second.cend(),
                                    std::back_inserter( F ) );
                    it->second = F;
                  }
              }
              if ( odd )
                { // odd: must copy forward also
                  q[ k ] += 1;
                  auto it = next_E.find( q );
                  if ( it == next_E.end() ) next_E[ q ] = pC.second;
                  else
                    {
                      Coordinates F;
                      std::set_union( pC.second.cbegin(),  pC.second.cend(),
                                      it->second.cbegin(), it->second.cend(),
                                      std::back_inserter( F ) );
                      it->second = F;
                    }
                }
            } // for ( const auto& pC : E )
          E.swap( next_E );
          next_E.clear();
        }
      // Build point range.
      PointRange R;
      for ( const auto& pC : E )
        {
          Point p = pC.first;
          for ( auto&& x : pC.second )
            {
              p[ myAxis ] = x;
              R.push_back( p );
            }
        }
      std::sort( R.begin(), R.end() );
      return R;
    }

    /// @}

    //------------------- specific services (interval insertion, removal) -------------
  public:
    /// @name specific services (interval insertion, removal)
    /// @{

    /// @note Specific to this data structure.
    /// @return an evaluation of the memory usage of this data structure.
    size_type memory_usage() const noexcept
    {
      size_type nb = 0;
      for ( const auto& pV : myData )
        {
          nb += sizeof( Interval )  * pV.second.capacity() + sizeof( void* );
          nb += sizeof( pV ) + sizeof( void* );
        }
      nb += sizeof( Self );
      return nb;
    }

    /// @param q any point
    ///
    /// @return the integral intervals corresponding to lattice point
    /// coordinates along the row of axis \a myAxis and containing \a
    /// q.
    Intervals& at( Point q )
    {
      q[ myAxis ] = 0;
      return myData[ q ];
    }

    /// Reset the intervals for a given point.
    /// @param p any point
    ///
    /// @note After this operation, there is no lattice point on the row
    /// containing point \a p.
    void reset( Point p )
    {
      p[ myAxis ] = 0;
      auto it = myData.find( p );
      if ( it != myData.end() ) myData.erase( it );
    }


    /// The axis along which data is stacked in intervals
    Dimension myAxis;
    /// Associate to each point its sequences of intervals
    Container myData;
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LatticeSetByIntervals_h

#undef LatticeSetByIntervals_RECURSES
#endif // else defined(LatticeSetByIntervals_RECURSES)
