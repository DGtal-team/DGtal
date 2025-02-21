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
 * @file UnorderedSetByBlock.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2020/04/24
 *
 */
#ifndef UNORDEREDSETBYBLOCK_HPP
#define UNORDEREDSETBYBLOCK_HPP

#include <unordered_map>
#include <boost/iterator/iterator_facade.hpp>
#include <climits>
#include "DGtal/base/Common.h"
#include "DGtal/base/Bits.h"
#include "DGtal/kernel/CUnsignedNumber.h"
#include "DGtal/kernel/CBoundedNumber.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/PointHashFunctions.h"

namespace DGtal
{

  /// Splits an integral-array element into an integral-array element
  /// and a 32 or 64 bit integer (depending on \a TWord). The expected
  /// behaviour is for an element e of dimension 3 (and 32 bits word):
  ///
  /// \code
  /// Splitter< Point > split; // default is split with 32 bits word
  /// auto v = split( Point( 117, 43, 52 ) );
  /// v.first  == 117 & 31
  /// v.second == Point( 117 - v.first, 43, 52 );
  /// \endcode
  ///
  /// \code
  /// Splitter< Point,uint64_t > split; // for 64 bits word
  /// auto v = split( Point( 117, 43, 52 ) );
  /// v.first  == 117 & 63
  /// v.second == Point( 117 - v.first, 43, 52 );
  /// \endcode
  ///
  /// @tparam TElement the type of array-like element.
  ///
  /// @tparam TWord the type for holding blocks (uint64_t or uint32_t generally),
  /// a model of concepts::CIntegralNumber and concepts::CUnsignedNumber.
  ///
  /// @see UnorderedSetByBlock
  template < typename TElement, typename TWord = uint32_t >
  requires DGtal::concepts::CBoundedNumber<TWord> && 
           DGtal::concepts::CUnsignedNumber<TWord>
  struct Splitter {
    typedef Splitter< TElement,TWord >   Self;
    typedef TElement                     Element;
    typedef TWord                        Word;
    typedef typename Element::Coordinate Coordinate;

    /// Splits an element \a e into a pair grouping its block
    /// coordinates and its bit within this block.
    ///
    /// @param e any lattice point
    ///
    /// @return a pair grouping its block coordinates and its bit
    /// within this block.
    static
    std::pair< Element, Coordinate >
    split( Element e )
    {
      auto block_coords =
        ( e[ 0 ] & static_cast<Coordinate>( sizeof( Word ) * CHAR_BIT - 1 ) );
      e[ 0 ] -= block_coords;
      return { e, block_coords };
    }

    /// Rejoins a splitted element (see \ref split).
    ///
    /// @param e the block coordinate of the element.
    /// @param x the bit coordinate of the element.
    ///
    /// @return the corresponding rejoined element.
    static
    Element
    join( Element e, Coordinate x )
    {
      e[ 0 ] |= x;
      return e;
    }

    /// Rejoins a splitted element (see \ref split).
    ///
    /// @param p a pair grouping the block coordinate and the bit
    /// coordinate of the element.
    ///
    /// @return the corresponding rejoined element.
    static
    Element
    join( const std::pair< Element, Coordinate >& p )
    {
      Element ge = p.first;
      ge[ 0 ] |= p.second;
      return ge;
    }
  };


  /// This data structure represents a set of elements that must be
  /// integral arrays (i.e. digital points). It is similar to an
  /// unordered_set, but much more compact in general. The container
  /// used is a unordered_map from point to a 32-bit word. The idea is
  /// to group consecutive elements/points along x axis by blocks of
  /// 32 bits.  If any point in a block is present, the corresponding
  /// element is present in the unordered_map and bits set to 1 in the
  /// word correspond to points present in the set. On average, memory
  /// occupancy is divided by four, the structure is four times faster
  /// for traversal and approximately same speed for
  /// queries/insertion/erase.
  ///
  /// Almost all standard operations of unordered_set in c++11 are implemented.
  ///
  /// @tparam Key the type of integral array.
  /// @tparam TSplitter the type for splitting a key into a block and a bit (see \ref Splitter).
  /// @tparam Hash the type that provides a hasher for Key.
  /// @tparam KeyEqual the type that provides an equality comparator for Key.
  /// @tparam UnorderedMapAllocator the type that provides an allocator for the underlying unordered_map container.
  ///
  /// Specialized versions of Splitter are written for usual digital
  /// points of Z2i or Z3i.
  ///
  /// @code
  /// #include "DGtal/kernel/UnorderedSetByBlock.h"
  /// ...
  /// typedef DGtal::PointVector< 3, int >  Point3i; // digital point in Z3;
  /// std::unordered_set< Point3i >         aSet;    // usual data structure
  /// DGtal::UnorderedSetByBlock< Point3i > aSet2;   // this data structure
  /// // same code after for aSet or aSet2.
  /// @endcode
  template < typename Key,
             typename TSplitter = Splitter< Key >,
             class Hash = std::hash<Key>,
             class KeyEqual = std::equal_to<Key>,
             class UnorderedMapAllocator = std::allocator<
               std::pair<const Key, typename TSplitter::Word >
               >
             >
  struct UnorderedSetByBlock {
    typedef UnorderedSetByBlock< Key, TSplitter, Hash, KeyEqual > Self;
    typedef TSplitter               Splitter;
    typedef typename Splitter::Word Word;
    typedef typename Splitter::Coordinate Coordinate;
    /// The underlying container, an unordered_map.
    typedef std::unordered_map< Key, Word, Hash, KeyEqual,
                                UnorderedMapAllocator > Container;

    // Standard types
    /// Key
    typedef Key key_type;
    /// Key
    typedef Key value_type;
    /// Unsigned integer type (usually std::size_t)
    typedef typename Container::size_type size_type;
    /// Signed integer type (usually std::ptrdiff_t)
    typedef typename Container::difference_type difference_type;
    /// Hash
    typedef Hash     hasher;
    /// KeyEqual
    typedef KeyEqual key_equal;
    /// Allocator
    typedef typename Container::allocator_type allocator_type;
    /// Reference to value_type/Key
    typedef Key& reference;
    /// Const reference to value_type/Key
    typedef const Key& const_reference;
    /// Pointer to value_type/Key
    typedef Key* pointer;
    /// Const Pointer to value_type/Key
    typedef const Key* const_pointer;

  public:
    // ---------------------- iterators --------------------------------

    /// Read iterator on set elements. Model of ForwardIterator.
    struct const_iterator
      : public boost::iterator_facade< const_iterator, Key const,
                                       boost::forward_traversal_tag,
                                       Key const >
    {
      friend struct UnorderedSetByBlock< Key, TSplitter, Hash, KeyEqual >;
      /// Default constructor
      const_iterator() : collection( nullptr ), it(),
                         bit( static_cast<Coordinate>(0) ),
                         current( static_cast<Word>(0) ) {}

      /// Constructor from set and container iterator
      /// @param aSet a reference to the visited unordered block set
      /// @param anIt an iterator in the container of this set.
      const_iterator( const Self& aSet, typename Container::const_iterator anIt )
        : collection( &aSet ), it( anIt )
      {
        if ( it != collection->my_elements.cend() )
          {
            current  = it->second;
            bit      = static_cast<Coordinate>( Bits::leastSignificantBit( current ) );
          }
        else
          {
            current = static_cast<Word>(0);
            bit     = static_cast<Coordinate>(0);
          }
      }

      /// Constructor from set, container iterator and starting bit
      /// @param aSet a reference to the visited unordered block set
      /// @param anIt an iterator in the container of this set.
      /// @param aBit the bit index in the word pointed by \a anIt.
      const_iterator( const Self& aSet, typename Container::const_iterator anIt,
                      Coordinate aBit )
        : collection( &aSet ), it( anIt ), bit( aBit )
      {
        if ( it != collection->my_elements.cend() )
          {
            current  = it->second;
            current &= ~( ( static_cast<Word>(1) << bit ) - static_cast<Word>(1) );
          }
        else
          current = static_cast<Word>(0);
      }

      /// Constructor from set and starting key.
      /// @param aSet a reference to the visited unordered block set
      /// @param key any key (if it is in the set, the iterator point on the key, otherwise it is iterator `cend()`.
      const_iterator( const Self& aSet, const Key& key )
        : collection( &aSet )
      {
        auto se  = collection->my_splitter.split( key );
        it       = collection->my_elements.find( se.first );
        if ( it != collection->my_elements.cend() )
          {
            bit     = se.second;
            current = it->second & ~( (static_cast<Word>(1) << bit )
                                      - static_cast<Word>(1) );
          }
        else
          {
            bit     = static_cast<Coordinate>(0);
            current = static_cast<Word>(0);
          }
      }

    private:
      friend class boost::iterator_core_access;
      void increment()
      {
        ASSERT( current != static_cast<Word>(0)
                && "Invalid increment on const_iterator" );
        current &= ~( static_cast<Word>(1) << bit );
        if ( current == static_cast<Word>(0) )
          {
            ++it;
            if ( it != collection->my_elements.cend() )
              {
                current = it->second;
                bit     = static_cast<Coordinate>(Bits::leastSignificantBit( current ));
              }
            else
              {
                current = static_cast<Word>(0);
                bit     = static_cast<Coordinate>(0); // NB: LSB(0) is undefined
              }
          }
        else
          bit = static_cast<Coordinate>(Bits::leastSignificantBit( current ));
      }

      bool equal( const const_iterator & other ) const
      {
        ASSERT( collection == other.collection );
        return it == other.it && bit == other.bit;
      }

      const Key dereference() const
      {
        return collection->my_splitter.join( it->first, bit );
      }

      /// the collection that this iterator is traversing.
      const Self*                  collection;
      /// the hidden iterator that traverses the block map.
      typename Container::const_iterator it;
      /// the current position in the block.
      Coordinate                   bit;
      /// the current value of the block, where visited bits have been erased.
      Word                         current;

    };

    /// Read-write iterator on set elements. Model of ForwardIterator.
    struct iterator
      : public boost::iterator_facade< iterator, Key const,
                                       boost::forward_traversal_tag,
                                       Key const >
    {
      friend struct UnorderedSetByBlock< Key, TSplitter, Hash, KeyEqual >;
      /// Default constructor
      iterator() : collection( nullptr ), it(),
                   bit( static_cast<Coordinate>(0) ),
                   current( static_cast<Word>(0) ) {}

      /// Constructor from set and container iterator
      /// @param aSet a reference to the visited unordered block set
      /// @param anIt an iterator in the container of this set.
      iterator( Self& aSet, typename Container::iterator anIt )
        : collection( &aSet ), it( anIt )
      {
        if ( it != collection->my_elements.end() )
          {
            current  = it->second;
            bit      = static_cast<Coordinate>(Bits::leastSignificantBit( current ));
          }
        else
          {
            current = static_cast<Word>(0);
            bit     = static_cast<Coordinate>(0);
          }
      }

      /// Constructor from set, container iterator and starting bit
      /// @param aSet a reference to the visited unordered block set
      /// @param anIt an iterator in the container of this set.
      /// @param aBit the bit index in the word pointed by \a anIt.
      iterator( Self& aSet, typename Container::iterator anIt, Coordinate aBit )
        : collection( &aSet ), it( anIt ), bit( aBit )
      {
        if ( it != collection->my_elements.end() )
          {
            current  = it->second;
            current &= ~( ( static_cast<Word>(1) << bit )
                          - static_cast<Word>(1) );
          }
        else
          current = static_cast<Word>(0);
      }

      /// Constructor from set and starting key.
      /// @param aSet a reference to the visited unordered block set
      ///
      /// @param key any key (if it is in the set, the iterator point
      /// on the key, otherwise it is iterator `end()`.
      iterator( Self& aSet, const Key& key )
        : collection( &aSet )
      {
        auto se  = collection->my_splitter.split( key );
        it       = collection->my_elements.find( se.first );
        if ( it != collection->my_elements.end() )
          {
            bit     = se.second;
            current = it->second & ~( (static_cast<Word>(1) << bit )
                                      - static_cast<Word>(1) );
          }
        else
          {
            bit     = static_cast<Coordinate>(0);
            current = static_cast<Word>(0);
          }
      }

      /// Constructor from const_iterator
      /// @param other any const_iterator
      iterator( const const_iterator& other )
        : collection( other.collection ), it( other.it ),
          bit( other.bit ), current( other.current )
      {}

      /// Move constructor from const_iterator
      /// @param other any const_iterator
      iterator( const_iterator&& other )
        : collection( std::move( other.collection ) ),
          it( std::move( other.it ) ),
          bit( std::move( other.bit ) ),
          current( std::move( other.current ) )
      {}

      /// Conversation to const_iterator
      /// @return the corresponding const_iterator
      operator const_iterator() const
      {
        return const_iterator( *collection, it, bit );
      }

    private:
      friend class boost::iterator_core_access;
      void increment()
      {
        ASSERT( current != static_cast<Word>(0)
                && "Invalid increment on iterator" );
        current &= ~( static_cast<Word>(1) << bit );
        if ( current == static_cast<Word>(0) )
          {
            ++it;
            if ( it != collection->my_elements.end() )
              {
                current = it->second;
                bit     = static_cast<Coordinate>(Bits::leastSignificantBit( current ));
              }
            else
              {
                current = static_cast<Word>(0);
                bit     = static_cast<Coordinate>(0); // NB: LSB(0) is undefined
              }
          }
        else
          bit = static_cast<Coordinate>(Bits::leastSignificantBit( current ));
      }

      bool equal( const iterator & other ) const
      {
        ASSERT( collection == other.collection );
        return it == other.it && bit == other.bit;
      }

      // The equality comparator with const_iterator must also be
      // provided to respect the standard that says that iterator and
      // const_iterator should always be comparable.
      bool equal( const const_iterator & other ) const
      {
        ASSERT( collection == other.collection );
        return it == other.it && bit == other.bit;
      }

      const Key dereference() const
      {
        return collection->my_splitter.join( it->first, bit );
      }

      /// the collection that this iterator is traversing.
      Self*                        collection;
      /// the hidden iterator that traverses the block map.
      typename Container::iterator it;
      /// the current position in the block.
      Coordinate                   bit;
      /// the current value of the block, where visited bits have been erased.
      Word                         current;

    };

    // ------------------------- standard services ----------------------------------
    /// @name Standard services (construction, initialization, assignment)
    /// @{
  public:

    /// Main constructor.
    /// @param bucket_count the initial number of buckets for the underlying container.
    /// @param splitter the splitter object for keys.
    /// @param hash the hash object for keys.
    /// @param equal the key equality comparator object for keys.
    /// @param alloc the allocator for the underlying container.
    UnorderedSetByBlock( size_type bucket_count = 23,
                         const Splitter & splitter = Splitter(),
                         const Hash& hash = Hash(),
                         const key_equal& equal = key_equal(),
                         const UnorderedMapAllocator& alloc = UnorderedMapAllocator())
      : my_splitter( splitter ),
        my_elements( bucket_count, hash, equal, alloc ),
        my_size( 0 ) {}

    /// Default destructor
    ~UnorderedSetByBlock() = default;

    /// Copy constructor
    /// @param other the object to clone
    UnorderedSetByBlock( const Self& other )
      : my_splitter( other.my_splitter ),
        my_elements( other.my_elements ),
        my_size( other.my_size )
    {}

    /// Move constructor
    /// @param other the object to clone
    UnorderedSetByBlock( Self&& other )
      : my_splitter( std::move( other.my_splitter ) ),
        my_elements( std::move( other.my_elements ) ),
        my_size( std::move( other.my_size ) )
    {}

    /// Assignment
    /// @param other the object to clone
    /// @return a reference to this
    Self& operator=( const Self& other )
    {
      if ( this != &other )
        {
          my_splitter = other.my_splitter;
          my_elements = other.my_elements;
          my_size     = other.my_size;
        }
      return *this;
    }
    /// Default move assignment
    /// @param other the object to clone
    /// @return a reference to this
    Self& operator=( Self&& other )
    {
      my_splitter = std::move( other.my_splitter );
      my_elements = std::move( other.my_elements );
      my_size     = std::move( other.my_size );
      return *this;
    }

    /// @}

    // ---------------------- iterator services -----------------------------
    /// @name Iterator services
    /// @{
  public:

    /// @return an iterator of the first stored element
    iterator begin()
    {
      return iterator( *this, my_elements.begin() );
    }
    /// @return an iterator past the last stored element
    iterator end()
    {
      return iterator( *this, my_elements.end() );
    }

    /// @return an iterator of the first stored element
    const_iterator begin() const
    {
      return const_iterator( *this, my_elements.cbegin() );
    }
    /// @return an iterator past the last stored element
    const_iterator end() const
    {
      return const_iterator( *this, my_elements.cend() );
    }

    /// @return an iterator of the first stored element
    const_iterator cbegin() const
    {
      return const_iterator( *this, my_elements.cbegin() );
    }
    /// @return an iterator past the last stored element
    const_iterator cend() const
    {
      return const_iterator( *this, my_elements.cend() );
    }

    /// @}

    // ---------------------- capacity services -----------------------------
    /// @name Capacity services
    /// @{
  public:

    /// @return 'true' iff the container is empty
    bool empty() const noexcept { return my_elements.empty(); }
    /// @return the number of elements stored in the container.
    size_type size() const noexcept { return my_size; }
    /// @return the maximum number of elements that can be stored in the container.
    size_type max_size() const noexcept { return my_elements.max_size(); }

    /// @note Specific to this data structure.
    /// @return the number of blocks stored in the container.
    size_type blocks() const noexcept { return my_elements.size(); }

    /// @note Specific to this data structure.
    /// @return an evaluation of the memory usage of this data structure.
    size_type memory_usage() const noexcept
    {
      size_type mem = (my_elements.bucket_count()+1) * sizeof( void* )
        + 2 * sizeof( size_type );
      mem += blocks() * ( sizeof( void* )       /* next */
                          + sizeof( Key )       /* key */
                          + sizeof( Word )      /* value */
                          + sizeof( size_type ) /* hash  */
                          + sizeof( void* )     /* dyn. alloc. */ );
      return mem;
    }

    /// @note Specific to this data structure.
    /// @return an evaluation of the memory usage of the same data
    /// stored in an unordered set.
    size_type memory_usage_unordered_set() const noexcept
    {
      size_type mem = (my_elements.bucket_count()+1) * sizeof( void* )
        + 2 * sizeof( size_type );
      mem += size() * ( sizeof( void* )       /* next */
                        + sizeof( Key )       /* key */
                        + sizeof( size_type ) /* hash  */
                        + sizeof( void* )     /* dyn. alloc. */ );
      return mem;
    }

    /// @}

    // ---------------------- modifier services -----------------------------
    /// @name Modifier services
    /// @{
  public:

    /// Clears the container
    void clear() noexcept
    {
      my_elements.clear();
      my_size = 0;
    }

    /// Exchanges the contents of the container with those of
    /// other. Does not invoke any move, copy, or swap operations on
    /// individual elements.
    ///
    /// @param other the other set to exchange with.
    ///
    /// @note All iterators and references remain valid. The
    /// past-the-end iterator is invalidated. The Hash and KeyEqual
    /// objects must be Swappable, and they are exchanged using
    /// unqualified calls to non-member swap.
    void swap( Self& other ) noexcept
    {
      std::swap( my_splitter, other.my_splitter );
      std::swap( my_elements, other.my_elements );
      std::swap( my_size, other.my_size );
    }

    /**
     *  @brief Attempts to insert an element into the set.
     *  @param  value  Element to be inserted.
     *  @return  A pair, of which the first element is an iterator that points
     *           to the possibly inserted element, and the second is a bool
     *           that is true if the element was actually inserted.
     *
     *  This function attempts to insert an element into the set.  A set
     *  relies on unique keys and thus an element is only inserted if it is
     *  not already present in the set.
     *
     *  Insertion requires amortized constant time.
     */
    std::pair<iterator,bool> insert( const value_type& value )
    {
      const auto se = my_splitter.split( value );
      auto it = my_elements.find( se.first );
      if ( it == my_elements.end() )
        {
          auto   p = my_elements.insert
            ( std::make_pair( se.first,
                              static_cast<Word>(1) << se.second ) );
          my_size += 1;
          return std::make_pair( iterator( *this, p.first, se.second ), true );
        }
      else
        {
          bool exist = it->second & ( static_cast<Word>(1) << se.second );
          if ( ! exist )
            {
              it->second |= static_cast<Word>(1) << se.second;
              my_size    += 1;
            }
          return std::make_pair( iterator( *this, it, se.second ), ! exist );
        }
    }

    /**
     *  @brief Inserts a range of element into the set.
     *
     *  @tparam InputIterator any model of input iterator 
     *  @param[in]  first an iterator pointing on the first element of the range
     *  @param[in]  last an iterator pointing after the last element of the range
     *
     *  This function inserts a range of elements into the set.  A set
     *  relies on unique keys and thus an element is only inserted if it is
     *  not already present in the set.
     *
     *  Each insertion requires amortized constant time.
     */
    template <typename InputIterator>
    void  insert( InputIterator first, InputIterator last )
    {
      for ( ; first != last; ++first ) insert( *first );
    }
    
    /**
     *  @brief Attempts to build and insert an element into the set.
     *  @param __args  Arguments used to generate an element.
     *  @return  A pair, of which the first element is an iterator that points
     *           to the possibly inserted element, and the second is a bool
     *           that is true if the element was actually inserted.
     *
     *  This function attempts to build and insert an element into the set.
     *  A set relies on unique keys and thus an element is only inserted if
     *  it is not already present in the set.
     *
     *  Insertion takes amortized constant time.
     */
    template<typename... _Args>
    std::pair<iterator, bool>
    emplace(_Args&&... __args)
    {
      const auto se = my_splitter.split( Key( std::forward<_Args>(__args)... ) );
      auto it = my_elements.find( se.first );
      if ( it == my_elements.end() )
        {
          auto   p =
            my_elements.insert( std::make_pair( se.first,
                                                static_cast<Word>(1) << se.second ) );
          my_size += 1;
          return std::make_pair( iterator( *this, p.first, se.second ), true );
        }
      else
        {
          bool exist = it->second & ( static_cast<Word>(1) << se.second );
          if ( ! exist )
            {
              it->second |= static_cast<Word>(1) << se.second;
              my_size    += 1;
            }
          return std::make_pair( iterator( *this, it, se.second ), ! exist );
        }
    }

    /// Removes specified element from the container.
    /// @param pos a valid iterator in this data structure
    /// @return the iterator following the last removed element.
    ///
    /// @note References and iterators to the erased elements are
    /// invalidated. Other iterators and references are not
    /// invalidated.
    ///
    /// @pre The iterator pos must be valid and dereferenceable. Thus
    /// the end() iterator (which is valid, but is not
    /// dereferenceable) cannot be used as a value for pos.
    iterator erase( const_iterator pos ) noexcept
    {
      ASSERT( this == pos.collection );
      ASSERT( pos  != cend() );
      ASSERT( ( pos.it->second & ( static_cast<Word>(1) << pos.bit ) )
              != static_cast<Word>(0) );
      my_size -= 1;
      Word & w = const_cast< Word& >( pos.it->second );
      if ( ( w &= ~( static_cast<Word>(1) << pos.bit ) )
           == static_cast<Word>(0) )
        return iterator( *this, my_elements.erase( pos.it ) );
      else
        return iterator( *this, my_elements.erase( pos.it, pos.it ), pos.bit );
    }

    /// Removes the elements in the range [first; last), which must be
    /// a valid range in *this.
    ///
    /// @param first an iterator such that [first; last) is a valid
    /// range in this data structure
    ///
    /// @param last an iterator such that [first; last) is a valid
    /// range in this data structure
    ///
    /// @return the iterator following the last removed element.
    ///
    /// @note References and iterators to the erased elements are
    /// invalidated. Other iterators and references are not
    /// invalidated.
    ///
    iterator erase( const_iterator first, const_iterator last ) noexcept
    {
      ASSERT( this == first.collection );
      ASSERT( this == last.collection );
      if ( first == cend() ) return end();
      auto itB = first.it;
      auto itE = last.it;
      Word mask = static_cast<Word>(0);
      // Take care of range over one block only
      if ( itB == itE )
        {
          while ( first != last )
            {
              mask    |= static_cast<Word>(1) << first.bit;
              my_size -= 1;
              ++first;
            }
          my_elements[ itB->first ] &= ~mask;
          return iterator( *this,
                           my_elements.find( itE->first ),
                           last.bit ); // must be valid
        }
      // Take care of first element.
      while ( first.it == itB )
        {
          mask    |= static_cast<Word>(1) << first.bit;
          my_size -= 1;
          ++first;
        }
      // Erase first block if empty
      if ( ( my_elements[ itB->first ] &= ~mask )
           == static_cast<Word>(0) )
        my_elements.erase( itB );
      // Count erased elements in main range.
      for ( itB = first.it; itB != itE; ++itB )
        my_size -= Bits::nbSetBits( itB->second );
      // Erase elements in main range
      my_elements.erase( first.it, itE );
      // Take care of last element.
      if ( itE == my_elements.cend() ) return end();
      itB   = itE;
      first = const_iterator( *this, itB );
      mask  = static_cast<Word>(0);
      while ( first != last )
        {
          mask    |= static_cast<Word>(1) << first.bit;
          my_size -= 1;
          ++first;
        }
      // Erase last block if empty
      if ( ( my_elements[ itB->first ] &= ~mask )
           == static_cast<Word>(0) )
        my_elements.erase( itB );
      return iterator( *this,
                       my_elements.find( itE->first ),
                       last.bit ); // must be valid or end.
    }

    /// Removes specified element from the container, if it exists.
    /// @param key the value to erase from the set
    /// @return the number of value removed from the set (either 0 or 1 ).
    ///
    /// @note References and iterators to the erased elements are
    /// invalidated. Other iterators and references are not
    /// invalidated.
    size_type erase( const key_type& key )
    {
      auto it = find( key );
      if ( it != end() )
        {
          erase( it );
          return 1;
        }
      else return 0;
    }

    /// @}

    // ---------------------- lookup services -----------------------------
    /// @name Lookup services
    /// @{
  public:

    /// Finds an element with key equivalent to key.
    /// @param key the value to look-up.
    ///
    /// @return an iterator pointing to \a key or `end()` if the key
    /// is not the set.
    iterator find( const Key& key )
    {
      const auto se = my_splitter.split( key );
      const auto it = my_elements.find( se.first );
      if ( it == my_elements.end() ) return end();
      const bool exist = it->second & ( static_cast<Word>(1) << se.second );
      if ( exist ) return iterator( *this, it, se.second );
      else         return end();
    }

    /// Finds an element with key equivalent to key.
    /// @param key the value to look-up.
    ///
    /// @return a const iterator pointing to \a key or `end()` if the key
    /// is not the set.
    const_iterator find( const Key& key ) const
    {
      const auto se = my_splitter.split( key );
      const auto it = my_elements.find( se.first );
      if ( it == my_elements.cend() ) return cend();
      const bool exist = it->second & ( static_cast<Word>(1) << se.second );
      if ( exist ) return const_iterator( *this, it, se.second );
      else         return cend();
    }

    /// @param key the value to look-up.
    /// @return the number of elements with key that compares equal to
    /// the specified argument key, which is either 1 or 0 since this
    /// container does not allow duplicates.
    size_type count( const Key& key ) const
    {
      const auto se = my_splitter.split( key );
      const auto it = my_elements.find( se.first );
      if ( it == my_elements.cend() ) return 0;
      const bool exist = it->second & ( static_cast<Word>(1) << se.second );
      return exist ? 1 : 0;
    }

    /// Returns the bounds of a range that includes all the elements
    /// that compare equal to k. In set containers, where keys are
    /// unique, the range will include one element at most.
    ///
    /// @param key the value to look-up.
    ///
    /// @return a range containing the sought element or an empty
    /// range if \a key is not in this set.
    std::pair<iterator,iterator>
    equal_range( const Key & key )
    {
      iterator first = find( key );
      if ( first != end() )
        {
          iterator last = first;
          return std::make_pair( first, ++last );
        }
      else return std::make_pair( first, first );
    }

    /// Returns the bounds of a range that includes all the elements
    /// that compare equal to k. In set containers, where keys are
    /// unique, the range will include one element at most.
    ///
    /// @param key the value to look-up.
    ///
    /// @return a range containing the sought element or an empty
    /// range if \a key is not in this set.
    std::pair<const_iterator,const_iterator>
    equal_range( const Key & key ) const
    {
      const_iterator first = find( key );
      if ( first != end() )
        {
          const_iterator last = first;
          return std::make_pair( first, ++last );
        }
      else return std::make_pair( first, first );
    }

    /// @}

    // ---------------------- set services -------------------------
    /// @name Set services
    /// @{
  public:

    /// @param other any unordered set with same sort of elements
    /// @return 'true' if and only if this set includes \a other, and
    /// 'false' otherwise.
    ///
    /// @note Much faster that using \ref count or \ref find on each
    /// element, since it proceeds block by block.
    bool includes( const Self& other ) const
    {
      return internal_includes_by_map_iterator( other );
    }

  protected:
    /// Performs includes operation using underlying container iterator.
    /// @param other any unordered set with same sort of elements
    /// @return 'true' if and only if this set includes \a other, and
    /// 'false' otherwise.
    bool internal_includes_by_map_iterator( const Self& other ) const
    {
      auto       itMap_other    = other.my_elements.cbegin();
      const auto itEndMap_other = other.my_elements.cend();
      const auto itEndMap_this  = my_elements.cend();
      for ( ; itMap_other != itEndMap_other; ++itMap_other )
        {
          const auto itMap_this = my_elements.find( itMap_other->first );
          if ( itMap_this == itEndMap_this ) return false;
          const Word w_this  = itMap_this->second;
          const Word w_other = itMap_other->second;
          if ( ( w_this & w_other ) != w_other ) return false;
        }
      return true;
    }

    /// Performs includes operation using underlying container
    /// iterator. Verbose version for debug.
    /// @param other any unordered set with same sort of elements
    /// @return 'true' if and only if this set includes \a other, and
    /// 'false' otherwise.
    bool internal_trace_includes_by_map_iterator( const Self& other ) const
    {
      trace.info() << "[trace_includes_v1] #this=" << size()
                   << " #other=" << other.size() << std::endl;
      auto       itMap_other    = other.my_elements.cbegin();
      const auto itEndMap_other = other.my_elements.cend();
      const auto itEndMap_this  = my_elements.cend();
      for ( ; itMap_other != itEndMap_other; ++itMap_other )
        {
          trace.info() << "other: cell=" << itMap_other->first
                << " value=" << std::hex << itMap_other->second << std::endl;
          const auto itMap_this = my_elements.find( itMap_other->first );
          if ( itMap_this != itEndMap_this )
            trace.info() << "this : cell=" << itMap_this->first
                  << " value=" << std::hex << itMap_this->second << std::endl;
          else
            trace.info() << "this : end cell" << std::endl;
          if ( itMap_this == itEndMap_this ) return false;
          const Word w_this  = itMap_this->second;
          const Word w_other = itMap_other->second;
          if ( ( w_this & w_other ) != w_other ) return false;
        }
      return true;
    }

    /// Performs includes operation using iterators and big steps,
    /// slightly slower than internal_includes_by_map_iterator.
    /// @param other any unordered set with same sort of elements
    /// @return 'true' if and only if this set includes \a other, and
    /// 'false' otherwise.
    bool internal_includes_by_iterator( const Self& other ) const
    {
      auto it_other  = other.cbegin();
      auto itEnd_other = other.cend();
      while ( it_other != itEnd_other )
        {
          const auto it_this = find( *it_other );
          if ( it_this == cend() ) return false;
          auto   itMap_other = it_other.it;
          const Word w_this  = it_this.it->second;
          const Word w_other = itMap_other->second;
          if ( ( w_this & w_other ) != w_other ) return false;
          it_other = const_iterator( other, ++itMap_other );
        }
      return true;
    }

    /// Performs includes operation using iterators and big steps,
    /// slightly slower than internal_includes_by_map_iterator. Verbose
    /// version for debug.
    ///
    /// @param other any unordered set with same sort of elements
    /// @return 'true' if and only if this set includes \a other, and
    /// 'false' otherwise.
    bool internal_trace_includes_by_iterator( const Self& other ) const
    {
      trace.info() << "[trace_includes_v2] #this=" << size()
                   << " #other=" << other.size() << std::endl;
      auto it_other  = other.cbegin();
      auto itEnd_other = other.cend();
      while ( it_other != itEnd_other )
        {
          trace.info() << "other: cell=" << it_other.it->first
                << " value=" << std::hex << it_other.it->second << std::endl;
          const auto it_this = find( *it_other );
          if ( it_this != cend() )
            trace.info() << "this : cell=" << it_this.it->first
                  << " value=" << std::hex << it_this.it->second << std::endl;
          else
            trace.info() << "this : end cell" << std::endl;
          if ( it_this == cend() ) return false;
          auto   itMap_other = it_other.it;
          const Word w_this  = it_this.it->second;
          const Word w_other = itMap_other->second;
          if ( ( w_this & w_other ) != w_other ) return false;
          it_other = const_iterator( other, ++itMap_other );
        }
      return true;
    }

    /// @}

    // ---------------------- hash policy services -----------------------------
    /// @name Hash policy services
    /// @{
  public:

    /**
     * Sets the number of buckets to the number needed to accomodate
     * at least count elements without exceeding maximum load factor
     * and rehashes the container, i.e. puts the elements into
     * appropriate buckets considering that total number of buckets
     * has changed. Effectively calls `rehash(std::ceil(count /
     * max_load_factor()))`.
     *
     * @param block_count new capacity of the container (should be thought
     * in terms of number of expected blocks).
     */
    void reserve( size_type block_count )
    {
      my_elements.reserve( block_count );
    }

    /// @}

  private:
    // -------------------------- data ---------------------------------
    /// The splitter object
    Splitter  my_splitter;
    /// the unordered_set containing the elements
    Container my_elements;
    /// the number of elements
    size_type my_size;
  };


  /// This functions swaps in O(1) the given two sets. Calls
  /// UnorderedSetByBlock::swap.
  ///
  /// @tparam Key the type of integral array.
  /// @tparam TSplitter the type for splitting a key into a block and a bit (see \ref Splitter).
  /// @tparam Hash the type that provides a hasher for Key.
  /// @tparam KeyEqual the type that provides an equality comparator for Key.
  /// @tparam UnorderedMapAllocator the type that provides an allocator for the underlying unordered_map container.
  template < typename Key,
             typename TSplitter,
             class Hash,
             class KeyEqual,
             class UnorderedMapAllocator >
  void swap
  ( UnorderedSetByBlock< Key, TSplitter, Hash, KeyEqual, UnorderedMapAllocator >& s1,
    UnorderedSetByBlock< Key, TSplitter, Hash, KeyEqual, UnorderedMapAllocator >& s2 )
    noexcept
  {
    s1.swap( s2 );
  }

} // namespace DGtal

#endif // #ifndef UNORDEREDSETBYBLOCK_HPP
