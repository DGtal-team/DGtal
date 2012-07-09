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
 * @file LabelledMap.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/07/05
 *
 * Header file for module LabelledMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(LabelledMap_RECURSES)
#error Recursive header files inclusion detected in LabelledMap.h
#else // defined(LabelledMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LabelledMap_RECURSES

#if !defined LabelledMap_h
/** Prevents repeated inclusion of headers. */
#define LabelledMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstring>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Labels.h"
//#include "DGtal/base/FakeKeyValuePair.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class LabelledMap
  /**
     Description of template class 'LabelledMap' <p> \brief Aim:
     Represents a map label -> data, where the label is an integer
     between 0 and a constant L-1. It is based on a binary coding of
     labels and a mixed list/array structure. The assumption is that
     the number of used labels is much less than L. The objective is
     to minimize the memory usage.

@verbatim
V[ 0 ] is the data of the first set label.
V[ 1 ] is the data of the second set label.
...

if less than 3 datas and N = 2
+------+------+------+------+------+
|labels| V[0] | V[1] | ...  |  0   |
+------+------+------+------+------+

if only 3 datas and N = 2
+------+------+------+------+------+
|labels| V[0] | V[1] | V[2] | V[3] |
+------+------+------+------+------+

if more than 3 datas and N = 2, M = 4
+------+------+------+------+------+        +------+------+------+------+------+
|labels| V[0] | V[1] | V[2] | ptr --------> | V[3] | V[4] | V[5] | V[6] | ptr --------> ...
+------+------+------+------+------+        +------+------+------+------+------+

@endverbatim

     This structure is related to the IndexedListWithBlocks, except
     that it stores the mapping label -> index. The (maximum) number
     of possible labels is fixed at instantiation for optimisation
     purposes.

     Such a structure is useful when:
     - the expected size of this container is small, but may sometimes increase.
     - the user wishes sometimes to insert a new data or erase another data.
     - the user wishes to have an access to the datas that is as fast as possible given a valid label.
     - one wishes to limit as possible the memory usage.
     - generally this structure is embedded as the data of a big array.

     Model of boost::PairAssociativeContainer and
     boost::SimpleAssociativeContainer.

     @tparam TData the type for the datas stored in the list.
     @tparam L the maximum number of labels.
     @tparam TWord the integer used to store the labels (if L >= log_2( digits( TWord ) ) then several consecutive words are stored.), e.g. DGtal::uint8_t.
     @tparam N the number of data stored in the first block.
     @tparam M the number of data stored in the further blocks.

     NB: In the following, we use the notations
     - n is the size of the container
     - b is the number of blocks ( b = 1 + (size()-N) / M ).
   */
  template <typename TData, unsigned int L, typename TWord,
            unsigned int N, unsigned int M>
  class LabelledMap
  {
    BOOST_STATIC_ASSERT( L >= 1 );
    BOOST_STATIC_ASSERT( N >= 1 );
    BOOST_STATIC_ASSERT( M >= 2 );
  public:
    // ----------------------- Public types ------------------------------
    typedef TData Data;
    typedef TWord Word;
    typedef Labels<L, Word> LabelsType;
    typedef typename LabelsType::Label Label;
    typedef Label Key;
    typedef std::pair<const Key, Data> Value;
    //typedef FakeKeyValuePair<Key, Data> Value;

    typedef typename LabelsType::ConstIterator LabelsConstIterator;
    typedef std::ptrdiff_t DifferenceType;
    typedef std::size_t SizeType;
    typedef Value& Reference;
    typedef Value* Pointer;
    typedef const Value& ConstReference;
    typedef const Value* ConstPointer;

    //class Iterator;      //< Forward declaration
    class ConstIterator; //< Forward declaration

    // ----------------------- Standard types ------------------------------
    typedef Value value_type;
    typedef Data data_type;
    typedef DifferenceType difference_type;
    typedef Reference reference;
    typedef Pointer pointer;
    typedef ConstReference const_reference;
    typedef ConstPointer const_pointer;
    typedef SizeType size_type;
    typedef ConstIterator iterator;
    typedef ConstIterator const_iterator;
    
    struct FirstBlock; //< Forward declaration
    struct AnyBlock; //< Forward declaration

    union BlockPointer {
      FirstBlock* first;
      AnyBlock* any;
    };

    /// Used in first block to finish it or to point to the next block.
    union DataOrBlockPointer {
      Data lastData; // used when at the end of the list
      AnyBlock* nextBlock;  // used otherwise
    };

    /// Represents the first block in the container.
    /// Internal structure.
    struct FirstBlock {
      inline
      FirstBlock() 
      { data.nextBlock = 0; }

      inline
      void insert( unsigned int idx, unsigned int size, const Data & v )
      {
	if ( size <= N )
	  {
	    ASSERT( idx <= size );
	    // works also in the case we use 'data' to store a N+1-th data.
	    std::copy_backward( datas + idx, datas + size, datas + size + 1 );
	    datas[ idx ] = v;
	  }
	else if ( size == (N+1) )
	  {
	    ASSERT( idx <= size );
	    // This cannot be tested.
	    // ASSERT( data.nextBlock == 0 );
	    AnyBlock* next = new AnyBlock;
	    if ( idx < N )
	      {
		next->datas[ 0 ] = datas[ N - 1 ];
		next->datas[ 1 ] = data.lastData;
		std::copy_backward( datas + idx, datas + N - 1, datas + N );
		datas[ idx ] = v;
	      }
	    else if ( idx == N )
	      {
		next->datas[ 0 ] = v;
		next->datas[ 1 ] = data.lastData;
	      }
	    else if ( idx > N )
	      {
		next->datas[ 0 ] = data.lastData;
		next->datas[ 1 ] = v;
	      }
	    data.nextBlock = next;
	  }
	else // size > N + 1
	  {
	    if ( idx < N )
	      {
		Data v1 = datas[ N - 1 ];
		std::copy_backward( datas + idx, datas + N - 1, datas + N );
		data.nextBlock->insert( 0, size - N, v1 );
		datas[ idx ] = v;
	      }
	    else
	      data.nextBlock->insert( idx - N, size - N, v );
	  }
	++size;
      }

      inline 
      void erase( unsigned int idx, unsigned int size )
      {
	// std::cerr << "FirstBlock::erase(" << idx << ")"
	// 	  << " this=" << this
	// 	  << " next=" << data.nextBlock
	// 	  << std::endl;
	ASSERT( idx < size );
	if ( size <= ( N + 1 ) )
	  {
	    // works also in the case we use 'data' to store a N+1-th data.
	    std::copy( datas + idx + 1, datas + size, datas + idx );
	  }
	else // size > N + 1
	  {
	    if ( idx < N )
	      {
		std::copy( datas + idx + 1, datas + N, datas + idx );
		datas[ N - 1 ] = data.nextBlock->datas[ 0 ];
		data.nextBlock = data.nextBlock->erase( 0, size - N );
	      }
	    else
	      data.nextBlock = data.nextBlock->erase( idx - N, size - N );
	  }
	--size;
      }

      Data datas[ N ];
      DataOrBlockPointer data;
    };

    /// Represents a block (except the first) in the container.
    /// Internal structure.
    struct AnyBlock {
      inline AnyBlock() : next( 0 ) {}

      inline
      void insert( unsigned int idx, unsigned int size, const Data & v )
      {
        ASSERT( idx <= size );
	if ( idx >= M ) 
	  {
	    if ( next == 0 )
	      {
		ASSERT( idx == M );
		next = new AnyBlock;
                next->datas[ 0 ] = v;
	      }
            else
              next->insert( idx - M, size - M, v );
	  }
	else 
	  { // idx < M
            if ( size < ( M - 1) )
              {
                std::copy_backward( datas + idx, datas + size, 
                                    datas + size + 1 );
                datas[ idx ] = v;
              }
            else
              {
                Data v1 = datas[ M - 1 ];
                std::copy_backward( datas + idx, datas + M - 1, datas + M );
                datas[ idx ] = v;
                if ( size >= M )
                  {
                    if ( next == 0 )
                      {
                        ASSERT( size == M );
                        next = new AnyBlock;
                        next->datas[ 0 ] = v1;
                      }
                    else
                      next->insert( 0, size - M, v1 );
                  }
              }
	  }
      }

      inline 
      AnyBlock* erase( unsigned int idx, unsigned int size )
      {
	// std::cerr << "AnyBlock::erase(" << idx << "," << size << ")" 
	// 	  << " this=" << this
	// 	  << " next=" << next
	// 	  << std::endl;
        if ( size == 1 )
          {
            ASSERT( idx == 0 );
            delete this;
            return 0;
          }
	if ( idx < M )
	  {
	    std::copy( datas + idx + 1, datas + M, datas + idx );
	    if ( next != 0 )
	      {
		ASSERT( size > M );
		datas[ M - 1 ] = next->datas[ 0 ];
                next = next->erase( 0, size - M );
	      }
	  }
	else
	  next = next->erase( idx - M, size - M );
	return this;
      }


      Data datas[ M ];
      AnyBlock* next;
    };

    /**
       Pseudo-random iterator to visit LabelledMap (it is
       only a random forward iterator).  Model of
       boost::ForwardIterator. Provides also + and += arithmetic.
    */
    class BlockIterator {
    public:
      typedef BlockIterator Self;
      typedef TData Value;
      typedef Value* Pointer;
      typedef Value& Reference;
      typedef std::ptrdiff_t DifferenceType; //< only positive offsets allowed.

      // ----------------------- std types ----------------------------------
      typedef Value value_type;
      typedef std::size_t size_type;
      typedef DifferenceType difference_type;
      typedef Pointer pointer;
      typedef Reference reference;
      //typedef const Reference const_reference;
      typedef std::forward_iterator_tag iterator_category;


    protected:
      unsigned int myIdx;      //< current index in \a myDatas of the iterator
      unsigned int myNbDatas; //< number of valid datas in array \a myDatas
      Data* myDatas;         //< array of \a myNbDatas datas.
      AnyBlock* myNext;        //< pointer to next block or 0 if last block.

      friend class LabelledMap;

    protected:
      /**
	 Constructor from first block and index. Used by class LabelledMap.
      */
      BlockIterator( FirstBlock & block, unsigned int idx, unsigned int size );
      
    public:
      /**
	 Default destructor.
      */
      ~BlockIterator();

      /**
	 Default constructor.
      */
      BlockIterator();

      /**
	 Copy constructor.
	 @param other the object to clone.
      */
      BlockIterator( const BlockIterator & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       */
      Self & operator= ( const Self & other );
      
      /**
	 Dereference operator.
	 @return the current data of the iterator, if valid.
      */
      Reference operator*() const;
     
      /**
	 Pointer dereference operator.
	 @return a non-mutable pointer on the current data.
      */  
      Pointer operator->() const;
      
      /** 
	  Pre-increment operator.
	  @return a reference to itself.
      */
      Self& operator++();
      
      /** 
	  Post-increment operator.
	  @return a reference to itself.
      */
      Self operator++( int );

      /** 
	  Addition operator. Moves the iterator at position + \a n.
	  @param n any positive integer
	  @return a reference to itself.
      */
      Self& operator+=( DifferenceType n );

      /** 
	  Positive offset dereference operator. Moves the iterator at position + \a n.
	  @param n any positive integer
	  @return a reference to itself.
      */
      Reference operator[]( DifferenceType n ) const;
    
      /**
	 Equality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on the same element.
      */
      bool operator==( const Self & other ) const;
      
      /**
	 Inequality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on different elements.
      */
      bool operator!=( const Self & other ) const;
      
      
    };


    /**
       Pseudo-random iterator to visit LabelledMap (it is
       only a random forward iterator).  Model of
       boost::ForwardIterator. Provides also + and += arithmetic.
    */
    class BlockConstIterator {
    public:
      typedef BlockConstIterator Self;
      typedef TData Value;
      typedef const Value* Pointer;
      typedef const Value& Reference;
      typedef std::ptrdiff_t DifferenceType; //< only positive offsets allowed.

      // ----------------------- std types ----------------------------------
      typedef Value value_type;
      typedef std::size_t size_type;
      typedef DifferenceType difference_type;
      typedef Pointer pointer;
      typedef Reference reference;
      // typedef Reference const_reference;
      typedef std::forward_iterator_tag iterator_category;


    protected:
      unsigned int myIdx;      //< current index in \a myDatas of the iterator
      unsigned int myNbDatas; //< number of valid datas in array \a myDatas
      const Data* myDatas;   //< array of \a myNbDatas datas.
      const AnyBlock* myNext;  //< pointer to next block or 0 if last block.

      friend class LabelledMap;

    protected:
      /**
	 Constructor from first block and index.
         Used by class LabelledMap.
      */
      BlockConstIterator( const FirstBlock & block, unsigned int idx, unsigned int size );
      
    public:
      /**
	 Default destructor.
      */
      ~BlockConstIterator();

      /**
	 Default constructor.
      */
      BlockConstIterator();

      /**
	 Copy constructor.
	 @param other the object to clone.
      */
      BlockConstIterator( const BlockConstIterator & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       */
      Self & operator= ( const Self & other );
      
      /**
	 Dereference operator.
	 @return the current data of the iterator, if valid.
      */
      Reference operator*() const;
     
      /**
	 Pointer dereference operator.
	 @return a non-mutable pointer on the current data.
      */  
      Pointer operator->() const;
      
      /** 
	  Pre-increment operator.
	  @return a reference to itself.
      */
      Self& operator++();
      
      /** 
	  Post-increment operator.
	  @return a reference to itself.
      */
      Self operator++( int );

      /** 
	  Addition operator. Moves the iterator at position + \a n.
	  @param n any positive integer
	  @return a reference to itself.
      */
      Self& operator+=( DifferenceType n );

      /** 
	  Positive offset dereference operator. Moves the iterator at position + \a n.
	  @param n any positive integer
	  @return a reference to itself.
      */
      Reference operator[]( DifferenceType n ) const;
    
      /**
	 Equality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on the same element.
      */
      bool operator==( const Self & other ) const;
      
      /**
	 Inequality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on different elements.
      */
      bool operator!=( const Self & other ) const;
      
      
    };

    // ----------------------- Iterator services ------------------------------
  public:

    // /**
    //    This class allows to visit all stored pairs (key,value).
    // */
    // class Iterator {
    // public:
    //   friend class LabelledMap;
    //   typedef Iterator Self;
    //   typedef typename LabelledMap<TData, L, TWord, N, M>::Value Value;
    //   typedef Value* Pointer;
    //   typedef Value& Reference;
    //   typedef std::ptrdiff_t DifferenceType;

    //   // ----------------------- std types ----------------------------------
    //   typedef Value value_type;
    //   typedef std::size_t size_type;
    //   typedef DifferenceType difference_type;
    //   typedef Pointer pointer;
    //   typedef Reference reference;
    //   // typedef Reference const_reference;
    //   typedef std::forward_iterator_tag iterator_category;

    // private:
    //   /// Iterator to visit keys.
    //   LabelsConstIterator myLabelsIt; 
    //   /// Iterator to visit datas.
    //   BlockIterator myBlockIt;
    //   Value myValue;

    // protected:
    //   /**
    //      Constructor. Internal. Used by LabelledMap.
    //   */
    //   Iterator( LabelsConstIterator lIt, BlockIterator bIt );

    // public:
    //   /**
    //      Default destructor.
    //   */
    //   ~Iterator();

    //   /**
    //      Default constructor.
    //   */
    //   Iterator();

    //   /**
    //      Copy constructor.
    //      @param other the object to clone.
    //   */
    //   Iterator( const Iterator & other );

    //   /**
    //    * Assignment.
    //    * @param other the object to copy.
    //    * @return a reference on 'this'.
    //    */
    //   Self & operator= ( const Self & other );
      
    //   /**
    //      Dereference operator.
    //      @return the current data of the iterator, if valid.
    //   */
    //   Reference operator*() const;
     
    //   /**
    //      Pointer dereference operator.
    //      \b Warning: Not thread-safe !! Use operator* instead.
    //      @return a non-mutable pointer on the current data.
    //   */  
    //   Pointer operator->() const;
      
    //   /** 
    //       Pre-increment operator.
    //       @return a reference to itself.
    //   */
    //   Self& operator++();
      
    //   /** 
    //       Post-increment operator.
    //       @return a reference to itself.
    //   */
    //   Self operator++( int );

    //   /**
    //      Equality operator.
    //      @param other any other iterator.
    //      @return 'true' iff the iterators points on the same element.
    //   */
    //   bool operator==( const Self & other ) const;
      
    //   /**
    //      Inequality operator.
    //      @param other any other iterator.
    //      @return 'true' iff the iterators points on different elements.
    //   */
    //   bool operator!=( const Self & other ) const;

    // };


    /**
       This class allows to visit all stored pairs (key,value).
    */
    class ConstIterator {
    public:
      friend class LabelledMap;
      typedef ConstIterator Self;
      typedef const typename LabelledMap<TData, L, TWord, N, M>::Value Value;
      typedef Value* Pointer;
      typedef Value Reference;
      typedef std::ptrdiff_t DifferenceType;

      // ----------------------- std types ----------------------------------
      typedef Value value_type;
      typedef std::size_t size_type;
      typedef DifferenceType difference_type;
      typedef Pointer pointer;
      typedef Reference reference;
      // typedef Reference const_reference;
      typedef std::forward_iterator_tag iterator_category;

    private:
      /// ConstIterator to visit keys.
      LabelsConstIterator myLabelsIt; 
      /// ConstIterator to visit datas.
      BlockConstIterator myBlockIt;

    protected:
      /**
	 Constructor. Internal. Used by LabelledMap.
      */
      ConstIterator( LabelsConstIterator lIt, BlockConstIterator bIt );

    public:
      /**
	 Default destructor.
      */
      ~ConstIterator();

      /**
	 Default constructor.
      */
      ConstIterator();

      /**
	 Copy constructor.
	 @param other the object to clone.
      */
      ConstIterator( const ConstIterator & other );

      /**
       * Assignment.
       * @param other the object to copy.
       * @return a reference on 'this'.
       */
      Self & operator= ( const Self & other );
      
      /**
	 Dereference operator.
	 @return the current data of the iterator, if valid.
      */
      Reference operator*() const;
     
      /**
	 Pointer dereference operator.
         \b Warning: Not thread-safe !! Use operator* instead.
	 @return a non-mutable pointer on the current data.
      */  
      Pointer operator->() const;
      
      /** 
	  Pre-increment operator.
	  @return a reference to itself.
      */
      Self& operator++();
      
      /** 
	  Post-increment operator.
	  @return a reference to itself.
      */
      Self operator++( int );

      /**
	 Equality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on the same element.
      */
      bool operator==( const Self & other ) const;
      
      /**
	 Inequality operator.
	 @param other any other iterator.
	 @return 'true' iff the iterators points on different elements.
      */
      bool operator!=( const Self & other ) const;

    };
    
    /// non-mutable class via iterators.
    typedef ConstIterator Iterator;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    LabelledMap();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    LabelledMap ( const LabelledMap & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    LabelledMap & operator= ( const LabelledMap & other );

    /**
     * Destructor.
     */
    ~LabelledMap();

    // ----------------------- Container services -----------------------------
  public:
    
    /**
       The number of datas stored in the structure. O(1) complexity.
     */
    SizeType size() const;

    /**
       @return 'true' if and only if the container is empty. O(1)
     */
    bool empty() const;

    /**
       The maximum number of datas storable in the structure.
     */
    SizeType max_size() const;

    /**
       The number of datas currently allocated in the structure.
     */
    SizeType capacity() const;

    /**
       Removes all the datas stored in the structure. O(b)
       complexity.
     */
    void clear();

    /**
       Insertion of a new data at given label. Non-standard (should
       return a pair<iterator,bool>).

       @param val a pair<label,data>
    */
    void insert( const Value & val );

    /**
       Random unprotected read-write access to data at position \a idx
       @param idx the index of the data in the container.
       @return a reference on the data.
       @pre idx < size()
       NB: O( b ), E = O( 1 + ceil( ( idx - N ) / M ) )
    */
    Data & blockAt( unsigned int idx );

    /**
       Random unprotected read access to data at position \a idx
       @param idx the index of the data in the container.
       @return a const reference on the data.
       @pre idx < size()
       NB: O( b ), E = O( 1 + ceil( ( idx - N ) / M ) )
    */
    const Data & blockAt( unsigned int idx ) const;

    /**
       Insertion of a new data at given position. The former data at
       this position and the next ones are shifted.

       @param idx the index of the data in the container.
       @pre idx <= size() (if size(), inserts at the end.
       @param data the data to insert.
       NB: O( n ), E = O( n - idx )
    */
    void blockInsert( unsigned int idx, const Data & data );

    /**
       Removal of a data at a given position. Following datas are shifted.

       @param idx the index of the data in the container.
       @pre idx < size()
       NB: O( n ), E = O( n - idx )
    */
    void blockErase( unsigned int idx );

    // /// @return an iterator pointing on the first element in the container.
    // Iterator begin();

    // /// @return an iterator pointing after the last element in the container.
    // Iterator end();

    /// @return an iterator pointing on the first element in the container.
    ConstIterator begin() const;

    /// @return an iterator pointing after the last element in the container.
    ConstIterator end() const;

    /// @return an iterator pointing on the first element in the container.
    BlockIterator blockBegin();

    /// @return an iterator pointing after the last element in the container.
    BlockIterator blockEnd();

    /// @return an iterator pointing on the first element in the container.
    BlockConstIterator blockBegin() const;

    /// @return an iterator pointing after the last element in the container.
    BlockConstIterator blockEnd() const;

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

    /// Stores the labels for this sequence of datas.
    LabelsType myLabels;

    /**
       Stores the first block of datas.
    */
    FirstBlock myFirstBlock;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class LabelledMap


  /**
   * Overloads 'operator<<' for displaying objects of class 'LabelledMap'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'LabelledMap' to write.
   * @return the output stream after the writing.

   @tparam TData the type for the datas stored in the list.
   @tparam N the number of data stored in the first block.
   @tparam M the number of data stored in the further blocks.

   */
  template  <typename TData, unsigned int L, typename TWord,
             unsigned int N, unsigned int M>
  std::ostream&
  operator<< ( std::ostream & out, 
               const LabelledMap<TData, L, TWord, N, M> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/LabelledMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LabelledMap_h

#undef LabelledMap_RECURSES
#endif // else defined(LabelledMap_RECURSES)
