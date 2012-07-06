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
 * @file IndexedListWithBlocks.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/07/05
 *
 * Header file for module IndexedListWithBlocks.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IndexedListWithBlocks_RECURSES)
#error Recursive header files inclusion detected in IndexedListWithBlocks.h
#else // defined(IndexedListWithBlocks_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IndexedListWithBlocks_RECURSES

#if !defined IndexedListWithBlocks_h
/** Prevents repeated inclusion of headers. */
#define IndexedListWithBlocks_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <cstring>
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class IndexedListWithBlocks
  /**
     Description of template class 'IndexedListWithBlocks' <p> \brief
     Aim: Represents a mixed list/array structure which is useful in
     some context.

     Such a structure is useful when:
     - the user knows at which position/index lies its value.
     - the expected size of this container is small, but may sometimes increase.
     - the user wishes sometimes to insert a new value or erase another value. Note that in this case the user knows that further indices have changed.
     - the user wishes to have a random access to the values that is as fast as possible.
     - one wishes to limit as possible the memory usage.
     - generally this structure is embedded as the value of a big array.

     @tparam TValue the type for the values stored in the list.
     @tparam N the number of value stored in the first block.
     @tparam M the number of value stored in the further blocks.

     NB: In the following, we use the notations
     - n is the size of the container
     - b is the number of blocks ( b = 1 + (size()-N) / M ).
   */
  template <typename TValue, unsigned int N, unsigned int M>
  class IndexedListWithBlocks
  {
  public:
    // ----------------------- Public types ------------------------------
    typedef TValue Value;
    
    struct FirstBlock; //< Forward declaration
    struct AnyBlock; //< Forward declaration

    union BlockPointer {
      FirstBlock* first;
      AnyBlock* any;
    };

    /// Used in blocks to finish it or to point to the next block.
    union ValueOrBlockPointer {
      Value lastValue; // used when at the end of the list
      AnyBlock* nextBlock;  // used otherwise
    };

    /// Represents the first block in the container.
    struct FirstBlock {
      inline
      FirstBlock() : size( 0 )
      { data.nextBlock = 0; }

      inline
      Value shiftFromTill( unsigned int i1, unsigned int i2 )
      {
        Value* p1 = values + i1; 
        Value tmp = *p2;
        std::memmove( p1, p1 + 1, ( i2 - i1 ) * sizeof(Value) );
        return tmp;
      }
      unsigned int size;
      Value values[ N ];
      ValueOrBlockPointer data;
    };

    /// Represents a block (except the first) in the container.
    struct AnyBlock {
      inline
      AnyBlock()
      { data.nextBlock = 0; }

      inline
      Value shiftFromTill( unsigned int i1, unsigned int i2 )
      {
        Value* p1 = values + i1; 
        Value tmp = *p2;
        std::memmove( p1, p1 + 1, ( i2 - i1 ) * sizeof(Value) );
        return tmp;
      }

      Value values[ M ];
      ValueOrBlockPointer data;
    };

    struct Iterator {
      unsigned int idx;
      unsigned int nbValues;
      Value* values;
      AnyBlock* next;
    };

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    IndexedListWithBlocks();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    IndexedListWithBlocks ( const IndexedListWithBlocks & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    IndexedListWithBlocks & operator= ( const IndexedListWithBlocks & other );

    /**
     * Destructor.
     */
    ~IndexedListWithBlocks();

    // ----------------------- Container services -----------------------------
  public:
    
    /**
       The number of values stored in the structure. O(1) complexity.
     */
    unsigned int size() const;

    /**
       Removes all the values stored in the structure. O(b)
       complexity.
     */
    void clear();

    /**
       Random unprotected read-write access to value at position \a idx
       @param idx the index of the value in the container.
       @return a reference on the value.
       @pre idx < size()
       NB: O( b ), E = O( 1 + ceil( ( idx - N ) / M ) )
    */
    Value & operator[]( unsigned int idx );

    /**
       Random unprotected read access to value at position \a idx
       @param idx the index of the value in the container.
       @return a const reference on the value.
       @pre idx < size()
       NB: O( b ), E = O( 1 + ceil( ( idx - N ) / M ) )
    */
    const Value & operator[]( unsigned int idx ) const;

    /**
       Insertion of a new value at given position. The former value at
       this position and the next ones are shifted.

       @param idx the index of the value in the container.
       @pre idx <= size() (if size(), inserts at the end.
       @param value the value to insert.
       NB: O( n ), E = O( n - idx )
    */
    void insert( unsigned int idx, const Value & value );

    /**
       Removal of a value at a given position. Following values are shifted.

       @param idx the index of the value in the container.
       @pre idx < size()
       NB: O( n ), E = O( n - idx )
    */
    void erase( unsigned int idx );

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

    /**
       Stores the first block of values.
    */
    FirstBlock myFirstBlock;

    // ------------------------- Hidden services ------------------------------
  protected:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class IndexedListWithBlocks


  /**
   * Overloads 'operator<<' for displaying objects of class 'IndexedListWithBlocks'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'IndexedListWithBlocks' to write.
   * @return the output stream after the writing.

   @tparam TValue the type for the values stored in the list.
   @tparam N the number of value stored in the first block.
   @tparam M the number of value stored in the further blocks.

   */
  template  <typename TValue, unsigned int N, unsigned int M>
  std::ostream&
  operator<< ( std::ostream & out, 
               const IndexedListWithBlocks<TValue, N, M> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/IndexedListWithBlocks.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IndexedListWithBlocks_h

#undef IndexedListWithBlocks_RECURSES
#endif // else defined(IndexedListWithBlocks_RECURSES)
