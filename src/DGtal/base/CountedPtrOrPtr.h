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
 * @file CountedPtrOrPtr.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2013/11/06
 *
 * Header file for module CountedPtrOrPtr.cpp
 *
 * Taken from http://ootips.org/yonat/4dev/smart-pointers.html
 *
 * This file is part of the DGtal library.
 */

#if defined(CountedPtrOrPtr_RECURSES)
#error Recursive header files inclusion detected in CountedPtrOrPtr.h
#else // defined(CountedPtrOrPtr_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CountedPtrOrPtr_RECURSES

#if !defined CountedPtrOrPtr_h
/** Prevents repeated inclusion of headers. */
#define CountedPtrOrPtr_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  template <typename T> class CountedConstPtrOrConstPtr;

  /////////////////////////////////////////////////////////////////////////////
  // template class CountedPtrOrPtr
  /**
   * Description of template class 'CountedPtrOrPtr' <p> \brief Aim:
   * Smart or simple pointer on \c T. It can be a smart pointer based
   * on reference counts or a simple pointer on \c T depending of a
   * boolean value. This is useful when instantiating from an Alias<T>
   * object, letting the user specify if it uses smart pointers or
   * simply pointers.
   *
   * @tparam T any type.
   * @see CountedPtr
   */
  template <typename T>
  class CountedPtrOrPtr
  {
  public:
    friend class CountedConstPtrOrConstPtr<T>;

    // ----------------------- Standard services ------------------------------
  public:

    typedef T element_type;
    typedef typename CountedPtr<T>::Counter Counter;

    /**
       allocate a new counter (smart if isCountedPtr==true)
    */
    inline explicit CountedPtrOrPtr( T* p = 0, bool isCountedPtr = true )
      : myAny(0), myIsCountedPtr( isCountedPtr )
    { 
      if ( isCountedPtr ) {
	if (p) myAny = static_cast<void*>( new Counter( p ) );
      }
      else
	myAny = static_cast<void*>( p );
    }

    ~CountedPtrOrPtr()
    { if ( myIsCountedPtr ) release(); }

    CountedPtrOrPtr( const CountedPtr<T> & r ) throw()
      : myIsCountedPtr( true )
    { 
      acquire(r.myCounter); 
    }

    CountedPtrOrPtr(const CountedPtrOrPtr& r) throw()
      : myIsCountedPtr( r.myIsCountedPtr )
    { 
      if ( myIsCountedPtr )
	acquire( r.counterPtr() ); 
      else
	myAny = r.myAny;
    }

    CountedPtrOrPtr& operator=(const CountedPtrOrPtr& r)
    {
      if ( this != & r ) {
	if ( myIsCountedPtr ) release();
	if ( r.myIsCountedPtr ) acquire( r.myCounter );
	else myAny = r.myAny;
	myIsCountedPtr = r.myIsCountedPtr;
      }
      return *this;
    }

    CountedPtrOrPtr& operator=(const CountedPtr<T>& r)
    {
      if ( this != & r ) {
	if ( myIsCountedPtr ) release();
	acquire( r.myCounter );
	myIsCountedPtr = true;
      }
      return *this;
    }

    T& operator*()  const throw()   { return myIsCountedPtr ? ( * counterPtr()->ptr ) : ( * ptr() ); }
    T* operator->() const throw()   { return myIsCountedPtr ? counterPtr()->ptr : ptr(); }
    T* get()        const throw()   { return myIsCountedPtr ? ( myAny ? counterPtr()->ptr : 0 ) : ptr(); }
    bool unique()   const throw()
    {
      return myIsCountedPtr
	? ( myAny ? counterPtr()->count == 1 : true )
	: true;
    }

    /**
     * For debug.
     */
    unsigned int count() const      { return myIsCountedPtr ? counterPtr()->count : 0; }

    inline T* drop() 
    { // Gives back the pointer without deleting him. Delete only the counter.
      if ( myIsCountedPtr ) {
	T* tmp = counterPtr()->ptr;
	ASSERT( counterPtr()->count == 1 );
	delete counterPtr();
	myAny = 0; 
	return tmp;
      } else {
	return ptr();
      }
    }

private:

    void* myAny;
    bool myIsCountedPtr;

	    
    inline Counter* counterPtr() const
    { return static_cast<Counter*>( myAny ); }
    
    inline T* ptr() const
    { return static_cast<T*>( myAny ); }
	    
    inline void acquire(Counter* c) throw()
    { // increment the count
      myAny = static_cast<void*>( c );
      if (c) ++c->count;
    }

    void release()
    { // decrement the count, delete if it is 0
        if (myAny) {
	  Counter * counter = counterPtr();
	  if (--counter->count == 0) {
	    delete counter->ptr;
	    delete counter;
	  }
	  myAny = 0;
        }
    }


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

    // ------------------------- Hidden services ------------------------------
  protected:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class CountedPtrOrPtr


  /**
   * Overloads 'operator<<' for displaying objects of class 'CountedPtrOrPtr'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CountedPtrOrPtr' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const CountedPtrOrPtr<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/CountedPtrOrPtr.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CountedPtrOrPtr_h

#undef CountedPtrOrPtr_RECURSES
#endif // else defined(CountedPtrOrPtr_RECURSES)
