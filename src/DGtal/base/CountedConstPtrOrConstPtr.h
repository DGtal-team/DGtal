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
 * @file CountedConstPtrOrConstPtr.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2013/11/06
 *
 * Header file for module CountedConstPtrOrConstPtr.cpp
 *
 * Taken from http://ootips.org/yonat/4dev/smart-pointers.html
 *
 * This file is part of the DGtal library.
 */

#if defined(CountedConstPtrOrConstPtr_RECURSES)
#error Recursive header files inclusion detected in CountedConstPtrOrConstPtr.h
#else // defined(CountedConstPtrOrConstPtr_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CountedConstPtrOrConstPtr_RECURSES

#if !defined CountedConstPtrOrConstPtr_h
/** Prevents repeated inclusion of headers. */
#define CountedConstPtrOrConstPtr_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/CountedPtrOrPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CountedConstPtrOrConstPtr
  /**
   * Description of template class 'CountedConstPtrOrConstPtr' <p> \brief Aim:
   * Smart or simple const pointer on \c T. It can be a smart pointer based
   * on reference counts or a simple pointer on \c T depending of a
   * boolean value. This is useful when instantiating from a ConstAlias<T>
   * object, letting the user specify if it uses smart pointers or
   * simply pointers.
   *
   * @tparam T any type.
   * @see CountedPtr
   */
  template <typename T>
  class CountedConstPtrOrConstPtr
  {
  public:
    friend class CountedPtr<T>;
    friend class CountedPtrOrPtr<T>;

    // ----------------------- Standard services ------------------------------
  public:

    typedef T element_type;
    typedef typename CountedPtr<T>::Counter Counter;

    /**
       allocate a new counter (smart if isCountedPtr==true)
    */
    inline explicit CountedConstPtrOrConstPtr( const T* p = 0, bool isCountedPtr = true )
      : myAny(0), myIsCountedPtr( isCountedPtr )
    { 
      if ( isCountedPtr ) {
	if (p) myAny = static_cast<void*>( new Counter( const_cast<T*>( p ) ) );
      }
      else
	myAny = const_cast<void*>( static_cast<const void*>( p ) );
    }

    ~CountedConstPtrOrConstPtr()
    { if ( myIsCountedPtr ) release(); }

    CountedConstPtrOrConstPtr( const CountedPtr<T> & r ) throw()
      : myIsCountedPtr( true )
    { 
      acquire(r.myCounter); 
    }

    CountedConstPtrOrConstPtr(const CountedConstPtrOrConstPtr& r) throw()
      : myIsCountedPtr( r.myIsCountedPtr )
    { 
      if ( myIsCountedPtr )
	acquire( r.counterPtr() ); 
      else
	myAny = r.myAny;
    }

    CountedConstPtrOrConstPtr(const CountedPtrOrPtr<T>& r) throw()
      : myIsCountedPtr( r.myIsCountedPtr )
    { 
      if ( myIsCountedPtr )
	acquire( r.counterPtr() ); 
      else
	myAny = r.myAny;
    }

    CountedConstPtrOrConstPtr& operator=(const CountedConstPtrOrConstPtr& r)
    {
      if ( this != & r ) {
	if ( myIsCountedPtr ) release();
	if ( r.myIsCountedPtr ) acquire( r.myCounter );
	else myAny = r.myAny;
	myIsCountedPtr = r.myIsCountedPtr;
      }
      return *this;
    }

    CountedConstPtrOrConstPtr& operator=(const CountedPtrOrPtr<T>& r)
    {
      if ( this != & r ) {
	if ( myIsCountedPtr ) release();
	if ( r.myIsCountedPtr ) acquire( r.myCounter );
	else myAny = r.myAny;
	myIsCountedPtr = r.myIsCountedPtr;
      }
      return *this;
    }

    CountedConstPtrOrConstPtr& operator=(const CountedPtr<T>& r)
    {
      if ( this != & r ) {
	if ( myIsCountedPtr ) release();
	acquire( r.myCounter );
	myIsCountedPtr = true;
      }
      return *this;
    }

    const T& operator*()  const throw()   { return myIsCountedPtr ? ( * counterPtr()->ptr ) : ( * ptr() ); }
    const T* operator->() const throw()   { return myIsCountedPtr ? counterPtr()->ptr : ptr(); }
    const T* get()        const throw()   { return myIsCountedPtr ? ( myAny ? counterPtr()->ptr : 0 ) : ptr(); }
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

    inline const T* drop() 
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

  }; // end of class CountedConstPtrOrConstPtr


  /**
   * Overloads 'operator<<' for displaying objects of class 'CountedConstPtrOrConstPtr'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CountedConstPtrOrConstPtr' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const CountedConstPtrOrConstPtr<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/CountedConstPtrOrConstPtr.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CountedConstPtrOrConstPtr_h

#undef CountedConstPtrOrConstPtr_RECURSES
#endif // else defined(CountedConstPtrOrConstPtr_RECURSES)
