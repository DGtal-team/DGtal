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
 * @file CountedPtr.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/08
 *
 * Header file for module CountedPtr.cpp
 *
 * Taken from http://ootips.org/yonat/4dev/smart-pointers.html
 *
 * This file is part of the DGtal library.
 */

#if defined(CountedPtr_RECURSES)
#error Recursive header files inclusion detected in CountedPtr.h
#else // defined(CountedPtr_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CountedPtr_RECURSES

#if !defined CountedPtr_h
/** Prevents repeated inclusion of headers. */
#define CountedPtr_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  template <typename T> class CountedPtrOrPtr;
  template <typename T> class CountedConstPtrOrConstPtr;

  /////////////////////////////////////////////////////////////////////////////
  // template class CountedPtr
  /**
   * Description of template class 'CountedPtr' <p>
   * \brief Aim: Smart pointer based on reference counts.
   *
   * Taken from http://ootips.org/yonat/4dev/smart-pointers.html
   */
  template <typename T>
  class CountedPtr
  {
  public:
    friend class CountedPtrOrPtr<T>;
    friend class CountedConstPtrOrConstPtr<T>;

    // ----------------------- Standard services ------------------------------
  public:
    struct Counter {
        Counter(T* p = 0, unsigned c = 1) : ptr(p), count(c) {}
        T*          ptr;
        unsigned    count;
    };

    typedef T element_type;

    explicit CountedPtr(T* p = 0) // allocate a new counter
        : myCounter(0) { if (p) myCounter = new Counter(p);}
    ~CountedPtr()
        {release();}
    CountedPtr(const CountedPtr& r) throw()
        {acquire(r.myCounter);}
    CountedPtr& operator=(const CountedPtr& r)
    {
        if (this != &r) {
            release();
            acquire(r.myCounter);
        }
        return *this;
    }

    T& operator*()  const throw()   {return *myCounter->ptr;}
    T* operator->() const throw()   {return myCounter->ptr;}
    T* get()        const throw()   {return myCounter ? myCounter->ptr : 0;}
    bool unique()   const throw()
    {return (myCounter ? myCounter->count == 1 : true);}

    /**
     * For debug.
     */
    unsigned int count() const      {return myCounter->count;}

    inline T* drop() 
    { // Gives back the pointer without deleting him. Delete only the Counter.
      T* tmp = myCounter->ptr;
      ASSERT( myCounter->count == 1 );
      delete myCounter;
      myCounter = 0; 
      return tmp;
    }
private:

    Counter* myCounter;

    void acquire(Counter* c) throw()
    { // increment the count
        myCounter = c;
        if (c) ++c->count;
    }

    void release()
    { // decrement the count, delete if it is 0
        if (myCounter) {
            if (--myCounter->count == 0) {
                delete myCounter->ptr;
                delete myCounter;
            }
            myCounter = 0;
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

  }; // end of class CountedPtr


  /**
   * Overloads 'operator<<' for displaying objects of class 'CountedPtr'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CountedPtr' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const CountedPtr<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/CountedPtr.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CountedPtr_h

#undef CountedPtr_RECURSES
#endif // else defined(CountedPtr_RECURSES)
