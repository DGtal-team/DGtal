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
 * @file CGALPointVector.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/09/28
 *
 * Header file for module CGALPointVector.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CGALPointVector_RECURSES)
#error Recursive header files inclusion detected in CGALPointVector.h
#else // defined(CGALPointVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CGALPointVector_RECURSES

#if !defined CGALPointVector_h
/** Prevents repeated inclusion of headers. */
#define CGALPointVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include <CGAL/Origin.h>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CGALPointVector
  /**
   * Description of template class 'CGALPointVector' <p>
   * \brief Aim: implement a wrapper of CGAL::Point_d in DGtal.
   */
  template <typename T>
  class CGALPointVector< DGtal::Dimension dim, typename Kernel>: public CGAL::Point_d<Kernel>
  {
    // ----------------------- Standard services ------------------------------
  public:

    typedef CGAL::Point_d<Kernel> Container;
    typedef typename Kernel::FT Value;
    
    CGALPointVector(): CGAL::Point_d<Kernel>(dim,CGAL::ORIGIN)
    {
      for(DGta::Dimension i =0; i< dim; ++i)
	this->operator[](i)=0;
    }
    
    /**
     * Destructor.
     */
    ~CGALPointVector();

    // ----------------------- Iterators --------------------------------------
  public:

    template <typename Value>
    class Iterator
    {
    public:
      typedef std::bidirectional_iterator_tag iterator_category; 
      typedef Value value_type;
      typedef ptrdiff_t difference_type;
      typedef Value* pointer;
      typedef Value& reference;
      typedef typename TPoint::Dimension Dimension;
      
      Iterator(const CGALPointVector *container, size_t aPos  )
       : myContainer( container ),  myCurrentPos( aPos)
      {
      }

      const Value & operator*() const
      {
	ASSERT(myCurrentPos <  myContainer->dimension()); // we must be between [begin,end]
	return myContainer[myCurrentPos];
      }
      Value & operator*()
      {
	ASSERT(myCurrentPos <  myContainer->dimension()); // we must be between [begin,end]
	return myContainer[myCurrentPos];
      }

      /**
       * Operator ==
       *
       */
      bool operator== ( const CGALPointVector_Iterator<Value> &it ) const
      {
	return ( myContainer[myCurrentPos]==it.myContainer[myCurrentPos] );
      }
      
      /**
       * Operator !=
       *
       */
      bool operator!= ( const CGALPointVector_Iterator<Value> &aIt ) const
      {
	return ( myContainer[myCurrentPos]==it.myContainer[myCurrentPos] );
      }

      /**
       * Operator ++ (++it)
       */
      CGALPointVector_Iterator<Value> &operator++()
      {
	myCurrentPos++;
	return *this;
      }

      /**
       * Operator ++ (it++)
       */
      CGALPointVector_Iterator<Value> operator++ ()
      {
	CGALPointVector_Iterator<Value> tmp = *this;
	myCurrentPos++;
	return tmp;
      }

      /**
       * Operator -- (--it)
       */
      CGALPointVector_Iterator<Value> &operator--()
      {
	myCurrentPos--;
	return *this;
      }

      /**
       * Operator -- (it--)
       */
      CGALPointVector_Iterator<Value> operator-- ( )
      {
	CGALPointVector_Iterator<Value> tmp = *this;
	myCurrentPos--;
	return tmp;
      }

    private:
      ///Second index of the iterator position
      DGtal::Dimension myCurrentPos;
      ///Current point
      const CGALPointVector<> * myContainer;
    };


    typedef typename Container::Cartesian_const_iterator const_iterator;
    typedef typename Container::Cartesian_const_iterator ConstIterator;
    typedef typename CGALPointVector_Iterator<> Iterator;
    

    Iterator begin() 
    {
      return CGALPointVector_Iterator(this,0);
    }
    
    Iterator end() 
    {
      return CGALPointVector_Iterator(this,this->dimension());
    }

    ConstIterator begin() const 
    {
      return this->cartesian_begin();
    }

    ConstIterator end() const 
    {
      return this->cartesian_end();
    }

    DGtal::Dimension size() const
    {
      return this->dimension();
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

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    CGALPointVector();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    CGALPointVector ( const CGALPointVector & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    CGALPointVector & operator= ( const CGALPointVector & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class CGALPointVector


  /**
   * Overloads 'operator<<' for displaying objects of class 'CGALPointVector'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CGALPointVector' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const CGALPointVector<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/kernel/extensions//CGALPointVector.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CGALPointVector_h

#undef CGALPointVector_RECURSES
#endif // else defined(CGALPointVector_RECURSES)
