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
 * @file ArithDSSIterator.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/11/26
 *
 * Header file for module ArithDSSIterator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ArithDSSIterator_RECURSES)
#error Recursive header files inclusion detected in ArithDSSIterator.h
#else // defined(ArithDSSIterator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ArithDSSIterator_RECURSES

#if !defined ArithDSSIterator_h
/** Prevents repeated inclusion of headers. */
#define ArithDSSIterator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class ArithDSSIterator
/**
 * Description of class 'ArithDSSIterator' <p>
 * \brief Aim:
 */
  
  template<typename TInteger, int connectivity = 8>
    class ArithDSSIterator
    {
      // ----------------------- Standard services ------------------------------
    public:
    
    typedef ArithDSSIterator Self;
    typedef TInteger Integer;
    typedef PointVector<2,Integer> Point;     
    
    typedef Point Value;
    
    // stl iterator types.
    typedef std::input_iterator_tag iterator_category;
    typedef Point value_type;
    typedef std::ptrdiff_t difference_type; 
    typedef const Point* pointer;
    typedef const Point& reference;
    
    TInteger remainder(Point Q);

    
    
    reference operator*() const;
        
    pointer operator->() const;
   
    Self& operator++();
        
    bool operator==(const Self & other) const;
    
    bool operator!=(const Self & other) const;
    
    
    /**
     * Constructor with initialisation of the DSS parameters and begin point
     * @param a an integer
     * @param b an integer
     * @param mu an integer
     * @param p a point
     */
    
    ArithDSSIterator(const Integer a, const Integer b, const Integer mu, const Point & p);
        
        /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    ArithDSSIterator(){};



    /**
     * Destructor.
     */
    ~ArithDSSIterator(){};

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

    Integer myA;
    Integer myB;
    Integer myMu;
    Point myP;

    // ------------------------- Hidden services ------------------------------
protected:

    
public:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    ArithDSSIterator ( const ArithDSSIterator & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    //ArithDSSIterator & operator= ( const ArithDSSIterator & other );

    
    Self& operator=(const Self & other);
    


    // ------------------------- Internals ------------------------------------
private:
    
    }; // end of class ArithDSSIterator
  
  
/**
 * Overloads 'operator<<' for displaying objects of class 'ArithDSSIterator'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'ArithDSSIterator' to write.
 * @return the output stream after the writing.
 */
  // std::ostream&
  //operator<< ( std::ostream & out, const ArithDSSIterator & object );
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/ArithDSSIterator.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ArithDSSIterator_h

#undef ArithDSSIterator_RECURSES
#endif // else defined(ArithDSSIterator_RECURSES)
