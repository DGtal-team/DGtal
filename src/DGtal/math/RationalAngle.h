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
 * @file RationalAngle.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/12/20
 *
 * Header file for module RationalAngle.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RationalAngle_RECURSES)
#error Recursive header files inclusion detected in RationalAngle.h
#else // defined(RationalAngle_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RationalAngle_RECURSES

#if !defined RationalAngle_h
/** Prevents repeated inclusion of headers. */
#define RationalAngle_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class RationalAngle
/**
 * Description of class 'RationalAngle' <p>
 * \brief Aim:
 */
template<typename TInteger>
class RationalAngle
{
  typedef TInteger Integer;
  typedef RationalAngle<Integer> Self;
  
    // ----------------------- Standard services ------------------------------
 public:
  
  RationalAngle(Integer a, Integer b);
  
  

    /**
     * Destructor.
     */
  ~RationalAngle(){};

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
 
    Integer myP;
    Integer myQ;
    


   // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
 public:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    RationalAngle(){};

public:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    RationalAngle ( const RationalAngle & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Self & operator= ( const RationalAngle & other );
    
    /** 
     * Comparison operators
     * @param other the angle to compare with
     * @return a boolean
     */
    bool operator== (const RationalAngle & other);
    
    bool operator< (const RationalAngle & other);
    
    bool operator> (const RationalAngle & other);
    
    bool operator<= (const RationalAngle & other);
    
    bool operator>= (const RationalAngle & other);
    
    bool operator!= (const RationalAngle & other);
    
    Self plusPI_2 ();
      
    Self minusPI_2();
    
    Self plusPI ();
      
    Self minusPI();
    

    // ------------------------- Internals ------------------------------------
private:

}; // end of class RationalAngle


/**
 * Overloads 'operator<<' for displaying objects of class 'RationalAngle'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'RationalAngle' to write.
 * @return the output stream after the writing.
 */
 /* std::ostream& */
 /*   operator<< ( std::ostream & out, const RationalAngle & object ); */


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/math//RationalAngle.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RationalAngle_h

#undef RationalAngle_RECURSES
#endif // else defined(RationalAngle_RECURSES)
