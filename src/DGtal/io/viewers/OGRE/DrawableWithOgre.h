/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

#pragma once

/**
* @file DrawableWithOgre.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/26
*
* Header file for module DrawableWithOgre.cpp
*
* This file is part of the DGtal library.
*/

#if defined(DrawableWithOgre_RECURSES)
#error Recursive header files inclusion detected in DrawableWithOgre.h
#else // defined(DrawableWithOgre_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DrawableWithOgre_RECURSES

#if !defined XXX_h
/** Prevents repeated inclusion of headers. */
#define DrawableWithOgre_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class DrawableWithOgre
/**
* Description of class 'DrawableWithOgre' <p>
* \brief Aim: Permits to create a dynamic polymorphism for dgtal objects
*/
class DrawableWithOgre
{
    // ----------------------- Standard services ------------------------------
    
public:

/**
* Constructor.
*/
    DrawableWithOgre();

/**
* Destructor.
*/
    ~DrawableWithOgre();

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
    
    
/**
* Returns the name of the associated DGtalObject
*/
virtual const string className() const ;



    // ------------------------- Protected Datas ------------------------------
protected:
  
  
  // the object type
  std::string typeObject;
  

    // ------------------------- Hidden services ------------------------------
protected:


protected:

    /**
* Copy constructor.
* @param other the object to clone.
* Forbidden by default.
*/
    DrawableWithOgre ( const DrawableWithOgre & other );

    /**
* Assignment.
* @param other the object to copy.
* @return a reference on 'this'.
* Forbidden by default.
*/
    DrawableWithOgre & operator= ( const DrawableWithOgre & other );

    // ------------------------- Internals ------------------------------------
protected:

}; // end of class DrawableWithOgre


/**
* Overloads 'operator<<' for displaying objects of class 'DrawableWithOgre'.
* @param out the output stream where the object is written.
* @param object the object of class 'DrawableWithOgre' to write.
* @return the output stream after the writing.
*/
std::ostream&
operator<< ( std::ostream & out, const DrawableWithOgre & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.


// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DrawableWithOgre_h

#undef DrawableWithOgre_RECURSES
#endif // else defined(DrawableWithOgre_RECURSES)