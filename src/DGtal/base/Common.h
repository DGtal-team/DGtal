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
//LICENSE-END
#pragma once

/** 
 * @file Common.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * 
 * @date 2009/12/10
 * 
 * Header file for module Common.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Common_RECURSES)
#error Recursive header files inclusion detected in Common.h
#else // defined(Common_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Common_RECURSES

#if !defined Common_h
#define Common_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <exception>
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/concept/requires.hpp>
#include <boost/serialization/access.hpp>


#if ( (defined(WIN32)) )
#define _USE_MATH_DEFINES
#endif //win32
#include <cmath>

#include "DGtal/utils/Trace.h"
#include "DGtal/utils/TraceWriterTerm.h"
#include "DGtal/utils/TraceWriterFile.h"
#include "DGtal/utils/Assert.h"
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/base/Exceptions.h"
#include "Board/Board.h"


//////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////// 
namespace DGtal 
{
  
  /** DGtal Global variables
   *
   **/
  
  extern TraceWriterTerm traceWriterTerm;
  extern Trace trace;
  
  /**
   * Interface that specifies that an object can draw itself on a
   * board. 
   * @todo (JOL) Put this class elsewhere.
   */
  struct DrawableWithBoard {
    /**
     * Operation to override. Does nothing by default.
     *
     * @param board any object of type Board.
     */
    virtual void selfDraw( LibBoard::Board & board ) const {}
  };

  /////////////////////////////////////////////////////////////////////////////
  // class Common
  /** 
   * Description of class 'Common' <p>
   * Aim: Holds the current configuration.
   */
  class Common
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    /**
     * Destructor. 
     */
    ~Common();

    // ----------------------- Interface --------------------------------------
  public:
    
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay( std::ostream & out ) const;

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
    Common();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Common( const Common & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Common & operator=( const Common & other );
  
    // ------------------------- Internals ------------------------------------
  private:
  
  }; // end of class Common


  /**
   * Overloads 'operator<<' for displaying objects of class 'Common'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Common' to write.
   * @return the output stream after the writing.
   */
  inline
    std::ostream&
    operator<<( std::ostream & out, const Common & object )
  {
    object.selfDisplay( out );
    return out;
  }

  
} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Common_h

#undef Common_RECURSES
#endif // else defined(Common_RECURSES)
