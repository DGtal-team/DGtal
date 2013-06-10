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
 * @file   Board3D.h
 * @author Aline Martin
 * @date   vendredi 7 juin 2013
 * 
 * @brief
 *
 * Header file for module Board3D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Board3D_RECURSES)
#error Recursive header files inclusion detected in Board3D.h
#else // defined(Board3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Board3D_RECURSES

#if !defined Board3D_h
/** Prevents repeated inclusion of headers. */
#define Board3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <algorithm>
#include <map>

#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{


  /////////////////////////////////////////////////////////////////////////////
  // class Board3D
  /**
   * Description of class 'Board3D' <p>
   * @brief Class for OBJ export
   */
  class Board3D : public Display3D
  {
  public:

  
    /*!
     * \brief Constructor.
     */
    Board3D();
  
    /*!
     * \brief Destructor.
     */
    ~Board3D(){};
  

    /**
     * @return the style name used for drawing this object.
     */
    std::string className() const
    {
      return "Board3D";
    }

    /**
     * Save a OBJ image.
     * @param filename filename of the image to save.
     */
    void saveOBJ(const std::string & filename);


  
    /**
     * The associated map type for storing possible modes used for
     * displaying for digital objects.
     */
    //typedef std::map< std::string, std::string > ModeMapping;

    //  /**
    //    * The associated map type for storing the default styles of
    //    * digital objects.
    //    */
    //   typedef std::map< std::string,CountedPtr<DrawableWithDisplay3D> > StyleMapping;
  
    DGtal::Color myDefaultColor;  //!< default color

  

 
    /**
     * Set the default color for future drawing.
     *
     * @param aColor a DGtal::Color (allow to set a trasnparency value).
     *
     **/  
    Board3D & operator<<(const DGtal::Color & aColor);



    /**
     * Draws the drawable [object] in this board. It should satisfy
     * the concept CDrawableWithDisplay3D, which requires for instance a
     * method setStyle( Board3D & ).
     *
     * @param object any drawable object.
     * @return a reference on 'this'.
     */
    template <typename TDrawableWithDisplay3D>
    Board3D & operator<<( const  TDrawableWithDisplay3D & object );

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

  public:
  

    // ------------------------- Private Datas --------------------------------
  private:
 

  
  protected :
    /**
     *  init function (should be in Constructor).
     */
    virtual void init();

  private:

  }; // end of class Board3D
  
 
 

  
  /**
   * Overloads 'operator<<' for displaying objects of class 'Board3D'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Board3D' to write.
   * @return the output stream after the writing.
   */
  std::ostream&
  operator<< ( std::ostream & out, const Board3D & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/boards/Board3D.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Board3D_h

#undef Board3D_RECURSES
#endif // else defined(Board3D_RECURSES)
