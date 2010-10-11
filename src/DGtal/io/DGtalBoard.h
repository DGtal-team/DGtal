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
 * @file DGtalBoard.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/10/11
 *
 * Header file for module DGtalBoard.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DGtalBoard_RECURSES)
#error Recursive header files inclusion detected in DGtalBoard.h
#else // defined(DGtalBoard_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DGtalBoard_RECURSES

#if !defined DGtalBoard_h
/** Prevents repeated inclusion of headers. */
#define DGtalBoard_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/Object.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
    /**
     * Specifies the drawing mode for domains.
     */
    enum DomainDrawMode { GRID = 0, PAVING = 1 };

/////////////////////////////////////////////////////////////////////////////
// class DGtalBoard
/**
 * Description of class 'DGtalBoard' <p> \brief Aim: This class
 * specializes a 'Board' class so as to display DGtal objects more
 * naturally (with <<). The user has simply to declare a DGtalBoard
 * object and uses stream operators to display most digital
 * objects. Furthermore, one can use this class to modify the current
 * style for drawing.
 */
  class DGtalBoard : public LibBoard::Board
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    /**
     * Destructor.
     */
    ~DGtalBoard();
    
    /** 
     * Constructs a new board and sets the background color, if any.
     * 
     * @param backgroundColor A color for the drawing's background.
     */
    DGtalBoard( const LibBoard::Color & backgroundColor 
		= LibBoard::Color::None );

    /** 
     * Copy constructor.
     * 
     * @param other The object to be copied.
     */
    DGtalBoard( const DGtalBoard & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DGtalBoard & operator= ( const DGtalBoard & other );

    /**
     * Draws the drawable [object] in this board. It should satisfy
     * the concept CDrawableWithBoard, which requires for instance a
     * method selfDraw( LibBoard::Board ).
     *
     * @param domain any domain.
     * @return a reference on 'this'.
     */
    template <typename CDrawableWithBoard>
    DGtalBoard & operator<<( const CDrawableWithBoard & object );

    /**
     * Draws the domain in this board. Choose GRID or PAVING according
     * to the current state of this object.
     *
     * @param domain any domain.
     * @return a reference on 'this'.
     * @see DrawDomainGrid, DrawDomainPaving
     */
    template <typename TSpace>
    DGtalBoard & operator<<( const HyperRectDomain<TSpace> & domain );

    /**
     * Draws the object in this board. Draws adjacencies according to
     * the current state of this object.
     *
     * @param domain any domain.
     * @return a reference on 'this'.
     * @see DrawObjectAdjacencies
     */
    template <typename TDigitalTopology, typename TDigitalSet>
    DGtalBoard & 
    operator<<( const Object<TDigitalTopology,TDigitalSet> & object );

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

    // ------------------------- Public Datas ------------------------------
  public:
    DomainDrawMode myDomainDrawMode;
    bool myDrawObjectAdjacencies;

    // ------------------------- Private Datas --------------------------------
private:

    // ------------------------- Hidden services ------------------------------
protected:

private:


    // ------------------------- Internals ------------------------------------
private:

}; // end of class DGtalBoard


/**
 * Overloads 'operator<<' for displaying objects of class 'DGtalBoard'.
 * @param out the output stream where the object is written.
 * @param object the object of class 'DGtalBoard' to write.
 * @return the output stream after the writing.
 */
std::ostream&
operator<< ( std::ostream & out, const DGtalBoard & object );

  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawDomainGrid {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myDomainDrawMode = GRID;
    }
  };

  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawDomainPaving {
    void selfDraw( DGtalBoard & board ) const
    {
      board.myDomainDrawMode = PAVING;
    }
  };

  /**
   * Modifier class in a DGtalBoard stream. Realizes the concept
   * CDrawableWithDGtalBoard.
   */
  struct DrawObjectAdjacencies {
    DrawObjectAdjacencies( bool drawAdj = true )
      : myDrawAdj( drawAdj )
    {}
    void selfDraw( DGtalBoard & board ) const
    {
      board.myDrawObjectAdjacencies = myDrawAdj;
    }
    bool myDrawAdj;
  };

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/io/DGtalBoard.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DGtalBoard_h

#undef DGtalBoard_RECURSES
#endif // else defined(DGtalBoard_RECURSES)
