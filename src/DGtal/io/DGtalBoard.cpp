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
/**
 * @file DGtalBoard.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/10/11
 *
 * Implementation of methods defined in DGtalBoard.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/io/DGtalBoard.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/io/DGtalBoard.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class DGtalBoard
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
DGtal::DGtalBoard::~DGtalBoard()
{
}

/** 
 * Constructs a new board and sets the background color, if any.
 * 
 * @param backgroundColor A color for the drawing's background.
 */
DGtal::DGtalBoard::DGtalBoard( const LibBoard::Color & backgroundColor )
  : LibBoard::Board( backgroundColor ),
    myDomainDrawMode( GRID ),
    myDrawObjectAdjacencies( false ),
    myStyles()
{
}

/** 
 * Copy constructor.
 * 
 * @param other The object to be copied.
 */
DGtal::DGtalBoard::DGtalBoard( const DGtalBoard & other )
  : LibBoard::Board( other ),
    myDomainDrawMode( other.myDomainDrawMode ),
    myDrawObjectAdjacencies( other.myDrawObjectAdjacencies ),
    myStyles( other.myStyles )
{
}

/**
 * Assignment.
 * @param other the object to copy.
 * @return a reference on 'this'.
 */
DGtal::DGtalBoard & 
DGtal::DGtalBoard::operator= ( const DGtalBoard & other )
{
  if ( this != &other )
    {
      LibBoard::Board::operator=( other );
      myDomainDrawMode = other.myDomainDrawMode;
      myDrawObjectAdjacencies = other.myDrawObjectAdjacencies;
      myStyles = other.myStyles;
    }
  return *this;
}


///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::DGtalBoard::selfDisplay ( std::ostream & out ) const
{
    out << "[DGtalBoard]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::DGtalBoard::isValid() const
{
    return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
