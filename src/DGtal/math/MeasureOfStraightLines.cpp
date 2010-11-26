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

/** 
 * @file MeasureOfStraightLines.cpp
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/03/04
 * 
 * Implementation of methods defined in MeasureOfStraightLines.h 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/math/MeasureOfStraightLines.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/math/MeasureOfStraightLines.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

///////////////////////////////////////////////////////////////////////////////
// class MeasureOfStraightLines
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :




///////////////////////////////////////////////////////////////////////////////
// Interface - public :


/**
 * Constructor.
 */
DGtal::MeasureOfStraightLines::MeasureOfStraightLines()
{
   ///Default value
   myEpsilon = 0.0005;
 }


 /**
   * Destructor.
   */
DGtal::MeasureOfStraightLines:: ~MeasureOfStraightLines()
 {
 }

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void 
DGtal::MeasureOfStraightLines::selfDisplay( std::ostream & out ) const
{
  out << "[MeasureOfStraightLines]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool 
DGtal::MeasureOfStraightLines::isValid() const
{
  return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
