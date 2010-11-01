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
 * @file StdDefs.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/11/01
 *
 * Implementation of methods defined in StdDefs.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/helpers/StdDefs.h"
// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/helpers/StdDefs.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

namespace DGtal {
  namespace Z2i {
    // static const Adj4 adj4 = Adj4();
    // static const Adj8 adj8 = Adj4();
    // static const DT4_8 dt4_8( adj4, adj8, JORDAN_DT );
    // static const DT8_4 dt8_4( adj8, adj4, JORDAN_DT );
  }
}

///////////////////////////////////////////////////////////////////////////////
// class StdDefs
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
DGtal::StdDefs::~StdDefs()
{
}



///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::StdDefs::selfDisplay ( std::ostream & out ) const
{
    out << "[StdDefs]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::StdDefs::isValid() const
{
    return true;
}



///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
