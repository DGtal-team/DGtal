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
 * @file testLambdaMST3D.cpp
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/03
 *
 * Functions for testing class LambdaMST3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <vector>
#include <iterator>
#include "DGtalCatch.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/estimation/LambdaMST3DBy2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "LambdaMST3DBy2D test" )
{

    typedef vector < Point > Container;
    typedef Container::const_iterator ConstIterator;
    LambdaMST3DBy2D < ConstIterator > lmst;

    // Input points
    Container contour;
    contour.push_back ( Point ( 18, 25, 18 ) );
    contour.push_back ( Point ( 17, 25, 19 ) );
    contour.push_back ( Point ( 16, 25, 20 ) );
    contour.push_back ( Point ( 15, 25, 21 ) );
    contour.push_back ( Point ( 14, 25, 22 ) );
    contour.push_back ( Point ( 13, 25, 23 ) );
    contour.push_back ( Point ( 12, 25, 24 ) );
    contour.push_back ( Point ( 11, 25, 25 ) );
    contour.push_back ( Point ( 10, 25, 26 ) );
    contour.push_back ( Point ( 9, 25, 27 ) );
    contour.push_back ( Point ( 8, 25, 28 ) );

    lmst.init ( contour.cbegin ( ), contour.cend ( ), LambdaMST3DBy2D < ConstIterator >::MAIN_AXIS::X ) ;
    lmst.eval ( contour.front ( ) );

    lmst.init ( contour.cbegin ( ), contour.cend ( ), LambdaMST3DBy2D < ConstIterator >::MAIN_AXIS::X );
    vector < RealVector > tangent;
    lmst.eval ( contour.cbegin ( ), contour.cend ( ), back_insert_iterator < vector < RealVector > > ( tangent ) );
}

///////////////////////////////////////////////////////////////////////////////
