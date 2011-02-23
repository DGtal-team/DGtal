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
 * @file test_SpaceND.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/03/03
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_SpaceND <p>
 * Aim: simple test of \ref SpaceND
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"


using namespace DGtal;
using namespace std;


/**
* Test instanciation of SpaceND
*
**/
bool testSimpleSpace()
{

    SpaceND<6> aSpace6;
    SpaceND<2,DGtal::int64_t> aSpace2;

    cout << "aSpace6 = " << aSpace6 << endl;
    cout << "aSpace2 = " << aSpace2 << endl;
    SpaceND<6>::Subcospace<2>::Type aSpace4 = aSpace6.subcospace<2>();
// won't work
// aSpace6::SpaceType::Subcospace<2>::Type aSpace4 = aSpace6.getSubcospace<2>();
    cout << "aSpace6.subcospace<2> = " << aSpace4 << endl;

    return true;
}

// Temporary test to check if template is valid.
bool testInteger()
{
  // JOL: Integer => Cinteger
  //   Integer<int> concept;
   return true;
}


int main()
{

  if ( testSimpleSpace() && testInteger() )
        return 0;
    else
        return 1;
}

/** @ingroup Tests **/
