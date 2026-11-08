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
 * @file
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/04/23
 *
 * Functions for testing class cpp11.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
///////////////////////////////////////////////////////////////////////////////
#include <boost/type_traits/is_arithmetic.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/utility/enable_if.hpp>
#include "DGtal/base/Common.h"

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class cpp11.
///////////////////////////////////////////////////////////////////////////////

template< typename TC, typename TD >
class _A
{
public:
	typedef TC _C;

  _A( const _C& c );
};

template< typename TC, typename TD >
inline
_A< TC, TD >::_A( const _C& c )
{
  boost::ignore_unused_variable_warning( c );
  std::cout << "Generic" << std::endl;
}



template< typename TC > //int specialization
class _B : public _A< TC, int >
{
public:
    typedef _A< TC, int > Super;
	typedef typename Super::_C _C; //Compile en rajoutant cette ligne (et en changeant les appels à Super::C par C)

	_B( const _C& c );
};

template <typename TC>
inline
_B<TC>::_B( const _C& c) : Super(c)
{
  std::cout << "Specialized" << std::endl;
}



int main( int /*argc*/, char** /*argv*/ )
{
  _B<int> a(1);

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
