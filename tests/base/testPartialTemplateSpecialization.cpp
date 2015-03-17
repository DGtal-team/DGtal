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
 * @file testcpp11.cpp
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


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class cpp11.
///////////////////////////////////////////////////////////////////////////////

template< typename TC, unsigned short d = 8 >
class A
{
public:
	typedef TC C; 

	A( const C& c );
};

template< typename TC, unsigned short d >
inline
A< TC, d >::A( const C& c )
{
	std::cout << "Generic" << std::endl;
}





template< typename TC >
class B : public A< TC, 4 >
{
public: 
    typedef A< TC, 4 > Super;

	B( const typename Super::C& c );
};

template <typename TC>
inline
B<TC>::B( const typename Super::C& c )
  : Super(c) 
{
	std::cout << "Specialized" << std::endl;
}



int main( int argc, char** argv )
{
  B<int> a(1);

  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
