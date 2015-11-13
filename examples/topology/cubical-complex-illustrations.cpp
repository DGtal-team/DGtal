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
 * @file cubical-complex-collapse.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/08/28
 *
 * An example file named cubical-complex-collapse.cpp.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <map>
#include <unordered_map>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/CubicalComplex.h"
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


int main( int argc, char** argv )
{
  using namespace DGtal::Z2i;
  typedef CubicalComplex< KSpace >     CC;

  trace.beginBlock( "Creating Cubical Complex" );
  KSpace K;
  K.init( Point( 0,0 ), Point( 5,3 ), true );
  CC complex( K );
  Domain domain( Point( 0,0 ), Point( 5,3 ) );
  complex.insertCell( K.uSpel( Point(1,1) ) );
  complex.insertCell( K.uSpel( Point(2,1) ) );
  complex.insertCell( K.uSpel( Point(3,1) ) );
  complex.insertCell( K.uSpel( Point(2,2) ) );
  complex.insertCell( K.uSpel( Point(3,2) ) );
  complex.insertCell( K.uSpel( Point(4,2) ) );
  complex.close();
  trace.endBlock();
  
  Board2D board;
  board << domain;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color(80,80,100), Color(180,180,200) ) )
        << complex;
  board.saveTikZ( "cubical-complex-illustrations-1.tikz" );

  CC S( K );
  S.insertCell( K.uCell( Point( 5, 4 ) ) ); // a linel
  S.insertCell( K.uCell( Point( 4, 4 ) ) ); // a pointel
  S.insertCell( K.uCell( Point( 7, 5 ) ) ); // a pixel
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color::Black, Color(60,60,60) ) )
        << S;
  board.saveTikZ( "cubical-complex-illustrations-2.tikz" );
  board.clear();

  board << domain;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color(80,80,100), Color(180,180,200) ) )
        << complex;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color::Red, Color(255,120,120) ) )
        << complex.closure( S );
  board.saveTikZ( "cubical-complex-illustrations-3.tikz" );
  board.clear();

  board << domain;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color(80,80,100), Color(180,180,200) ) )
        << complex;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color::Blue, Color(120,120,255) ) )
        << complex.star( S );
  board.saveTikZ( "cubical-complex-illustrations-4.tikz" );
  board.clear();

  board << domain;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color(80,80,100), Color(180,180,200) ) )
        << complex;
  board << CustomStyle( complex.className(), 
                        new CustomColors( Color::Green, Color(120,255,120) ) )
        << complex.link( S );
  board.saveTikZ( "cubical-complex-illustrations-5.tikz" );
  board.clear();
  
  
  
  return 0;
}
///////////////////////////////////////////////////////////////////////////////
