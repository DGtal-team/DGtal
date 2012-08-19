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
 * @file testVoronoiMap.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/14
 *
 * Functions for testing class VoronoiMap.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/volumes/distance/VoronoiMap.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/io/boards/Board2D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VoronoiMap.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testVoronoiMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing VoronoiMap2D ..." );

  Z2i::Point a(-10,-10);
  Z2i::Point b(10,10);
  Z2i::Domain domain(a,b);

  Z2i::DigitalSet mySet(domain);

  for(Z2i::Domain::ConstIterator it = domain.begin(), itend = domain.end(); 
      it != itend;
      ++it)
    mySet.insertNew( *it );
  
  mySet.erase( Z2i::Point(0,-6));
  mySet.erase( Z2i::Point(6,0));
  mySet.erase( Z2i::Point(-6,0));
 
  typedef SetPredicate<Z2i::DigitalSet> Predicate;
  Predicate myPredicate(mySet);

  //typedef NotPointPredicate<Predicate> NegPredicate;
  //NegPredicate myNegPredicate( myPredicate );

  typedef VoronoiMap<Z2i::Space, Predicate, 2> Voro2;
  
  Voro2 voro(domain, myPredicate);

  Voro2::OutputImage output = voro.compute();
  
  for(int j=-10; j <= 10; j++)
    {    
      for(int i=-10; i<=10; i++)
        trace.info() << "("<<output( Z2i::Point(i,j))[0]<<","<< output( Z2i::Point(i,j))[1]<<") ";
      trace.info()<<std::endl;
    }


  Board2D board;
  for(Voro2::OutputImage::Domain::ConstIterator it = output.domain().begin(), itend = output.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = output(*it);
      unsigned char c = (p[1]*13 + p[0] * 7) % 256;
      board << CustomStyle( (*it).className(), new CustomColors(Color(c,c,c),Color(c,c,c)))
            << (*it);
    }

  board.saveCairo("Voromap.png", Board2D::CairoPNG);

  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  


  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class VoronoiMap" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testVoronoiMap(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
