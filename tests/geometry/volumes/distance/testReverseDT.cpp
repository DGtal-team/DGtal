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
 * @file testReverseDT.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/03
 *
 * Functions for testing class ReverseDT.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/geometry/volumes/distance/ReverseDistanceTransformation.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/images/imagesSetsUtils/SimpleThresholdForegroundPredicate.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;



template<typename Image>
void randomSeeds(Image &input, const unsigned int nb, const int value)
{
  typename Image::Point p, low = input.lowerBound();
  typename Image::Vector ext;

  ext = input.extent();

  for (unsigned int k = 0 ; k < nb; k++)
  {
    for (unsigned int dim = 0; dim < Image::dimension; dim++)
    {
      p[dim] = rand() % (ext[dim]) +  low[dim];
    }
    input.setValue(p, value);
  }
}



///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ReverseDT.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testReverseDT()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef HueShadeColorMap<DGtal::uint64_t, 2> Hue;

  trace.beginBlock ( "Testing Reverse DT in 2D ..." );
  
  Z2i::Point a (2, 2 );
  Z2i::Point b ( 15, 15 );

  typedef ImageSelector< Z2i::Domain,  int>::Type Image;
  Image image (Z2i::Domain( a, b ));
  
  for ( unsigned k = 0; k < 49; k++ )
    {
      a[0] = ( k / 7 ) + 5;
      a[1] = ( k % 7 ) + 5;
      image.setValue ( a, 128 );
    }

  a = Z2i::Point(2,2);
 
  typedef SimpleThresholdForegroundPredicate<Image> Predicate;
  Predicate aPredicate(image,0);
 
  DistanceTransformation<Z2i::Space, Predicate, 2 > dt(Z2i::Domain(a,b), aPredicate);
  typedef DistanceTransformation<Z2i::Space, Predicate,2>::OutputImage ImageDT;

  ImageDT result = dt.compute (  );
  trace.info() << result<< std::endl;
  //ReverseDT  
  trace.warning()<<"DT:"<<endl;
  ImageDT::ConstIterator it = result.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (*it) << " ";
      ++it;
    }
    std::cout << std::endl;
  }


  ReverseDistanceTransformation< ImageDT, 2 > reverseDT;
  typedef ReverseDistanceTransformation< ImageDT, 2 >::OutputImage ImageRDT;

  ImageRDT reconstruction = reverseDT.reconstruction( result );
  
  // board.clear();
  //drawImage<Hue>(board, result, (DGtal::int64_t)0, (DGtal::int64_t)10);
  // board.saveSVG ( "image-REDTtest.svg" );

  trace.warning()<<"REDT:"<<endl;
  ImageRDT::ConstIterator it2 = reconstruction.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (int)(*it2) << " ";
      ++it2;
    }
    std::cout << std::endl;
  }

  //Checking
  bool ok=true;
  ImageRDT::ConstIterator itrec = reconstruction.begin(), itend = reconstruction.end();
  Image::ConstIterator  itinit = image.begin();
  for( ; itrec != itend; ++itrec,++itinit)
    if ((*itrec) == 0)
      ok = ok & ((*itinit) == 0);

  nbok += ok ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

bool testReverseDTL1()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef HueShadeColorMap<DGtal::uint64_t, 2> Hue;

  trace.beginBlock ( "Testing Reverse DT in 2D with L1 metric ..." );
  
  Z2i::Point a (2, 2 );
  Z2i::Point b ( 15, 15 );

  typedef ImageSelector< Z2i::Domain, unsigned int>::Type Image;
  Image image ( Z2i::Domain( a, b ));
  
  for ( unsigned k = 0; k < 49; k++ )
    {
      a[0] = ( k / 7 ) + 5;
      a[1] = ( k % 7 ) + 5;
      image.setValue ( a, 128 );
    }
  a = Z2i::Point(2, 2 );
 
 
  typedef SimpleThresholdForegroundPredicate<Image> Predicate;
  Predicate aPredicate(image,0);
 
  DistanceTransformation<Z2i::Space, Predicate, 1> dt(Z2i::Domain(a,b), aPredicate);
  typedef DistanceTransformation<Z2i::Space, Predicate,1>::OutputImage ImageDT;

  ImageDT result = dt.compute (  );


  //ReverseDT  
  trace.warning()<<"DT:"<<endl;
  ImageDT::ConstIterator it = result.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (int)(*it) << " ";
      ++it;
    }
    std::cout << std::endl;
  }


  ReverseDistanceTransformation< ImageDT, 1 > reverseDT;
  typedef ReverseDistanceTransformation< ImageDT, 1 >::OutputImage ImageRDT;

  ImageRDT reconstruction = reverseDT.reconstruction( result );
 
  trace.warning()<<"REDT:"<<endl;
  ImageRDT::ConstIterator it2 = reconstruction.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (int)(*it2) << " ";
      ++it2;
    }
    std::cout << std::endl;
  }

  //Checking
  bool ok=true;
  ImageRDT::ConstIterator itrec = reconstruction.begin(), itend = reconstruction.end();
  Image::ConstIterator  itinit = image.begin();
  for( ; itrec != itend; ++itrec,++itinit)
    if ((*itrec) == 0)
      ok = ok & ((*itinit) == 0);

  nbok += ok ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}
bool testReverseDTL1simple()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef HueShadeColorMap<DGtal::uint64_t, 2> Hue;

  trace.beginBlock ( "Testing Reverse DT in 2D with L1 metric ..." );
  
  Z2i::Point a (2, 2 );
  Z2i::Point b ( 15, 15 );

  typedef ImageSelector< Z2i::Domain, unsigned int>::Type Image;
 

  typedef SimpleThresholdForegroundPredicate<Image> Predicate;
  typedef DistanceTransformation<Z2i::Space, Predicate,1>::OutputImage ImageDT;


  ImageDT result ( Z2i::Domain(a, b ));
  result.setValue(Z2i::Point(5,7), 3);
  result.setValue(Z2i::Point(9,7), 4);

  //ReverseDT  
  trace.warning()<<"DT:"<<endl;
  ImageDT::ConstIterator it = result.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (int)(*it) << " ";
      ++it;
    }
    std::cout << std::endl;
  }


  ReverseDistanceTransformation< ImageDT, 1 > reverseDT;
  typedef ReverseDistanceTransformation< ImageDT, 1 >::OutputImage ImageRDT;

  ImageRDT reconstruction = reverseDT.reconstruction( result );
 
  trace.warning()<<"REDT:"<<endl;
  ImageRDT::ConstIterator it2 = reconstruction.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (int)(*it2) << " ";
      ++it2;
    }
    std::cout << std::endl;
  }

 
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testReverseDTSet()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef HueShadeColorMap<DGtal::uint64_t, 2> Hue;

  trace.beginBlock ( "Testing Reverse DT in 2D ..." );
  
  Z2i::Point a (2, 2 );
  Z2i::Point b ( 15, 15 );

  typedef ImageSelector< Z2i::Domain, unsigned int>::Type Image;
  Image image (Z2i::Domain( a, b ));
  
  for ( unsigned k = 0; k < 49; k++ )
    {
      a[0] = ( k / 7 ) + 5;
      a[1] = ( k % 7 ) + 5;
      image.setValue ( a, 128 );
    }
  a = Z2i::Point(2, 2 );

 
  typedef SimpleThresholdForegroundPredicate<Image> Predicate;
  Predicate aPredicate(image,0);
 
  DistanceTransformation<Z2i::Space, Predicate, 2> dt(Z2i::Domain(a,b), aPredicate);
  typedef DistanceTransformation<Z2i::Space, Predicate,2>::OutputImage ImageDT;

 
  ImageDT result = dt.compute (  );


  //ReverseDT  
  trace.warning()<<"DT:"<<endl;
  ImageDT::ConstIterator it = result.begin();
  for (unsigned int y = 2; y < 16; y++)
    {
    for (unsigned int x = 2; x < 16; x++)
    {
      std::cout << (*it) << " ";
      ++it;
    }
    std::cout << std::endl;
  }

  
  ReverseDistanceTransformation< ImageDT, 2 > reverseDT;

  typedef DigitalSetBySTLSet<Z2i::Domain> Set;

  
  Set reconstruction(result.domain());
  reverseDT.reconstructionAsSet<Set>( reconstruction, result );
  
  Board2D board;
  board << reconstruction;
  board.saveSVG ( "image-REDTtestSet.svg" );

  trace.warning()<<"REDT:"<<endl;
  for(Set::ConstIterator it2 = reconstruction.begin(), 
  itend2 = reconstruction.end();
      it2!=itend2;
      ++it2)
    trace.info() << (*it2) << " ";
  
  //Checking
  bool ok=true;

  nbok += ok ? 1 : 0; 
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
  trace.beginBlock ( "Testing class ReverseDT" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testReverseDT() && testReverseDTSet() 
    && testReverseDTL1() && testReverseDTL1simple(); // && ... other tests
  
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
