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
 * @file testImageContainerBenchmark.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/22
 *
 * Functions for testing class ImageContainerBenchmark.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


template<typename Image>
double builtinIteratorScan(const Image &aImage)
{
  long int cpt = 0;
  double timer;

  trace.beginBlock( "Built-in iterator simple scan ..." );
  for ( typename Image::ConstIterator it = aImage.begin(), itend = aImage.end();
      it != itend ;
      ++it)
    cpt += (long int)(*it);

  timer =  trace.endBlock();
  trace.info() << "Cpt=" << cpt << endl;
  return timer;
}

template<typename Image, typename Domain>
double domainIteratorScan(const Image &aImage, const Domain &aDomain)
{
  long int cpt = 0;
  double timer;

  trace.beginBlock( "Domain iterator simple scan ..." );
  for ( typename Domain::ConstIterator it = aDomain.begin(),
      itend = aDomain.end(); it != itend; ++it)
    cpt += (long int) aImage( (*it) );

  timer =  trace.endBlock();
  trace.info() << "Cpt=" << cpt << endl;
  return timer;
}

template<typename Point, typename Image, typename Domain>
bool testSuite(unsigned int dim, unsigned int n)
{
  double alloc, builtinconstiter, domainiter;

  Point a = Point::zero, b;
  for (unsigned int i = 0; i < dim; i++)
    b[i] = n;

  try
  {

    Domain aDomain(a, b);
    trace.info() << aDomain << endl;

    trace.beginBlock("init");
    Image image( aDomain);
    alloc = trace.endBlock();

    builtinconstiter = builtinIteratorScan(image);
    domainiter = domainIteratorScan<Image, Domain>(image, aDomain);

    trace.warning() << "Dim= " << dim
    << " n=" << n << " Alloc=" << alloc
    << " Built-in Iter=" << builtinconstiter
    << " DomainIter=" << domainiter
    << endl;

    std::cout << dim << " " << n << " " << alloc << " " << builtinconstiter
        << " " << domainiter << std::endl;

    trace.endBlock();
    return true;
  }
  catch (bad_alloc& ba)
  {
    trace.error() << "bad_alloc caught: " << ba.what() << endl;
    trace.warning() << "Dim= " << dim
    << " n=" << n << " Alloc= XX"
    << " Constiter= XX" << endl;
    std::cout << dim << " " << n << " " << std::endl;
    return false;
  }
}


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ImageContainerBenchmark.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testImageContainerBenchmark()
{
  unsigned int dim = 2;

  std::cout << "#dim n alloc built-in-Iter domain-Iter" << std::endl;

  for (unsigned int n = 100; n < 2000 ; n = n + 100)
  {
    trace.beginBlock("Begin test suite");
    typedef SpaceND<2> Space;
    typedef Space::Point Point;
    typedef HyperRectDomain<Space> Domain;
    //Default image selector = STLVector
    typedef ImageSelector<Domain, int>::Type Image;



    if (!testSuite<Point, Image, Domain>(dim, n))
    {
      trace.endBlock();
      break;
    }

    trace.endBlock();
  }


  dim++;

  for (unsigned int n = 100; n < 2048 ; n = n + 100)
  {
    trace.beginBlock("Begin test suite");
    typedef SpaceND<3> Space;
    typedef Space::Point Point;
    typedef HyperRectDomain<Space> Domain;
    //Default image selector = STLVector
    typedef ImageSelector<Domain, int>::Type Image;

    if (!testSuite<Point, Image, Domain>(dim, n))
    {
      trace.endBlock();
      break;
    }

    trace.endBlock();
  }

  dim++;

  for (unsigned int n = 50; n < 200 ; n = n + 20)
  {
    trace.beginBlock("Begin test suite");
    typedef SpaceND<4> Space;
    typedef Space::Point Point;
    typedef HyperRectDomain<Space> Domain;
    //Default image selector = STLVector
    typedef ImageSelector<Domain, int>::Type Image;

    if (!testSuite<Point, Image, Domain>(dim, n))
    {
      trace.endBlock();
      break;
    }

    trace.endBlock();
  }

  dim++;

  for (unsigned int n = 50; n < 100 ; n = n + 10)
  {
    trace.beginBlock("Begin test suite");
    typedef SpaceND<5> Space;
    typedef Space::Point Point;
    typedef HyperRectDomain<Space> Domain;
    //Default image selector = STLVector
    typedef ImageSelector<Domain, int>::Type Image;

    if (!testSuite<Point, Image, Domain>(dim, n))
    {
      trace.endBlock();
      break;
    }

    trace.endBlock();
  }


  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ImageContainerBenchmark" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testImageContainerBenchmark(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
