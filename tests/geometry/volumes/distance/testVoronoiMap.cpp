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
#include "DGtal/images/CConstImage.h"
#include "DGtal/geometry/volumes/distance/VoronoiMap.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/InexactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VoronoiMap.
///////////////////////////////////////////////////////////////////////////////


template<typename Point>
double mynorm(const Point &point, const double p)
{
  double res=0.0;
  for(unsigned int i=0; i< Point::dimension; i++)
    res +=  std::pow ( (double)abs(point[i]) , p);
  
  return std::pow(res, 1.0/p);
}

template <typename VoroMap>
void saveVoroMap(const std::string &filename,const VoroMap &output,const double p)
{
  typedef HueShadeColorMap<double,2> Hue;
  double maxdt=0.0;
  
  for ( typename VoroMap::Domain::ConstIterator it = output.domain().begin(), itend = output.domain().end();
	it != itend; ++it)
    {
      typename VoroMap::Value point = output(*it);
      if ( mynorm(point-(*it),p) > maxdt)
        maxdt = mynorm(point-(*it),p);
    }
  trace.error() << "MaxDT="<<maxdt<<std::endl;
      
  Board2D board;
  Hue hue(0,maxdt);
  
  for(typename VoroMap::Domain::ConstIterator it = output.domain().begin(), 
        itend = output.domain().end();
      it != itend; ++it)
    {
      typename VoroMap::Value point = output(*it);
      board << CustomStyle( (*it).className(), new CustomColors( hue(mynorm(point- (*it),p)), 
                                                                 hue(mynorm(point- (*it),p))))
            << (*it);
    }

  board.saveSVG(filename.c_str());
}


/* Is Validate the VoronoiMap
 */
template < typename Set, typename Image>
bool checkVoronoiL2(const Set &aSet, const Image & voro)
{
  typedef typename Image::Point Point;
  
  for(typename Image::Domain::ConstIterator it = voro.domain().begin(), itend = voro.domain().end();
      it != itend; ++it)
    {
      Point psite  = voro(*it);
      Point p = (*it);
      DGtal::int64_t d=0;
      for(DGtal::Dimension i=0; i<Point::dimension;++i)
	d+= (p[i]-psite[i])*(p[i]-psite[i]);
      
      for(typename Set::ConstIterator itset = aSet.begin(), itendSet = aSet.end(); 
          itset != itendSet;
          ++itset)
	{
	  DGtal::int64_t dbis=0;
	  for(DGtal::Dimension i=0; i<Point::dimension;++i)
	    dbis+= (p[i]-(*itset)[i])*(p[i]-(*itset)[i]);
	  if ( dbis < d)
	    {
	      trace.error() << "DT Error at "<<p<<"  Voro:"<<psite<<" ("<<d<<")  from set:"
			    << (*itset) << "("<<dbis<<")"<<std::endl;
	      return false;
	    }
	}
    }
  return true;
}


bool testCheckConcept()
{
  typedef ExactPredicateLpSeparableMetric<Z3i::Space,2> L2Metric;
  BOOST_CONCEPT_ASSERT(( CConstImage< VoronoiMap<Z3i::Space, Z3i::DigitalSet, L2Metric> >));
  
  return true;
}

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
  

  Z2i::DigitalSet sites(domain);
  
  sites.insertNew( Z2i::Point(0,-6));
  sites.insertNew( Z2i::Point(6,0));
  sites.insertNew( Z2i::Point(-6,0));
  
  for(Z2i::DigitalSet::ConstIterator it = sites.begin(), itend = sites.end();
      it != itend; ++it)
    mySet.erase (*it);
  


  typedef ExactPredicateLpSeparableMetric<Z2i::Space, 2> L2Metric;
  typedef VoronoiMap<Z2i::Space, Z2i::DigitalSet, L2Metric> Voro2;
  L2Metric l2;
  Voro2 voro(&domain, &mySet,&l2);
 
  for(int j=-10; j <= 10; j++)
    {    
      for(int i=-10; i<=10; i++)
        trace.info() << "("<<voro( Z2i::Point(i,j))[0]<<","<< voro( Z2i::Point(i,j))[1]<<") ";
      trace.info()<<std::endl;
    }

  trace.info()<<"Exporting o SVG"<<std::endl;
 
  Board2D board;
  for(Voro2::OutputImage::Domain::ConstIterator it = voro.domain().begin(), 
        itend = voro.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = voro(*it);
      unsigned char c = (p[1]*13 + p[0] * 7) % 256;
      board << CustomStyle( (*it).className(), new CustomColors(Color(c,c,c),Color(c,c,c)))
            << (*it);
    }

  board.saveSVG("Voromap.svg");

  nbok += checkVoronoiL2(sites,voro) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Voronoi diagram is valid !" << std::endl;
  trace.endBlock();
  
  


  return nbok == nb;
}



/**
 * Example of a test. To be completed.
 *
 */
template<typename Set>
bool testVoronoiMapFromSites2D(const Set &aSet, const std::string &name)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Set mySet(aSet.domain());
  
  for(typename Set::Domain::ConstIterator it = aSet.domain().begin(), itend = aSet.domain().end(); 
      it != itend;
      ++it)
    mySet.insertNew( *it );
  
  
  for(typename Set::ConstIterator it = aSet.begin(), itend = aSet.end();
      it != itend; ++it)
    mySet.erase (*it);
  

  trace.beginBlock(" Voro computation");
  typedef ExactPredicateLpSeparableMetric<typename Set::Space,2> L2Metric;
  typedef VoronoiMap<typename Set::Space, Set, L2Metric> Voro2;  
  L2Metric l2;
  Voro2 voro(aSet.domain(), mySet, l2 );

  trace.endBlock();

  trace.beginBlock(" Voronoi computation l_6");
  typedef ExactPredicateLpSeparableMetric<typename Set::Space,6> L6Metric;
  L6Metric l6;
  typedef VoronoiMap<typename Set::Space, Set, L6Metric> Voro6;
  Voro6 voro6(aSet.domain(), mySet, l6 );
  trace.endBlock();



  trace.beginBlock(" DT computation");
  typedef DistanceTransformation<typename Set::Space, Set, L2Metric> DT;
  DT dt(aSet.domain(), mySet, l2);
  trace.endBlock();


  if ( (aSet.domain().upperBound()[1] - aSet.domain().lowerBound()[1]) <20)
    {
      for(int j= aSet.domain().lowerBound()[1]; j <= aSet.domain().upperBound()[1]; j++)
	{    
	  for(int i=aSet.domain().lowerBound()[0]; i<=aSet.domain().upperBound()[0]; i++)
	    if ( aSet.find( Z2i::Point(i,j) ) != aSet.end() )
	      std::cout <<"X ";
	    else
	      std::cout<<"0 ";
	  trace.info()<<std::endl;
	}
  
      trace.info() << std::endl;
      
      for(int j= aSet.domain().lowerBound()[1]; j <= aSet.domain().upperBound()[1]; j++)
	{    
	  for(int i=aSet.domain().lowerBound()[0]; i<=aSet.domain().upperBound()[0]; i++)
	    trace.info() << "("<<voro( Z2i::Point(i,j))[0]<<","<< voro( Z2i::Point(i,j))[1]<<") ";
	  trace.info()<<std::endl;
	}
    }

  Board2D board;
  board << voro.domain();
  for(typename Voro2::OutputImage::Domain::ConstIterator it = voro.domain().begin(), itend = voro.domain().end();
      it != itend; ++it)
    {
      if (!mySet(*it))
	board  << (*it);
    }
  std::string filename= "Voromap-"+name+"-orig.svg";
  board.saveSVG(filename.c_str());
  
  board.clear();
  board << voro.domain();
  board.setPenColor(Color(0,0,0));
  for(typename Voro2::OutputImage::Domain::ConstIterator it = voro.domain().begin(), itend = voro.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = voro(*it);
      if ((p != (*it)) && (p != voro.domain().upperBound() + Z2i::Point::diagonal(1))
	  &&  (p != voro.domain().lowerBound()))
	Display2DFactory::draw( board,   p - (*it), (*it)); 
    }

  filename= "Voromap-"+name+"-diag.svg";
  board.saveSVG(filename.c_str());
  

  board.clear();
  board << voro.domain();
  for(typename Voro2::OutputImage::Domain::ConstIterator it = voro.domain().begin(), itend = voro.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = voro(*it);
      unsigned char c = (p[1]*13 + p[0] * 7) % 256;
      //    if ((p != (*it)) && (p != voro.domain().upperBound() + Z2i::Point::diagonal(1))
      //	  &&  (p != voro.domain().lowerBound()))
	board << CustomStyle( (*it).className(), new CustomColors(Color(c,c,c),Color(c,c,c)))
	      << (*it);;
    }

  filename= "Voromap-"+name+".svg";
  board.saveSVG(filename.c_str());
  filename= "Voromap-"+name+"-hue.svg";
  saveVoroMap(filename.c_str(),voro,2);


  
  board.clear();
  for(typename Voro6::OutputImage::Domain::ConstIterator it = voro6.domain().begin(), 
        itend = voro6.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = voro6(*it);
      if (p != (*it))
	Display2DFactory::draw( board,   p - (*it), (*it)); 
    }

  filename= "Voromap-diag-l6-"+name+".svg";
  board.saveSVG(filename.c_str());

  board.clear();
  for(typename Voro6::OutputImage::Domain::ConstIterator it = voro6.domain().begin(), itend = voro6.domain().end();
      it != itend; ++it)
    {
      Z2i::Point p = voro6(*it);
      unsigned char c = (p[1]*13 + p[0] * 7) % 256;
      board << CustomStyle( (*it).className(), new CustomColors(Color(c,c,c),Color(c,c,c)))
            << (*it);;
    }

  filename= "Voromap-l6"+name+".svg";
  board.saveSVG(filename.c_str());
  filename= "Voromap-hue-l6-"+name+".svg";
  saveVoroMap(filename.c_str(),voro6,3);

 
  nbok += checkVoronoiL2(aSet,voro) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Voronoi diagram is valid !" << std::endl;
  
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
template<typename Set>
bool testVoronoiMapFromSites(const Set &aSet)
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Set mySet(aSet.domain());
  
  for(typename Set::Domain::ConstIterator it = aSet.domain().begin(), 
	itend = aSet.domain().end(); 
      it != itend;
      ++it)
    mySet.insertNew( *it );
  
  
  for(typename Set::ConstIterator it = aSet.begin(), itend = aSet.end();
      it != itend; ++it)
    mySet.erase (*it);
  


  trace.beginBlock(" Voronoi computation");
  typedef ExactPredicateLpSeparableMetric<typename Set::Space,2> L2Metric;
  typedef VoronoiMap<typename Set::Space, Set, L2Metric> Voro2;
  L2Metric l2;
  Voro2 voro(aSet.domain(), mySet, l2);
  trace.endBlock();


  trace.beginBlock(" Voronoi computation l_3");
  typedef ExactPredicateLpSeparableMetric<typename Set::Space,3> L3Metric;
  typedef VoronoiMap<typename Set::Space, Set, L3Metric> Voro3;
  L3Metric l3;
  Voro3 voro3(aSet.domain(), mySet, l3);
  trace.endBlock();


  trace.beginBlock(" DT computation");
  typedef DistanceTransformation<typename Set::Space, Set, L2Metric> DT;
  DT dt(aSet.domain(), mySet, l2);

  trace.endBlock();


  trace.beginBlock("Validating the Voronoi Map");
  nbok += (checkVoronoiL2(aSet,voro)   )? 1 : 0; 
  trace.endBlock();
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Voronoi diagram is valid !" << std::endl;
  
  return nbok == nb;
}


bool testSimple2D()
{
  
  Z2i::Point a(-10,-10);
  Z2i::Point b(10,10);
  Z2i::Domain domain(a,b);
  
  Z2i::DigitalSet sites(domain);
  bool ok;

  trace.beginBlock("Simple2D");
  sites.insertNew( Z2i::Point(0,-6));
  sites.insertNew( Z2i::Point(6,0));
  sites.insertNew( Z2i::Point(-6,0));

  ok = testVoronoiMapFromSites2D<Z2i::DigitalSet>(sites,"simple");
  trace.endBlock();

  return ok;

}

bool testSimpleRandom2D()
{

 Z2i::Point a(0,0);
 Z2i::Point b(64,64);
  Z2i::Domain domain(a,b);

  Z2i::DigitalSet sites(domain);
  bool ok;

  trace.beginBlock("Random 2D");
  for(unsigned int i = 0 ; i < 64; ++i)
    {
      Z2i::Point p(  rand() % (b[0]) -  a[0],  rand() % (b[1]) +  a[1]  );
      sites.insert( p );
    }
  ok = testVoronoiMapFromSites2D<Z2i::DigitalSet>(sites,"random");
  trace.endBlock();

  trace.beginBlock("Random 2D (dense)");
  for(unsigned int i = 0 ; i < 64*64-64; ++i)
    {
      Z2i::Point p(  rand() % (b[0]) -  a[0],  rand() % (b[1]) +  a[1]  );
      sites.insert( p );
    }
  ok = testVoronoiMapFromSites2D<Z2i::DigitalSet>(sites,"random-dense");
  trace.endBlock();

  return ok;

}


bool testSimple3D()
{

  Z3i::Point a(-10,-10,-10);
  Z3i::Point b(10,10,10);
  Z3i::Domain domain(a,b);

  Z3i::DigitalSet sites(domain);
  bool ok;

  trace.beginBlock("Simple3D");
  sites.insertNew( Z3i::Point(0,0,-6));
  sites.insertNew( Z3i::Point(6,0,0));
  sites.insertNew( Z3i::Point(-6,0,3));

  ok = testVoronoiMapFromSites<Z3i::DigitalSet>(sites);
  trace.endBlock();

  return ok;

}

bool testSimpleRandom3D()
{

  Z3i::Point a(0,0,0);
  Z3i::Point b(64,64,64);
  Z3i::Domain domain(a,b);
  
  Z3i::DigitalSet sites(domain);
  bool ok;
  
  trace.beginBlock("Random 3D");
  for(unsigned int i = 0 ; i < 64; ++i)
    {
      Z3i::Point p(  rand() % (b[0]) -  a[0], 
                     rand() % (b[1]) +  a[1],
                     rand() % (b[2]) +  a[2]  );
      sites.insert( p );
    }
  ok = testVoronoiMapFromSites<Z3i::DigitalSet>(sites);
  trace.endBlock();

  return ok;

}



bool testSimple4D()
{

  typedef SpaceND<4> Space4;
  Space4::Point a(0,0,0,0);
  Space4::Point b(5,5,5,5);
  HyperRectDomain<Space4> domain(a,b);

  DigitalSetBySTLSet< HyperRectDomain<Space4> > sites(domain);
  bool ok;

  trace.beginBlock("Simple4D");
  sites.insertNew( Space4::Point(1,4,1,1));
  sites.insertNew( Space4::Point(3,1,3,1));
  sites.insertNew( Space4::Point(0,0,0,0));

  ok = testVoronoiMapFromSites<  DigitalSetBySTLSet< HyperRectDomain<Space4> >  >(sites);
  trace.endBlock();

  return ok;

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

  bool res = testCheckConcept() 
    && testVoronoiMap() 
    && testSimple2D()
    &&  testSimpleRandom2D()
    && testSimple3D() 
    && testSimpleRandom3D()
    && testSimple4D()
    ; // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
