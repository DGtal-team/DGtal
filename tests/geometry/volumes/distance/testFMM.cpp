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
 * @file testFMM.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/01/16
 *
 * @brief Functions for the fast marching method.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iomanip>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/BasicPointPredicates.h"

//DT
#include "DGtal/images/ImageSelector.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"

//FMM
#include "DGtal/geometry/volumes/distance/IncrementalMetricComputers.h"
#include "DGtal/geometry/volumes/distance/FMM.h"

//Display
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/boards/Board2D.h"

//shape and digitizer
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/geometry/curves/representation/GridCurve.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

//////////////////////////////////////////////////////////////////////////////
// digital circle generator
template <typename TPoint>
class BallPredicate 
{
  public:
    typedef TPoint Point;

public: 

  BallPredicate(double aCx, double aCy, double aR ): 
    myCx(aCx), myCy(aCy), myR(aR)
  { ASSERT(myR > 0); }; 

  bool operator()(const TPoint& aPoint) const 
  {
    double d = std::sqrt( std::pow( (myCx-aPoint[0] ), 2) 
			  + std::pow( (myCy-aPoint[1] ), 2) );  
    if (d <= myR) return true; 
    else return false; 
  };
private: 
  double myCx, myCy, myR; 
};

template<typename TKSpace>
void
ballGenerator(const int& size, double aCx, double aCy, double aR, GridCurve<TKSpace>& gc)
{

  ASSERT( aR < (double) size ); 
 
  // Types
  typedef TKSpace KSpace;  
  typedef typename KSpace::SCell SCell;
  typedef GridCurve<KSpace> GridCurve; 
  typedef typename KSpace::Space Space;  
  typedef typename Space::Point Point;

  KSpace K;
  bool ok = K.init( Point(-size,-size), Point(size,size), true );
  if ( ! ok )
    {
      std::cerr << " error in creating KSpace." << std::endl;
    }
  try 
    {
      BallPredicate<Point> dig(aCx, aCy, aR); 
      // Extracts shape boundary
      SurfelAdjacency<KSpace::dimension> SAdj( true );
      SCell bel = Surfaces<KSpace>::findABel( K, dig, 10000 );
      // Getting the consecutive surfels of the 2D boundary
      std::vector<Point> points;
      Surfaces<KSpace>::track2DBoundaryPoints( points, K, SAdj, dig, bel );
      gc.initFromVector(points); 
    }
  catch ( InputException e )
    {
      std::cerr << " error in finding a bel." << std::endl;
    }
}

// template<typename TKSpace>
// void
// ballGenerator(const int& size, GridCurve<TKSpace>& gc)
// {
//   double radius = (rand()%size);
//   trace.info() << " #ball c(" << 0 << "," << 0 << ") r=" << radius << endl; 
//   ballGenerator<TKSpace>( size, 0.0, 0.0, radius, gc ); 
// }

template< typename TIterator >
void draw( const TIterator& itb, const TIterator& ite, std::string basename) 
{
  typedef typename std::iterator_traits<TIterator>::value_type Pair; 
  typedef typename Pair::first_type Point; 
  typedef typename Pair::second_type Value; 
  GradientColorMap<Value, DGtal::CMAP_GRAYSCALE> colorMap(0,10); 

  Board2D b; 
  b.setUnit ( LibBoard::Board::UCentimeter );

  TIterator it = itb; 
  for ( ; it != ite; ++it)
    {
      Point p = it->first;
      b << CustomStyle( p.className(), new CustomFillColor( colorMap( it->second) ) );
      b << p;
    }

  std::stringstream s; 
  s << basename << ".eps"; 
  b.saveEPS(s.str().c_str());
} 


/**
 * Simple 2d distance transform
 *
 */
bool testDisplayDT2d(int size, int area, double distance)
{

  static const DGtal::Dimension dimension = 2; 

  //type definitions 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point;
 
  typedef IncrementalEuclideanMetricComputer<dimension> MetricComputer; 
  typedef MetricComputer::Distance Distance;  

  typedef FMM<MetricComputer, DomainPredicate<Domain> > FMM; 

  //init
  Point c(0,0); 
  Point up(size, size); 
  Point low(-size,-size); 

  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( c, 0.0 ) );

  MetricComputer mc; 
  Domain d(low, up); 
  DomainPredicate<Domain> dp(d);

  //computation
  trace.beginBlock ( "Display FMM results " );
 
  FMM fmm(map, mc, dp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  trace.endBlock();

  //display
  std::stringstream s; 
  s << "DTFromPoint-" << size << "-" << area << "-" << distance; 
  draw(map.begin(), map.end(), s.str());

  return fmm.isValid(); 
}

bool testDispalyDTFromCircle(int size, int area, double distance)
{

  static const DGtal::Dimension dimension = 2; 

  //type definitions 
  typedef KhalimskySpaceND<dimension,int> KSpace; 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point;
 
  typedef IncrementalEuclideanMetricComputer<dimension> MetricComputer; 
  typedef MetricComputer::Distance Distance;  

  //init domain
  Point c(0,0); 
  Point up(size, size); 
  Point low(-size,-size); 
  Domain d(low, up); 

  //generate digital circle
  GridCurve<KSpace> gc;   
  double radius = (rand()%size);
  trace.info() << " #ball c(" << 0 << "," << 0 << ") r=" << radius << endl; 
  ballGenerator<KSpace>( size, 0, 0, radius, gc ); 

  trace.beginBlock ( "Interior " );

  //init
  std::map<Point, Distance> map; 
  GridCurve<KSpace>::OuterPointsRange rin = gc.getOuterPointsRange();
  //typedef FMM<MetricComputer, DomainPredicate<Domain> > FMM; 
  typedef FMM<MetricComputer, BallPredicate<Point> > FMM;
  FMM::initInnerPoints(rin.begin(), rin.end(), map, 0.5); 

  //computation
  MetricComputer mc; 
  //DomainPredicate<Domain> dp(d);
  BallPredicate<Point> bp(0,0,radius); 
  FMM fmm(map, mc, bp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  //display
  std::stringstream s; 
  s << "DTFromCircle-" << size << "-" << area << "-" << distance; 
  draw(map.begin(), map.end(), s.str());

  trace.endBlock();

  return fmm.isValid(); 

}


/**
 * Comparison with the separable distance transform
 *
 */
bool testComparison(int size, int area, double distance)
{

  static const DGtal::Dimension dimension = 3; 

  //type definitions 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point;

  //init
  Point c(0,0,0); 
  Point up(size, size, size); 
  Point low(-size,-size,-size); 
  Domain d(low, up); 

  //image construction
  typedef ImageSelector<Domain, unsigned int>::Type Image;
  Image image (low, up);
  Domain::Iterator dit = d.begin(); 
  Domain::Iterator ditEnd = d.end(); 
  for ( ; dit != ditEnd; ++dit)
    {
      image.setValue(*dit, 128); 
    }
  image.setValue(c, 0); 

  //computation
  trace.beginBlock ( " FMM " ); 
 
  typedef IncrementalLInfinityMetricComputer<dimension, int> MetricComputer; 
  typedef MetricComputer::Distance Distance;  

  typedef FMM<MetricComputer, DomainPredicate<Domain> > FMM; 

  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( c, 0 ) );

  MetricComputer mc; 
  DomainPredicate<Domain> dp(d);

  FMM fmm(map, mc, dp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  trace.endBlock();

  trace.beginBlock ( " DT " );

  typedef DistanceTransformation<Image, 0> DT; 
  DT dt;
  DT::OutputImage resultImage = dt.compute ( image );
  trace.info() << resultImage << std::endl; 

  trace.endBlock();

  bool flagIsOk = true; 

  trace.beginBlock ( " Comparison " );
  //all points of result must be in map and have the same distance
  Domain::ConstIterator it = d.begin(); 
  Domain::ConstIterator itEnd = d.end(); 
     for ( ; ( (it != itEnd)&&(flagIsOk) ); ++it)
       {
	 std::map<Point, Distance>::iterator itMap = map.find(*it); 
	 if (itMap == map.end())
	   flagIsOk = false; 
	 else 
	   {
	     if (resultImage(*it) != itMap->second)
	       flagIsOk = false; 
	   }
       }
  trace.endBlock();

  return flagIsOk; 

}



///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main ( int argc, char** argv )
{
  trace.beginBlock ( "Testing FMM" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  int size = 50; 
  bool res =  
    testDisplayDT2d( size, (2*size+1)*(2*size+1), std::sqrt(2*size*size) )
&& testDisplayDT2d( size, (2*size+1)*(2*size+1), size )
&& testDisplayDT2d( size, 2*size*size, std::sqrt(2*size*size) )
;

  res = res && testDispalyDTFromCircle(size, size*size, size);   

   size = 20; 
   res = res && testComparison( size, (2*size+1)*(2*size+1)*(2*size+1)+1, size+1 )
;
  //&& ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
