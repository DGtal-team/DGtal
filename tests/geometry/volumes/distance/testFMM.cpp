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
#include <functional>

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
#include "DGtal/geometry/volumes/distance/FirstOrderIncrementalMetric.h"
#include "DGtal/geometry/volumes/distance/FMM.h"

//Display
#include "DGtal/io/colormaps/HueShadeColorMap.h"
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
// 
template <Dimension dim, int norm>
struct MetricTraits
{
  typedef FirstOrderIncrementalMetric<PointVector<dim,int>, 
    LInfinityFirstOrderIncrementalMetricHelper<dim,int> > Metric;  
};
//partial specialization
template <Dimension dim>
struct MetricTraits<dim, 1>
{
  typedef FirstOrderIncrementalMetric<PointVector<dim,int>, 
    L1FirstOrderIncrementalMetricHelper<dim,int> > Metric;  
};

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

template< typename TIterator >
void draw( const TIterator& itb, const TIterator& ite, const int& size, std::string basename) 
{
  typedef typename std::iterator_traits<TIterator>::value_type Pair; 
  typedef typename Pair::first_type Point; 
  typedef typename Pair::second_type Value; 
  HueShadeColorMap<unsigned char, 2> colorMap(0,3*size);

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
 
  typedef FirstOrderIncrementalMetric<Point> Metric; 
  typedef Metric::Value Distance;  

  typedef FMM<Metric, DomainPredicate<Domain> > FMM; 

  //init
  Point c = Point::diagonal(0); 
  Point up = Point::diagonal(size); 
  Point low = Point::diagonal(-size); 

  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( c, 0.0 ) );

  Metric mc; 
  Domain d(low, up); 
  DomainPredicate<Domain> dp(d);

  //computation
  trace.beginBlock ( "Display 2d FMM results " );
 
  FMM fmm(map, mc, dp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  trace.endBlock();

  //display
  std::stringstream s; 
  s << "DTFrom2dPt-" << size << "-" << area << "-" << distance; 
  draw(map.begin(), map.end(), size, s.str());

  return fmm.isValid(); 
}

/**
 * Simple 3d distance transform
 * and slice display
 */
bool testDisplayDT3d(int size, int area, double distance)
{

  static const DGtal::Dimension dimension = 3; 

  //type definitions 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point;
 
  typedef FirstOrderIncrementalMetric<Point> Metric; 
  typedef Metric::Value Distance;  

  typedef FMM<Metric, DomainPredicate<Domain> > FMM; 

  //init
  Point c = Point::diagonal(0); 
  Point up = Point::diagonal(size); 
  Point low = Point::diagonal(-size); 

  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( c, 0.0 ) );

  Metric mc; 
  Domain d(low, up); 
  DomainPredicate<Domain> dp(d);

  //computation
  trace.beginBlock ( "Display 3d FMM results " );
 
  FMM fmm(map, mc, dp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  trace.endBlock();

  {  //display
    HueShadeColorMap<unsigned char, 2> colorMap(0,2*size);

    Board2D b; 
    b.setUnit ( LibBoard::Board::UCentimeter );

    std::map<Point,Distance>::iterator it = map.begin(); 
    for ( ; it != map.end(); ++it)
      {
	Point p3 = it->first;
	if (p3[2] == 0)
	  {
	    PointVector<2,Point::Coordinate> p2(p3[0], p3[1]); 
	    b << CustomStyle( p2.className(), new CustomFillColor( colorMap( it->second) ) )
	      << p2;
	  }
      }

    std::stringstream s; 
    s << "DTFrom3dPt-" << size << "-" << area << "-" << distance
      << ".eps"; 
    b.saveEPS(s.str().c_str());
  }

  return fmm.isValid(); 
}

bool testDispalyDTFromCircle(int size)
{

  static const DGtal::Dimension dimension = 2; 

  //space and domain 
  typedef KhalimskySpaceND<dimension,int> KSpace; 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point;
 
  //Metric
  typedef FirstOrderIncrementalMetric<Point> Metric; 
  typedef Metric::Value Distance;  
  Metric mc; 

  //Digital circle generation
  GridCurve<KSpace> gc;   
  double radius = (rand()%size);
  trace.info() << " #ball c(" << 0 << "," << 0 << ") r=" << radius << endl; 
  ballGenerator<KSpace>( size, 0, 0, radius, gc ); 


  unsigned int nbok = 0;
  unsigned int nb = 0;

  double dmaxInt = 0; 
  trace.beginBlock ( "Interior " );
  {
    typedef BallPredicate<Point> Predicate; 
    typedef FMM<Metric, Predicate > FMM;

    //init
    std::map<Point, Distance> map; 
    GridCurve<KSpace>::OuterPointsRange r = gc.getOuterPointsRange();
    FMM::initInnerPoints(r.begin(), r.end(), map, 0.5); 

    //computation
    Predicate bp(0,0,radius); 
    FMM fmm(map, mc, bp); 
    fmm.compute(); 
    trace.info() << fmm << std::endl;
    nbok += (fmm.isValid()?1:0); 
    trace.info() << nbok << "/" << ++nb << std::endl; 

    //max
    {
      std::map<Point, Distance>::const_iterator it = map.begin(); 
      std::map<Point, Distance>::const_iterator itEnd = map.end(); 
      for ( ; it != itEnd; ++it)
	{
	  if (it->second > dmaxInt) dmaxInt = it->second; 
	}
      trace.info() << dmaxInt << std::endl;
    }

    //display
    std::stringstream s; 
    s << "DTInCircle-" << size; 
    draw(map.begin(), map.end(), size, s.str());
  }
  trace.endBlock();

  double dmaxExt = 0; 
  trace.beginBlock ( "Exterior " );
  {
    typedef NotPointPredicate<BallPredicate<Point> > PointPredicate; 
    typedef BinaryPointPredicate<PointPredicate, 
      DomainPredicate<Domain> > Predicate; 
    typedef FMM<Metric, Predicate > FMM;

    //init
    std::map<Point, Distance> map; 
    GridCurve<KSpace>::InnerPointsRange r = gc.getInnerPointsRange();
    FMM::initInnerPoints(r.begin(), r.end(), map, 0.5); 

    //computation
    DomainPredicate<Domain> dp( Domain(Point(-size,-size), Point(size,size)) ); 
    PointPredicate bp( BallPredicate<Point>(0,0,radius) );
    Predicate pred( bp, dp, andBF2 ); 
    FMM fmm(map, mc, pred); 
    fmm.compute(); 
    trace.info() << fmm << std::endl; 
    nbok += (fmm.isValid()?1:0); 
    trace.info() << nbok << "/" << ++nb << std::endl; 

    //max
    {
      std::map<Point, Distance>::const_iterator it = map.begin(); 
      std::map<Point, Distance>::const_iterator itEnd = map.end(); 
      for ( ; it != itEnd; ++it)
	{
	  if (it->second > dmaxExt) dmaxExt = it->second; 
	}
      trace.info() << dmaxExt << std::endl;
    }

    //display
    std::stringstream s; 
    s << "DTOutCircle-" << size; 
    draw(map.begin(), map.end(), size, s.str());
  }
  trace.endBlock();

  double dmin = 2*size*size; 
  double dmax = 0; 
  trace.beginBlock ( "Both " );
  {
    typedef DomainPredicate<Domain> Predicate; 
    typedef FMM<Metric, Predicate > FMM;

    //init
    std::map<Point, Distance> map; 
    GridCurve<KSpace>::IncidentPointsRange r = gc.getIncidentPointsRange();
    FMM::initIncidentPoints(r.begin(), r.end(), map, 0.5, true); 

    //computation
    DomainPredicate<Domain> dp( Domain(Point(-size,-size), Point(size,size)) ); 
    FMM fmm(map, mc, dp); 
    fmm.compute(); 
    trace.info() << fmm << std::endl;
    nbok += (fmm.isValid()?1:0); 
    trace.info() << nbok << "/" << ++nb << std::endl; 

    //min, max
    {
      std::map<Point, Distance>::const_iterator it = map.begin(); 
      std::map<Point, Distance>::const_iterator itEnd = map.end(); 
      for ( ; it != itEnd; ++it)
	{
	  if (it->second > dmax) dmax = it->second; 
	  if (it->second < dmin) dmin = it->second; 
	}
      trace.info() << dmin << ", " << dmax << std::endl;
    }

    //display
    std::stringstream s; 
    s << "DTfromCircle-" << size; 
    draw(map.begin(), map.end(), size, s.str());
  }
  trace.endBlock();

  trace.beginBlock ( "Comparison " );
  {
    double epsilon = 0.0001;
    nbok += ( ( (std::abs(-dmaxInt - dmin) < epsilon) 
		&& (std::abs(dmaxExt - dmax) < epsilon) )?1:0); 
    trace.info() << nbok << "/" << ++nb << std::endl; 
  }
  trace.endBlock();

  return (nb == nbok); 

}


/**
 * Comparison with the separable distance transform
 *
 */
template<Dimension dim, int norm>
bool testComparison(int size, int area, double distance)
{

  static const DGtal::Dimension dimension = dim; 

  //type definitions 
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef typename Domain::Point Point;

  //init
  Point c = Point::diagonal(0); 
  Point up = Point::diagonal(size); 
  Point low = Point::diagonal(-size); 
  Domain d(low, up); 

  //image construction
  typedef typename ImageSelector<Domain, unsigned int>::Type Image;
  Image image ( Domain(low, up) );
  typename Domain::Iterator dit = d.begin(); 
  typename Domain::Iterator ditEnd = d.end(); 
  for ( ; dit != ditEnd; ++dit)
    {
      image.setValue(*dit, 128); 
    }
  image.setValue(c, 0); 

  //computation
  trace.beginBlock ( " FMM computation " ); 
 
  typedef typename MetricTraits<dimension,norm>::Metric Metric; 
  typedef typename Metric::Value Distance;  
  typedef FMM<Metric, DomainPredicate<Domain> > FMM; 

  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( c, 0 ) );

  Metric mc; 
  DomainPredicate<Domain> dp(d);

  FMM fmm(map, mc, dp, area, distance); 
  fmm.compute(); 
  trace.info() << fmm << std::endl; 

  trace.endBlock();

  trace.beginBlock ( " DT computation " );

  typedef DistanceTransformation<Image, norm> DT; 
  DT dt;
  typename DT::OutputImage resultImage = dt.compute ( image );
  trace.info() << resultImage << std::endl; 

  trace.endBlock();

  bool flagIsOk = true; 

  trace.beginBlock ( " Comparison " );
  //all points of result must be in map and have the same distance
  typename Domain::ConstIterator it = d.begin(); 
  typename Domain::ConstIterator itEnd = d.end(); 
     for ( ; ( (it != itEnd)&&(flagIsOk) ); ++it)
       {
	 typename std::map<Point, Distance>::iterator itMap = map.find(*it); 
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

  //2d L2 tests
  int size = 50; 
  bool res =   
    testDisplayDT2d( size, (2*size+1)*(2*size+1), std::sqrt(2*size*size) )
    && testDisplayDT2d( size, (2*size+1)*(2*size+1), size )
    && testDisplayDT2d( size, 2*size*size, std::sqrt(2*size*size) )
    && testDispalyDTFromCircle(size);   

  size = 50; 
  //3d L2 test
  res = res && testDisplayDT3d( size, 4*size*size*size, std::sqrt(size*size*size) )
    ; 

  //3d L1 and Linf comparison
  size = 20; 
  res = res  
    && testComparison<3,1>( size, (2*size+1)*(2*size+1)*(2*size+1)+1, 3*size+1 )
    && testComparison<3,0>( size, (2*size+1)*(2*size+1)*(2*size+1)+1, size+1 )
;
  //&& ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
