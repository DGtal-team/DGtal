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
 * @file exampleFMM2D.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/23
 *
 * @brief The aim of this example
 * is to use the FMM (fast marching method) class
 * in order to extrapolate a speed field out of an interface, 
 * such that the extended field is constant on rays normal to the interface.  
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iomanip>
#include <functional>

#include "DGtal/base/Common.h"

//space, domain and image
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetFromMap.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/topology/SCellsFunctors.h"

//tracking
#include "DGtal/topology/helpers/Surfaces.h"


//FMM
#include "DGtal/geometry/volumes/distance/FMM.h"

//Display
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


//////////////////////////////////////////////////////////////////////////////
// ball
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

template <typename TPoint>
class BallFunctor 
{
public:
  typedef TPoint Point;
  typedef double Value; 
public: 

  BallFunctor(double aCx, double aCy, double aR ): 
    myCx(aCx), myCy(aCy), myR(aR)
  { ASSERT(myR > 0); }; 

  Value operator()(const TPoint& aPoint) const 
  {
    double d = std::sqrt( std::pow( (myCx-aPoint[0] ), 2) 
			  + std::pow( (myCy-aPoint[1] ), 2) );  
    return (d - myR);
  };
private: 
  double myCx, myCy, myR; 
};

template< typename TIterator >
void draw( const TIterator& itb, const TIterator& ite, const int& size, std::string basename) 
{
  typedef typename std::iterator_traits<TIterator>::value_type Pair; 
  typedef typename Pair::first_type Point; 
  typedef typename Pair::second_type Value; 
  HueShadeColorMap<unsigned char, 2> colorMap(0,3*size);

  Board2D b; 
  b.setUnit ( LibBoard::Board::UCentimeter );
 
  for (TIterator it = itb; it != ite; ++it)
    {
      Point p = it->first;
      b << CustomStyle( p.className(), new CustomFillColor( colorMap( it->second) ) );
      b << p;
    }

  std::stringstream s; 
  s << basename << ".eps"; 
  b.saveEPS(s.str().c_str());
} 


//////////////////////////////////////////////////////////////////////////////
// main task
bool perform()
{

  static const DGtal::Dimension dimension = 2; 

  //Domain
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point; 
  int size = 50; 
  Domain d(Point::diagonal(-size), Point::diagonal(size)); 
  double h = 1.0/(double)size; 

  //Predicate, which implicitely define the interface
  int radius = (size/2);
  typedef BallPredicate<Point> Predicate; 
  Predicate predicate( 0, 0, radius ); 
  trace.info() << " # circle of radius 0.5 "
	       << "digitized in a square of size 1 "
	       << "at step h=" << h << endl; 

  //Digital circle generation
  typedef KhalimskySpaceND< dimension, int > KSpace; 
  KSpace K; K.init( Point::diagonal(-size), Point::diagonal(size), true); 
  SurfelAdjacency<KSpace::dimension> SAdj( true );
  KSpace::SCell bel = Surfaces<KSpace>::findABel( K, predicate, 10000 );
  std::vector<KSpace::SCell> vSCells;
  Surfaces<KSpace>::track2DBoundary( vSCells, K, SAdj, predicate, bel );

  //Image of distance values and associated set
  typedef ImageContainerBySTLMap<Domain,double> DistanceImage; 
  typedef DigitalSetFromMap<DistanceImage> Set; 
  DistanceImage dmap( d ); 
  Set set(dmap); 

  //initialisation
  //! [FMMFullDef]
  typedef L2SecondOrderLocalDistance<DistanceImage, Set> Distance; 
  typedef FMM<DistanceImage, Set, Domain::Predicate, Distance > FMM;
  //! [FMMFullDef]

  typedef BallFunctor<Point> Functor; 
  Functor functor( 0, 0, radius ); 
  FMM::initFromBelsRange( K, 
			  vSCells.begin(), vSCells.end(), 
			  functor, dmap, set ); 

  //Image of speed values
  typedef ImageContainerBySTLMap<Domain,double> SpeedImage; 
  SpeedImage smap(d, 0.0); 

  SCellToIncidentPoints<KSpace> belToPair( K );
  for (std::vector<KSpace::SCell>::const_iterator it = vSCells.begin(),
	 itEnd = vSCells.end(); it != itEnd; ++it) 
    {
      SCellToIncidentPoints<KSpace>::Output pair = belToPair( *it );
      Point in = pair.first; 
      Point out = pair.second; 
      //speed equal to the x-coordinate
      smap.setValue( in, in[0] ); 
      smap.setValue( out, out[0] ); 
    }


  //Extrapolating speed away from the interface 
  SpeedExtrapolator<DistanceImage, Set, SpeedImage> speedFunctor(dmap, set, smap); 
  const double maxWidth = 10.0; 
  Distance distFunctor(dmap, set); 
  FMM fmm(dmap, set, d.predicate(), d.size(), maxWidth, distFunctor ); 
  Point lastPt = Point::diagonal(0);      //last point
  double lastDist = 0.0;                  //its distance
  while ( (fmm.computeOneStep( lastPt, lastDist )) 
	  && (std::abs( lastDist ) < maxWidth) )
    {
      //new speed value
      smap.setValue( lastPt, speedFunctor( lastPt ) ); 
    }

  trace.info() << fmm << std::endl;

  //display - you should see constant colors 
  //on rays normal to the interface. 
  std::stringstream s; 
  s << "SpeedExt-" << radius; 
  draw(smap.begin(), smap.end(), size, s.str());

  return true; 

}





///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main ( int argc, char** argv )
{
  trace.beginBlock ( "Example 2d FMM" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  //computation
  perform(); 

  trace.endBlock();
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
