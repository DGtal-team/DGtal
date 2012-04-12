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
 * @file exampleFMM.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/23
 *
 * The aim of this example
 * is to illustrate the FMM (fast marching method) algorithm
 * for incremental distance transform. 
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
// annulus
// template <typename TPoint>
// class AnnulusPredicate 
// {
// public:
//   typedef TPoint Point;

// public: 

//   AnnulusPredicate(double aCx, double aCy, double aR, double aW = 1.5): 
//     myCx(aCx), myCy(aCy), myR(aR), myW(aW)
//   { ASSERT(myR > 0); ASSERT(myW > 0); }; 

//   bool operator()(const TPoint& aPoint) const 
//   {
//     double d = std::sqrt( std::pow( (myCx-aPoint[0] ), 2) 
// 			  + std::pow( (myCy-aPoint[1] ), 2) );  
//     if ( (d <= (myR+myW))&&(d >= (myR-myW)) ) return true; 
//     else return false; 
//   };
// private: 
//   double myCx, myCy, myR, myW; 
// };

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



//////////////////////////////////////////////////////////////////////////////
// main task
bool performDT(int size)
{

  static const DGtal::Dimension dimension = 2; 

  //Domain
  typedef HyperRectDomain< SpaceND<dimension, int> > Domain; 
  typedef Domain::Point Point; 
  Domain d(Point::diagonal(-size), Point::diagonal(size)); 
  double h = 1.0/(double)size; 

  //predicate
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


  { ///low accuracy
    //Image and set
    typedef ImageContainerBySTLMap<Domain,double> Image; 
    typedef DigitalSetFromMap<Image> Set; 
    Image map( d ); 
    Set set(map); 

    //initialisation
    //! [FMMDef]
    typedef FMM<Image, Set, Predicate > FMM;
    //! [FMMDef]

    //! [FMMInit1]
    FMM::initFromBelsRange( K, 
			    vSCells.begin(), vSCells.end(), 
			    map, set, 0.5 ); 
    //! [FMMInit1]

    //computation
    //! [FMMUsage]
    FMM fmm(map, set, predicate); 
    fmm.compute(); 
    trace.info() << fmm << std::endl;
    //! [FMMUsage]

    //max
    double truth = radius*h; 
    double found = ( std::max(std::abs(fmm.max()),std::abs(fmm.min())) )*h;
    trace.info() << " # radius (low accuracy)" << std::endl; 
    trace.info() << " # truth: " << truth << std::endl; 
    trace.info() << " # found: " << found << std::endl; 
    trace.info() << " # diff.: " << std::abs(found-truth) << std::endl; 

  }

  { ///high accuracy
    //Image and set
    typedef ImageContainerBySTLMap<Domain,double> Image; 
    typedef DigitalSetFromMap<Image> Set; 
    Image map( d ); 
    Set set(map); 
    //  map.setValue(Point(0,0), 0.0); 

    //initialisation
    //! [FMMDef2]
    typedef FMM<Image, Set, Predicate > FMM;
    //! [FMMDef2]
    typedef BallFunctor<Point> Functor; 
    Functor functor( 0, 0, radius ); 
    //! [FMMInit2]
    FMM::initFromBelsRange( K, 
			    vSCells.begin(), vSCells.end(), 
			    functor, map, set ); 
    //! [FMMInit2]

    //computation
    //! [FMMUsage2]
    FMM fmm(map, set, predicate); 
    fmm.compute(); 
    trace.info() << fmm << std::endl;
    //! [FMMUsage2]

    //max
    double truth = radius*h; 
    double found = ( std::max(std::abs(fmm.max()),std::abs(fmm.min())) )*h;
    trace.info() << " # radius (high accuracy)" << std::endl; 
    trace.info() << " # truth: " << truth << std::endl; 
    trace.info() << " # found: " << found << std::endl; 
    trace.info() << " # diff.: " << std::abs(found-truth) << std::endl; 
  }

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

  trace.info() << "NB: You can increase the resolution as follows: "
	       << std::endl << argv[0] << " 30 (default)" << std::endl; 


  //size parameter
  int size = 30; 
  if (argc > 1) 
    {
      std::istringstream s( argv[1] ); 
      s >> size;  
    }
  //computation
  performDT(size); 

  trace.endBlock();
  return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
