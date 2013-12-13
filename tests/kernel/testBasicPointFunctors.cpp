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
 * @file testBasicPointFunctors.cpp
 * @ingroup Tests
 *
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/02
 *
 * This file is part of the DGtal library
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

using namespace DGtal;
using namespace std;

template <typename TFunctor, typename TArg, typename TRes >
void checkingConcepts()
{
  BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, TArg, TRes > ));
}

bool testProjector()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Checking projection on a subspace" );
  {
  //a 3d point
  PointVector<3,int> p(0,1,2); 

  //projectors
  typedef Projector<SpaceND<2,int> > Projector2D; 
  std::vector<Dimension> v1, v2; 
  v1.push_back(0);   v1.push_back(2); 
  v2.push_back(2);   v2.push_back(1);
  Projector2D proj1, proj2, proj3; 
  proj1.init(v1.begin(), v1.end()); 
  proj2.init(v2.begin(), v2.end()); 
 
  //comparison
  PointVector<2,int> res1(0,2); 
  trace.info() << "p " << p << " => " << proj1(p) << " == " << res1 << std::endl;
  nbok += ( proj1(p) == res1 ) ? 1 : 0; 
  nb++;  

  PointVector<2,int> res2(2,1);
  trace.info() << "p " << p << " => " << proj2(p) << " == " << res2 << std::endl;
  nbok += ( proj2(p) == res2 ) ? 1 : 0; 
  nb++;  

  PointVector<2,int> res3(0,1);
  trace.info() << "p " << p << " => " << proj3(p) << " == " << res3 << std::endl;
  nbok += ( proj3(p) == res3 ) ? 1 : 0; 
  nb++;  
  }
  trace.endBlock();

  trace.beginBlock ( "Checking projection on a greater space" );
  {
  //a 2d point
  PointVector<2,int> p(5,2); 

  //projectors
  typedef Projector<SpaceND<3,int> > Projector3D; 
  std::vector<Dimension> v1, v2, v4; 
  v1.push_back(0);   v1.push_back(2); v1.push_back(1);  
  v2.push_back(1);   v2.push_back(0);
  v4.push_back(1);
  Projector3D proj1, proj2, proj3; 
  proj1.init(v1.begin(), v1.end()); 
  proj2.init(v2.begin(), v2.end());
  Projector3D proj4(-1);  
  proj4.init(v4.begin(), v4.end()); 

  //SliceRotator2D
  PointVector<3, int> pt1(0,0, 0);
  PointVector<3, int> pt2(10,10, 10);
  HyperRectDomain<SpaceND<3, int> >  domain (pt1, pt2);
  SliceRotator2D< HyperRectDomain<SpaceND<3, int> >, int> sliceRot(2, domain, 6, 1,  pt1, 0.1);
  SliceRotator2D< HyperRectDomain<SpaceND<3, int> >, int> sliceRot2(2, domain, 7, 2, 3.14);
  PointVector<2, int> pt(5,5);  
  PointVector<2, int> pt_2(10, 9);  
  PointVector<3, int> ptR(4,5,6);  
  PointVector<3, int> ptR2(0, 1, 7);  
 
  trace.info() << "pt " << pt << " => " << sliceRot(pt) << " == " << ptR << std::endl;
  nbok += ( sliceRot(pt) == ptR ) ? 1 : 0; 
  nb++;
  
  trace.info() << "pt " << pt_2 << " => " << sliceRot2(pt_2) << " == " << ptR2 << std::endl;
  nbok += ( sliceRot2(pt_2) == ptR2 ) ? 1 : 0; 
  nb++;

  //Point2DEmbedderIn3D
  PointVector<3,int> ptOrigin3D(3,3,3);
  Point2DEmbedderIn3D< HyperRectDomain<SpaceND<3, int> >, int> embedder(domain,
									ptOrigin3D,
									PointVector<3,int>(6,6,3),
									PointVector<3,int>(3,3,5),
									PointVector<3,int>(0,0,0));
  PointVector<2, int> ptb(0, 0);  
  PointVector<2, int> pt_2b(4, 2);  
    trace.info() << "pt " << ptb << " => " << embedder(ptb) << " == " << PointVector<3,int>(3,3,3) << std::endl;
  nbok += ( embedder(ptb) == PointVector<3,int>(3,3,3) ) ? 1 : 0; 
  nb++;
  
  trace.info() << "pt " << pt_2b << " => " << embedder(pt_2b) << " == " << PointVector<3,int>(5,5,5) << std::endl;
  nbok += ( embedder(pt_2b) ==  PointVector<3,int>(5,5,5)  ) ? 1 : 0; 
  nb++;

  //Point2DEmbedderIn3D (constructor from normal point)
  PointVector<3,int> pt2Origin3D(5,5,3);
  Point2DEmbedderIn3D< HyperRectDomain<SpaceND<3, int> >, int> embedder2(domain,
                                                                         pt2Origin3D,
                                                                         PointVector<3,int>(0,0,3),
									 4); 
  PointVector<2, int> pt2b(0, 0);  
  PointVector<2, int> pt2_2b(2, 2);  
    trace.info() << "pt " << pt2b << " => " << embedder2(pt2b) << " == " << PointVector<3,int>(3,5,3) << std::endl;
    nbok += ( embedder2(pt2b) == PointVector<3,int>(3,5,3) ) ? 1 : 0; 
  nb++;
  
  trace.info() << "pt " << pt2_2b << " => " << embedder2(pt2_2b) << " == " << PointVector<3,int>(5,4,3) << std::endl;
  nbok += ( embedder2(pt2_2b) ==  PointVector<3,int>(5,4,3)  ) ? 1 : 0; 
  nb++;


  //comparison
  PointVector<3,int> res1(5,0,2); 
  trace.info() << "p " << p << " => " << proj1(p) << " == " << res1 << std::endl;
  nbok += ( proj1(p) == res1 ) ? 1 : 0; 
  nb++;  

  PointVector<3,int> res2(2,5,0);
  trace.info() << "p " << p << " => " << proj2(p) << " == " << res2 << std::endl;
  nbok += ( proj2(p) == res2 ) ? 1 : 0; 
  nb++;  

  PointVector<3,int> res3(5,2,0);
  trace.info() << "p " << p << " => " << proj3(p) << " == " << res3 << std::endl;
  nbok += ( proj3(p) == res3 ) ? 1 : 0; 
  nb++;  

  PointVector<3,int> res4(2,-1,-1);
  trace.info() << "p " << p << " => " << proj4(p) << " == " << res4
	       << "(-1 as default value)" << std::endl;
  nbok += ( proj4(p) == res4 ) ? 1 : 0; 
  nb++;  
  }
  trace.endBlock();

  return nbok == nb;
}

//////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing basic point functors" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;


  checkingConcepts<Projector<SpaceND<2,int> >, PointVector<6,int>, PointVector<2,int> >(); 
  //for instance, this does not compile because 
  //the point of dim 6 is projected on a point of dim 2 (and not 3)
  //checkingConcepts<Projector<SpaceND<2,int> >, PointVector<6,int>, PointVector<3,int> >(); 

  bool res = testProjector();

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}

/** @ingroup Tests **/
