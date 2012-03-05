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
 * @file testLocalConvolutionNormalVectorEstimator.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/03/03
 *
 * Functions for testing class LocalConvolutionNormalVectorEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigTest.h"
#include "DGtal/base/Common.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/DigitalSetBoundary.h"
#include "DGtal/topology/ImplicitDigitalSurface.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/ExplicitDigitalSurface.h"
#include "DGtal/topology/LightExplicitDigitalSurface.h"
#include "DGtal/topology/BreadthFirstVisitor.h"
#include "DGtal/topology/helpers/FrontierPredicate.h"
#include "DGtal/topology/helpers/BoundaryPredicate.h"
#include "DGtal/topology/CUndirectedSimpleLocalGraph.h"
#include "DGtal/topology/CUndirectedSimpleGraph.h"

#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"

#include "DGtal/images/ImageSelector.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/geometry/surfaces/estimation/LocalConvolutionNormalVectorEstimator.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

struct MyFunctor {

  double operator()(double /*r*/) {return 0.0;}

};

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LocalConvolutionNormalVectorEstimator.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testLocalConvolutionNormalVectorEstimator()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing convolution neighborhood ..." );
  
  
  std::string filename = testPath + "samples/cat10.vol";

  typedef ImageSelector < Z3i::Domain, int>::Type Image;
  Image image = VolReader<Image>::importVol(filename);
  DigitalSet set3d (image.domain());
  SetPredicate<DigitalSet> set3dPredicate( set3d );
  SetFromImage<DigitalSet>::append<Image>(set3d, image, 
                                          0,256);
 
  KSpace ks;
  bool space_ok = ks.init( image.domain().lowerBound(), 
                           image.domain().upperBound(), true );
  if (!space_ok)
    {
      trace.error() << "Error in the Khamisky space construction."<<std::endl;
      return 2;
    }
  trace.endBlock();
  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  MySurfelAdjacency surfAdj( true ); // interior in all directions.
  
  trace.beginBlock( "Set up digital surface." );
  typedef LightImplicitDigitalSurface<KSpace, SetPredicate<DigitalSet> > 
    MyDigitalSurfaceContainer;
  typedef DigitalSurface<MyDigitalSurfaceContainer> MyDigitalSurface;
  SCell bel = Surfaces<KSpace>::findABel( ks, set3dPredicate, 100000 );
  MyDigitalSurfaceContainer* ptrSurfContainer = 
    new MyDigitalSurfaceContainer( ks, set3dPredicate, surfAdj, bel );
  MyDigitalSurface digSurf( ptrSurfContainer ); // acquired
 
  MyDigitalSurface::ConstIterator it = digSurf.begin();

  //Fake embedder
  typedef int MyEmbedder;

  typedef LocalConvolutionNormalVectorEstimator<MyDigitalSurface,MyFunctor, MyEmbedder> MyEstimator;
 

  MyFunctor f; 
  MyEstimator myNormalEstimator(digSurf, f, MyEmbedder());

  myNormalEstimator.init(1.0, 5);
  
  MyEstimator::Quantity res = myNormalEstimator.eval(it);
  
  trace.info() << "Normal vector at begin() : "<< res << std::endl;

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
  trace.beginBlock ( "Testing class LocalConvolutionNormalVectorEstimator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testLocalConvolutionNormalVectorEstimator(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
