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
 * @file testDigitalSurfaceBoostGraphInterface.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2013/01/20
 *
 * Functions for testing class DigitalSurfaceBoostGraphInterface.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/queue.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/graph/DigitalSurfaceBoostGraphInterface.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/io/readers/MPolynomialReader.h"
/// JOL: Since using boost::vertices is enforced before we define it,
/// the compiler is unable to find our function boost::vertices. We
/// *must* include graph_concepts.hpp after defining our graph wrapper.
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/breadth_first_search.hpp>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalSurfaceBoostGraphInterface.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDigitalSurfaceBoostGraphInterface()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;


  using namespace Z3i;

  // Construct the implicit shape
  typedef Space::RealPoint RealPoint;
  typedef RealPoint::Coordinate Ring;
  typedef MPolynomial<3, Ring> Polynomial3;
  typedef MPolynomialReader<3, Ring> Polynomial3Reader;
  typedef ImplicitPolynomial3Shape<Space> ImplicitShape;
  typedef GaussDigitizer<Space,ImplicitShape> DigitalShape; 
  typedef DigitalShape::PointEmbedder DigitalEmbedder;

  // Implicit shape is an ellipse
  trace.beginBlock( "Constructing implicit shape." );
  double p1[] = {-2,-2,-2};
  double p2[] = { 2, 2, 2};
  std::string poly_str = "x*x+y*y+2*z*z-1";
  double step = 0.02;
  Polynomial3 P;
  Polynomial3Reader reader;
  std::string::const_iterator iter 
    = reader.read( P, poly_str.begin(), poly_str.end() );
  if ( iter != poly_str.end() )
    {
      std::cerr << "ERROR: I read only <" 
                << poly_str.substr( 0, iter - poly_str.begin() )
                << ">, and I built P=" << P << std::endl;
      return 1;
    }
  trace.info() << "- P = " << P << std::endl;
  ImplicitShape ishape( P );
  DigitalShape dshape;
  dshape.attach( ishape );
  dshape.init( RealPoint( p1 ), RealPoint( p2 ), step );
  Domain domain = dshape.getDomain();
  trace.endBlock();

  // Construct the Khalimsky space from the image domain
  trace.beginBlock ( "Construct the Khalimsky space from the image domain ..." );
  KSpace K;
  bool space_ok = K.init( domain.lowerBound(), 
                          domain.upperBound(), true // necessary
                          );
  if (!space_ok)
    {
      trace.error() << "Error in the Khamisky space construction."<<std::endl;
      return false;
    }
  trace.endBlock();

  //! [trackImplicitPolynomialSurfaceToOFF-SurfelAdjacency]
  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  typedef KSpace::Surfel Surfel;
  typedef KSpace::SurfelSet SurfelSet;
  typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
  typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;
  trace.beginBlock ( "Extract surface ..." );

  MySurfelAdjacency surfAdj( true ); // interior in all directions.
  MySetOfSurfels theSetOfSurfels( K, surfAdj );
  Surfel bel = Surfaces<KSpace>::findABel( K, dshape, 100000 );
  Surfaces<KSpace>::trackBoundary( theSetOfSurfels.surfelSet(),
                                   K, surfAdj, 
                                   dshape, bel );
  MyDigitalSurface digSurf( theSetOfSurfels );
  trace.info() << "Digital surface has " << digSurf.size() << " surfels."
               << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing Graph concepts for DigitalSurface ..." );
  typedef MyDigitalSurface Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie DigitalSurface::Vertex
  typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie DigitalSurface::Arc

  BOOST_CONCEPT_ASSERT(( boost::VertexListGraphConcept<Graph> ));
  BOOST_CONCEPT_ASSERT(( boost::AdjacencyGraphConcept<Graph> ));
  BOOST_CONCEPT_ASSERT(( boost::IncidenceGraphConcept<Graph> ));
  // BOOST_CONCEPT_ASSERT(( BidirectionalGraphConcept<Graph> ));
  trace.endBlock();

  trace.beginBlock ( "Testing IncidenceGraph interface with breadth_first_visit ..." );
  // get the property map for coloring vertices.
  typedef std::map< vertex_descriptor, boost::default_color_type > StdColorMap;
  StdColorMap colorMap;
  boost::associative_property_map< StdColorMap > propColorMap( colorMap );
  // get the property map for storing distances
  typedef std::map< vertex_descriptor, unsigned long > StdDistanceMap;
  StdDistanceMap distanceMap;
  boost::associative_property_map< StdDistanceMap > propDistanceMap( distanceMap );
  boost::queue< vertex_descriptor > Q; // std::queue does not have top().
  vertex_descriptor start = *( digSurf.begin() );
  boost::breadth_first_visit // boost graph breadth first visiting algorithm.
    ( digSurf, // the graph
      start, // the starting vertex
      Q, // the buffer for breadth first queueing
      boost::make_bfs_visitor( boost::record_distances( propDistanceMap, boost::on_tree_edge() ) ), // only record distances
      propColorMap  // necessary for the visiting vertices
      );
  trace.endBlock();

  // Display results
  trace.beginBlock ( "Display breadth_first_visit result ..." );
  typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;
  std::pair<vertex_iter, vertex_iter> vp;
  unsigned long maxD = 0;
  vertex_descriptor furthest = start;
  unsigned long nbV = 0;
  for ( vp = boost::vertices( digSurf ); vp.first != vp.second; ++vp.first, ++nbV )
    {
      unsigned long d = boost::get( propDistanceMap, *vp.first );
      if ( d > maxD ) 
        {
          maxD = d;
          furthest = *vp.first;
        }
    }
  trace.info() << "- d[ " << start << " ] = " << boost::get( propDistanceMap, start ) << std::endl;
  trace.info() << "- d[ " << furthest << " ] = " << maxD << std::endl;
  ++nb, nbok += ( nbV == digSurf.size() ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "nb vertices is ok" << std::endl;
  ++nb, nbok += ( maxD == 189 ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "maxD == 189" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int /* argc */, char** /* argv */ )
{
  trace.beginBlock ( "Testing class DigitalSurfaceBoostGraphInterface" );

  bool res = testDigitalSurfaceBoostGraphInterface(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
