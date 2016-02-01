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
 * @file testObjectBoostGraphInterface.cpp
 * @ingroup Tests
 * @author Pablo Hernandez-Cerdan. Institute of Fundamental Sciences. Massey University. Palmerston North, New Zealand
 *
 * @date 2016/02/01
 *
 * Functions for testing class ObjectBoostGraphInterface.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtalCatch.h"
#include <iostream>
#include <queue>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/queue.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/graph/ObjectBoostGraphInterface.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/SurfelAdjacency.h"
/// JOL: Since using boost::vertices is enforced before we define it,
/// the compiler is unable to find our function boost::vertices. We
/// *must* include graph_concepts.hpp after defining our graph wrapper.
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/stoer_wagner_min_cut.hpp>
#include <boost/graph/boykov_kolmogorov_max_flow.hpp>
#include <boost/graph/filtered_graph.hpp>
///////////////////////////////////////////////////////////////////////////
using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////
// Fixture for object from a diamond set and DT26_6 topology.
///////////////////////////////////////////////////////////////////////////
struct Fixture_object_diamond_with_hole {
  ///////////////////////////////////////////////////////////
  // type aliases
  ///////////////////////////////////////////////////////////
  using Point  = DGtal::Z3i::Point;
  using Domain = DGtal::Z3i::Domain;
  using KSpace = DGtal::Z3i::KSpace;

  using FixtureSurfelAdjacency =
    DGtal::SurfelAdjacency<DGtal::Z3i::KSpace::dimension> ;
  using FixtureDigitalTopology =
    DGtal::Z3i::DT26_6;
  using FixtureDigitalSet =
    DGtal::DigitalSetByAssociativeContainer<DGtal::Z3i::Domain , std::unordered_set< typename DGtal::Z3i::Domain::Point> >;
  using FixtureObject =
    DGtal::Object<FixtureDigitalTopology, FixtureDigitalSet>;

  ///////////////////////////////////////////////////////////
  // fixture data
  FixtureObject obj_fixture;
  ///////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////
  // Constructor
  ///////////////////////////////////////////////////////////
  Fixture_object_diamond_with_hole() {
    using namespace DGtal;

    // trace.beginBlock ( "Create Fixture_object_diamond" );
    Point p1( -10, -10, -10 );
    Point p2( 10, 10, 10 );
    Domain domain( p1, p2 );
    Point c( 0, 0, 0 );

    // diamond of radius 4
    FixtureDigitalSet diamond_set( domain );
    for ( auto it = domain.begin(); it != domain.end(); ++it )
    {
      if ( (*it - c ).norm1() <= 3 ) diamond_set.insertNew( *it );
    }
    diamond_set.erase( c );

    KSpace K;
    bool space_ok = K.init( domain.lowerBound(),
        domain.upperBound(), true // necessary
        );
    FixtureSurfelAdjacency surfAdj( true ); // interior in all directions.

    using FixtureDigitalSet = DigitalSetByAssociativeContainer<Z3i::Domain , std::unordered_set< typename Z3i::Domain::Point> >;
    FixtureDigitalTopology::ForegroundAdjacency adjF;
    FixtureDigitalTopology::BackgroundAdjacency adjB;
    FixtureDigitalTopology topo(adjF, adjB, DGtal::DigitalTopologyProperties::JORDAN_DT);
    obj_fixture = FixtureObject(topo,diamond_set);

    // trace.endBlock();
  }
 };

TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Basic Graph functions", "[interface]" ){
  GIVEN( "A diamond object with graph properties" ){
    typedef FixtureObject Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
    typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
    typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

    THEN( "Vertices, Num Vertices" ){
      auto num_verts = boost::num_vertices(obj_fixture);
      auto verts = boost::vertices(obj_fixture);
      size_t count_verts{0};
      for(auto v = verts.first; v != verts.second; ++v, ++count_verts){};
      REQUIRE(count_verts == num_verts);
    }
    THEN( "Edges, Num Edges" ){
      auto num_edges = boost::num_edges(obj_fixture);
      auto edges = boost::edges(obj_fixture);
      size_t count_edges{0};
      for(auto v = edges.first; v != edges.second; ++v, ++count_edges){};
      REQUIRE(count_edges == num_edges);
    }
    THEN( "Out Edges, Out Degree, Degree, and Adjacencies of a vertex" ){
      // Map relating point and known degree in the diamond.
      std::map<Point, size_t> point_numedges;
      Point north{ 0 , 0 , 3 };
      point_numedges[north] = 5;

      Point x3{ 3 , 0 , 0 };
      point_numedges[x3] = 5;

      Point x1y1z1{ 1 , 1 , 1 };
      point_numedges[x1y1z1] = 15;

      Point cz2{ 0 , 0 , 2 };
      point_numedges[cz2] = 14;

      Point cz1{ 0 , 0 , 1 };
      point_numedges[cz1] = 21;

      for(auto && pm : point_numedges){
        auto out_edges = boost::out_edges(pm.first, obj_fixture);
        size_t count_edges{0};
        for(auto v = out_edges.first; v != out_edges.second; ++v, ++count_edges){};
        REQUIRE(count_edges == pm.second);
        auto out_degree = boost::out_degree(pm.first, obj_fixture);
        REQUIRE(out_degree == count_edges);
        auto degree = obj_fixture.degree(pm.first);
        REQUIRE(degree == out_degree);
        auto adj_vp = boost::adjacent_vertices(pm.first, obj_fixture);
        size_t count_verts{0};
        for(auto v = adj_vp.first; v != adj_vp.second; ++v, ++count_verts){};
        REQUIRE(count_verts == degree);
      }
    }
  }
}

TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Boost Graph Concepts", "[concepts]" ){
  GIVEN( "A diamond object with graph properties" ){

    typedef FixtureObject Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
    typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
    typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

    THEN( "Check Graph Concepts" ){

      BOOST_CONCEPT_ASSERT(( boost::VertexListGraphConcept<Graph> ));
      BOOST_CONCEPT_ASSERT(( boost::AdjacencyGraphConcept<Graph> ));
      BOOST_CONCEPT_ASSERT(( boost::IncidenceGraphConcept<Graph> ));
      BOOST_CONCEPT_ASSERT(( boost::EdgeListGraphConcept<Graph> ));
      // BOOST_CONCEPT_ASSERT(( boost::BidirectionalGraphConcept<Graph> ));
    }
  }
}

TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Breadth first visit and search", "[breadth]" ){
  GIVEN( "A diamond object with graph properties" ){
    typedef FixtureObject Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
    typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
    typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

    // get the property map for coloring vertices.
    typedef std::map< vertex_descriptor, boost::default_color_type > StdColorMap;
    StdColorMap colorMap;
    boost::associative_property_map< StdColorMap > propColorMap( colorMap );
    // get the property map for storing distances
    typedef std::map< vertex_descriptor, uint64_t > StdDistanceMap;
    StdDistanceMap distanceMap;
    boost::associative_property_map< StdDistanceMap > propDistanceMap( distanceMap );
    boost::queue< vertex_descriptor > Q; // std::queue does not have top().
    vertex_descriptor start = *( obj_fixture.begin() );

    THEN( "Test IncidenceGraph interface with breadth_first_visit" ){

      boost::breadth_first_visit // boost graph breadth first visiting algorithm.
        ( obj_fixture, // the graph
          start, // the starting vertex
          Q, // the buffer for breadth first queueing
          boost::make_bfs_visitor( boost::record_distances( propDistanceMap, boost::on_tree_edge() ) ), // only record distances
          propColorMap  // necessary for the visiting vertices
        );

      uint64_t maxD = 0;
      vertex_descriptor furthest = start;
      uint64_t nbV = 0;
      for ( std::pair<vertex_iterator, vertex_iterator>
          vp = boost::vertices( obj_fixture ); vp.first != vp.second; ++vp.first, ++nbV )
      {
        uint64_t d = boost::get( propDistanceMap, *vp.first );
        if ( d > maxD )
        {
          maxD = d;
          furthest = *vp.first;
        }
      }

      REQUIRE( nbV == obj_fixture.size() );
      INFO("- d[ " << start << " ] = " << boost::get( propDistanceMap, start));
      INFO("- d[ " << furthest << " ] = " << maxD);
      CHECK( maxD == 6 );
    }

    THEN( "Test IncidenceGraph interface with breadth_first_search" ){
      typedef std::map< vertex_descriptor, vertex_descriptor > PredecessorMap;
      PredecessorMap predecessorMap;
      boost::associative_property_map< PredecessorMap >
        propPredecessorMap( predecessorMap );

      boost::breadth_first_search(
         obj_fixture,
         start,
         Q,
         boost::make_bfs_visitor( boost::record_predecessors( propPredecessorMap, boost::on_tree_edge() ) ), // only record predecessors
         propColorMap  // necessary for the visiting vertices
         );

      INFO("predecessorMap");
      INFO("starting point: " << *( obj_fixture.begin() ) );
      size_t visited{0};
      for(auto && v : predecessorMap){
        ++visited;
        INFO(v.first << " : " << v.second) ;
      }
      // +1 to count the starting point;
      CHECK((visited + 1) == obj_fixture.size());
    }
  }
}
TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Connected Components", "[connected]" ){
  GIVEN( "A diamond object with graph properties and an isolated vertex" ){
    typedef FixtureObject Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
    typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
    typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

    // Add an isolated point in the domain.
    obj_fixture.pointSet().insertNew(FixtureObject::Point(0,0,7));

    THEN( "VertexListGraph interface with connected_components" ){
      // get the property map for labelling vertices.
      // get the property map for coloring vertices.
      typedef std::map< vertex_descriptor, boost::default_color_type > StdColorMap;
      StdColorMap colorMap;
      boost::associative_property_map< StdColorMap > propColorMap( colorMap );

      typedef std::map< vertex_descriptor, vertices_size_type > StdComponentMap;
      StdComponentMap componentMap;
      boost::associative_property_map< StdComponentMap > propComponentMap( componentMap );
      vertices_size_type nbComp =
        boost::connected_components // boost graph connected components algorithm.
        ( obj_fixture, // the graph
          propComponentMap, // the mapping vertex -> label
          boost::color_map( propColorMap ) // this map is used internally when computing connected components.
        );
      CHECK(nbComp == 2);

      THEN( "Filter graph and get components" ){
        using ComponentGraph =
          boost::filtered_graph<
          Graph,
          function<bool(edge_descriptor)>,
          function<bool(vertex_descriptor)>
            >;
        auto &g = obj_fixture;

        std::vector<ComponentGraph> component_graphs;

        for (size_t i = 0; i < nbComp; i++)
          component_graphs.emplace_back(g,
              [componentMap,i,&g](edge_descriptor e) {
              return componentMap.at(boost::source(e,g) )==i
              || componentMap.at(boost::target(e,g))==i;
              },
              [componentMap,i](vertex_descriptor v) {
              return componentMap.at(v)==i;
              });


        CHECK( component_graphs.size() == 2);
        std::vector<FixtureObject> obj_components;
        // Copying object and clear pointSet instead of creating from scratch. Lazy?
        FixtureObject obj_copy(obj_fixture);
        obj_copy.pointSet().clear();

        // Create new object from the component_graph.
      for (auto && c : component_graphs){
        obj_components.push_back(obj_copy);
        for (auto && vp = boost::vertices(c); vp.first != vp.second ; ++vp.first){
          obj_components.back().pointSet().insertNew(*vp.first);
        }
      }

      // THEN( "[visualize] visualize the components" ){
      // //#include "DGtal/io/viewers/Viewer3D.h"
      //   int argc(1);
      //   char** argv(nullptr);
      //   QApplication app(argc, argv);
      //   Viewer3D<> viewer;
      //   viewer.show();
      //
      //   viewer.setFillColor(Color(255, 255, 255, 255));
      //   viewer << obj_components[0].pointSet();
      //   viewer.setFillColor(Color(20, 30, 30, 255));
      //   viewer << obj_components[1].pointSet();
      //   viewer.setFillColor(Color(10, 10, 10, 20));
      //   viewer << obj_fixture.pointSet();
      //   viewer << Viewer3D<>::updateDisplay;
      //   app.exec();
      // }

      // TODO use copy_graph directly to an Object.
      // using StdVertexIndexMap = std::map< vertex_descriptor, vertices_size_type > ;
      // StdVertexIndexMap vertexIndexMap;
      // boost::associative_property_map< StdVertexIndexMap > propVertexIndexMap( vertexIndexMap );
      // boost::copy_graph( obj_fixture, obj_copy,
      //     boost::vertex_copy( my_vertex_copier<Graph,Graph,StdVertexIndexMap>( obj_fixture, obj_copy, vertexIndexMap ) )
      //     .edge_copy( my_edge_copier<Graph,Graph>( obj_fixture, obj_copy ) )
      //     .vertex_index_map( propVertexIndexMap )
      //     );

      }// filtered_graph
    }// connected_components
  }//given
}//scenario

////////////////////////////////////////////////////////
// copiers between Object and boost::adjacency_list
////////////////////////////////////////////////////////
struct vertex_position_t {
  typedef boost::vertex_property_tag kind;
};

struct vertex_position {
  Z3i::Point myP;
  vertex_position() : myP()
  {
  }
};

typedef boost::property< boost::vertex_index_t, std::size_t,
        boost::property<vertex_position_t, vertex_position> > VertexProperties;


template <typename Graph1, typename Graph2, typename VertexIndexMap>
struct my_vertex_copier {
  typedef typename boost::property_map< Graph2, boost::vertex_index_t>::type graph_vertex_index_map;
  typedef typename boost::property_map< Graph2, vertex_position_t>::type graph_vertex_position_map;
  typedef typename boost::graph_traits< Graph1 >::vertex_descriptor Vertex1;
  typedef typename boost::graph_traits< Graph2 >::vertex_descriptor Vertex2;

  const Graph1 & myG1;
  graph_vertex_index_map graph_vertex_index;
  graph_vertex_position_map graph_vertex_position;
  VertexIndexMap & myIndexMap;

  my_vertex_copier(const Graph1& g1, Graph2& g2, VertexIndexMap & indexMap )
    : myG1( g1 ),
    graph_vertex_index( boost::get( boost::vertex_index_t(), g2 ) ),
    graph_vertex_position( boost::get( vertex_position_t(), g2) ),
    myIndexMap( indexMap )
  {}

  void operator()( const Vertex1& v1, const Vertex2& v2 ) const {
    //  std::size_t idx = myIndexMap[ v1 ];
    // Does not work !
    // put( graph_vertex_index, v2, idx);
    vertex_position pos;
    pos.myP = v1;
    //std::cout << "vertex " << idx << " at " << pos.myP << std::endl;
    put( graph_vertex_position, v2, pos);
  }
};
template <typename Graph1, typename Graph2>
struct my_edge_copier {
  my_edge_copier(const Graph1& UNUSED1, Graph2& UNUSED2)
  {}
  template <typename Edge1, typename Edge2>
    void operator()(const Edge1& /*v1*/, Edge2& /*v2*/) const {
      // does nothing
    }
};

TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Copy graph", "[copy]" ){
  GIVEN( "A diamond object with graph properties" ){
    typedef FixtureObject Graph;
    typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
    typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
    typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
    typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;

    THEN( "Test copy_graph"){

      // trace.beginBlock ( "Testing AdjacencyListGraph with copy_graph ..." );
      using StdVertexIndexMap = std::map< vertex_descriptor, vertices_size_type > ;
      StdVertexIndexMap vertexIndexMap;
      boost::associative_property_map< StdVertexIndexMap > propVertexIndexMap( vertexIndexMap );
      using BGraph = boost::adjacency_list< boost::vecS, boost::vecS, boost::undirectedS, VertexProperties > ;
      BGraph bG;
      boost::copy_graph( obj_fixture, bG,
          boost::vertex_copy( my_vertex_copier<Graph,BGraph,StdVertexIndexMap>( obj_fixture, bG, vertexIndexMap ) )
          .edge_copy( my_edge_copier<Graph,BGraph>( obj_fixture, bG ) )
          .vertex_index_map( propVertexIndexMap )
          );
      using vertex_descriptor_2 = boost::graph_traits< BGraph >::vertex_descriptor ;
      using vertex_iterator_2 = boost::graph_traits< BGraph >::vertex_iterator ;
      using GraphVertexPositionMap = boost::property_map< BGraph, vertex_position_t>::type ;
      GraphVertexPositionMap vertexPos( boost::get( vertex_position_t(), bG) );
      for ( std::pair<vertex_iterator_2, vertex_iterator_2>
          vp = boost::vertices( bG ); vp.first != vp.second; ++vp.first )
      {
        vertex_descriptor_2 v1 = *vp.first;
        vertex_position pos = boost::get( vertexPos, v1 );
        // trace.info() << "- " << v1 << " was at " << pos.myP << std::endl;
      }
      INFO("after copy: Boost graph has " << num_vertices( bG ) << " vertices.");
      CHECK(boost::num_vertices( bG ) == boost::num_vertices( obj_fixture ));
      // trace.endBlock();

    }
  }
}


// TEST_CASE_METHOD(Fixture_object_diamond_with_hole, "Wagner-Stoe min-cut and Boykov-Kolmogorov max-flow", "[mincut][maxflow]" ){
//   GIVEN( "A diamond object with graph properties, and a breadth_first_visit" ){
//     typedef FixtureObject Graph;
//     typedef boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor; // ie Object::Vertex
//     typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor; // ie Object::Arc
//     typedef boost::graph_traits<Graph>::vertices_size_type vertices_size_type; // ie Object::Size
//     typedef boost::graph_traits<Graph>::vertex_iterator vertex_iterator;
//     typedef boost::graph_traits<Graph>::out_edge_iterator out_edge_iterator;
//     typedef boost::graph_traits<Graph>::edge_iterator edge_iterator;
//
//     ///////////////////////////////////////
//     // Breadth first
//
//     // get the property map for coloring vertices.
//     using StdColorMap = std::map< vertex_descriptor, boost::default_color_type > ;
//     StdColorMap colorMap;
//     boost::associative_property_map< StdColorMap > propColorMap( colorMap );
//     // get the property map for storing distances
//     using StdDistanceMap = std::map< vertex_descriptor, uint64_t > ;
//     StdDistanceMap distanceMap;
//     boost::associative_property_map< StdDistanceMap > propDistanceMap( distanceMap );
//     boost::queue< vertex_descriptor > Q; // std::queue does not have top().
//     vertex_descriptor start = *( obj_fixture.begin() );
//
//     boost::breadth_first_visit // boost graph breadth first visiting algorithm.
//       ( obj_fixture, // the graph
//         start, // the starting vertex
//         Q, // the buffer for breadth first queueing
//         boost::make_bfs_visitor( boost::record_distances( propDistanceMap, boost::on_tree_edge() ) ), // only record distances
//         propColorMap  // necessary for the visiting vertices
//       );
//
//     uint64_t maxD = 0;
//     vertex_descriptor furthest = start;
//     uint64_t nbV = 0;
//     for ( std::pair<vertex_iterator, vertex_iterator>
//         vp = boost::vertices( obj_fixture ); vp.first != vp.second; ++vp.first, ++nbV )
//     {
//       uint64_t d = boost::get( propDistanceMap, *vp.first );
//       if ( d > maxD )
//       {
//         maxD = d;
//         furthest = *vp.first;
//       }
//     }
//     THEN( "Test Wagner Stoer min-cut"){
//
//       trace.beginBlock ( "Testing UndirectedGraph interface with Wagner-Stoer min cut ..." );
//       // get the property map for weighting edges.
//       using weight_type = double ;
//       using StdWeightMap = std::map< edge_descriptor, weight_type > ;
//       StdWeightMap weightMap;
//       boost::associative_property_map< StdWeightMap > propWeightMap( weightMap );
//       using StdVertexIndexMap = std::map< vertex_descriptor, vertices_size_type > ;
//       StdVertexIndexMap vertexIndexMap;
//       boost::associative_property_map< StdVertexIndexMap > propVertexIndexMap( vertexIndexMap );
//       vertices_size_type idxV = 0;
//       // The weight is smaller for edges traversing plane z=0 than anywhere else.
//       // The min cut thus cuts the sphere in two approximate halves.
//       for ( std::pair<vertex_iterator, vertex_iterator>
//           vp = boost::vertices( obj_fixture ); vp.first != vp.second; ++vp.first, ++idxV )
//       {
//         vertex_descriptor v1 = *vp.first;
//         vertexIndexMap[ v1 ] = idxV;
//         for ( std::pair<out_edge_iterator, out_edge_iterator>
//             ve = boost::out_edges( v1, obj_fixture ); ve.first != ve.second; ++ve.first )
//         {
//           vertex_descriptor v2 = boost::target( *ve.first, obj_fixture );
//           if ( v1 < v2 )
//           {
//             // KSpace::SCell sep = obj_fixture.separator( *ve.first );
//             // weight_type weight = ( K.sKCoord( sep, 2 ) == 0 ) ? 0.01 : 1.0;
//             weight_type weight = 0.5;
//             weightMap[ *ve.first ] = weight;
//             // weightMap[ obj_fixture.opposite( *ve.first ) ] = weight;
//           }
//         }
//       }
//       // get the parity map for assigning a set to each vertex.
//       using StdParityMap = std::map< vertex_descriptor, bool > ;
//       StdParityMap parityMap;
//       boost::associative_property_map< StdParityMap > propParityMap( parityMap );
//
//       weight_type total_weight =
//         boost::stoer_wagner_min_cut // boost wagner stoer min cut algorithm.
//         ( obj_fixture, // the graph
//           propWeightMap, // the mapping edge -> weight
//           boost::parity_map( propParityMap ) // this map stores the vertex assignation
//           .vertex_index_map( propVertexIndexMap )
//         );
//       trace.info() << "- total weight = " << total_weight << std::endl;
//       uint64_t nb0 = 0;
//       uint64_t nb1 = 0;
//       for ( std::pair<vertex_iterator, vertex_iterator>
//           vp = boost::vertices( obj_fixture ); vp.first != vp.second; ++vp.first, ++idxV )
//       {
//         vertex_descriptor v1 = *vp.first;
//         INFO("- " << v1 << " in " << parityMap[ v1 ] );
//         if ( parityMap[ v1 ] ) ++nb1;
//         else ++nb0;
//       }
//       INFO("parityMap: True components: " << nb1 << " False: " << nb0);
//       INFO("- parity[ " << start << " ] = " << parityMap[ start ]);
//       INFO("- parity[ " << furthest << " ] = " << parityMap[ furthest ]);
//       CHECK( parityMap[start] != parityMap[furthest]);
//       CHECK(total_weight < 1.0);
//
//       trace.endBlock();
//
//       THEN( "Test Boykov-Kolmogorov max flow"){
//         using weight_type = double ;
//         using StdWeightMap = std::map< edge_descriptor, weight_type > ;
//         StdWeightMap weightMap;
//         boost::associative_property_map< StdWeightMap > propWeightMap( weightMap );
//         using StdVertexIndexMap = std::map< vertex_descriptor, vertices_size_type > ;
//         StdVertexIndexMap vertexIndexMap;
//
//         trace.beginBlock ( "Testing EdgeListGraph and IncidenceGraph interfaces with Boykov-Kolmogorov max flow ..." );
//         typedef double capacity_type;
//         // get the property map for edge capacity.
//         typedef std::map< edge_descriptor, weight_type > StdEdgeCapacityMap;
//         StdEdgeCapacityMap edgeCapacityMap;
//         boost::associative_property_map< StdEdgeCapacityMap > propEdgeCapacityMap( edgeCapacityMap );
//         // get the property map for edge residual capacity.
//         typedef std::map< edge_descriptor, weight_type > StdEdgeResidualCapacityMap;
//         StdEdgeResidualCapacityMap edgeResidualCapacityMap;
//         boost::associative_property_map< StdEdgeResidualCapacityMap > propEdgeResidualCapacityMap( edgeResidualCapacityMap );
//         // get the property map for reverse edge.
//         typedef std::map< edge_descriptor, edge_descriptor > StdReversedEdgeMap;
//         StdReversedEdgeMap reversedEdgeMap;
//         boost::associative_property_map< StdReversedEdgeMap > propReversedEdgeMap( reversedEdgeMap );
//         // get the property map for vertex predecessor.
//         typedef std::map< vertex_descriptor, edge_descriptor > StdPredecessorMap;
//         StdPredecessorMap predecessorMap;
//         boost::associative_property_map< StdPredecessorMap > propPredecessorMap( predecessorMap );
//         // We already have vertex color map, vertex distance map and vertex index map.
//         uint64_t nbEdges = 0;
//         // The weight is smaller for edges traversing plane z=0 than anywhere else.
//         // The min cut thus cuts the sphere in two approximate halves.
//         for ( std::pair<edge_iterator, edge_iterator>
//             ve = boost::edges( obj_fixture ); ve.first != ve.second; ++ve.first, ++nbEdges )
//         {
//           edge_descriptor e = *ve.first;
//           edge_descriptor rev_e = obj_fixture.opposite( e );
//           vertex_descriptor v1 = boost::source( e, obj_fixture );
//           vertex_descriptor v2 = boost::target( e, obj_fixture );
//           ASSERT( boost::source( rev_e, obj_fixture ) == v2 );
//           ASSERT( boost::target( rev_e, obj_fixture ) == v1 );
//           if ( v1 < v2 )
//           {
//             // KSpace::SCell sep = obj_fixture.separator( *ve.first );
//             // capacity_type capacity = ( K.sKCoord( sep, 2 ) == 0 ) ? 0.01 : 1.0;
//             capacity_type capacity = 0.5;
//             edgeCapacityMap[ e ] = capacity;
//             edgeCapacityMap[ obj_fixture.opposite( e ) ] = capacity;
//             reversedEdgeMap[ e ] = obj_fixture.opposite( e );
//             reversedEdgeMap[ obj_fixture.opposite( e ) ] = e;
//           }
//         }
//         trace.info() << "- nb edges = " << nbEdges << std::endl;
//         distanceMap.clear();
//         colorMap.clear();
//         capacity_type max_flow =
//           boost::boykov_kolmogorov_max_flow // boykov kolmogorov max flow algorithm.
//           ( obj_fixture, // the graph
//             propEdgeCapacityMap, propEdgeResidualCapacityMap,
//             propReversedEdgeMap, propPredecessorMap, propColorMap, propDistanceMap, propVertexIndexMap,
//             start, furthest );
//         INFO( "- max flow = " << max_flow)
//           CHECK(abs(max_flow) == Approx(total_weight));
//         trace.endBlock();
//       }// maxflow
//     }// mincut
//   }
// }



///////////////////////////////////////////////////////////////////////////////
