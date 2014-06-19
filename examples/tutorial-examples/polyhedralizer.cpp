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
 * @file polyedrisation.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/06/16
 *
 * An example file named polyedrisation.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <queue>
#include <QtGui/qapplication.h>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

//! [polyhedralisation-includes-readvol]
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/imagesSetsUtils/SimpleThresholdForegroundPredicate.h"
//! [polyhedralisation-includes-readvol]

#include "DGtal/io/Display3D.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"

#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/ImplicitDigitalSurface.h"

#include "DGtal/graph/BreadthFirstVisitor.h"
#include "DGtal/geometry/surfaces/COBANaivePlaneComputer.h"
#include "DGtal/geometry/surfaces/ChordNaivePlaneComputer.h"
#include "DGtal/geometry/surfaces/ChordGenericNaivePlaneComputer.h"

#include "DGtal/kernel/SimpleMatrix.h"
#include "DGtal/math/EigenDecomposition.h"

///////////////////////////////////////////////////////////////////////////////


//! [polyhedralisation-typedefs]
using namespace std;
using namespace DGtal;
using namespace Z3i;
//! [polyhedralisation-typedefs]

template <typename T1, typename T2>
struct PairSorted2nd
{
  typedef PairSorted2nd<T1,T2> Self;
  inline PairSorted2nd( const T1& t1, const T2& t2 ) : first( t1 ), second( t2 ) {}
  bool operator<( const Self& other ) const 
  {
    return second < other.second;
  }
  T1 first;
  T2 second;
};

template <typename T1, typename T2, typename T3>
struct Triple
{
  T1 first;
  T2 second;
  T3 third;
  Triple( T1 t1 = T1(), T2 t2 = T2(), T3 t3 = T3() )
    : first( t1 ), second( t2 ), third( t3 )
  {}
};

template <typename RealVector,
          typename ConstIterator>
double LSF( RealVector& N, ConstIterator itB, ConstIterator itE )
{
  typedef typename RealVector::Component Component;
  typedef SimpleMatrix<Component,3,3> Matrix;
  Matrix A; A.clear();
  unsigned int nb = 0;
  RealVector G = RealVector::zero; // centre de gravite.
  for ( ConstIterator it = itB; it != itE; ++it )
    {
      G += RealVector( (*it)[ 0 ], (*it)[ 1 ], (*it)[ 2 ] );
      ++nb;
    }
  G /= nb;
  for ( ConstIterator it = itB; it != itE; ++it )
    {
      RealVector p( (*it)[ 0 ], (*it)[ 1 ], (*it)[ 2 ] );
      p -= G;
      for ( Dimension i = 0; i < 3; ++i )
        for ( Dimension j = 0; j < 3; ++j )
          A.setComponent( i, j, A( i, j ) + p[ i ] * p[ j ] );
    }
  // A matrice de Gram
  // On cherche V tq V^t A V / |V|^2 est minimum. C'est la première valeur propre.
  Matrix V;
  RealVector values;
  EigenDecomposition<3,Component>::getEigenDecomposition( A, V, values );
  N = V.column( 0 ); // first eigenvector;
  // N /= N.norm();
  double mu = 0.0;
  for ( ConstIterator it = itB; it != itE; ++it )
    mu += N.dot( *it );
  trace.info() << N << " " << (mu/(double)nb) << " " << nb << std::endl;
  return mu/(double)nb;
}


int main( int argc, char** argv )
{
  QApplication application(argc,argv);
  string inputFilename = argc > 1 ? argv[ 1 ] : "/export/lachaud/Images/3d/vol/OctaFlower/octa-flower-129.vol";
  //string inputFilename = "/export/lachaud/GITHUB/DGtal/examples/samples/cat10.vol";
  int threshold = argc > 2 ? atoi( argv[ 2 ] ) : 1;
  int widthNum = argc > 3 ? atoi( argv[ 3 ] ) : 1;
  int widthDen = argc > 4 ? atoi( argv[ 4 ] ) : 1;

  trace.beginBlock( "Reading vol file into an image." );
  //! [polyhedralisation-readVol]
  //typedef ImageSelector < Domain, int>::Type Image;
  typedef ImageContainerBySTLVector< Domain, int> Image;
  Image image = VolReader<Image>::importVol(inputFilename);
  //! [polyhedralisation-readVol]

  typedef SimpleThresholdForegroundPredicate<Image> DigitalObject;
  DigitalObject digitalObject( image, threshold );
  //! [polyhedralisation-readVol]
  trace.endBlock();

  //! [polyhedralisation-KSpace]
  trace.beginBlock( "Construct the Khalimsky space from the image domain." );
  KSpace ks;
  bool space_ok = ks.init( image.domain().lowerBound(), image.domain().upperBound(), true );
  if (!space_ok)
    {
      trace.error() << "Error in the Khamisky space construction."<<std::endl;
      return 2;
    }
  trace.endBlock();
  //! [polyhedralisation-KSpace]

  //! [polyhedralisation-SurfelAdjacency]
  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  MySurfelAdjacency surfAdj( false ); // exterior in all directions.
  //! [polyhedralisation-SurfelAdjacency]

  //! [polyhedralisation-ExtractingSurface]
  trace.beginBlock( "Extracting boundary by tracking the surface. " );
  typedef KSpace::SCell SCell;
  typedef KSpace::Surfel Surfel;
  Surfel start_surfel = Surfaces<KSpace>::findABel( ks, digitalObject, 100000 );
  typedef ImplicitDigitalSurface< KSpace, DigitalObject > MyContainer;
  typedef DigitalSurface< MyContainer > MyDigitalSurface;
  MyContainer container( ks, digitalObject, surfAdj, start_surfel );
  MyDigitalSurface digSurf( container );
  trace.info() << "Digital surface has " << digSurf.size() << " surfels."
               << std::endl;
  trace.endBlock();
  //! [polyhedralisation-ExtractingSurface]

  //! [polyhedralisation-ComputingPlaneSize]
  // First pass to find biggest planes.
  trace.beginBlock( "1) Segmentation first pass. Computes all planes so as to sort vertices by the plane size." );
  typedef DGtal::int64_t InternalInteger;
  //typedef ChordNaivePlaneComputer<Z3, Z3::Point, InternalInteger> NaivePlaneComputer;
  typedef ChordGenericNaivePlaneComputer<Z3,Z3::Point, InternalInteger> NaivePlaneComputer;
  typedef MyDigitalSurface::ConstIterator ConstIterator;
  // Initialisation
  std::map<Surfel,unsigned int> v2size;
  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    v2size[ *it ] = 0;

  // Compute planes
  typedef BreadthFirstVisitor<MyDigitalSurface> Visitor;
  int j = 0;
  int nb = digSurf.size();
  NaivePlaneComputer planeComputer;
  std::vector<Point> layer;
  std::vector<Surfel> layer_surfel;
  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    {
      if ( ( (++j) % 50 == 0 ) || ( j == nb ) ) trace.progressBar( j, nb );
      Surfel v = *it;
      int axis = ks.sOrthDir( v );
      planeComputer.init( widthNum, widthDen );
      // The visitor takes care of all the breadth-first traversal.
      Visitor visitor( digSurf, v );
      layer.clear();
      layer_surfel.clear();
      Visitor::Size currentSize = visitor.current().second;
      int n = 0;
      while ( ! visitor.finished() )
        {
          Visitor::Node node = visitor.current();
          v = node.first;
          int axis = ks.sOrthDir( v );
          Point p = ks.sCoords( ks.sDirectIncident( v, axis ) );
          if ( node.second != currentSize )
            {
              bool isExtended = planeComputer.extend( layer.begin(), layer.end() );
              if ( isExtended )
                {
                  for ( std::vector<Surfel>::const_iterator it_layer = layer_surfel.begin(), 
                          it_layer_end = layer_surfel.end(); it_layer != it_layer_end; ++it_layer )
                    {
                      ++v2size[ *it_layer ];
                      ++n;
                    }
                  layer_surfel.clear();
                  layer.clear();
                  currentSize = node.second;
                }
              else
                break;
            }
          layer_surfel.push_back( v );
          layer.push_back( p );
          visitor.expand();
        }
      //trace.info() << "Vertex " << v << " has size " << n << std::endl;
    }
  trace.endBlock();


  // Prepare queue
  typedef PairSorted2nd<Surfel,int> SurfelWeight;
  std::priority_queue<SurfelWeight> Q;
  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    Q.push( SurfelWeight( *it, v2size[ *it ] ) );
  //! [polyhedralisation-ComputingPlaneSize]

  //! [polyhedralisation-segment]
  // Segmentation into planes
  trace.beginBlock( "2) Segmentation second pass. Visits vertices from the one with biggest plane to the one with smallest plane." );
  typedef Triple<NaivePlaneComputer, Color, RealVector> SegmentedPlane;
  std::set<Surfel> processedVertices;
  std::vector<SegmentedPlane*> segmentedPlanes;
  std::map<Surfel,SegmentedPlane*> v2plane;
  j = 0;
  while ( ! Q.empty() )
    {
      if ( ( (++j) % 50 == 0 ) || ( j == nb ) ) trace.progressBar( j, nb );
      Surfel v = Q.top().first;
      Q.pop();
      if ( processedVertices.find( v ) != processedVertices.end() ) // already in set
        continue; // process to next vertex

      SegmentedPlane* ptrSegment = new SegmentedPlane;
      segmentedPlanes.push_back( ptrSegment ); // to delete them afterwards.
      v2plane[ v ] = ptrSegment;
      ptrSegment->first.init( widthNum, widthDen );
      ptrSegment->second = Color( 255, 0, 0, 255 );
      ptrSegment->third = RealVector::zero;
      // The visitor takes care of all the breadth-first traversal.
      Visitor visitor( digSurf, v );
      layer.clear();
      layer_surfel.clear();
      Visitor::Size currentSize = visitor.current().second;
      while ( ! visitor.finished() )
        {
          Visitor::Node node = visitor.current();
          v = node.first;
          Dimension axis = ks.sOrthDir( v );
          Point p = ks.sCoords( ks.sDirectIncident( v, axis ) );
          if ( node.second != currentSize )
            {
              bool isExtended = ptrSegment->first.extend( layer.begin(), layer.end() );
              if ( isExtended )
                {
                  for ( std::vector<Surfel>::const_iterator it_layer = layer_surfel.begin(), 
                          it_layer_end = layer_surfel.end(); it_layer != it_layer_end; ++it_layer )
                    {
                      Surfel s = *it_layer;
                      Dimension k = ks.sOrthDir( s );
                      Vector tn = ks.sCoords( ks.sIndirectIncident( s, k ) ) - ks.sCoords( ks.sDirectIncident( s, k ) );
                      ptrSegment->third += RealVector( tn[ 0 ], tn[ 1 ], tn[ 2 ] );
                      processedVertices.insert( *it_layer );
                      if ( v2plane.find( *it_layer ) == v2plane.end() )
                        v2plane[ *it_layer ] = ptrSegment;
                    }
                  layer.clear();
                  layer_surfel.clear();
                  currentSize = node.second;
                }
              else break;
            }
          layer_surfel.push_back( v );
          layer.push_back( p );
          if ( processedVertices.find( v ) != processedVertices.end() )
            // surfel is already in some plane.
            visitor.ignore();
          else
            visitor.expand();
        }
      if ( visitor.finished() ) 
        {
          trace.warning() << "Visitor finished." << std::endl;
          for ( std::vector<Surfel>::const_iterator it_layer = layer_surfel.begin(), 
                  it_layer_end = layer_surfel.end(); it_layer != it_layer_end; ++it_layer )
            {
              Surfel s = *it_layer;
              Dimension k = ks.sOrthDir( s );
              Vector tn = ks.sCoords( ks.sIndirectIncident( s, k ) ) - ks.sCoords( ks.sDirectIncident( s, k ) );
              ptrSegment->third += RealVector( tn[ 0 ], tn[ 1 ], tn[ 2 ] );
              processedVertices.insert( *it_layer );
              if ( v2plane.find( *it_layer ) == v2plane.end() )
                v2plane[ *it_layer ] = ptrSegment;
            }
        }
      // Assign random color for each plane.
      ptrSegment->second = Color( random() % 192 + 64, random() % 192 + 64, random() % 192 + 64, 255 );
    }
  trace.endBlock();
  //! [polyhedralisation-segment]

  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    {
      Surfel s = *it;
      if ( v2plane.find( s ) == v2plane.end() )
        trace.warning() << "Surfel " << s << " not found in v2plane." << std::endl;
    }
  //! [polyhedralisation-visualization]
  typedef Viewer3D<Space,KSpace> MyViewer3D;
  typedef Display3DFactory<Space,KSpace> MyDisplay3DFactory;
  MyViewer3D viewer( ks );
  viewer.show(); 
  viewer << SetMode3D( start_surfel.className(), "Basic" );
  for ( std::map<Surfel,SegmentedPlane*>::const_iterator 
          it = v2plane.begin(), itE = v2plane.end();
        it != itE; ++it )
    {
      viewer << CustomColors3D( it->second->second, it->second->second );
      RealVector normal;
      it->second->first.getUnitNormal( normal );
      if ( it->second->third.dot( normal ) < 0.0 ) normal = -normal;
      MyDisplay3DFactory::drawOrientedSurfelWithNormal( viewer, it->first, normal, false );
      viewer << it->first;
    }
  viewer << MyViewer3D::updateDisplay;
  application.exec();
  //! [polyhedralisation-visualization]

  //! [polyhedralisation-MakeMesh]
  typedef unsigned int Number;
  typedef Mesh<RealPoint> MyMesh;
  typedef MyMesh::MeshFace MeshFace;
  typedef MyDigitalSurface::FaceSet FaceSet;
  typedef MyDigitalSurface::VertexRange VertexRange;
  // Numbers all vertices.
  std::map<Surfel, Number> index;
  Number nbv = 0;
  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    index[ *it ] = nbv++;
  MyMesh polyhedron( true );
  // Project all vertices onto their plane
  typedef NaivePlaneComputer::Primitive Primitive;
  double maxl = (double)widthNum/(double)widthDen;
  for ( ConstIterator it = digSurf.begin(), itE= digSurf.end(); it != itE; ++it )
    {
      //Point p = ks.sCoords( ks.sDirectIncident( *it, ks.sOrthDir( *it ) ) );
      Point p = ks.sKCoords( *it );
      RealPoint rp( (double)p[ 0 ]/2.0, (double)p[ 1 ]/2.0, (double)p[ 2 ]/2.0 );
      // Check neighbors
      VertexRange neighbors;
      std::back_insert_iterator<VertexRange> writeIt = std::back_inserter( neighbors );
      digSurf.writeNeighbors( writeIt, *it );
      std::set<SegmentedPlane*> segPlanes;
      segPlanes.insert( v2plane[ *it ] );
      for ( VertexRange::const_iterator itn = neighbors.begin(), itnE = neighbors.end(); itn != itnE; ++itn )
        segPlanes.insert( v2plane[ *itn ] );
      RealPoint rq;
      if (true) //( segPlanes.size() == 1 )
        {
          NaivePlaneComputer& computer = v2plane[ *it ]->first;
          RealVector normal;
          double mu = LSF( normal, computer.begin(), computer.end() );
          double lambda = mu - rp.dot( normal );
          rq = rp + lambda*normal;
          // Primitive PS = v2plane[ *it ]->first.primitive();
          // RealVector normal = PS.normal();
          // double lambda = PS.mu() + PS.nu()/2.0 - rp.dot( normal );
          // rq = rp + lambda*normal;
        }
      else
        {
          typedef SimpleMatrix<double,3,3> Matrix;
          Matrix A; A.identity();
          RealVector B = rp;
          for ( std::set<SegmentedPlane*>::const_iterator itp = segPlanes.begin(), itpE = segPlanes.end();
                itp != itpE; ++itp )
            {
              Primitive PS = (*itp)->first.primitive();
              RealVector normal = PS.normal();
              for ( Dimension i = 0; i < 3; ++i )
                for ( Dimension j = 0; j < 3; ++j )
                  {
                    double val = A( i, j ) + normal[ i ] * normal[ j ];
                    A.setComponent( i, j, val );
                  }
              B += (PS.mu() + PS.nu()/2.0) * normal;
            }
          rq = A.inverse() * B;
        }
      polyhedron.addVertex( rq );
    }
  // Define faces of the mesh.
  // Outputs closed faces.
  FaceSet faces = digSurf.allClosedFaces();
  for ( typename FaceSet::const_iterator itf = faces.begin(), itf_end = faces.end();
        itf != itf_end; ++itf )
    {
      MeshFace mface( itf->nbVertices );
      VertexRange vtcs = digSurf.verticesAroundFace( *itf );
      int i = 0;
      for ( typename VertexRange::const_iterator itv = vtcs.begin(), itv_end = vtcs.end();
            itv != itv_end; ++itv )
        mface[ i++ ] = index[ *itv ];
      polyhedron.addFace( mface, Color( 255, 243, 150, 255 ) ); //v2plane[ *vtcs.begin() ]->second );
    }
  viewer.clear();
  viewer.show();
  viewer << polyhedron;
  viewer << MyViewer3D::updateDisplay;
  application.exec();
  //! [polyhedralisation-visualization]


  //! [polyhedralisation-freeMemory]
  for ( std::vector<SegmentedPlane*>::iterator 
          it = segmentedPlanes.begin(), itE = segmentedPlanes.end(); 
        it != itE; ++it )
    delete *it;
  segmentedPlanes.clear();
  v2plane.clear();
  //! [polyhedralisation-freeMemory]


  // viewer << Viewer3D<>::updateDisplay;
  // application.exec();


  return 0;
}
