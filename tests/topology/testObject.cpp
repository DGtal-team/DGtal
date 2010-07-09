/**
 * @file testObject.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/08
 *
 * Functions for testing class Object.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/sets/DigitalSetConverter.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/Object.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Object.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testObject()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef SpaceND< int, 2 > Z2;
  typedef Z2::Point Point;
  typedef MetricAdjacency< Z2, 1 > Adj4;
  typedef MetricAdjacency< Z2, 2 > Adj8;
  typedef DigitalTopology< Adj4, Adj8 > DT48;
  typedef HyperRectDomain< Z2 > DomainType; 
  Adj4 adj4;
  Adj8 adj8;
  DT48 dt48( adj4, adj8, JORDAN_DT );
  typedef DigitalSetSelector< DomainType, MEDIUM_DS+HIGH_BEL_DS >::Type 
     MediumSet;
//   typedef DigitalSetSelector< DomainType, SMALL_DS >::Type 
//     MediumSet;
  Point p1( { -500, -500 } );
  Point p2( { 500, 500 } );
  Point c( { 0, 0 } );
  Point l( { 449, 0 } );
  DomainType domain( p1, p2 );
  MediumSet disk( domain );
  trace.beginBlock ( "Creating disk( r=450.0 ) ..." );
  for ( DomainType::ConstIterator it = domain.begin(); 
	it != domain.end();
	++it )
    {
      if ( (*it - c ).norm() < 450.0 )
	// insertNew is very important for vector container.
	disk.insertNew( *it );
    }
  trace.endBlock();

  trace.beginBlock ( "Testing Object instanciation and smart copy  ..." );
  Object<DT48, MediumSet> disk_object( dt48, disk );
  nbok += disk_object.size() == 636101 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Disk (r=450.0) " << disk_object << std::endl;
  trace.info() << "  size=" << disk_object.size() << std::endl;
  Object<DT48, MediumSet> disk_object2( disk_object );
  nbok += disk_object2.size() == 636101 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Disk2 (r=450.0) " << disk_object2 << std::endl;
  trace.info() << "  size=" << disk_object2.size() << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing copy on write system ..." );
  trace.info() << "Removing center point in Disk." << std::endl;
  disk_object.pointSet().erase( c );
  disk_object2.pointSet().insert( c );
  nbok += disk_object.size() == 636100 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Disk - c (r=450.0) " << disk_object << std::endl;
  trace.info() << "  size=" << disk_object.size() << std::endl;
  nbok += disk_object2.size() == 636101 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Disk2 + c (r=450.0) " << disk_object2 << std::endl;
  trace.info() << "  size=" << disk_object2.size() << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing neighborhoods ..." );
  typedef Object<DT48, MediumSet>::SmallSet SmallSet;
  Object<DT48, SmallSet> neigh = disk_object.neighborhood( c );
  nbok += neigh.size() == 4 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "N_4(Disk, c).size() = " << neigh.size() 
	       << " == 4" << std::endl;
  neigh = disk_object.properNeighborhood( l );
  nbok += neigh.size() == 3 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "N*_4(Disk, " << l << ").size() = " << neigh.size()
	       << " == 3" << std::endl;
  typedef Object<DT48, MediumSet>::SizeType SizeType;
  SizeType size = disk_object.properNeighborhoodSize( l );
  nbok += size == 3 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "#N*_4(Disk, " << l << ") = " << size
	       << " == 3" << std::endl;

  neigh = disk_object2.neighborhood( c );
  nbok += neigh.size() == 5 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "N_4(Disk2, c).size() = " << neigh.size() 
	       << " == 5" << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing set converters ..." );
  DigitalSetConverter<SmallSet>::assign
    ( neigh.pointSet(), disk_object.pointSet() );
  nbok += neigh.size() == 636100 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "neigh = disk_object, size() = " << neigh.size() 
	       << " == 636100" << std::endl;
  Object<DT48, SmallSet> neigh2 = disk_object2.neighborhood( c );
  DigitalSetConverter<SmallSet>::assign
    ( neigh.pointSet(), neigh2.pointSet() );
  nbok += neigh.size() == 5 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "neigh = N_4(Disk2, c), size() = " << neigh.size() 
	       << " == 5" << std::endl;
  trace.endBlock();

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class Object" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testObject(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
