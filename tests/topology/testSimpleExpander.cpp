/**
 * @file testSimpleExpander.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/20
 *
 * Functions for testing class SimpleExpander.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/DomainPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/sets/DigitalSetConverter.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/DomainMetricAdjacency.h"
#include "DGtal/topology/DomainAdjacency.h"
#include "DGtal/topology/DigitalTopology.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/Expander.h"
#include "Board/Board.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

struct SelfDrawStyleCustom
{
  SelfDrawStyleCustom(LibBoard::Board & aboard)
  {
    aboard.setFillColorRGBi(0,169,0);
  }
};

struct SelfDrawStyleCustomRed
{
  SelfDrawStyleCustomRed(LibBoard::Board & aboard)
  {
    aboard.setFillColorRGBi(169,0,0);
  }
};
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SimpleExpander.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleExpander()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "(4,8) Filling ..." );

  typedef int Integer;                // choose your digital line here.
  typedef SpaceND<2> Z2;          // Z^2
  typedef Z2::Point Point;
  typedef MetricAdjacency<Z2,1> Adj4; // 4-adjacency type
  typedef MetricAdjacency<Z2,2> Adj8; // 8-adjacency type
  typedef DigitalTopology< Adj8, Adj4 > DT8_4; //8,4 topology type
  typedef HyperRectDomain< Z2 > Domain; 
  typedef Domain::ConstIterator DomainConstIterator;
  typedef DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
  typedef Object<DT8_4, DigitalSet> ObjectType;


  typedef Object<DT8_4::ReverseTopology, DigitalSet> ObjectTypeReverseTopo;

  typedef Expander<ObjectTypeReverseTopo> ObjectExpanderReverseTopo;
  typedef Expander<ObjectType> ObjectExpander;

  Point p1( -5, -5 );
  Point p2( 5, 5 );
  Domain domain( p1, p2 );

  Adj4 adj4;                          // instance of 4-adjacency
  Adj8 adj8;                          // instance of 8-adjacency
  DT8_4 dt8_4(adj8,adj4, JORDAN_DT );
  DT8_4::ReverseTopology dt4_8(adj4,adj8, JORDAN_DT );

  //We construct a simple "house" set
  DigitalSet houseSet( domain );

  for ( int k=-3; k <3 ; k++)
    {
      houseSet.insert(Point(k,-3));
      houseSet.insert(Point(-3,k));
      houseSet.insert(Point(3,k));
      houseSet.insert(Point(k,3));
    }

  //We compute the complement
  DigitalSet houseSetCompl( domain);
  houseSetCompl.assignFromComplement( houseSet );
   
  //We create the objects associated to the sets
  ObjectType house8( dt8_4, houseSet );
  ObjectType houseCompl8( dt8_4, houseSetCompl );
  ObjectTypeReverseTopo house4(dt4_8, houseSet);
  ObjectTypeReverseTopo houseCompl4( dt4_8, houseSetCompl );
  

  //Board Export init
  Board board;
  board.setUnit(Board::UCentimeter);
   
  //Border=4 Filling=4
  board.clear();
  domain.selfDrawAsGrid(board);
  house4.selfDrawWithAdjacencies(board);
  ObjectExpanderReverseTopo expander(houseCompl4, Point(0,0));
  while (!expander.finished())
    {
      for ( ObjectExpander::ConstIterator it = expander.begin();
	    it != expander.end();
	    ++it )
        std::cout << " " << *it;
      
      expander.nextLayer();
    } 
  expander.core().selfDraw<SelfDrawStyleCustom>(board);
  board.saveSVG("house4-4.svg");
  
  //Border=4 Filling=8
  board.clear();
  domain.selfDrawAsGrid(board);
  house4.selfDrawWithAdjacencies(board);
  ObjectExpander expander8(houseCompl8, Point(0,0));
  while (!expander8.finished())
    {
      for ( ObjectExpander::ConstIterator it = expander8.begin();
	    it != expander8.end();
	    ++it )
        std::cout << " " << *it;
      
      expander8.nextLayer();
    } 
  expander8.core().selfDraw<SelfDrawStyleCustom>(board);
  board.saveSVG("house4-8.svg");
  
  //Border=8 Filling=8
  board.clear();
  domain.selfDrawAsGrid(board);
  house8.selfDrawWithAdjacencies(board); 
  ObjectExpander expander88(houseCompl8, Point(0,0));
  while (!expander88.finished())
    {
      for ( ObjectExpander::ConstIterator it = expander88.begin();
	    it != expander88.end();
	    ++it )
        std::cout << " " << *it;
      
      expander88.nextLayer();
    } 
  expander88.core().selfDraw<SelfDrawStyleCustom>(board);
  board.saveSVG("house8-8.svg");

  //Border=8 Filling=4
  board.clear();
  domain.selfDrawAsGrid(board);
  house8.selfDrawWithAdjacencies(board);
  ObjectExpanderReverseTopo expander84(houseCompl4, Point(0,0));
  while (!expander84.finished())
    {
      for ( ObjectExpander::ConstIterator it = expander84.begin();
	    it != expander84.end();
	    ++it )
        std::cout << " " << *it;
      
      expander84.nextLayer();
    } 
  expander84.core().selfDraw<SelfDrawStyleCustom>(board);
  board.saveSVG("house8-4.svg");


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
   trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSimpleExpander(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
