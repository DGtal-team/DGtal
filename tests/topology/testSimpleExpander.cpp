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
  typedef SpaceND<int,2> Z2;          // Z^2
  typedef Z2::Point Point;
  typedef MetricAdjacency<Z2,1> Adj4; // 4-adjacency type
  typedef MetricAdjacency<Z2,2> Adj8; // 8-adjacency type
  typedef DigitalTopology< Adj8, Adj4 > DT8_4; //8,4 topology type
  typedef HyperRectDomain< Z2 > Domain; 
  typedef Domain::ConstIterator DomainConstIterator;
  typedef DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
  typedef Object<DT8_4, DigitalSet> ObjectType;


  typedef Object<DT8_4::ReverseTopology, DigitalSet> ObjectTypeReverseTopo;

  typedef Expander<ObjectTypeReverseTopo> ObjectExpander;

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

  DigitalSet houseSetCompl( domain);
  houseSetCompl = houseSet.computeComplement();
   
  ObjectType house8( dt8_4, houseSet );
  ObjectTypeReverseTopo house4(dt4_8, houseSet);

  //Board Export
  Board board;
  board.setUnit(Board::UCentimeter);
  
  domain.selfDrawAsGrid(board);
  house8.selfDrawWithAdjacencies(board);
  board.saveSVG("house8.svg");
  
  house4.selfDrawWithAdjacencies(board);
  board.saveSVG("house4.svg");
  
  houseSetCompl.selfDraw<SelfDrawStyleCustom>(board);
  board.saveSVG("house4-compl.svg");
  
  /* ObjectExpander expander(house_compl, Point(0,0));
  while (!expander.finished())
    {
      for ( ObjectExpander::ConstIterator it = expander.begin();
	    it != expander.end();
	    ++it )
        std::cout << " " << *it;
      
      expander.nextLayer();
    }
  */


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
