/**
 * @file testObjectBorder.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/09/17
 *
 * Functions for testing class ObjectBorder.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iterator>
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
///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ObjectBorder.
///////////////////////////////////////////////////////////////////////////////

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

/**
 * Simple test to illustrate the border extraction of a simple 2D
 * object considering different topologies.
 *
 */
bool testObjectBorder()
{ 
  trace.beginBlock ( "Testing Object Borders in 2D ..." );

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


  Point p1( -20, -10 );
  Point p2( 20, 10 );
  Domain domain( p1, p2 );

  Adj4 adj4;                          // instance of 4-adjacency
  Adj8 adj8;                          // instance of 8-adjacency
  DT8_4 dt8_4(adj8,adj4, JORDAN_DT );

  Point c( 0, 0 );
  
  //We construct a simple 3-bubbles set
  DigitalSet bubble_set( domain );
  for ( DomainConstIterator it = domain.begin(); it != domain.end(); ++it )
    {
      int x = (*it)[0];
      int y = (*it)[1];
      if (( x*x+y*y < 82) || 
	  (  (x-14)*(x-14)+(y+1)*(y+1)< 17) ||
	  (  (x+14)*(x+14)+(y-1)*(y-1)< 17) )
	bubble_set.insertNew( *it);
    }
   
  ObjectType bubble( dt8_4, bubble_set );
  
  //Connectedness Check
  if (bubble.computeConnectedness() == ObjectType::CONNECTED)
    trace.info() << "The object is (8,4)connected."<<endl;
  else
    trace.info() << "The object is not (8,4)connected."<<endl;

  //Border Computation
  ObjectType bubbleBorder = bubble.border();
  if (bubbleBorder.computeConnectedness() == ObjectType::CONNECTED)
   trace.info() << "The object (8,4) border is connected."<<endl;
  else
    trace.info() << "The object (8,4) border is not connected."<<endl;
  
  //Board Exoirt
  Board board;
  board.setUnit(Board::UCentimeter);
 
  domain.selfDrawAsGrid(board);
  bubble_set.selfDraw(board);
  board.saveSVG("bubble-set.svg");
  
  bubbleBorder.selfDrawWithAdjacencies<SelfDrawStyleCustom>(board);  
  board.saveSVG("bubble-object-border.svg");
  
  board.clear();

  //////////////////////:
  //the same with the reverse topology
  typedef Object<DT8_4::ReverseTopology, DigitalSet> ObjectType48;
  DT8_4::ReverseTopology dt4_8 = dt8_4.reverseTopology();

  ObjectType48 bubble2( dt4_8, bubble_set );
  
  //Border Computation
  ObjectType48 bubbleBorder2 = bubble2.border();
  if (bubbleBorder2.computeConnectedness() == ObjectType48::CONNECTED)
    trace.info() << "The object (4,8) border is connected."<<endl;
  else
    trace.info() << "The object (4,8) border is not connected."<<endl;
  
  domain.selfDrawAsGrid(board);
  bubble_set.selfDraw(board);
  
  bubbleBorder2.selfDrawWithAdjacencies<SelfDrawStyleCustom>(board);  
  board.saveSVG("bubble-object-border-48.svg");
  
  //We split the border according to its components
  vector<ObjectType48> borders(30);
  int nbComponents; 
  
  vector<ObjectType48>::iterator it=borders.begin();
  nbComponents = bubbleBorder2.writeComponents( it );
  
  trace.info() << "The Bubble object has "<< nbComponents<< " (4,8)-connected components"<<endl;
  
  bool flag=true;
  for(unsigned int k=0;k<nbComponents ; k++)
    {
      if (flag)
	borders[k].selfDrawWithAdjacencies<SelfDrawStyleCustom>(board);
      else
	borders[k].selfDrawWithAdjacencies<SelfDrawStyleCustomRed>(board);
      flag = !flag;
    }
  
  board.saveSVG("bubble-object-color-borders-48.svg");
  trace.endBlock();
  
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
 
  bool res = testObjectBorder(); // && ... other tests
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
