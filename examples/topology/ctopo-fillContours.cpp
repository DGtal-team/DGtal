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
 * @file ctopo-fillContours.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2014/06/16
 *
 * An example file named ctopo-fillContours.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGTal/geometry/curves/FreemanChain.h"
#include "DGtal/io/boards/Board2D.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example ctopo-fillContours" );
  DGtal::KhalimskySpaceND< 2, int > K; 
  K.init(Z2i::Point(0, 10),  Z2i::Point(20, 30), false);
  // We choose a direct and indirect oriented contour.
  FreemanChain<int> fc1 ("001001001001001111101111011222222223222222322233333330301033333003", 6, 14);
  FreemanChain<int> fc2 ("1111000033332222", 6, 20);  
  
  Board2D aBoard;
  Board2D aBoard2;
  aBoard << K.lowerBound() << K.upperBound() ;
  aBoard2 << K.lowerBound() << K.upperBound() ;
    
  //From the FreemanChain we can get a vector of SCell with sign defined from the FreemanChain orientation:
  std::set<DGtal::KhalimskySpaceND< 2, int >::SCell> boundarySCell;
  FreemanChain<int>::getInterPixelLinels(K, fc1, boundarySCell, false); 
  
  aBoard << CustomStyle((*boundarySCell.begin()).className(),  new CustomColors(DGtal::Color::Red, DGtal::Color::Red) );
  for( std::set<DGtal::KhalimskySpaceND< 2, int >::SCell>::const_iterator it= boundarySCell.begin();  it!= boundarySCell.end(); it++){
    aBoard << *it;
  }
  
  // We can also add other freeman chains with indirect orientation to construct a hole in interior of the shape:
  std::set<DGtal::KhalimskySpaceND< 2, int >::SCell> boundarySCellhole;
  FreemanChain<int>::getInterPixelLinels(K, fc2, boundarySCellhole, false); 
    
  aBoard << CustomStyle((*boundarySCell.begin()).className(),  new CustomColors(DGtal::Color::Blue, DGtal::Color::Blue) );
  aBoard2 << CustomStyle((*boundarySCell.begin()).className(),  new CustomColors(DGtal::Color::Blue, DGtal::Color::Blue) );
  
  for( std::set<DGtal::KhalimskySpaceND< 2, int >::SCell>::const_iterator it= boundarySCellhole.begin();  it!= boundarySCellhole.end(); it++){
    aBoard << *it;
    aBoard2 << *it;
    boundarySCell.insert(*it);
  }

  
  
  // Now we can compute the unsigned cell associated to interior pixels: 
  std::set< DGtal::KhalimskySpaceND< 2, int >::Cell> interiorCell;
  Surfaces<DGtal::KhalimskySpaceND< 2, int > >::uComputeInterior(K, boundarySCell, interiorCell, false);  

  aBoard << CustomStyle((*interiorCell.begin()).className(),  new CustomColors(DGtal::Color::None, Color(200, 200, 200)) );
  for(std::set<DGtal::KhalimskySpaceND< 2, int >::Cell>::const_iterator it = interiorCell.begin(); it!=interiorCell.end(); it++){
    aBoard << *it;
  }
  
  
  // We can also compute the unsigned cell associated to interior and exterior pixels: 
  std::set< DGtal::KhalimskySpaceND< 2, int >::Cell> interiorCellHole;
  std::set< DGtal::KhalimskySpaceND< 2, int >::Cell> exteriorCellHole;
  Surfaces<DGtal::KhalimskySpaceND< 2, int > >::uComputeInterior(K, boundarySCellhole, interiorCellHole, true);  
  Surfaces<DGtal::KhalimskySpaceND< 2, int > >::uComputeExterior(K, boundarySCellhole, exteriorCellHole, false);  
  

  aBoard2 << CustomStyle((*interiorCellHole.begin()).className(),  new CustomColors(DGtal::Color::None, Color(200, 200, 200)) );
  for(std::set<DGtal::KhalimskySpaceND< 2, int >::Cell>::const_iterator it = interiorCellHole.begin(); it!=interiorCellHole.end(); it++){
    aBoard2 << *it;
  }
  aBoard2 << CustomStyle((*exteriorCellHole.begin()).className(),  new CustomColors(DGtal::Color::None, Color(100, 100, 100)) );
  for(std::set<DGtal::KhalimskySpaceND< 2, int >::Cell>::const_iterator it = exteriorCellHole.begin(); it!=exteriorCellHole.end(); it++){
    aBoard2 << *it;
  }
  
  
  aBoard.saveEPS("example_ctopo-fillContours.eps");
  aBoard.saveEPS("example_ctopo-fillContours.fig");

  aBoard2.saveEPS("example_ctopo-fillContours2.eps");
  aBoard2.saveEPS("example_ctopo-fillContours2.fig");
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
