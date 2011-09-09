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
 * @file gridcurve.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/09
 *
 * @brief An example file for GridCurve.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/2d/GridCurve.h"

#include "DGtal/topology/helpers/Surfaces.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i; 

///////////////////////////////////////////////////////////////////////////////
template <typename CI>
void displayAll( const CI& ciBegin, const CI& ciEnd ) 
{
  if ( isNotEmpty(ciBegin, ciEnd) ) 
  { //if the range is not empty
    CI i( ciBegin); 
    do 
    {
      trace.info() << *i << endl;
      i++;
    } while (i != ciEnd); 
  }    
}

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  trace.beginBlock ( "Example for 2d gridcurves" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  string square = examplesPath + "samples/smallSquare.dat";
  
  Curve c1,c2; 
  
  trace.emphase() << "Input" << endl;
  trace.info() << "\t from a data file " << endl;
  {
    fstream inputStream;
    inputStream.open (square.c_str(), ios::in);
    c1.initFromVectorStream(inputStream);
    inputStream.close();
  }
  trace.info() << "\t from a digital set " << endl;
  {
   // domain
   Point lowerBound( -50, -50 );
   Point upperBound( 50, 50 );
   Domain domain( lowerBound, upperBound );
    
   // digital set: diamond of radius 30 centered at the origin
   Point o( 0, 0 );
   DigitalSet set( domain );
   for ( Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it )
   {
     if ( (*it - o ).norm1() <= 30 ) set.insertNew( *it );
   }
    
    vector<Point> boundaryPoints;                              //boundary points to retrieve
    K2 ks; ks.init( lowerBound, upperBound, true );   //Khalimsky space 
    SurfelAdjacency<K2::dimension> sAdj( true );     //adjacency
    SetPredicate<DigitalSet> predicate( set );             //predicate

    //tracking and init grid curve
    SCell s = Surfaces<KSpace>::findABel( ks, predicate, 1000 );
    Surfaces<KSpace>::track2DBoundaryPoints( boundaryPoints, ks, sAdj, predicate, s );
    c2.initFromVector( boundaryPoints );
  }
  
// @TODO trace.info() << "\t from a FreemanChain (from a file) " << endl; 


  trace.emphase() << "Output" << endl;
  trace.info() << "\t standard output " << endl;
  {
    trace.info() << c1 << std::endl;
  }
  trace.info() << "\t into a data file " << endl;
  {
    ofstream outputStream("myGridCurve.dat"); 
    if (outputStream.is_open()) 
      c2.writeVectorToStream(outputStream);
    outputStream.close();
  }
  trace.info() << "\t into a vector graphics file " << endl;
  {
    Board2D aBoard;
    aBoard.setUnit(Board2D::UCentimeter);
    aBoard << c2; 
    aBoard.saveEPS( "myGridCurve.eps", Board2D::BoundingBox, 5000 );
  }
  
  // @TODO trace.info() << "\t into a FreemanChain " << endl; 
  
  trace.emphase() << "Ranges Ouput" << endl;
  {
    Board2D aBoard;
    aBoard.setUnit(Board2D::UCentimeter);

    Point low(-1,-1);
    Point up(3,3);
    Domain aDomain( low,up );

    {//1cellsRange
      Curve::SCellsRange r = c1.get1SCellsRange(); 
      
      trace.info() << r << endl;
      
      aBoard << SetMode(aDomain.styleName(), "Grid") << aDomain; 
      aBoard << r; 
      aBoard.saveEPS( "My1CellsRange.eps", Board2D::BoundingBox, 5000 );
      aBoard.clear(); 
    }
    {//IncientPointsRange
      Curve::IncidentPointsRange r = c1.getIncidentPointsRange(); 
      
      trace.info() << r << endl;
      
      aBoard << SetMode(aDomain.styleName(), "Grid") << aDomain; 
      aBoard << r; 
      aBoard.saveEPS( "MyIncidentPointsRange.eps", Board2D::BoundingBox, 5000 );
      aBoard.clear(); 
    }
    {//CodesRange
      Curve::CodesRange r = c1.getCodesRange(); 
      
      trace.info() << r << endl;
    }
  }
  
  trace.emphase() << "Ranges Iterators" << endl;
  {
    typedef Curve::PointsRange Range; 
    Range r = c1.getPointsRange(); 
    
    trace.info() << "\t iterate over the range" << endl;
    Range::ConstIterator it = r.begin(); 
    Range::ConstIterator itEnd = r.end(); 
    for ( ; it != itEnd; ++it)
    {
      trace.info() << *it << endl;
    }
      
    trace.info() << "\t iterate over the range in the reverse way" << endl;
    Range::ConstReverseIterator rit = r.rbegin(); 
    Range::ConstReverseIterator ritEnd = r.rend(); 
    for ( ; rit != ritEnd; ++rit) 
    {
      trace.info() << *rit << endl;
    }
      
    trace.info() << "\t iterate over the range in a circular way" << endl;
    Range::ConstCirculator c = r.c();
    //set the starting element wherever you want... 
    for (unsigned i = 0; i < 20; ++i) ++c; 
    //... and circulate
    Range::ConstCirculator cend( c );
    do 
    {
      trace.info() << *c << endl;
      c++;
    } while (c!=cend);      
    
    trace.info() << "\t Generic function working with any (circular)iterator" << endl;
    displayAll<Range::ConstIterator>(r.begin(),r.end()); 
    displayAll<Range::ConstReverseIterator>(r.rbegin(),r.rend()); 
    displayAll<Range::ConstCirculator>(r.c(),r.c()); 
    
  }
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
