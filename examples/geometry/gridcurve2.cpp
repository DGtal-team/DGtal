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

 DigitalSet getDiamondSet( ) 
 {
 
   Point p1( -50, -50 );
   Point p2( 50, 50 );
   Domain domain( p1, p2 );
   // diamond of radius 30 centered at the origin
   Point c( 0, 0 );
   DigitalSet aSet( domain );
   for ( Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it )
   {
     if ( (*it - c ).norm1() <= 30 ) aSet.insertNew( *it );
   }

   return aSet; 
 }


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
   Point c( 0, 0 );
   DigitalSet set( domain );
   for ( Domain::ConstIterator it = domain.begin(); it != domain.end(); ++it )
   {
     if ( (*it - c ).norm1() <= 30 ) set.insertNew( *it );
   }
    
    vector<Point> boundaryPoints;                              //boundary points to retrieve
    K2 ks; ks.init( lowerBound, upperBound, true );   //Khalimsky space 
    SurfelAdjacency<K2::dimension> sAdj( true );     //adjacency
    SetPredicate<DigitalSet> predicate( set );             //predicate

    //tracking and init grid curve
    SCell s = Surfaces<KSpace>::findABel( ks, predicate, 1000 );
    Surfaces<KSpace>::track2DBoundaryPoints( boundaryPoints, ks, sAdj, predicate, s );
    c1.initFromVector( boundaryPoints );
  }
  
// @TODO trace.info() << "\t from a FreemanChain (from a file) " << endl; 


  trace.emphase() << "Output" << endl;
  trace.info() << "\t standard output " << endl;
  {
    trace.info() << c1 << std::endl;
  }
  trace.info() << "\t into a data file " << endl;
  trace.info() << "\t into a vector graphics file " << endl;
  // @TODO trace.info() << "\t from a FreemanChain (from a file) " << endl; 
  
  trace.emphase() << "Ranges Ouput" << endl;
  
  trace.emphase() << "Ranges Iterators" << endl;
  
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
