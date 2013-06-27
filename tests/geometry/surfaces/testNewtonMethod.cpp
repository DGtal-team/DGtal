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
 * @file testNewtonMethod.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/11/28
 *
 * Functions for testing newton method (approximation of a surface).
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/shapes/parametric/Ball2D.h"
#include "DGtal/topology/SCellsFunctors.h"

///////////////////////////////////////////////////////////////////////////////


using namespace DGtal;

template< typename SCell, typename KSpace, typename Shape >
double getRatioSurface( SCell & inner, SCell & outter, KSpace & k, Shape & shape, unsigned int nbIter )
{
    SCellToMidPoint< KSpace > midpoint( k );

    Z2i::RealPoint low = midpoint( inner );
    Z2i::RealPoint high = midpoint( outter );
    Z2i::RealPoint normal = high - low;
    double normOriginalNormal = normal.norm();

    unsigned int iter = 0;

    Z2i::RealPoint test;
    while ( iter <= nbIter )
    {
        test = Z2i::RealPoint( low + normal / 2.0 );
        DGtal::Orientation inout = shape.orientation( test );
        if( inout == ON )
        {
            return (( test - midpoint( inner )).norm() / normOriginalNormal );
        }
        else if( inout == OUTSIDE )
        {
            high = test;
            normal /= 2.0;
        }
        else
        {
            low = test;
            normal /= 2.0;
        }

        ++iter;
    }

    return (( test - midpoint( inner )).norm() / normOriginalNormal );
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing newton method (approximation of a surface).
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testNewtonMethod ( )
{
    /// Construction of a shape

    double radius = 20;
    typedef Ball2D< Z2i::Space > MyShape;
    MyShape shape( 0, 0, radius );

    /// Construction of 2 points on the surface of the digitzed shape
    double h = 1;
    Z2i::Point aa( - radius / h - 5, - radius / h - 5 );
    Z2i::Point bb( radius / h + 5, radius / h + 5 );
    Z2i::Domain domain( aa, bb );
    Z2i::KSpace kSpace;
    bool space_ok = kSpace.init( domain.lowerBound(), domain.upperBound(), true );
    if ( !space_ok )
    {
      trace.error() << "Error in the Khalimsky space construction." << std::endl;
      return 2;
    }

    typedef Z2i::KSpace::SCell SCell;
    SCell inner( Z2i::Point( 2 * ( 20 / h ) - 1, 1 ), true );
    SCell outter( Z2i::Point( 2 * ( 20 / h ) + 1, 1 ), false );

    /// Getting ratio of each
    std::cout << "ratio: " <<getRatioSurface( inner, outter, kSpace, shape, 50 ) << std::endl;

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing Newton Method" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << std::endl;

  bool res = testNewtonMethod( ); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
