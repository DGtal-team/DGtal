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
 * @file generateSimplicityTables3D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/06/22
 *
 * An example file named generateSimplicityTables3D. Creates precomputed
 * tables for determining whether some point is simple within an
 * object.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <bitset>
#include "DGtal/topology/Object.h"
#include "DGtal/helpers/StdDefs.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

/**
   Given a digital topology \a dt, generates tables that tells if the
   central point is simple for the specified configuration. The
   configuration is determined by a sequence of bits, the first bit
   for the point in the neighborhood, the second bit for the second
   point, etc. When set to one, the point is in the neighborhood.

   @tparam Object the type of object whose simpleness we wish to
   precompute. Includes the topology.

   @tparam Map the type used to store the mapping configuration -> bool.

   @param dt an instance of the digital topology.
   @param map (modified) the mapping configuration -> bool.
*/
template <typename Object, typename Map>
void 
generateSimplicityTable( const typename Object::DigitalTopology & dt,
			 Map & map )
{
  typedef typename Object::DigitalSet DigitalSet;
  typedef typename Object::Point Point;
  typedef typename DigitalSet::Domain Domain;
  typedef typename Domain::ConstIterator DomainConstIterator;

  Point p1 = Point::diagonal( -1 );
  Point p2 = Point::diagonal(  1 );
  Point c = Point::diagonal( 0 );
  Domain domain( p1, p2 );
  DigitalSet shapeSet( domain );
  Object shape( dt, shapeSet );
  unsigned int k = 0;
  for ( DomainConstIterator it = domain.begin(); it != domain.end(); ++it )
    if ( *it != c ) ++k;
  ASSERT( ( k < 32 )
	  && "[generateSimplicityTable] number of configurations is too high." );
  unsigned int nbCfg = 1 << k;
  for ( unsigned int cfg = 0; cfg < nbCfg; ++cfg )
    {
      if ( ( cfg % 1000 ) == 0 )
	{
	  trace.progressBar( (double) cfg, (double) nbCfg );
	}
      shape.pointSet().clear();
      shape.pointSet().insert( c );
      unsigned int mask = 1;
      for ( DomainConstIterator it = domain.begin(); it != domain.end(); ++it )
	{
	  if ( *it != c )
	    {
	      if ( cfg & mask ) shape.pointSet().insert( *it );
	      mask <<= 1;
	    }
	}
      bool simple = shape.isSimple( c );
      map[ cfg ] = simple;
    }
}



int main( int /*argc*/, char** /*argv*/ )
{
  typedef std::bitset<67108864> ConfigMap; // 2^26

  using namespace Z3i;
  trace.beginBlock ( "Generate 3d table for 26_6 topology" );
  // Too big for stack. Use heap instead.
  ConfigMap* table26_6 = new ConfigMap;
  generateSimplicityTable< Object6_26 >( dt6_26, *table26_6 );
  trace.endBlock();

  ofstream file26_6( "simplicity_table6_26.txt" );
  file26_6 << *table26_6;
  file26_6.close();

  delete table26_6;
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
