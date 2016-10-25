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
 * @file
 *
 * @author Pablo Hernandez-Cerdan. Institute of Fundamental Sciences.
 * Massey University. Palmerston North, New Zealand
 *
 * @date 2016/03/25
 *
 * Generation of Look Up Tables for predicates function in a voxel
 * neighborhood.
 *
 * This file is part of the DGtal library.
 */

#if defined(NeighborhoodConfigurationsGenerators_RECURSES)
#error Recursive header files inclusion detected in NeighborhoodConfigurationsGenerators.h
#else // defined(NeighborhoodConfigurationsGenerators_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NeighborhoodConfigurationsGenerators_RECURSES

#if !defined NeighborhoodConfigurationsGenerators_h
/** Prevents repeated inclusion of headers. */
#define NeighborhoodConfigurationsGenerators_h
//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <bitset>
#include <memory>
#include "DGtal/topology/Object.h"
#include "DGtal/helpers/StdDefs.h"
#include <unordered_map>
#include "boost/dynamic_bitset.hpp"
#include <DGtal/topology/helpers/NeighborhoodConfigurationsHelper.h>

namespace DGtal {
  namespace functions {

  /**
    Given a digital topology \a dt, generates tables that tells if the
    central point is simple for the specified configuration. The
    configuration is determined by a sequence of bits, the first bit
    for the point in the neighborhood, the second bit for the second
    point, etc. When set to one, the point is in the neighborhood.

    @tparam TObject the type of object whose simpleness we wish to
    precompute. Includes the topology.
    @tparam TMap the type used to store the mapping configuration -> bool.

    @param dt an instance of the digital topology.
    @param map (modified) the mapping configuration -> bool.
  */
  template <typename TObject, typename TMap>
  void
  generateSimplicityTable(
      const typename TObject::DigitalTopology & dt,
      TMap & map )
  {
    typedef typename TObject::DigitalSet DigitalSet;
    typedef typename TObject::Point Point;
    typedef typename DigitalSet::Domain Domain;
    typedef typename Domain::ConstIterator DomainConstIterator;

    Point p1 = Point::diagonal( -1 );
    Point p2 = Point::diagonal(  1 );
    Point c = Point::diagonal( 0 );
    Domain domain( p1, p2 );
    DigitalSet shapeSet( domain );
    TObject shape( dt, shapeSet );
    unsigned int k = 0;
    for ( DomainConstIterator it = domain.begin(); it != domain.end(); ++it )
      if ( *it != c ) ++k;
    ASSERT( ( k < 32 )
        && "[generateSimplicityTable] number of configurations is too high." );
    unsigned int nbCfg = 1 << k;
    for ( NeighborhoodConfiguration cfg = 0; cfg < nbCfg; ++cfg )
    {
      if ( ( cfg % 1000 ) == 0 )
      {
        trace.progressBar( (double) cfg, (double) nbCfg );
      }
      shape.pointSet().clear();
      shape.pointSet().insert( c );
      NeighborhoodConfiguration mask = 1;
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

  }// namespace functions
}// namespace DGtal
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NeighborhoodConfigurationsGenerators_h

#undef NeighborhoodConfigurationsGenerators_RECURSES
#endif // else defined(NeighborhoodConfigurationsGenerators_RECURSES)
