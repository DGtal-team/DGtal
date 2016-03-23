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
 * @file NeighborhoodConfigurationsHelpers.h
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

#if defined(NeighborhoodConfigurationsHelpers_RECURSES)
#error Recursive header files inclusion detected in NeighborhoodConfigurationsHelpers.h
#else // defined(NeighborhoodConfigurationsHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NeighborhoodConfigurationsHelpers_RECURSES

#if !defined NeighborhoodConfigurationsHelpers_h
/** Prevents repeated inclusion of headers. */
#define NeighborhoodConfigurationsHelpers_h
//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <bitset>
#include <memory>
#include "DGtal/topology/Object.h"
#include "DGtal/helpers/StdDefs.h"
#include <unordered_map>
#include "boost/dynamic_bitset.hpp"

namespace DGtal {
  namespace functions {
  /**
   * Get the occupancy configuration of the neighborhood of a point. The neighborhood is considered maximal regardless the topology of the object (3x3 cube for 3D point, 2x2 square for 2D).
   *
   * @tparam TObject Object type.

   * @param input_obj input object. Used to check what points are occupied.
   * @param center of the neighborhood. It doesn't matter if center belongs or not to \b input_object.
   * @param mapPointToMask map[Point]->configuration, where Point is inside a 3x3 cube centered in {0,0,0} in 3D, or 2x2 in 2D.
   * @see mapPointToBitMask
   *
   * @return bit configuration
   */
  template<typename TObject>
  unsigned int
  getOccupancyConfiguration(
      const          TObject        & input_obj,
      const typename TObject::Point & center,
      const std::unordered_map<
              typename TObject::Point, unsigned int> & mapPointToMask);

  /**
   * Get the occupancy configuration of the neighborhood of a point in a cubical
   * complex.
   * The neighborhood size is considered DxD for dimension D of the point
   * (ie 3x3 cube for 3D point).
   *
   * @tparam TComplex Complex type.

   * @param input_complex input complex. Used to check what points are occupied.
   * @param center of the neighborhood. It doesn't matter if center belongs
   * or not to \b input_complex.
   * @param mapPointToMask map[Point]->configuration, where Point is
   * inside a DxD cube centered in {0,0,..} in ND.
   *
   * @note This doesn't work with KSpace coordinates, these must be converted
   * to digital coordinates before: @see KhalimskySpaceND::uCoords(3,cell)
   * @see mapPointToBitMask
   *
   * @return bit configuration
   */

  template<typename TComplex>
  unsigned int
  getComplexOccupancyConfiguration(
      const          TComplex        & input_complex,
      const typename TComplex::Point & center,
      const std::unordered_map<
              typename TComplex::Point, unsigned int> & mapPointToMask);

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
      TMap & map );

  }// namespace functions
}// namespace DGtal
///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/NeighborhoodConfigurationsHelpers.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NeighborhoodConfigurationsHelpers_h

#undef NeighborhoodConfigurationsHelpers_RECURSES
#endif // else defined(NeighborhoodConfigurationsHelpers_RECURSES)
