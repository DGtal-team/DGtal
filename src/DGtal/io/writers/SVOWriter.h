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

#pragma once

/**
 * @file SVOWriter.h
 * @author Bastien Doignies (bastien.doignies@liris.cnrs.fr)
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2025/10/20
 *
 * This file is part of the DGtal library.
 */

#include <fstream>
#include <boost/endian/conversion.hpp>
#include "DGtal/kernel/sets/DigitalSetByOctree.h"

namespace DGtal 
{
  /**
   * @brief Helper class to write an octree to a file using SVO file format. 
   *
   *  @ref moduleIO
   *
   *  @tparam Space The space on which the Octree is defined
   *  @see SVOReader
   */
  template<class Space> // Left here for coherence with SVOReader which can not infere Space.
  class SVOWriter 
  {
    public:
      /**
       * @brief Exports an octree to a file
       *
       * @param filename Name of the output file
       * @param octree The octree to export
       * @param compressed Whether the octree is compressed
       *
       * @return Returns always true. Errors are handled by exception and written to trace.
       *
       */
      static bool exportSVO(const std::string& filename, const DigitalSetByOctree<Space>& octree, const bool compressed = true);
  };
}

#include "SVOWriter.ih"
