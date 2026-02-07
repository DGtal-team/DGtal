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
 * @file SVOReader.h
 * @author Bastien Doignies (bastien.doignies@liris.cnrs.fr)
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2025/10/20
 *
 * This file is part of the DGtal library.
 */

#include <fstream>
#include <map>
#include <boost/endian/conversion.hpp>
#include "DGtal/kernel/sets/DigitalSetByOctree.h"

namespace DGtal
{
  /**
   * @brief Class to read SVO file.
   *
   * The file format is defined in SVOWriter.h
   *
   * @tparam Space The space on which the octree is defined
   * @see SVOWriter
   */
  template <class Space>
  class SVOReader
  {
  public:
    using Octree = DigitalSetByOctree<Space>;
    /**
     * @brief Imports an octree from a file
     *
     * @param filename The file in which the octree is in
     */
    static Octree importSVO(const std::string& filename);
  };
}

#include "SVOReader.ih"
