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
   * @brief Helper class to write an octree to a file.
   *
   * The file format is structured in two sections in this order: 
   * the header and the data.
   *
   * Header:
   *  - The header uses ASCII.
   *  - The fields have a case sensitive name, the value is separated
   *  by a ':'. 
   *  - Each field is defined on its own line. A second ':' in a line would 
   *  be part of the value.
   *  - Spaces (' ', '\\r', '\\t') around the value are ignored. 
   *  - There is technically no specified order, although it is recommended 
   *  to follow the one below.
   *  - Fields are:
   *    - 'Format': Always 'SVO'.
   *    - 'Compression': Either '1' or '0', indicates if the data is compressed
   *    - 'Size': [uint], The number of voxels stored within the Octree.
   *    - 'State': [uint], Whether this is an SVO or SVDAG.
   *    - 'Dim': [uint], The dimension of the voxels.
   *    - 'LowerBound': int[Dim], Exactly 'Dim' integers separated by spaces 
   *    giving domain lower bound.
   *    - 'UpperBound': int[Dim], Exactly 'Dim' integers separated by spaces
   *    giving domain upper bound.
   *  The header ends with a single '.' on the last line.
   *
   * Data:
   *  - The data are always written in binary.
   *  - When compressed, the data are compressed by the zlib algorithm with
   *  the best compression level available (9).
   *  - The uncompressed data are ALWAYS defined in little-endian order. 
   *  - The data are laid out in the following order:
   *    - SizeSize: 8 bytes: Size of array length in bytes.
   *    - PtrSize:  8 bytes: Size of childen indices in bytes.
   *    - Depth: SizeSize bytes: Depth of the SVO.
   *    - ChildrenCounts: (SizeSize bytes)[Depth]: Number of children at each level.
   *    - Chidlren: (PtrSize bytes)[...] : Indices for each children in the order of
   *    Depth, Child index, Ptr Index.
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
       * @return Returns always true. Error are handle by exception and written to trace.
       *
       */
      static bool exportSVO(const std::string& filename, const DigitalSetByOctree<Space>& octree, const bool compressed = true);
  };
}

#include "SVOWriter.ih"
