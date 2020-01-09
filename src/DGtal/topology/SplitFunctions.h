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
 * @file SplitFunctions.h
 * @author Pablo Hernandez-Cerdan (\c pablo.hernandez.cerdan@outlook.com)
 *
 * @date 2018/01/08
 *
 * Defines functions associated to split domains for multi-threading purposes.
 *
 * This file is part of the DGtal library.
 */

#if defined(SplitFunctions_RECURSES)
#error Recursive header files inclusion detected in SplitFunctions.h
#else // defined(SplitFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SplitFunctions_RECURSES

#if !defined SplitFunctions_h
/** Prevents repeated inclusion of headers. */
#define SplitFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////
namespace DGtal
{
  namespace functions {
  /**
   * Compute the number of splits given a region defined by lowerBound and upperBound
   *
   * @tparam TPoint point type for lower and upper bound
   * @param requested_number_of_splits
   * @param lowerBound
   * @param upperBound
   * @param splits[] the splits for each dimension
   *
   * @return number of splits, lesser or equal than requested_number_of_splits
   */
    template <typename TPoint>
      size_t computeSplits(
          const size_t requested_number_of_splits,
          const TPoint & lowerBound,
          const TPoint & upperBound,
          unsigned int splits[]);

    /**
     * Get the ith split, where i has to be less than the number of splits
     * computed by @computeSplits.
     *
     * Returns two points corresponding to the output_lowerBound and output_upperRegion
     * of the ith split.
     *
     * @tparam TPoint point type for lower and upper bound
     * @param splitIndex the ith split to be computed
     * @param requested_number_of_splits
     * @param lowerBound
     * @param upperBound
     *
     * @return array containing output_lowerBound and output_upperBound
     */
    template <typename TPoint>
      std::array<TPoint, 2> getSplit(
          const size_t splitIndex,
          const size_t requested_number_of_splits,
          const TPoint & lowerBound,
          const TPoint & upperBound);

    /**
     * Split a CubicalComplex (or VoxelComplex) into sub_complexes
     *
     * @tparam TComplex
     * @param vc
     * @param requested_number_of_splits (the actual splits might be less)
     *
     * @return vector with sub_complexes
     *
     * @sa computeSplits
     * @sa getSplit
     */
    template < typename TComplex >
    std::vector<TComplex>
    splitComplex(
       const TComplex & vc ,
       const size_t requested_number_of_splits
       );

  } // namespace functions
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/SplitFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SplitFunctions_h

#undef SplitFunctions_RECURSES
#endif // else defined(SplitFunctions_RECURSES)
