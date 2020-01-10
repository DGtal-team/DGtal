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
   * @param[in] requested_number_of_splits number of splits (the actual splits might be less)
   * @param[in] lowerBound the lower bound point of the domain
   * @param[in] upperBound the upper bound point of the domain
   * @param[in,out] splits the splits for each dimension
   * For example, initialize splits with a std::vector, and use data to get the raw array
   * needed for this function.
   * @code
   * std::vector<unsigned int> splits(dimension);
   * auto number_of_splits = computeSplits(requested_number_of_splits,
   *                           lowerBound, upperBound, splits.data());
   * @endcode
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
     * computed by @sa computeSplits.
     *
     * Returns two points corresponding to the output_lowerBound and output_upperRegion
     * of the ith split.
     *
     * @tparam TPoint point type for lower and upper bound
     * @param[in] splitIndex the ith split to be computed
     * @param[in] requested_number_of_splits number of splits (the actual splits might be less)
     * @param[in] lowerBound the lower bound point of the domain
     * @param[in] upperBound the upper bound point of the domain
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
     * Return struct of @sa splitComplex
     *
     * Holds the splitted sub_complexes and information about the splits.
     * Including the domains with ghost layers if this was requested in @sa splitComplex.
     *
     * @tparam TComplex Complex type
     */
    template <typename TComplex>
    struct SplittedComplexes {
      /** Number of splits (might be less than the requested_number_of_splits) */
      size_t number_of_splits;
      /** Vector of sub_complexes, with the data copied from the original complex */
      std::vector<TComplex> sub_complexes;
      /** Two points defining the domain of the splits */
      using PointsPair = std::array<typename TComplex::Point, 2>;
      /** Vector with the domains of the original splits.
       * If wide_of_ghost_layer is 0, this is the domain of the complexes,
       * otherwise, the domain is stored in splits_with_ghost_layers. */
      std::vector<PointsPair> splits;
      /** Wide of the ghost layer that was added to the original split */
      size_t wide_of_ghost_layer;
      /** Vector with the domains of the splits with ghost layers added.
       * If wide_of_ghost_layer is NOT 0, this is the domain of the complexes,
       * otherwise is empty, and the domain is stored in splits. */
      std::vector<PointsPair> splits_with_ghost_layers;
    };

    /**
     * Split a CubicalComplex (or VoxelComplex) into sub_complexes
     *
     * @tparam TComplex
     * @param[in] vc input complex
     * @param[in] requested_number_of_splits (the actual splits might be less)
     * @param[in] wide_of_ghost_layer enlarge the splits with a number of voxels
     * The enlarged area (ghost layers) are shared between different sub_complexes
     * The ghost layers are useful for multi-threading purposes, where
     * they can be marked as FIXED (no thinning on this area), then thin each sub_complex
     * and finally create new sub_complexes containing the ghost domains, removed the FIXED
     * tag and thin them. This way we can perform a faster thinning for big domains.
     *
     * @return SplittedComplexes with the sub_complexes and the splits domain
     *
     * @sa computeSplits
     * @sa getSplit
     */
    template < typename TComplex >
    SplittedComplexes<TComplex>
    splitComplex(
       const TComplex & vc ,
       const size_t requested_number_of_splits,
       const size_t wide_of_ghost_layer = 0
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
