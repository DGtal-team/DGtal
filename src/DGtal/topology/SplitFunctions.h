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
  } // namespace functions


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
      /** number of splits per dimension */
      std::vector<unsigned int> splits;
      /** Two points defining the domain of the splits */
      using PointsPair = std::array<typename TComplex::Point, 2>;
      /** Vector with the domains of the original splits.
       * If wide_of_ghost_layer is 0, this is the domain of the complexes,
       * otherwise, the domain is stored in splits_with_ghost_layers. */
      std::vector<PointsPair> splits_domain;
      /** Wide of the ghost layer that was added to the original split */
      size_t wide_of_ghost_layer;
      /** Vector with the domains of the splits with ghost layers added.
       * If wide_of_ghost_layer is NOT 0, this is the domain of the complexes,
       * otherwise is empty, and the domain is stored in splits. */
      std::vector<PointsPair> splits_domain_with_ghost_layers;
    };

  namespace functions {

    /**
     * Split a CubicalComplex (or VoxelComplex) into sub_complexes.
     *
     * @tparam TComplex
     * @param[in] vc input complex
     * @param[in] requested_number_of_splits (the actual splits might be less)
     * @param[in] wide_of_ghost_layer enlarge the splits with a number of voxels
     * The enlarged area (ghost layers) are shared between different sub_complexes
     * The ghost layers are useful for multi-threading purposes, where
     * they can be marked as FIXED (no thinning on this area), then thin each sub_complex
     * and finally create new sub_complexes containing the ghost domains, remove the FIXED
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

    /**
     * Perform an union between out and each complex in complexes.
     * out |= complex;
     *
     * @tparam TComplex complex type
     * @param[in,out] out complex where the merge is stored
     * @param[in] complexes vector of complexes
     */
    template < typename TComplex >
    void
    mergeSubComplexes(TComplex & out,
        const std::vector<TComplex> &complexes);

    template < typename TComplex >
      TComplex
    extractSubComplex(
       const TComplex & vc ,
       const typename TComplex::Point & sub_lowerBound,
       const typename TComplex::Point & sub_upperBound
       );

    /**
     * Get the voxels from the border of the input vc.
     * If lowerBound_to_ignore and upperBound_to_ignore are set, the borders
     * defined by those bounds are ignored.
     *
     * lowerBound and upperBound are usually the lower and upper bound
     * of the input complex vc, but they can also be changed to get the
     * borders of a smaller hypercube contained in vc.
     *
     * @tparam TComplex
     * @param vc input complex
     * @param lowerBound the lowerBound of the hypercube defining a border (usually vc.lowerBound().
     * @param upperBound the upperBound of the hypercube defining a border (usually vc.upperBound().
     * @param wide_point the wide of the border (3D), defaults to nullptr, which is equivalent to {0,0,0}
     * it has to be greater than zero.
     * @param lowerBound_to_ignore lowerBound defining an hypercube of borders to ignore
     * @param upperBound_to_ignore upperBound defining an hypercube of borders to ignore
     *
     * @return vector with iterators of the border
     */
    template < typename TComplex >
      std::vector<typename TComplex::CellMapIterator>
      getBorderVoxels(
          TComplex & vc,
          const typename TComplex::Point & lowerBound,
          const typename TComplex::Point & upperBound,
          const typename TComplex::Point * wide_point = nullptr,
          const typename TComplex::Point * lowerBound_to_ignore = nullptr,
          const typename TComplex::Point * upperBound_to_ignore = nullptr
          );

    /**
     * Helper function to get the border voxels obtained from
     * @sa getBorderVoxels to the desired data.
     *
     * @tparam TComplex Complex type
     * @param vc input complex
     * @param border_iterators iterators from getBorderVoxels
     * @param data data of the cell to set in the border
     */
    template < typename TComplex >
      void
      setBorderData(
          TComplex & vc,
          const std::vector<typename TComplex::CellMapIterator> &border_iterators,
          const DGtal::uint32_t &data);
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
