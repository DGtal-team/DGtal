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
 * @file DomainSplitter.h
 * @author Bastien DOIGNIES (\c bastien.doignies@liris.cnrs.fr )
 * LIRIS (CNRS, UMR 5205), University of Lyon, France
 *
 * @date 2026/04/29
 *
 * This file is part of the DGtal library.
 */

#include "DGtal/kernel/domains/CDomain.h"
#include <vector>

namespace DGtal
{
  /**
   * @brief Data structure returned by Domain splitters
   *
   * This class offers the ability for the splitter to give information
   * about the surfaces or object within the splits.
   * For example, the hintVoxelCount field may allow to pre-allocate buffers.
   */
  template<typename Domain>
  struct SplitInfo
  {
    BOOST_CONCEPT_ASSERT(( concepts::CDomain< Domain > ));

    Domain domain; //< The actual split domain
    uint32_t hintVoxelCount = 0; //< An expected guess for the number of voxel
  };

  /**
   * @brief Splits a domain evenly along all dimensions
   *
   * @tparam The model of domain to split
   */
  template<typename Domain>
  struct RegularDomainSplitter
  {
    BOOST_CONCEPT_ASSERT(( concepts::CDomain< Domain > ));

    //Output spllitted domain type
    typedef std::vector<SplitInfo<Domain>> SplitDomainsInfo;

    /**
     * @brief Splits a domain
     *
     * This functions may split the domain in fewer part than wanted to ensure
     * even subdomains.
     *
     * @param d The domain to split
     * @param splitHint The targeted number of splits
     */
    SplitDomainsInfo operator()(const Domain& d, uint32_t splitHint) const
    {
      // Find best match possible for even splitting
      const uint32_t splitCount = std::floor(std::log(splitHint) / std::log(Domain::dimension));
      const uint32_t totalSplits = std::pow(splitCount, Domain::dimension);

      if (splitCount == 0)
        return { SplitInfo{d, 0} };

      auto splitSize = (d.upperBound() - d.lowerBound()) / (int32_t)splitCount;
      SplitDomainsInfo result;
      result.reserve(totalSplits);

      for (uint32_t i = 0; i < totalSplits; ++i)
      {
        auto start = d.lowerBound();
        auto idx = i;
        for (uint32_t j = 0; j < Domain::dimension; ++j)
        {
          auto k = idx % splitCount;
          start[j] += k * splitSize[j] + k; // +k ensure no overlap between domains
          idx /= splitCount;
        }

        // Make correction to ensure it remains within the domain
        auto end = start + splitSize;
        for (uint32_t j = 0; j < Domain::dimension; ++j)
          end[j] = std::clamp(end[j], d.lowerBound()[j], d.upperBound()[j]);

        result.emplace_back(Domain(start, end), 0);
      }

      return result;
    };
  };


  /**
   * @brief Splits a domain along one of the domain grid axis.
   *
   * @tparam The model of domain to split
   */
  template<typename Domain>
  struct AxisDomainSplitter
  {

    BOOST_CONCEPT_ASSERT(( concepts::CDomain< Domain > ));

    //Output spllitted domain type
    typedef std::vector<SplitInfo<Domain>> SplitDomainsInfo;

    /**
     * @brief Regularly splits a domain along one axis
     *
     * @param d The domain to split
     * @param splitHint The targeted number of splits (clamped to the width of the domain)
     * @param dim the split axis
     */
    SplitDomainsInfo operator()(const Domain& d, uint32_t splitHint, typename Domain::Dimension dim) const
    {
      SplitDomainsInfo result;
      if (splitHint == 0)
        return result;

      auto lower = d.lowerBound();
      auto upper = d.upperBound();
      auto length = upper[dim] - lower[dim] + 1;
      uint32_t splitCount = splitHint;
      if (splitCount > length)
        splitCount = length;

      result.reserve(splitCount);
      auto base = length / splitCount;
      auto rem = length % splitCount;

      auto start = lower;
      for (uint32_t i = 0; i < splitCount; ++i)
      {
        auto size = base + (i < rem ? 1 : 0);
        auto end = upper;
        end[dim] = start[dim] + size - 1;
        result.emplace_back(Domain(start, end), 0);
        start[dim] = end[dim] + 1;
      }

      return result;
    };
  };

}
